// Command stream_cartesian_demo connects to a Viam machine, plans a joint-space trajectory
// tracing a shape (a figure-8, a snake/raster pattern, or a repeated line, see -shape) in the
// XZ plane with armplanning.PlanMotion, and streams the resulting joint positions to an arm
// through the motion service's stream_start/stream_push/stream_flush DoCommands. The arm first
// moves to the shape's start point via regular (non-streamed) motion, then streams through the
// rest of the shape.
//
// Configure via env vars before running:
//
//	VIAM_ADDRESS      machine address, e.g. my-machine-main.abcd1234.viam.cloud
//	VIAM_API_KEY_ID   API key ID
//	VIAM_API_KEY      API key secret
//	ARM_NAME          arm component name (defaults to "arm")
//
// Flags:
//
//	-push-sleep duration   sleep this long between each stream_push call (default 0)
//	-target-runway-ms int  target_runway_in_arm_ms stream_start option; 0 omits it so the
//	                       server default applies (default 0)
//	-shape string          shape to trace: "fig8", "snake", or "line" (default "fig8")
//	-snake-rows int        number of raster passes, snake shape only (default 4)
//	-repeats int           back-and-forth repetitions, line shape only; one rep = one pass
//	                       each direction (default 4)
//	-fig8-y float          Y (mm) of the shape's plane (default 1200)
//	-fig8-center-x float   shape center X (mm) (default 0)
//	-fig8-center-z float   shape center Z (mm) (default 700)
//	-fig8-span-x float     shape peak-to-peak span in X (mm) (default 400)
//	-fig8-span-z float     shape peak-to-peak span in Z (mm) (default 300)
//	-num-waypoints int     number of poses per pass (default 200)
//
// Run with: go run . [-shape line] [-repeats 4] [-push-sleep 10ms] [-fig8-center-x 0 -fig8-center-z 700]
package main

import (
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"math"
	"os"
	"sync"
	"time"

	vizapi "github.com/viam-labs/motion-tools/client/api"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/utils/rpc"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/motion/builtin"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

// traceHolder tracks the most recent non-nil trace seen over the course of a run, so it can
// be written out on exit regardless of whether the run succeeded or failed partway through.
type traceHolder struct {
	mu    sync.Mutex
	trace interface{}
}

// set stores t, unless t is nil -- a nil trace (e.g. from a stream_status call made after the
// session already ended) must never clobber a real trace captured earlier.
func (h *traceHolder) set(t interface{}) {
	if t == nil {
		return
	}
	h.mu.Lock()
	h.trace = t
	h.mu.Unlock()
}

func (h *traceHolder) get() interface{} {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.trace
}

// drawPreview sends poses to a locally running visualizer (../visualization, `make up`) as a
// preview before any motion starts. Clears any previously drawn preview first so repeated runs
// don't accumulate overlapping figure-8s. Best-effort: if the visualizer isn't running, the
// caller just logs a warning and continues -- this is a debugging aid, not a hard dependency.
func drawPreview(name string, poses []spatialmath.Pose) error {
	if _, err := vizapi.RemoveAll(); err != nil && !errors.Is(err, vizapi.ErrVisualizerNotRunning) {
		return fmt.Errorf("clearing previous drawings: %w", err)
	}
	_, err := vizapi.DrawPosesAsArrows(vizapi.DrawPosesAsArrowsOptions{
		Name:  name,
		Poses: poses,
	})
	return err
}

func degrees(jp []referenceframe.Input) []float64 {
	out := make([]float64, len(jp))
	for i, v := range jp {
		out[i] = utils.RadToDeg(float64(v))
	}
	return out
}

// figureEightPoses returns numPoints poses tracing a figure-8 (a 1:2-frequency Lissajous
// curve) in the XZ plane at y, centered at (centerX, centerZ), spanning spanX peak-to-peak
// in X and spanZ peak-to-peak in Z. Orientation is fixed (tool pointing straight down) for
// every point. The first and last poses coincide (t=0 and t=2*pi), closing the loop.
func figureEightPoses(centerX, centerZ, y, spanX, spanZ float64, numPoints int) []spatialmath.Pose {
	ax, az := spanX/2, spanZ/2
	poses := make([]spatialmath.Pose, numPoints)
	for i := 0; i < numPoints; i++ {
		t := 2 * math.Pi * float64(i) / float64(numPoints-1)
		poses[i] = spatialmath.NewPoseFromProtobuf(&commonpb.Pose{
			X:  centerX + ax*math.Sin(t),
			Y:  y,
			Z:  centerZ + az*math.Sin(2*t),
			OX: 0, OY: 1, OZ: 0, Theta: 0,
		})
	}
	return poses
}

// snakePoses returns numPoints poses tracing a boustrophedon ("snake"/raster) pattern in the XZ
// plane at y, centered at (centerX, centerZ), spanning spanX peak-to-peak in X and spanZ
// peak-to-peak in Z: numRows horizontal passes across X, stepping down through Z between passes,
// alternating sweep direction each row so consecutive points never jump across the shape.
// Orientation is fixed (tool pointing straight down), matching figureEightPoses. Unlike the
// figure-8, this shape does not close on itself -- the last pose does not coincide with the first.
func snakePoses(centerX, centerZ, y, spanX, spanZ float64, numPoints, numRows int) []spatialmath.Pose {
	if numRows < 2 {
		numRows = 2
	}
	pointsPerRow := numPoints / numRows
	if pointsPerRow < 2 {
		pointsPerRow = 2
	}
	ax, az := spanX/2, spanZ/2
	poses := make([]spatialmath.Pose, 0, pointsPerRow*numRows)
	for row := 0; row < numRows; row++ {
		z := centerZ + az - 2*az*float64(row)/float64(numRows-1)
		leftToRight := row%2 == 0
		for i := 0; i < pointsPerRow; i++ {
			frac := float64(i) / float64(pointsPerRow-1)
			if !leftToRight {
				frac = 1 - frac
			}
			x := centerX - ax + 2*ax*frac
			poses = append(poses, spatialmath.NewPoseFromProtobuf(&commonpb.Pose{
				X:  x,
				Y:  y,
				Z:  z,
				OX: 0, OY: 1, OZ: 0, Theta: 0,
			}))
		}
	}
	return poses
}

// linePoses returns poses for a back-and-forth line in the XZ plane at y, centered at
// (centerX, centerZ), spanning spanX peak-to-peak in X. Each rep is one forward pass (left to
// right) plus one backward pass (right to left), sharing endpoints between consecutive legs so
// the arm never stops. numPoints is the number of poses per pass.
func linePoses(centerX, centerZ, y, spanX float64, numPoints, repeats int) []spatialmath.Pose {
	if numPoints < 2 {
		numPoints = 2
	}
	ax := spanX / 2

	// One left-to-right pass.
	fwd := make([]spatialmath.Pose, numPoints)
	for i := range numPoints {
		frac := float64(i) / float64(numPoints-1)
		fwd[i] = spatialmath.NewPoseFromProtobuf(&commonpb.Pose{
			X: centerX - ax + 2*ax*frac, Y: y, Z: centerZ,
			OX: 0, OY: 1, OZ: 0, Theta: 0,
		})
	}

	// Concatenate repeats, sharing endpoints between consecutive legs.
	poses := make([]spatialmath.Pose, 0, 1+repeats*2*(numPoints-1))
	poses = append(poses, fwd[0])
	for range repeats {
		poses = append(poses, fwd[1:]...) // left→right, skip shared left endpoint
		for i := numPoints - 2; i >= 0; i-- {
			poses = append(poses, fwd[i]) // right→left, skip shared right endpoint
		}
	}
	return poses
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}

func run() error {
	pushSleep := flag.Duration("push-sleep", 0, "sleep this long between each stream_push DoCommand call (e.g. 10ms, 50ms)")
	targetRunwayMs := flag.Int("target-runway-ms", 0, "target_runway_in_arm_ms stream_start option; 0 omits it so the server default applies")
	shape := flag.String("shape", "fig8", `shape to trace: "fig8", "snake", or "line"`)
	snakeRows := flag.Int("snake-rows", 4, "number of raster passes, snake shape only")
	lineRepeats := flag.Int("repeats", 4, "back-and-forth repetitions, line shape only; one rep = one pass each direction")
	figY := flag.Float64("fig8-y", 800, "Y (mm) of the shape's plane")
	figCenterX := flag.Float64("fig8-center-x", 0, "shape center X (mm)")
	figCenterZ := flag.Float64("fig8-center-z", 700, "shape center Z (mm)")
	figSpanX := flag.Float64("fig8-span-x", 400, "shape peak-to-peak span in X (mm)")
	figSpanZ := flag.Float64("fig8-span-z", 300, "shape peak-to-peak span in Z (mm)")
	numWaypoints := flag.Int("num-waypoints", 200, "number of poses to trace the shape with")
	flag.Parse()

	ctx := context.Background()
	logger := logging.NewLogger("stream_cartesian_demo")

	address := os.Getenv("VIAM_ADDRESS")
	apiKeyID := os.Getenv("VIAM_API_KEY_ID")
	apiKey := os.Getenv("VIAM_API_KEY")
	if address == "" || apiKeyID == "" || apiKey == "" {
		return fmt.Errorf("set VIAM_ADDRESS, VIAM_API_KEY_ID, and VIAM_API_KEY")
	}
	armName := os.Getenv("ARM_NAME")
	if armName == "" {
		armName = "arm"
	}

	machine, err := client.New(ctx, address, logger, client.WithDialOptions(rpc.WithEntityCredentials(
		apiKeyID,
		rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: apiKey,
		},
	)))
	if err != nil {
		return fmt.Errorf("connecting to machine: %w", err)
	}
	defer machine.Close(ctx) //nolint:errcheck

	myArm, err := arm.FromRobot(machine, armName)
	if err != nil {
		return fmt.Errorf("getting arm %q: %w", armName, err)
	}

	motionSvc, err := motion.FromRobot(machine, "builtin")
	if err != nil {
		return fmt.Errorf("getting motion service: %w", err)
	}

	// Always write out whatever trace we've captured, on every exit path -- success, a push
	// or flush error, or a panic -- overwriting trace.json each run rather than leaving a
	// stale file from a previous run lying around to be mistaken for this one's.
	th := &traceHolder{}
	defer func() {
		// One last best-effort grab, in case the run ended (e.g. a push failed) before the
		// background poller's next tick, or before stream_flush ever ran. Uses a fresh
		// context with its own timeout since ctx itself may already be in a bad state.
		fetchCtx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		if resp, err := motionSvc.DoCommand(fetchCtx, map[string]interface{}{builtin.DoStreamStatus: true}); err == nil {
			th.set(resp["trace"])
		}

		trace := th.get()
		if trace == nil {
			logger.Warn("no trace was captured for this run -- writing trace.json as null so a previous run's file is never mistaken for this one")
		}
		const traceFile = "trace.json"
		if err := writeJSONFile(traceFile, trace); err != nil {
			logger.Warnf("failed to write %s: %v", traceFile, err)
		} else if trace != nil {
			logger.Infof("wrote %s -- render with: python3 plot_pipeline_trace.py %s", traceFile, traceFile)
		}
	}()

	// Build a minimal single-arm frame system for planning.
	model, err := myArm.Kinematics(ctx)
	if err != nil {
		return fmt.Errorf("getting arm kinematics: %w", err)
	}
	fs := referenceframe.NewEmptyFrameSystem("")
	if err := fs.AddFrame(model, fs.World()); err != nil {
		return fmt.Errorf("building frame system: %w", err)
	}
	frameName := model.Name()

	var points []spatialmath.Pose
	switch *shape {
	case "fig8":
		points = figureEightPoses(*figCenterX, *figCenterZ, *figY, *figSpanX, *figSpanZ, *numWaypoints)
	case "snake":
		points = snakePoses(*figCenterX, *figCenterZ, *figY, *figSpanX, *figSpanZ, *numWaypoints, *snakeRows)
	case "line":
		points = linePoses(*figCenterX, *figCenterZ, *figY, *figSpanX, *numWaypoints, *lineRepeats)
	default:
		return fmt.Errorf(`unknown -shape %q: want "fig8", "snake", or "line"`, *shape)
	}
	startPose := points[0]
	logger.Infof("%s: center=(%.1f, %.1f, %.1f) spanX=%.1f spanZ=%.1f numWaypoints=%d",
		*shape, *figCenterX, *figY, *figCenterZ, *figSpanX, *figSpanZ, *numWaypoints)

	// Preview the shape and the robot's current pose in the visualizer (../visualization,
	// `make up`) before any motion starts, streamed or otherwise.
	if err := drawPreview(*shape, points); err != nil {
		logger.Warnf("could not draw preview to visualizer: %v", err)
	} else {
		logger.Info("drew preview to visualizer")
	}
	if _, err := vizapi.DrawRobot(vizapi.DrawRobotOptions{Ctx: ctx, Robot: machine, ID: "robot"}); err != nil {
		logger.Warnf("could not draw robot to visualizer: %v", err)
	} else {
		logger.Info("drew robot to visualizer")
	}

	// Get the arm to startPose via regular (non-streamed, blocking) motion first, so the
	// streamed segment only ever has to cover the well-behaved start->goal interpolation --
	// not an arbitrary, possibly-large jump from wherever the arm happened to be sitting.
	//
	// Goes through motionSvc.Move (the same constrained-planning path proven to respect
	// orientation elsewhere in this script) rather than myArm.MoveToPosition directly, since
	// MoveToPosition was observed landing on the identity orientation instead of the
	// requested one -- a discrepancy specific to that raw arm-component call.
	logger.Infof("moving to start pose (non-streamed)... %v", startPose)
	if _, err := motionSvc.Move(ctx, motion.MoveReq{
		ComponentName: armName,
		Destination:   referenceframe.NewPoseInFrame(referenceframe.World, startPose),
	}); err != nil {
		return fmt.Errorf("moving to start pose: %w", err)
	}
	logger.Info("at start pose")

	// Now that the arm is actually at startPose, its live joint positions both anchor the
	// start state's collision geometry AND match the IK solution the interpolated line
	// starts from -- the first pushed waypoint should be a near-zero delta from this.
	currentJP, err := myArm.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("reading current joint positions: %w", err)
	}
	logger.Infof("current joint positions (deg): %v", degrees(currentJP))

	startState := armplanning.NewPlanState(
		referenceframe.FrameSystemPoses{frameName: referenceframe.NewPoseInFrame(referenceframe.World, startPose)},
		referenceframe.FrameSystemInputs{frameName: currentJP},
	)

	// Give the planner every figure-8 point as an ordered goal so it's forced to trace the
	// shape rather than free to take any joint-space path between just two endpoints.
	goals := make([]*armplanning.PlanState, len(points))
	for i, pose := range points {
		goals[i] = armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{frameName: referenceframe.NewPoseInFrame(referenceframe.World, pose)},
			nil,
		)
	}

	logger.Info("planning...")
	plan, _, err := armplanning.PlanMotion(ctx, logger, &armplanning.PlanRequest{
		FrameSystem: fs,
		StartState:  startState,
		Goals:       goals,
	})
	if err != nil {
		return fmt.Errorf("planning: %w", err)
	}
	traj := plan.Trajectory()
	logger.Infof("planned %d waypoints", len(traj))
	const jointDeltaWarnDeg = 15.0
	var prevDeg []float64
	for i, step := range traj {
		jp, ok := step[frameName]
		if !ok {
			logger.Warnf("waypoint %d has no entry for frame %q (have: %v)", i, frameName, mapKeys(step))
			continue
		}
		deg := degrees(jp)
		if prevDeg == nil {
			logger.Infof("waypoint %d (deg): %v", i, deg)
		} else {
			maxDelta := 0.0
			deltas := make([]float64, len(deg))
			for j := range deg {
				deltas[j] = deg[j] - prevDeg[j]
				if abs := deltas[j]; abs < 0 {
					if -abs > maxDelta {
						maxDelta = -abs
					}
				} else if abs > maxDelta {
					maxDelta = abs
				}
			}
			if maxDelta > jointDeltaWarnDeg {
				logger.Warnf("waypoint %d: joint delta %.3f deg exceeds %.1f deg threshold -- possible IK branch flip or near-singularity jump",
					i, maxDelta, jointDeltaWarnDeg)
			}
		}
		prevDeg = deg
	}

	// Abort any session left over from a previous run before starting a new one.
	// stream_abort returns {running: false} (not an error) when no session exists, so this is
	// always safe.
	abortResp, err := motionSvc.DoCommand(ctx, map[string]interface{}{builtin.DoStreamAbort: true})
	if err != nil {
		return fmt.Errorf("%s (pre-start): %w", builtin.DoStreamAbort, err)
	}
	logger.Infof("%s (pre-start): %v", builtin.DoStreamAbort, abortResp)

	startCmd := map[string]interface{}{"arm": armName}
	if *targetRunwayMs > 0 {
		startCmd["options"] = map[string]interface{}{"target_runway_in_arm_ms": *targetRunwayMs}
	}
	startResp, err := motionSvc.DoCommand(ctx, map[string]interface{}{
		builtin.DoStreamStart: startCmd,
	})
	if err != nil {
		return fmt.Errorf("%s: %w", builtin.DoStreamStart, err)
	}
	logger.Infof("%s response: %v", builtin.DoStreamStart, startResp)

	// Poll stream_status in the background for the rest of the run, so we see the moment
	// the arm halts on its own (running flips false, error appears) rather than only
	// finding out once we start polling after stream_flush at the very end. trace:false
	// keeps this cheap -- the trace only ever grows over the session, and the only place
	// that matters is the deferred fetch at the very end, right before it's written to
	// trace.json; plotting only happens after the script exits, so there's nothing to gain
	// from re-fetching and re-serializing the whole thing on every poll.
	statusCtx, stopStatusPoll := context.WithCancel(ctx)
	defer stopStatusPoll()
	go func() {
		ticker := time.NewTicker(500 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-statusCtx.Done():
				return
			case <-ticker.C:
				resp, err := motionSvc.DoCommand(statusCtx, map[string]interface{}{
					builtin.DoStreamStatus: map[string]interface{}{"trace": false},
				})
				if err != nil {
					return
				}
				if running, _ := resp["running"].(bool); !running {
					logger.Infof("%s (poll): %v", builtin.DoStreamStatus, resp)
				}
			}
		}
	}()

	// Push one waypoint at a time (rather than one big batch) so progress is visible as we go.
	lastProgressLog := time.Now()
	for i, step := range traj {
		jp := step[frameName]
		vec := make([]interface{}, len(jp))
		for j, v := range jp {
			vec[j] = float64(v)
		}
		pushResp, err := motionSvc.DoCommand(ctx, map[string]interface{}{
			builtin.DoStreamPush: []interface{}{vec},
		})
		if err != nil {
			return fmt.Errorf("%s (waypoint %d/%d): %w", builtin.DoStreamPush, i+1, len(traj), err)
		}
		if pushResp["ok"] == nil {
			return fmt.Errorf("unexpected %s response: %v", builtin.DoStreamPush, pushResp)
		}
		if now := time.Now(); now.Sub(lastProgressLog) >= 2*time.Second || i == len(traj)-1 {
			logger.Infof("streamed %d/%d waypoints (%.1f%%)", i+1, len(traj), 100*float64(i+1)/float64(len(traj)))
			lastProgressLog = now
		}
		if *pushSleep > 0 && i < len(traj)-1 {
			time.Sleep(*pushSleep)
		}
	}

	statusResp, err := motionSvc.DoCommand(ctx, map[string]interface{}{
		builtin.DoStreamStatus: map[string]interface{}{"trace": false},
	})
	if err != nil {
		return fmt.Errorf("%s: %w", builtin.DoStreamStatus, err)
	}
	logger.Infof("%s (before flush): %v", builtin.DoStreamStatus, statusResp)

	// stream_flush blocks until the remaining trajectory has drained to the arm (or our ctx
	// expires, in which case it reports running:true and the session keeps draining on its
	// own). The status poll below is a safety net for that ctx-expiry case.
	flushResp, err := motionSvc.DoCommand(ctx, map[string]interface{}{builtin.DoStreamFlush: true})
	if err != nil {
		return fmt.Errorf("%s: %w", builtin.DoStreamFlush, err)
	}
	logger.Infof("%s: %v", builtin.DoStreamFlush, flushResp)

	// trace:false here too -- this loop only needs "running" to know when to stop waiting.
	// The deferred cleanup does its own dedicated (trace:true) fetch once this returns, and
	// that single fetch is what actually ends up in trace.json; the trace only ever grows
	// over the session's lifetime, so a post-mortem fetch already has everything.
	for {
		resp, err := motionSvc.DoCommand(ctx, map[string]interface{}{
			builtin.DoStreamStatus: map[string]interface{}{"trace": false},
		})
		if err != nil {
			return fmt.Errorf("%s (waiting for flush to complete): %w", builtin.DoStreamStatus, err)
		}
		if running, _ := resp["running"].(bool); !running {
			logger.Infof("stream ended: %v", resp)
			if errStr, ok := resp["error"]; ok {
				return fmt.Errorf("stream ended with error: %v", errStr)
			}
			break
		}
		time.Sleep(200 * time.Millisecond)
	}

	return nil
}

func writeJSONFile(path string, v interface{}) error {
	b, err := json.MarshalIndent(v, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, b, 0o600)
}

func mapKeys(m referenceframe.FrameSystemInputs) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}
