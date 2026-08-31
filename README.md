# stream_cartesian_demo

Test client for the motion service's arm-streaming DoCommands
(`stream_start` / `stream_push` / `stream_flush` / `stream_abort` / `stream_status`).
Traces a Cartesian shape (fig8, snake, or line) by pushing IK'd joint targets one
waypoint at a time, then writes `trace.json` (pipeline trace, on every exit path).

Used to reproduce and diagnose streaming starvation deaths
("arm's current trajectory failed") against a UR arm.

## Build

Requires local checkouts referenced by `replace` directives in `go.mod`
(`go.viam.com/rdk`, `motion-tools`):

    go build -o stream_cartesian_demo .

## Run

    export VIAM_ADDRESS=<machine FQDN>
    export VIAM_API_KEY_ID=<key id>
    export VIAM_API_KEY=<key>
    export ARM_NAME=<arm component name>   # default "arm"

    ./stream_cartesian_demo -shape snake -num-waypoints 12001 -target-runway-ms 150

Key flags: `-shape fig8|snake|line`, `-num-waypoints`, `-target-runway-ms`
(0 = server default), `-push-sleep` (delay between pushes), and the `-fig8-*`
plane/span options (also used by snake). See `main.go` for the full list.

## Rendering traces

    python3 plot_pipeline_trace.py trace.json    # writes pipeline_trace.html
