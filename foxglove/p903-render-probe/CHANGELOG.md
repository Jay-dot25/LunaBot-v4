# lunabot-p903-render-probe version history

## 0.2.0

- Add six read-only topic health panels that match the dashboard view groups.
- Show source freshness and age from the telemetry adapter plus receive-to-render
  p95 and an estimated source-to-render p95 inside the client panel.
- Declare `ts-loader` as a build dependency and include ES2022 TypeScript globals.

## 0.1.1

- Accept both ROS `nsec` and `nanosec` stamp field names so source-age metrics
  work with rosbridge and Foxglove ROS schemas.

## 0.0.0

- Alpha testing
