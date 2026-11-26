import { ExtensionContext } from "@foxglove/extension";

import { initCameraRenderProbe } from "./CameraRenderProbe";
import { initTopicHealthPanel, TopicHealthGroup } from "./TopicHealthPanel";

const topicHealthGroups: TopicHealthGroup[] = [
  {
    title: "System and mission health",
    topics: [
      { topic: "/lunabot/safety/health", type: "std_msgs/msg/String" },
      { topic: "/lunabot/safety/stopped", type: "std_msgs/msg/Bool" },
      { topic: "/lunabot/control/status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/mission/status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/autonomy/status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/evaluation/status", type: "std_msgs/msg/String" },
    ],
  },
  {
    title: "Sensors and perception",
    topics: [
      { topic: "/lunabot/camera/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/lunabot/camera/camera_info", type: "sensor_msgs/msg/CameraInfo" },
      { topic: "/lunabot/depth/image_raw", type: "sensor_msgs/msg/Image" },
      { topic: "/lunabot/depth/camera_info", type: "sensor_msgs/msg/CameraInfo" },
      { topic: "/lunabot/terrain/segmentation", type: "sensor_msgs/msg/Image" },
      { topic: "/lunabot/terrain/segmentation_confidence", type: "sensor_msgs/msg/Image" },
      { topic: "/lunabot/terrain/overlay", type: "sensor_msgs/msg/Image" },
      { topic: "/lunabot/lidar/scan", type: "sensor_msgs/msg/LaserScan" },
      { topic: "/lunabot/imu", type: "sensor_msgs/msg/Imu" },
      { topic: "/lunabot/joint_states", type: "sensor_msgs/msg/JointState" },
    ],
  },
  {
    title: "Map and localization",
    topics: [
      { topic: "/map", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/tf", type: "tf2_msgs/msg/TFMessage" },
      { topic: "/tf_static", type: "tf2_msgs/msg/TFMessage" },
      { topic: "/lunabot/odom", type: "nav_msgs/msg/Odometry" },
      { topic: "/goal_pose", type: "geometry_msgs/msg/PoseStamped" },
      { topic: "/plan", type: "nav_msgs/msg/Path" },
    ],
  },
  {
    title: "Terrain and risk layers",
    topics: [
      { topic: "/lunabot/terrain/semantic_map", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/lunabot/terrain/semantic_map/confidence", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/lunabot/terrain/semantic_map/age", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/lunabot/terrain/semantic_map/slope", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/lunabot/terrain/semantic_map/roughness", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/lunabot/terrain/semantic_map/crater_risk", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/lunabot/terrain/semantic_map/uncertainty", type: "nav_msgs/msg/OccupancyGrid" },
      { topic: "/lunabot/terrain/cost_map", type: "nav_msgs/msg/OccupancyGrid" },
    ],
  },
  {
    title: "Planning and evaluation",
    topics: [
      { topic: "/lunabot/terrain/plan", type: "nav_msgs/msg/Path" },
      { topic: "/lunabot/terrain/planner/status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/autonomy/replan_status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/evaluation/status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/mission/events", type: "std_msgs/msg/String" },
    ],
  },
  {
    title: "Mission and safety controls",
    topics: [
      { topic: "/lunabot/mission/status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/mission/events", type: "std_msgs/msg/String" },
      { topic: "/lunabot/safety/health", type: "std_msgs/msg/String" },
      { topic: "/lunabot/safety/stopped", type: "std_msgs/msg/Bool" },
      { topic: "/lunabot/control/status", type: "std_msgs/msg/String" },
      { topic: "/lunabot/dashboard/control_status", type: "std_msgs/msg/String" },
    ],
  },
];

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "camera-render-probe", initPanel: initCameraRenderProbe });
  for (const group of topicHealthGroups) {
    extensionContext.registerPanel({
      name: `Topic health — ${group.title}`,
      initPanel: (panelContext) => initTopicHealthPanel(panelContext, group),
    });
  }
}
