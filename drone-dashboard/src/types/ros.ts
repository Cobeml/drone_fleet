// ROS Message Type Definitions

export interface Header {
  frame_id: string;
  stamp: {
    sec: number;
    nsec: number;
  };
}

export interface Point {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Pose {
  position: Point;
  orientation: Quaternion;
}

export interface GeometryPose {
  pose: Pose;
}

export interface PoseStamped {
  header: Header;
  pose: Pose;
}

export interface NavPath {
  header: Header;
  poses: Array<{ pose: Pose }>;
}

export interface PointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

export interface PointCloud2 {
  header: Header;
  height: number;
  width: number;
  fields: PointField[];
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: Uint8Array;
  is_dense: boolean;
}

export interface FleetStatus {
  drones: DroneStatus[];
  timestamp: number;
}

export interface DroneStatus {
  id: string;
  status: 'active' | 'idle' | 'error' | 'offline';
  battery_level: number;
  mission_id?: string;
  health_status: {
    gps: boolean;
    imu: boolean;
    lidar: boolean;
    communication: boolean;
  };
}

export interface MissionWaypoint {
  id: string;
  position: Point;
  orientation?: Quaternion;
  action?: 'hover' | 'land' | 'takeoff' | 'scan';
  duration?: number; // seconds
}

export interface Mission {
  id: string;
  name: string;
  waypoints: MissionWaypoint[];
  status: 'planned' | 'active' | 'completed' | 'paused' | 'cancelled';
  drone_id?: string;
  start_time?: number;
  end_time?: number;
  progress?: number; // 0-100
}