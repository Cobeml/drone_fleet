// Drone Type Definitions

import { Point, Quaternion } from './ros';

export interface DroneState {
  id: string;
  name: string;
  position: Point;
  orientation: Quaternion;
  velocity: Point;
  batteryLevel: number;
  status: 'active' | 'idle' | 'error' | 'offline';
  missionId?: string;
  lastUpdate: number;
  altitude: number;
  speed: number;
  heading: number; // degrees
  signalStrength: number; // 0-100
  temperature: number; // celsius
  flightTime: number; // seconds
  distanceTraveled: number; // meters
}

export interface DroneTrajectory {
  droneId: string;
  points: Array<{
    position: Point;
    timestamp: number;
  }>;
  maxPoints?: number;
}

export interface DroneTelemetry {
  droneId: string;
  timestamp: number;
  gps: {
    latitude: number;
    longitude: number;
    altitude: number;
    satellites: number;
    hdop: number;
  };
  imu: {
    accelerometer: Point;
    gyroscope: Point;
    magnetometer: Point;
  };
  motors: Array<{
    id: number;
    rpm: number;
    temperature: number;
    current: number;
  }>;
}

export interface DroneCommand {
  droneId: string;
  command: 'takeoff' | 'land' | 'return_home' | 'pause' | 'resume' | 'emergency_stop';
  parameters?: any;
}

export interface DroneAlert {
  id: string;
  droneId: string;
  type: 'warning' | 'error' | 'info';
  category: 'battery' | 'signal' | 'temperature' | 'collision' | 'gps' | 'system';
  message: string;
  timestamp: number;
  acknowledged: boolean;
}