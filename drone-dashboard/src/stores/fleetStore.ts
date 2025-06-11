// Fleet Store - Global state management for drone fleet
import { create } from 'zustand';
import { DroneState, DroneAlert, DroneTrajectory } from '../types/drone';
import { Point, Quaternion } from '../types/ros';

interface FleetStore {
  // State
  drones: DroneState[];
  alerts: DroneAlert[];
  trajectories: Map<string, DroneTrajectory>;
  selectedDroneId: string | null;
  
  // Actions
  addDrone: (drone: DroneState) => void;
  updateDronePosition: (id: string, position: Point, orientation: Quaternion) => void;
  updateDroneStatus: (id: string, status: DroneState['status']) => void;
  updateDroneBattery: (id: string, batteryLevel: number) => void;
  updateDroneTelemetry: (id: string, updates: Partial<DroneState>) => void;
  removeDrone: (id: string) => void;
  selectDrone: (id: string | null) => void;
  
  // Alert management
  addAlert: (alert: DroneAlert) => void;
  acknowledgeAlert: (alertId: string) => void;
  clearAlerts: (droneId?: string) => void;
  
  // Trajectory management
  addTrajectoryPoint: (droneId: string, position: Point) => void;
  clearTrajectory: (droneId: string) => void;
  
  // Utility
  getDroneById: (id: string) => DroneState | undefined;
  getActiveDrones: () => DroneState[];
}

const MAX_TRAJECTORY_POINTS = 1000;

export const useFleetStore = create<FleetStore>((set, get) => ({
  // Initial state
  drones: [],
  alerts: [],
  trajectories: new Map(),
  selectedDroneId: null,
  
  // Actions
  addDrone: (drone) =>
    set((state) => ({
      drones: [...state.drones.filter((d) => d.id !== drone.id), drone],
    })),
  
  updateDronePosition: (id, position, orientation) =>
    set((state) => {
      const drone = state.drones.find((d) => d.id === id);
      if (!drone) return state;
      
      // Calculate heading from quaternion
      const heading = Math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
      ) * (180 / Math.PI);
      
      return {
        drones: state.drones.map((d) =>
          d.id === id
            ? {
                ...d,
                position,
                orientation,
                altitude: position.z,
                heading: heading < 0 ? heading + 360 : heading,
                lastUpdate: Date.now(),
              }
            : d
        ),
      };
    }),
  
  updateDroneStatus: (id, status) =>
    set((state) => ({
      drones: state.drones.map((d) =>
        d.id === id ? { ...d, status, lastUpdate: Date.now() } : d
      ),
    })),
  
  updateDroneBattery: (id, batteryLevel) =>
    set((state) => ({
      drones: state.drones.map((d) =>
        d.id === id ? { ...d, batteryLevel, lastUpdate: Date.now() } : d
      ),
    })),
  
  updateDroneTelemetry: (id, updates) =>
    set((state) => ({
      drones: state.drones.map((d) =>
        d.id === id ? { ...d, ...updates, lastUpdate: Date.now() } : d
      ),
    })),
  
  removeDrone: (id) =>
    set((state) => ({
      drones: state.drones.filter((d) => d.id !== id),
      selectedDroneId: state.selectedDroneId === id ? null : state.selectedDroneId,
    })),
  
  selectDrone: (id) =>
    set(() => ({
      selectedDroneId: id,
    })),
  
  // Alert management
  addAlert: (alert) =>
    set((state) => ({
      alerts: [...state.alerts, alert],
    })),
  
  acknowledgeAlert: (alertId) =>
    set((state) => ({
      alerts: state.alerts.map((a) =>
        a.id === alertId ? { ...a, acknowledged: true } : a
      ),
    })),
  
  clearAlerts: (droneId) =>
    set((state) => ({
      alerts: droneId
        ? state.alerts.filter((a) => a.droneId !== droneId)
        : [],
    })),
  
  // Trajectory management
  addTrajectoryPoint: (droneId, position) => {
    const trajectories = get().trajectories;
    const trajectory = trajectories.get(droneId) || {
      droneId,
      points: [],
    };
    
    trajectory.points.push({
      position,
      timestamp: Date.now(),
    });
    
    // Limit trajectory points
    if (trajectory.points.length > MAX_TRAJECTORY_POINTS) {
      trajectory.points = trajectory.points.slice(-MAX_TRAJECTORY_POINTS);
    }
    
    const newTrajectories = new Map(trajectories);
    newTrajectories.set(droneId, trajectory);
    
    set({ trajectories: newTrajectories });
  },
  
  clearTrajectory: (droneId) => {
    const trajectories = get().trajectories;
    const newTrajectories = new Map(trajectories);
    newTrajectories.delete(droneId);
    set({ trajectories: newTrajectories });
  },
  
  // Utility
  getDroneById: (id) => {
    return get().drones.find((d) => d.id === id);
  },
  
  getActiveDrones: () => {
    return get().drones.filter((d) => d.status === 'active');
  },
}));