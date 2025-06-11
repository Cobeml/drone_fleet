// Demo Data Generator for Testing
import { DroneState } from '../types/drone';
import { Point, Quaternion } from '../types/ros';

const DEMO_DRONE_NAMES = ['Alpha-01', 'Beta-02', 'Gamma-03', 'Delta-04'];

export function generateDemoDrone(index: number): DroneState {
  const id = `drone_${index + 1}`;
  const name = DEMO_DRONE_NAMES[index % DEMO_DRONE_NAMES.length];
  
  // Random starting position in NYC grid
  const position: Point = {
    x: (Math.random() - 0.5) * 400,
    y: 20 + Math.random() * 80,
    z: (Math.random() - 0.5) * 400,
  };
  
  // Random orientation
  const orientation: Quaternion = {
    x: 0,
    y: 0,
    z: Math.random() - 0.5,
    w: Math.sqrt(1 - (Math.random() - 0.5) ** 2),
  };
  
  // Random velocity
  const velocity: Point = {
    x: (Math.random() - 0.5) * 10,
    y: (Math.random() - 0.5) * 2,
    z: (Math.random() - 0.5) * 10,
  };
  
  const statuses: DroneState['status'][] = ['active', 'active', 'idle', 'offline'];
  const status = statuses[Math.floor(Math.random() * statuses.length)];
  
  return {
    id,
    name,
    position,
    orientation,
    velocity,
    batteryLevel: 20 + Math.random() * 80,
    status,
    lastUpdate: Date.now(),
    altitude: position.y,
    speed: Math.sqrt(velocity.x ** 2 + velocity.z ** 2),
    heading: Math.random() * 360,
    signalStrength: 60 + Math.random() * 40,
    temperature: 15 + Math.random() * 10,
    flightTime: Math.random() * 7200, // Up to 2 hours
    distanceTraveled: Math.random() * 10000,
  };
}

export function updateDemoDronePosition(drone: DroneState, deltaTime: number = 0.1): DroneState {
  if (drone.status !== 'active') {
    return drone;
  }
  
  // Simple circular motion for demo
  const time = Date.now() / 1000;
  const radius = 150 + drone.id.charCodeAt(drone.id.length - 1) % 50;
  const speed = 0.1 + (drone.id.charCodeAt(drone.id.length - 1) % 10) / 100;
  const angle = time * speed + (drone.id.charCodeAt(drone.id.length - 1) % 360);
  
  const newPosition: Point = {
    x: Math.cos(angle) * radius,
    y: 30 + Math.sin(time * 0.5) * 10 + drone.id.charCodeAt(drone.id.length - 1) % 20,
    z: Math.sin(angle) * radius,
  };
  
  // Calculate velocity based on position change
  const newVelocity: Point = {
    x: (newPosition.x - drone.position.x) / deltaTime,
    y: (newPosition.y - drone.position.y) / deltaTime,
    z: (newPosition.z - drone.position.z) / deltaTime,
  };
  
  // Update orientation based on movement direction
  const heading = Math.atan2(newVelocity.z, newVelocity.x) * (180 / Math.PI);
  
  // Simulate battery drain
  const batteryDrain = drone.status === 'active' ? 0.01 : 0.001;
  const newBatteryLevel = Math.max(0, drone.batteryLevel - batteryDrain);
  
  return {
    ...drone,
    position: newPosition,
    velocity: newVelocity,
    altitude: newPosition.y,
    speed: Math.sqrt(newVelocity.x ** 2 + newVelocity.z ** 2),
    heading: heading < 0 ? heading + 360 : heading,
    batteryLevel: newBatteryLevel,
    flightTime: drone.flightTime + deltaTime,
    distanceTraveled: drone.distanceTraveled + Math.sqrt(
      (newPosition.x - drone.position.x) ** 2 +
      (newPosition.y - drone.position.y) ** 2 +
      (newPosition.z - drone.position.z) ** 2
    ),
    lastUpdate: Date.now(),
  };
}