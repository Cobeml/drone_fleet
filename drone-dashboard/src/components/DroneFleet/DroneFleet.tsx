// Drone Fleet Visualization Component
import React from 'react';
import { useFleetStore } from '../../stores/fleetStore';
import { DroneModel } from './DroneModel';

export const DroneFleet: React.FC = () => {
  const drones = useFleetStore((state) => state.drones);
  const selectedDroneId = useFleetStore((state) => state.selectedDroneId);
  
  return (
    <group name="drone-fleet">
      {drones.map((drone) => (
        <DroneModel
          key={drone.id}
          drone={drone}
          selected={drone.id === selectedDroneId}
        />
      ))}
    </group>
  );
};