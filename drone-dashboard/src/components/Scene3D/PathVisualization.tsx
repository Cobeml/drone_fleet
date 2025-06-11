// Path Visualization Component
import React from 'react';
import { Line } from '@react-three/drei';
import { useFleetStore } from '../../stores/fleetStore';
import * as THREE from 'three';

export const PathVisualization: React.FC = () => {
  const trajectories = useFleetStore((state) => state.trajectories);
  const drones = useFleetStore((state) => state.drones);
  
  return (
    <group name="path-visualization">
      {Array.from(trajectories.entries()).map(([droneId, trajectory]) => {
        const drone = drones.find(d => d.id === droneId);
        if (!drone || trajectory.points.length < 2) return null;
        
        const points = trajectory.points.map(p => 
          new THREE.Vector3(p.position.x, p.position.y, p.position.z)
        );
        
        const color = drone.status === 'active' ? '#00ff00' : '#ffff00';
        
        return (
          <Line
            key={droneId}
            points={points}
            color={color}
            lineWidth={2}
            transparent
            opacity={0.6}
            dashed={false}
          />
        );
      })}
    </group>
  );
};