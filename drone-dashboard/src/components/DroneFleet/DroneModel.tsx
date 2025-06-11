// Individual Drone 3D Model Component
import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { Text, Billboard, Cone, Sphere } from '@react-three/drei';
import { DroneState } from '../../types/drone';

interface DroneModelProps {
  drone: DroneState;
  selected?: boolean;
}

const getStatusColor = (status: DroneState['status']): string => {
  switch (status) {
    case 'active':
      return '#00ff00';
    case 'idle':
      return '#ffff00';
    case 'error':
      return '#ff0000';
    case 'offline':
      return '#808080';
    default:
      return '#ffffff';
  }
};

export const DroneModel: React.FC<DroneModelProps> = ({ drone, selected = false }) => {
  const meshRef = useRef<THREE.Group>(null);
  const propellerRefs = useRef<THREE.Mesh[]>([]);
  
  // Drone body color based on status
  const bodyColor = useMemo(() => getStatusColor(drone.status), [drone.status]);
  
  // Animate propellers and update position
  useFrame((state, delta) => {
    if (meshRef.current) {
      // Update position
      meshRef.current.position.set(
        drone.position.x,
        drone.position.y,
        drone.position.z
      );
      
      // Update rotation from quaternion
      meshRef.current.quaternion.set(
        drone.orientation.x,
        drone.orientation.y,
        drone.orientation.z,
        drone.orientation.w
      );
      
      // Animate propellers
      if (drone.status === 'active') {
        propellerRefs.current.forEach((propeller) => {
          if (propeller) {
            propeller.rotation.y += delta * 50;
          }
        });
      }
    }
  });
  
  const propellerPositions: [number, number, number][] = [
    [2, 0.5, 2],
    [-2, 0.5, 2],
    [2, 0.5, -2],
    [-2, 0.5, -2],
  ];
  
  return (
    <group ref={meshRef} name={`drone-${drone.id}`}>
      {/* Drone body */}
      <mesh castShadow receiveShadow>
        <boxGeometry args={[3, 1, 3]} />
        <meshStandardMaterial 
          color={selected ? '#ffd700' : '#333333'}
          metalness={0.6}
          roughness={0.4}
        />
      </mesh>
      
      {/* Arms */}
      {propellerPositions.map((pos, index) => (
        <group key={index} position={pos}>
          {/* Arm */}
          <mesh castShadow>
            <cylinderGeometry args={[0.2, 0.2, Math.sqrt(pos[0] ** 2 + pos[2] ** 2) * 1.4]} />
            <meshStandardMaterial color="#444444" />
          </mesh>
          
          {/* Motor */}
          <mesh position={[0, 0.3, 0]} castShadow>
            <cylinderGeometry args={[0.5, 0.5, 0.4]} />
            <meshStandardMaterial color="#222222" />
          </mesh>
          
          {/* Propeller */}
          <mesh 
            ref={(el) => {
              if (el) propellerRefs.current[index] = el;
            }}
            position={[0, 0.6, 0]}
          >
            <boxGeometry args={[4, 0.1, 0.3]} />
            <meshStandardMaterial color="#666666" transparent opacity={0.8} />
          </mesh>
        </group>
      ))}
      
      {/* Status indicator light */}
      <Sphere args={[0.3]} position={[0, 1, 0]}>
        <meshStandardMaterial 
          color={bodyColor}
          emissive={bodyColor}
          emissiveIntensity={drone.status === 'active' ? 1 : 0.3}
        />
      </Sphere>
      
      {/* Direction indicator */}
      <Cone args={[0.3, 1, 3]} position={[0, 0, 2]} rotation={[Math.PI / 2, 0, 0]}>
        <meshStandardMaterial color="#ff0000" />
      </Cone>
      
      {/* Drone label */}
      <Billboard position={[0, 4, 0]}>
        <Text
          fontSize={1.5}
          color={selected ? '#ffd700' : '#ffffff'}
          anchorX="center"
          anchorY="middle"
          outlineWidth={0.1}
          outlineColor="#000000"
        >
          {drone.name || drone.id}
        </Text>
        <Text
          fontSize={1}
          color="#aaaaaa"
          anchorX="center"
          anchorY="top"
          position={[0, -1, 0]}
        >
          {`Battery: ${drone.batteryLevel}%`}
        </Text>
        <Text
          fontSize={0.8}
          color="#888888"
          anchorX="center"
          anchorY="top"
          position={[0, -2, 0]}
        >
          {`Alt: ${drone.altitude.toFixed(1)}m`}
        </Text>
      </Billboard>
      
      {/* Selection indicator */}
      {selected && (
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -1, 0]}>
          <ringGeometry args={[5, 6, 32]} />
          <meshStandardMaterial 
            color="#ffd700"
            emissive="#ffd700"
            emissiveIntensity={0.5}
            transparent
            opacity={0.5}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}
    </group>
  );
};