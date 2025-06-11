// NYC Buildings Component (Placeholder)
import React, { useMemo } from 'react';
import * as THREE from 'three';
import { useLoader } from '@react-three/fiber';
import { Box as ThreeBox } from '@react-three/drei';

// Placeholder building data for Midtown Manhattan area
const PLACEHOLDER_BUILDINGS = [
  // Empire State Building area
  { id: 'esb', position: [200, 0, 150], height: 380, width: 60, depth: 60, color: '#8B8989' },
  // Times Square area buildings
  { id: 'ts1', position: [100, 0, 100], height: 150, width: 40, depth: 40, color: '#696969' },
  { id: 'ts2', position: [120, 0, 80], height: 120, width: 35, depth: 35, color: '#778899' },
  { id: 'ts3', position: [80, 0, 120], height: 180, width: 45, depth: 45, color: '#696969' },
  // Madison Square Garden area
  { id: 'msg', position: [50, 0, 200], height: 50, width: 100, depth: 80, color: '#A9A9A9' },
  // Random buildings to fill the grid
  { id: 'b1', position: [-100, 0, 50], height: 100, width: 30, depth: 30, color: '#808080' },
  { id: 'b2', position: [-50, 0, -100], height: 150, width: 40, depth: 40, color: '#696969' },
  { id: 'b3', position: [150, 0, -50], height: 200, width: 50, depth: 50, color: '#778899' },
  { id: 'b4', position: [-150, 0, -150], height: 120, width: 35, depth: 35, color: '#808080' },
  { id: 'b5', position: [250, 0, 50], height: 180, width: 45, depth: 45, color: '#696969' },
  { id: 'b6', position: [-200, 0, 100], height: 160, width: 40, depth: 40, color: '#A9A9A9' },
  { id: 'b7', position: [0, 0, -200], height: 140, width: 38, depth: 38, color: '#778899' },
  { id: 'b8', position: [300, 0, -100], height: 170, width: 42, depth: 42, color: '#808080' },
];

interface BuildingProps {
  position: [number, number, number];
  height: number;
  width: number;
  depth: number;
  color: string;
}

const Building: React.FC<BuildingProps> = ({ position, height, width, depth, color }) => {
  const adjustedPosition: [number, number, number] = [
    position[0],
    position[1] + height / 2,
    position[2]
  ];
  
  return (
    <ThreeBox
      position={adjustedPosition}
      args={[width, height, depth]}
      castShadow
      receiveShadow
    >
      <meshStandardMaterial 
        color={color} 
        roughness={0.8}
        metalness={0.2}
      />
    </ThreeBox>
  );
};

export const NYCBuildings: React.FC = () => {
  // TODO: Replace with actual COLLADA/GLTF model loading
  // const model = useLoader(ColladaLoader, '/models/nyc_midtown.dae');
  
  const buildings = useMemo(() => {
    return PLACEHOLDER_BUILDINGS.map((building) => (
      <Building
        key={building.id}
        position={building.position as [number, number, number]}
        height={building.height}
        width={building.width}
        depth={building.depth}
        color={building.color}
      />
    ));
  }, []);
  
  return (
    <group name="nyc-buildings">
      {/* Ground plane */}
      <mesh 
        rotation={[-Math.PI / 2, 0, 0]} 
        position={[0, -0.1, 0]}
        receiveShadow
      >
        <planeGeometry args={[1000, 1000]} />
        <meshStandardMaterial color="#3a3a3a" roughness={1} />
      </mesh>
      
      {/* Buildings */}
      {buildings}
      
      {/* Placeholder text */}
      <group position={[0, 200, 0]}>
        {/* This will be replaced with actual NYC 3D model */}
      </group>
    </group>
  );
};