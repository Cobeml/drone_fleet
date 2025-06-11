// Lighting Setup for 3D Scene
import React from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';

export const LightingSetup: React.FC = () => {
  const { scene } = useThree();
  
  React.useEffect(() => {
    // Enable shadows in the scene
    scene.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
  }, [scene]);
  
  return (
    <>
      {/* Ambient light for overall illumination */}
      <ambientLight intensity={0.6} color="#ffffff" />
      
      {/* Main directional light (sun) */}
      <directionalLight
        position={[100, 150, 100]}
        intensity={1}
        color="#ffffff"
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-far={500}
        shadow-camera-left={-200}
        shadow-camera-right={200}
        shadow-camera-top={200}
        shadow-camera-bottom={-200}
        shadow-bias={-0.001}
      />
      
      {/* Secondary fill light */}
      <directionalLight
        position={[-50, 50, -50]}
        intensity={0.3}
        color="#87ceeb"
      />
      
      {/* Hemisphere light for sky/ground color variation */}
      <hemisphereLight
        args={['#87ceeb', '#f4e4c1', 0.3]}
        position={[0, 50, 0]}
      />
      
      {/* Point lights for specific area lighting */}
      <pointLight
        position={[0, 100, 0]}
        intensity={0.5}
        color="#ffd700"
        distance={300}
        decay={2}
      />
    </>
  );
};