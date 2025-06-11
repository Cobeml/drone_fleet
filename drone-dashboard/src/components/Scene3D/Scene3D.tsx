// Main 3D Scene Component
import React, { Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { 
  OrbitControls, 
  Grid, 
  Environment, 
  Sky,
  Stats,
  PerspectiveCamera
} from '@react-three/drei';
import { Box, CircularProgress } from '@mui/material';
import { DroneFleet } from '../DroneFleet/DroneFleet';
import { NYCBuildings } from './NYCBuildings';
import { PathVisualization } from './PathVisualization';
import { LightingSetup } from './LightingSetup';
import { PointCloudVisualization } from './PointCloudVisualization';

interface Scene3DProps {
  showGrid?: boolean;
  showStats?: boolean;
  showPointCloud?: boolean;
  showPaths?: boolean;
}

const LoadingFallback: React.FC = () => (
  <Box
    sx={{
      position: 'absolute',
      top: '50%',
      left: '50%',
      transform: 'translate(-50%, -50%)',
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      gap: 2,
    }}
  >
    <CircularProgress size={60} />
    <Box sx={{ color: 'text.secondary' }}>Loading 3D Scene...</Box>
  </Box>
);

export const Scene3D: React.FC<Scene3DProps> = ({
  showGrid = true,
  showStats = false,
  showPointCloud = false,
  showPaths = true,
}) => {
  return (
    <Box sx={{ width: '100%', height: '100%', position: 'relative', bgcolor: '#f0f0f0' }}>
      <Canvas
        shadows
        dpr={[1, 2]}
        gl={{ preserveDrawingBuffer: true }}
      >
        <PerspectiveCamera 
          makeDefault 
          position={[150, 150, 150]} 
          fov={60}
          near={0.1}
          far={10000}
        />
        
        <Suspense fallback={null}>
          {/* Skybox and Environment */}
          <Sky 
            distance={450000}
            sunPosition={[100, 50, 100]}
            inclination={0.6}
            azimuth={0.25}
          />
          <Environment preset="city" />
          
          {/* Lighting */}
          <LightingSetup />
          
          {/* Grid Helper */}
          {showGrid && (
            <Grid 
              args={[1000, 1000, 100, 100]} 
              position={[0, 0, 0]}
              cellSize={10}
              cellColor="#6f6f6f"
              sectionColor="#9d9d9d"
              fadeDistance={1000}
              fadeStrength={1}
              followCamera={false}
            />
          )}
          
          {/* NYC Buildings */}
          <NYCBuildings />
          
          {/* Drone Fleet */}
          <DroneFleet />
          
          {/* Path Visualization */}
          {showPaths && <PathVisualization />}
          
          {/* Point Cloud Visualization */}
          {showPointCloud && <PointCloudVisualization />}
          
          {/* Camera Controls */}
          <OrbitControls
            enablePan
            enableZoom
            enableRotate
            minDistance={10}
            maxDistance={500}
            maxPolarAngle={Math.PI / 2}
            makeDefault
          />
        </Suspense>
        
        {/* Performance Stats */}
        {showStats && <Stats />}
      </Canvas>
      
      <Suspense fallback={<LoadingFallback />}>
        {/* Additional UI overlays can go here */}
      </Suspense>
    </Box>
  );
};