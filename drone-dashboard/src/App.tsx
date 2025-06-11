import React, { useEffect } from 'react';
import { ThemeProvider, createTheme, CssBaseline } from '@mui/material';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { Layout } from './components/Layout/Layout';
import { Dashboard } from './components/Dashboard/Dashboard';
import { initializeROSConnection } from './services/ros/ROSConnection';
import { useFleetStore } from './stores/fleetStore';
import { generateDemoDrone, updateDemoDronePosition } from './utils/demoData';
import './App.css';

// Create a dark theme for the dashboard
const darkTheme = createTheme({
  palette: {
    mode: 'dark',
    primary: {
      main: '#1976d2',
    },
    secondary: {
      main: '#dc004e',
    },
    background: {
      default: '#0a0a0a',
      paper: '#1a1a1a',
    },
  },
  typography: {
    fontFamily: '"Inter", "Roboto", "Helvetica", "Arial", sans-serif',
  },
});

function App() {
  const addDrone = useFleetStore((state) => state.addDrone);
  const updateDroneTelemetry = useFleetStore((state) => state.updateDroneTelemetry);
  const addTrajectoryPoint = useFleetStore((state) => state.addTrajectoryPoint);
  
  useEffect(() => {
    // Initialize ROS connection
    const rosUrl = process.env.REACT_APP_ROS_URL || 'ws://localhost:9090';
    console.log('Initializing ROS connection to:', rosUrl);
    
    let demoMode = false;
    
    try {
      initializeROSConnection({
        url: rosUrl,
        reconnectInterval: 3000,
        maxReconnectAttempts: 10,
      });
    } catch (error) {
      console.error('Failed to initialize ROS connection:', error);
      demoMode = true;
    }
    
    // Start demo mode if ROS is not available
    if (demoMode || process.env.REACT_APP_DEBUG === 'true') {
      console.log('Starting demo mode...');
      
      // Generate demo drones
      const demoDrones = Array.from({ length: 2 }, (_, i) => generateDemoDrone(i));
      demoDrones.forEach(drone => addDrone(drone));
      
      // Update demo drone positions
      const interval = setInterval(() => {
        demoDrones.forEach(drone => {
          const updatedDrone = updateDemoDronePosition(drone);
          updateDroneTelemetry(drone.id, updatedDrone);
          
          // Add trajectory points for active drones
          if (updatedDrone.status === 'active') {
            addTrajectoryPoint(drone.id, updatedDrone.position);
          }
          
          // Update the reference for next iteration
          Object.assign(drone, updatedDrone);
        });
      }, 100); // Update at 10Hz
      
      return () => clearInterval(interval);
    }
  }, [addDrone, updateDroneTelemetry, addTrajectoryPoint]);
  
  return (
    <ThemeProvider theme={darkTheme}>
      <CssBaseline />
      <Router>
        <Routes>
          <Route path="/" element={<Layout />}>
            <Route index element={<Dashboard />} />
            <Route path="fleet" element={<Dashboard />} />
            <Route path="mission" element={<Dashboard />} />
            <Route path="settings" element={<Dashboard />} />
          </Route>
        </Routes>
      </Router>
    </ThemeProvider>
  );
}

export default App;
