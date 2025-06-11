// Main Dashboard Component
import React, { useState } from 'react';
import {
  Box,
  Stack,
  Paper,
  Typography,
  ToggleButton,
  ToggleButtonGroup,
  IconButton,
  Tooltip,
} from '@mui/material';
import {
  GridOn as GridIcon,
  CloudQueue as CloudIcon,
  Timeline as TimelineIcon,
  Speed as SpeedIcon,
  Fullscreen as FullscreenIcon,
} from '@mui/icons-material';
import { Scene3D } from '../Scene3D/Scene3D';
import { FleetOverview } from './FleetOverview';
import { KPICards } from './KPICards';
import { useFleetStore } from '../../stores/fleetStore';

export const Dashboard: React.FC = () => {
  const [showGrid, setShowGrid] = useState(true);
  const [showStats, setShowStats] = useState(false);
  const [showPointCloud, setShowPointCloud] = useState(false);
  const [showPaths, setShowPaths] = useState(true);
  const [fullscreen3D, setFullscreen3D] = useState(false);
  
  const drones = useFleetStore((state) => state.drones);
  
  const handleToggleOptions = (
    event: React.MouseEvent<HTMLElement>,
    newOptions: string[]
  ) => {
    setShowGrid(newOptions.includes('grid'));
    setShowStats(newOptions.includes('stats'));
    setShowPointCloud(newOptions.includes('cloud'));
    setShowPaths(newOptions.includes('paths'));
  };
  
  const activeOptions = [
    showGrid && 'grid',
    showStats && 'stats',
    showPointCloud && 'cloud',
    showPaths && 'paths',
  ].filter(Boolean) as string[];
  
  return (
    <Box sx={{ height: '100%', display: 'flex', flexDirection: 'column', p: 2 }}>
      {/* KPI Cards */}
      <Box sx={{ mb: 2 }}>
        <KPICards />
      </Box>
      
      {/* Main Content Area */}
      <Stack 
        direction={{ xs: 'column', md: 'row' }} 
        spacing={2} 
        sx={{ flexGrow: 1, height: 'calc(100% - 150px)' }}
      >
        {/* 3D Visualization */}
        <Box sx={{ 
          flex: fullscreen3D ? 1 : { xs: 1, md: 0.66 }, 
          height: '100%',
          minHeight: { xs: '400px', md: 'auto' }
        }}>
          <Paper
            sx={{
              height: '100%',
              position: 'relative',
              overflow: 'hidden',
              display: 'flex',
              flexDirection: 'column',
            }}
          >
            <Box
              sx={{
                p: 1,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'space-between',
                borderBottom: 1,
                borderColor: 'divider',
              }}
            >
              <Typography variant="h6">3D Environment</Typography>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                <ToggleButtonGroup
                  value={activeOptions}
                  onChange={handleToggleOptions}
                  size="small"
                  aria-label="3D view options"
                >
                  <ToggleButton value="grid" aria-label="show grid">
                    <Tooltip title="Show Grid">
                      <GridIcon />
                    </Tooltip>
                  </ToggleButton>
                  <ToggleButton value="stats" aria-label="show stats">
                    <Tooltip title="Show Performance Stats">
                      <SpeedIcon />
                    </Tooltip>
                  </ToggleButton>
                  <ToggleButton value="cloud" aria-label="show point cloud">
                    <Tooltip title="Show Point Cloud">
                      <CloudIcon />
                    </Tooltip>
                  </ToggleButton>
                  <ToggleButton value="paths" aria-label="show paths">
                    <Tooltip title="Show Paths">
                      <TimelineIcon />
                    </Tooltip>
                  </ToggleButton>
                </ToggleButtonGroup>
                <IconButton
                  size="small"
                  onClick={() => setFullscreen3D(!fullscreen3D)}
                  aria-label="fullscreen"
                >
                  <Tooltip title="Toggle Fullscreen">
                    <FullscreenIcon />
                  </Tooltip>
                </IconButton>
              </Box>
            </Box>
            <Box sx={{ flexGrow: 1, position: 'relative' }}>
              <Scene3D
                showGrid={showGrid}
                showStats={showStats}
                showPointCloud={showPointCloud}
                showPaths={showPaths}
              />
            </Box>
          </Paper>
        </Box>
        
        {/* Fleet Overview Panel */}
        {!fullscreen3D && (
          <Box sx={{ 
            flex: { xs: 1, md: 0.34 }, 
            height: '100%',
            minHeight: { xs: '400px', md: 'auto' }
          }}>
            <Paper
              sx={{
                height: '100%',
                overflow: 'auto',
                display: 'flex',
                flexDirection: 'column',
              }}
            >
              <Box
                sx={{
                  p: 2,
                  borderBottom: 1,
                  borderColor: 'divider',
                }}
              >
                <Typography variant="h6">Fleet Overview</Typography>
                <Typography variant="body2" color="text.secondary">
                  {drones.length} drones in fleet
                </Typography>
              </Box>
              <Box sx={{ flexGrow: 1, overflow: 'auto', p: 2 }}>
                <FleetOverview />
              </Box>
            </Paper>
          </Box>
        )}
      </Stack>
    </Box>
  );
};