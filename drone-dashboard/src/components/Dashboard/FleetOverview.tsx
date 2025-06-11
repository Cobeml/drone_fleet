// Fleet Overview Component
import React from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Chip,
  LinearProgress,
  IconButton,
  Stack,
  Tooltip,
  Avatar,
} from '@mui/material';
import {
  Battery20 as BatteryLowIcon,
  Battery60 as BatteryMedIcon,
  BatteryFull as BatteryFullIcon,
  FlightTakeoff as FlightIcon,
  LocationOn as LocationIcon,
  Speed as SpeedIcon,
  Height as HeightIcon,
  Timer as TimerIcon,
  GpsFixed as GpsIcon,
  GpsOff as GpsOffIcon,
} from '@mui/icons-material';
import { useFleetStore } from '../../stores/fleetStore';
import { DroneState } from '../../types/drone';

const getStatusColor = (status: DroneState['status']) => {
  switch (status) {
    case 'active':
      return 'success';
    case 'idle':
      return 'warning';
    case 'error':
      return 'error';
    case 'offline':
    default:
      return 'default';
  }
};

const getBatteryIcon = (batteryLevel: number) => {
  if (batteryLevel < 30) return <BatteryLowIcon />;
  if (batteryLevel < 70) return <BatteryMedIcon />;
  return <BatteryFullIcon />;
};

const getBatteryColor = (batteryLevel: number) => {
  if (batteryLevel < 20) return 'error';
  if (batteryLevel < 50) return 'warning';
  return 'success';
};

interface DroneCardProps {
  drone: DroneState;
  onSelect: () => void;
}

const DroneCard: React.FC<DroneCardProps> = ({ drone, onSelect }) => {
  const formatTime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    return `${hours}h ${minutes}m`;
  };
  
  return (
    <Card
      sx={{
        cursor: 'pointer',
        transition: 'all 0.3s',
        '&:hover': {
          transform: 'translateY(-2px)',
          boxShadow: 4,
        },
      }}
      onClick={onSelect}
    >
      <CardContent>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 2 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <Avatar
              sx={{
                bgcolor: getStatusColor(drone.status) + '.main',
                width: 32,
                height: 32,
              }}
            >
              <FlightIcon fontSize="small" />
            </Avatar>
            <Typography variant="h6">{drone.name || drone.id}</Typography>
          </Box>
          <Chip
            label={drone.status}
            color={getStatusColor(drone.status)}
            size="small"
          />
        </Box>
        
        {/* Battery Level */}
        <Box sx={{ mb: 2 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 0.5 }}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 0.5 }}>
              {getBatteryIcon(drone.batteryLevel)}
              <Typography variant="body2">Battery</Typography>
            </Box>
            <Typography variant="body2" fontWeight="bold">
              {drone.batteryLevel}%
            </Typography>
          </Box>
          <LinearProgress
            variant="determinate"
            value={drone.batteryLevel}
            color={getBatteryColor(drone.batteryLevel)}
            sx={{ height: 6, borderRadius: 3 }}
          />
        </Box>
        
        {/* Drone Stats */}
        <Stack spacing={1}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <LocationIcon fontSize="small" color="action" />
            <Typography variant="body2" color="text.secondary">
              {`${drone.position.x.toFixed(1)}, ${drone.position.y.toFixed(1)}, ${drone.position.z.toFixed(1)}`}
            </Typography>
          </Box>
          
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <HeightIcon fontSize="small" color="action" />
            <Typography variant="body2" color="text.secondary">
              Alt: {drone.altitude.toFixed(1)}m
            </Typography>
          </Box>
          
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <SpeedIcon fontSize="small" color="action" />
            <Typography variant="body2" color="text.secondary">
              Speed: {drone.speed.toFixed(1)}m/s
            </Typography>
          </Box>
          
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            <TimerIcon fontSize="small" color="action" />
            <Typography variant="body2" color="text.secondary">
              Flight: {formatTime(drone.flightTime)}
            </Typography>
          </Box>
          
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
            {drone.signalStrength > 50 ? (
              <GpsIcon fontSize="small" color="success" />
            ) : (
              <GpsOffIcon fontSize="small" color="error" />
            )}
            <Typography variant="body2" color="text.secondary">
              Signal: {drone.signalStrength}%
            </Typography>
          </Box>
        </Stack>
      </CardContent>
    </Card>
  );
};

export const FleetOverview: React.FC = () => {
  const drones = useFleetStore((state) => state.drones);
  const selectDrone = useFleetStore((state) => state.selectDrone);
  
  return (
    <Stack spacing={2}>
      {drones.length === 0 ? (
        <Box sx={{ textAlign: 'center', py: 4 }}>
          <Typography variant="body1" color="text.secondary">
            No drones connected
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Waiting for drone data...
          </Typography>
        </Box>
      ) : (
        drones.map((drone) => (
          <DroneCard
            key={drone.id}
            drone={drone}
            onSelect={() => selectDrone(drone.id)}
          />
        ))
      )}
    </Stack>
  );
};