// KPI Cards Component
import React from 'react';
import { Stack, Card, CardContent, Typography, Box, Chip } from '@mui/material';
import {
  FlightTakeoff as DronesIcon,
  Battery80 as BatteryIcon,
  Speed as SpeedIcon,
  Warning as AlertIcon,
  Timer as TimerIcon,
  CloudDone as CloudIcon,
} from '@mui/icons-material';
import { useFleetStore } from '../../stores/fleetStore';

interface KPICardProps {
  title: string;
  value: string | number;
  subtitle?: string;
  icon: React.ReactNode;
  color?: 'primary' | 'secondary' | 'success' | 'error' | 'warning' | 'info';
  chip?: React.ReactNode;
}

const KPICard: React.FC<KPICardProps> = ({
  title,
  value,
  subtitle,
  icon,
  color = 'primary',
  chip,
}) => {
  return (
    <Card sx={{ height: '100%' }}>
      <CardContent>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start' }}>
          <Box>
            <Typography variant="body2" color="text.secondary" gutterBottom>
              {title}
            </Typography>
            <Typography variant="h4" component="div" sx={{ mb: 1 }}>
              {value}
            </Typography>
            {subtitle && (
              <Typography variant="caption" color="text.secondary">
                {subtitle}
              </Typography>
            )}
          </Box>
          <Box sx={{ display: 'flex', flexDirection: 'column', alignItems: 'flex-end', gap: 1 }}>
            <Box
              sx={{
                bgcolor: `${color}.main`,
                color: 'white',
                p: 1,
                borderRadius: 2,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
              }}
            >
              {icon}
            </Box>
            {chip}
          </Box>
        </Box>
      </CardContent>
    </Card>
  );
};

export const KPICards: React.FC = () => {
  const drones = useFleetStore((state) => state.drones);
  const alerts = useFleetStore((state) => state.alerts);
  
  // Calculate metrics
  const totalDrones = drones.length;
  const activeDrones = drones.filter(d => d.status === 'active').length;
  const averageBattery = drones.length > 0
    ? Math.round(drones.reduce((sum, d) => sum + d.batteryLevel, 0) / drones.length)
    : 0;
  const averageSpeed = drones.filter(d => d.status === 'active').length > 0
    ? (drones.filter(d => d.status === 'active').reduce((sum, d) => sum + d.speed, 0) / activeDrones).toFixed(1)
    : '0.0';
  const totalFlightTime = Math.round(drones.reduce((sum, d) => sum + d.flightTime, 0) / 3600); // in hours
  const unacknowledgedAlerts = alerts.filter(a => !a.acknowledged).length;
  
  return (
    <Stack 
      direction={{ xs: 'column', sm: 'row' }} 
      spacing={2}
      sx={{ 
        flexWrap: 'wrap',
        '& > *': { 
          flex: { xs: '1 1 100%', sm: '1 1 calc(50% - 8px)', md: '1 1 calc(16.666% - 10px)' },
          minWidth: { xs: '100%', sm: 'calc(50% - 8px)', md: '200px' }
        }
      }}
    >
      <KPICard
        title="Total Drones"
        value={totalDrones}
        subtitle={`${activeDrones} active`}
        icon={<DronesIcon />}
        color="primary"
        chip={activeDrones > 0 ? <Chip label="Active" size="small" color="success" /> : null}
      />
      
      <KPICard
        title="Avg Battery"
        value={`${averageBattery}%`}
        subtitle="Fleet average"
        icon={<BatteryIcon />}
        color={averageBattery > 50 ? 'success' : averageBattery > 20 ? 'warning' : 'error'}
      />
      
      <KPICard
        title="Avg Speed"
        value={`${averageSpeed} m/s`}
        subtitle="Active drones"
        icon={<SpeedIcon />}
        color="info"
      />
      
      <KPICard
        title="Alerts"
        value={unacknowledgedAlerts}
        subtitle={`${alerts.length} total`}
        icon={<AlertIcon />}
        color={unacknowledgedAlerts > 0 ? 'error' : 'success'}
        chip={unacknowledgedAlerts > 0 ? <Chip label="New" size="small" color="error" /> : null}
      />
      
      <KPICard
        title="Flight Time"
        value={`${totalFlightTime}h`}
        subtitle="Total today"
        icon={<TimerIcon />}
        color="secondary"
      />
      
      <KPICard
        title="Coverage"
        value="85%"
        subtitle="Mission area"
        icon={<CloudIcon />}
        color="success"
      />
    </Stack>
  );
};