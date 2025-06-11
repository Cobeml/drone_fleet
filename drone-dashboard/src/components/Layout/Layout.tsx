// Main Layout Component
import React, { useState } from 'react';
import { Outlet, useNavigate, useLocation } from 'react-router-dom';
import {
  Box,
  Drawer,
  AppBar,
  Toolbar,
  Typography,
  IconButton,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Divider,
  Badge,
  Chip,
} from '@mui/material';
import {
  Menu as MenuIcon,
  Dashboard as DashboardIcon,
  FlightTakeoff as FleetIcon,
  Map as MissionIcon,
  Settings as SettingsIcon,
  Warning as WarningIcon,
  Wifi as WifiIcon,
  WifiOff as WifiOffIcon,
} from '@mui/icons-material';
import { useFleetStore } from '../../stores/fleetStore';
import { getROSConnection } from '../../services/ros/ROSConnection';

const drawerWidth = 240;

const menuItems = [
  { text: 'Dashboard', icon: <DashboardIcon />, path: '/' },
  { text: 'Fleet Management', icon: <FleetIcon />, path: '/fleet' },
  { text: 'Mission Planning', icon: <MissionIcon />, path: '/mission' },
  { text: 'Settings', icon: <SettingsIcon />, path: '/settings' },
];

export const Layout: React.FC = () => {
  const [mobileOpen, setMobileOpen] = useState(false);
  const navigate = useNavigate();
  const location = useLocation();
  const alerts = useFleetStore((state) => state.alerts);
  const drones = useFleetStore((state) => state.drones);
  const [isConnected, setIsConnected] = useState(false);
  
  React.useEffect(() => {
    try {
      const ros = getROSConnection();
      setIsConnected(ros.isConnected());
      
      const unsubscribe = ros.onConnectionChange((connected) => {
        setIsConnected(connected);
      });
      
      return unsubscribe;
    } catch (error) {
      console.error('ROS connection not initialized');
    }
  }, []);
  
  const handleDrawerToggle = () => {
    setMobileOpen(!mobileOpen);
  };
  
  const unacknowledgedAlerts = alerts.filter(a => !a.acknowledged);
  const activeDrones = drones.filter(d => d.status === 'active');
  
  const drawer = (
    <div>
      <Toolbar>
        <Typography variant="h6" noWrap component="div">
          Drone Fleet
        </Typography>
      </Toolbar>
      <Divider />
      <List>
        {menuItems.map((item) => (
          <ListItem key={item.text} disablePadding>
            <ListItemButton
              selected={location.pathname === item.path}
              onClick={() => navigate(item.path)}
            >
              <ListItemIcon>
                {item.icon}
              </ListItemIcon>
              <ListItemText primary={item.text} />
            </ListItemButton>
          </ListItem>
        ))}
      </List>
      <Divider />
      <Box sx={{ p: 2 }}>
        <Typography variant="caption" color="text.secondary">
          Fleet Status
        </Typography>
        <Box sx={{ mt: 1 }}>
          <Chip
            label={`${activeDrones.length} Active`}
            color="success"
            size="small"
            sx={{ mr: 1 }}
          />
          <Chip
            label={`${drones.length} Total`}
            color="default"
            size="small"
          />
        </Box>
      </Box>
    </div>
  );
  
  return (
    <Box sx={{ display: 'flex', height: '100vh' }}>
      <AppBar
        position="fixed"
        sx={{
          width: { sm: `calc(100% - ${drawerWidth}px)` },
          ml: { sm: `${drawerWidth}px` },
        }}
      >
        <Toolbar>
          <IconButton
            color="inherit"
            aria-label="open drawer"
            edge="start"
            onClick={handleDrawerToggle}
            sx={{ mr: 2, display: { sm: 'none' } }}
          >
            <MenuIcon />
          </IconButton>
          <Typography variant="h6" noWrap component="div" sx={{ flexGrow: 1 }}>
            Drone Fleet Dashboard
          </Typography>
          
          {/* Connection Status */}
          <Chip
            icon={isConnected ? <WifiIcon /> : <WifiOffIcon />}
            label={isConnected ? 'Connected' : 'Disconnected'}
            color={isConnected ? 'success' : 'error'}
            variant="outlined"
            sx={{ mr: 2 }}
          />
          
          {/* Alerts Badge */}
          <IconButton color="inherit">
            <Badge badgeContent={unacknowledgedAlerts.length} color="error">
              <WarningIcon />
            </Badge>
          </IconButton>
        </Toolbar>
      </AppBar>
      
      <Box
        component="nav"
        sx={{ width: { sm: drawerWidth }, flexShrink: { sm: 0 } }}
      >
        <Drawer
          variant="temporary"
          open={mobileOpen}
          onClose={handleDrawerToggle}
          ModalProps={{
            keepMounted: true, // Better open performance on mobile.
          }}
          sx={{
            display: { xs: 'block', sm: 'none' },
            '& .MuiDrawer-paper': { boxSizing: 'border-box', width: drawerWidth },
          }}
        >
          {drawer}
        </Drawer>
        <Drawer
          variant="permanent"
          sx={{
            display: { xs: 'none', sm: 'block' },
            '& .MuiDrawer-paper': { boxSizing: 'border-box', width: drawerWidth },
          }}
          open
        >
          {drawer}
        </Drawer>
      </Box>
      
      <Box
        component="main"
        sx={{
          flexGrow: 1,
          p: 0,
          width: { sm: `calc(100% - ${drawerWidth}px)` },
          height: '100vh',
          display: 'flex',
          flexDirection: 'column',
        }}
      >
        <Toolbar />
        <Box sx={{ flexGrow: 1, overflow: 'hidden' }}>
          <Outlet />
        </Box>
      </Box>
    </Box>
  );
};