# Drone Fleet Dashboard

A modern React-based web dashboard for monitoring and controlling a fleet of drones in a 3D NYC environment. Built with React, TypeScript, Three.js, and Material-UI, this dashboard provides real-time visualization and control capabilities for ROS2-based drone systems.

## Features

- **Real-time 3D Visualization**: Interactive 3D scene showing drones navigating NYC Midtown Manhattan
- **Fleet Management**: Monitor multiple drones with status, battery, position, and telemetry data
- **ROS2 Integration**: Connects to ROS2 backend via ROSbridge WebSocket
- **Mission Planning**: Interface for planning and monitoring drone missions
- **Performance Metrics**: Real-time KPIs and fleet statistics
- **Responsive Design**: Works on desktop and mobile devices
- **Dark Theme**: Optimized for monitoring environments

## Technology Stack

- **Frontend Framework**: React 18 with TypeScript
- **3D Visualization**: Three.js with React Three Fiber
- **UI Components**: Material-UI (MUI)
- **State Management**: Zustand
- **ROS Communication**: roslibjs
- **Routing**: React Router
- **Build Tool**: Create React App

## Prerequisites

- Node.js 16+ and npm/yarn
- ROS2 Humble (for backend integration)
- ROSbridge WebSocket server (optional, demo mode available)

## Installation

1. Clone the repository:
```bash
cd /workspace/drone-dashboard
```

2. Install dependencies:
```bash
npm install
```

3. Create environment configuration:
```bash
cp .env.example .env
```

4. Configure the ROS WebSocket URL in `.env`:
```
REACT_APP_ROS_URL=ws://localhost:9090
```

## Running the Application

### Development Mode
```bash
npm start
```
The application will run at `http://localhost:3000`

### Demo Mode
To run without ROS connection, set in `.env`:
```
REACT_APP_DEBUG=true
```

### Production Build
```bash
npm run build
```

## Project Structure

```
src/
├── components/
│   ├── Scene3D/          # 3D visualization components
│   ├── DroneFleet/       # Drone rendering and management
│   ├── Dashboard/        # Main dashboard views
│   ├── Layout/           # App layout and navigation
│   └── common/           # Shared components
├── hooks/
│   └── useROSSubscription.ts  # ROS topic subscription hooks
├── services/
│   └── ros/              # ROS connection management
├── stores/
│   └── fleetStore.ts     # Global state management
├── types/                # TypeScript type definitions
└── utils/                # Utility functions
```

## Key Features Implementation

### 3D Scene Components
- **Scene3D**: Main 3D canvas with camera controls
- **DroneModel**: Individual drone visualization with animations
- **NYCBuildings**: Placeholder for NYC building models
- **PathVisualization**: Drone trajectory rendering
- **LightingSetup**: Scene lighting configuration

### Dashboard Components
- **KPICards**: Fleet performance metrics
- **FleetOverview**: Individual drone status cards
- **Dashboard**: Main dashboard layout

### ROS Integration
- Subscribes to topics:
  - `/drone{n}/pose` - Drone position and orientation
  - `/drone{n}/battery` - Battery status
  - `/drone{n}/nav/path` - Navigation paths
  - `/fleet/status` - Overall fleet status

## Development

### Adding New Components
1. Create component in appropriate directory
2. Define TypeScript interfaces in `types/`
3. Connect to Zustand store if needed
4. Add ROS subscriptions using hooks

### Extending ROS Communication
1. Define message types in `types/ros.ts`
2. Create subscription hooks in `hooks/`
3. Update store actions in `stores/`

### Performance Optimization
- Use React.memo for expensive components
- Implement throttling for high-frequency updates
- Optimize 3D scene with LOD and culling

## Deployment

### Docker
```dockerfile
FROM node:16-alpine
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY . .
RUN npm run build
EXPOSE 3000
CMD ["npm", "start"]
```

### Environment Variables
- `REACT_APP_ROS_URL`: ROSbridge WebSocket URL
- `REACT_APP_DEBUG`: Enable demo mode

## Contributing

1. Follow TypeScript and React best practices
2. Maintain component modularity
3. Add proper type definitions
4. Test responsive behavior
5. Document new features

## License

MIT License - See LICENSE file for details

## Troubleshooting

### ROS Connection Issues
- Ensure ROSbridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check WebSocket URL in browser console
- Verify CORS settings if needed

### Performance Issues
- Reduce point cloud density
- Lower trajectory point limits
- Enable hardware acceleration in browser
- Use production build for better performance

## Future Enhancements

- [ ] Real NYC 3D building models integration
- [ ] Advanced mission planning interface
- [ ] Multi-drone coordination features
- [ ] Historical data analysis
- [ ] Weather integration
- [ ] Collision detection visualization
- [ ] Video streaming from drones
