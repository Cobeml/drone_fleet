// Custom hook for ROS topic subscriptions
import { useEffect, useState, useRef, useCallback } from 'react';
import { getROSConnection } from '../services/ros/ROSConnection';
import * as ROSLIB from 'roslib';

interface UseROSSubscriptionOptions {
  enabled?: boolean;
  throttleRate?: number; // ms
  onError?: (error: Error) => void;
}

export function useROSSubscription<T>(
  topicName: string,
  messageType: string,
  options: UseROSSubscriptionOptions = {}
): {
  data: T | null;
  error: Error | null;
  isSubscribed: boolean;
  lastUpdate: number | null;
} {
  const { enabled = true, throttleRate = 0, onError } = options;
  
  const [data, setData] = useState<T | null>(null);
  const [error, setError] = useState<Error | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [lastUpdate, setLastUpdate] = useState<number | null>(null);
  
  const topicRef = useRef<ROSLIB.Topic | null>(null);
  const lastUpdateRef = useRef<number>(0);
  
  const handleMessage = useCallback((message: T) => {
    const now = Date.now();
    
    // Apply throttling if specified
    if (throttleRate > 0 && now - lastUpdateRef.current < throttleRate) {
      return;
    }
    
    lastUpdateRef.current = now;
    setData(message);
    setLastUpdate(now);
    setError(null);
  }, [throttleRate]);
  
  useEffect(() => {
    if (!enabled) {
      return;
    }
    
    let mounted = true;
    
    const subscribe = async () => {
      try {
        const rosConnection = getROSConnection();
        
        if (!rosConnection.isConnected()) {
          throw new Error('ROS connection not established');
        }
        
        topicRef.current = rosConnection.subscribeTo<T>(
          topicName,
          messageType,
          (message) => {
            if (mounted) {
              handleMessage(message);
            }
          }
        );
        
        if (mounted) {
          setIsSubscribed(true);
          setError(null);
        }
      } catch (err) {
        if (mounted) {
          const error = err instanceof Error ? err : new Error('Failed to subscribe to topic');
          setError(error);
          setIsSubscribed(false);
          onError?.(error);
        }
      }
    };
    
    subscribe();
    
    return () => {
      mounted = false;
      if (topicRef.current) {
        try {
          const rosConnection = getROSConnection();
          rosConnection.unsubscribe(topicName);
        } catch (err) {
          console.error('Error unsubscribing from topic:', err);
        }
        topicRef.current = null;
      }
      setIsSubscribed(false);
    };
  }, [topicName, messageType, enabled, handleMessage, onError]);
  
  return {
    data,
    error,
    isSubscribed,
    lastUpdate,
  };
}

// Hook for publishing to ROS topics
export function useROSPublisher<T>(
  topicName: string,
  messageType: string
): {
  publish: (message: T) => void;
  isConnected: boolean;
  error: Error | null;
} {
  const [error, setError] = useState<Error | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  
  useEffect(() => {
    try {
      const rosConnection = getROSConnection();
      setIsConnected(rosConnection.isConnected());
      
      const unsubscribe = rosConnection.onConnectionChange((connected) => {
        setIsConnected(connected);
      });
      
      return unsubscribe;
    } catch (err) {
      setError(err instanceof Error ? err : new Error('Failed to initialize publisher'));
      setIsConnected(false);
    }
  }, []);
  
  const publish = useCallback((message: T) => {
    try {
      const rosConnection = getROSConnection();
      rosConnection.publish(topicName, messageType, message);
      setError(null);
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to publish message');
      setError(error);
      throw error;
    }
  }, [topicName, messageType]);
  
  return {
    publish,
    isConnected,
    error,
  };
}