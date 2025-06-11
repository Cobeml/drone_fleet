// ROS Connection Service
import * as ROSLIB from 'roslib';

export interface ROSConnectionOptions {
  url: string;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
}

export interface TopicSubscription {
  topic: ROSLIB.Topic;
  callback: (message: any) => void;
  messageType: string;
}

export class ROSConnection {
  private ros: ROSLIB.Ros;
  private topics: Map<string, TopicSubscription> = new Map();
  private services: Map<string, ROSLIB.Service> = new Map();
  private connected: boolean = false;
  private reconnectAttempts: number = 0;
  private options: Required<ROSConnectionOptions>;
  private connectionListeners: Array<(connected: boolean) => void> = [];
  private reconnectTimer?: NodeJS.Timeout;

  constructor(options: ROSConnectionOptions) {
    this.options = {
      url: options.url,
      reconnectInterval: options.reconnectInterval || 3000,
      maxReconnectAttempts: options.maxReconnectAttempts || 10,
    };

    this.ros = new ROSLIB.Ros({
      url: this.options.url,
    });

    this.setupEventHandlers();
  }

  private setupEventHandlers(): void {
    this.ros.on('connection', () => {
      console.log('Connected to ROSbridge WebSocket');
      this.connected = true;
      this.reconnectAttempts = 0;
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = undefined;
      }
      this.notifyConnectionListeners(true);
      this.resubscribeTopics();
    });

    this.ros.on('error', (error) => {
      console.error('ROSbridge WebSocket error:', error);
    });

    this.ros.on('close', () => {
      console.log('Disconnected from ROSbridge WebSocket');
      this.connected = false;
      this.notifyConnectionListeners(false);
      this.attemptReconnect();
    });
  }

  private attemptReconnect(): void {
    if (this.reconnectAttempts >= this.options.maxReconnectAttempts) {
      console.error('Max reconnection attempts reached');
      return;
    }

    this.reconnectAttempts++;
    console.log(`Attempting to reconnect (${this.reconnectAttempts}/${this.options.maxReconnectAttempts})...`);

    this.reconnectTimer = setTimeout(() => {
      this.ros.connect(this.options.url);
    }, this.options.reconnectInterval);
  }

  private resubscribeTopics(): void {
    // Resubscribe to all topics after reconnection
    this.topics.forEach((subscription, topicName) => {
      console.log(`Resubscribing to topic: ${topicName}`);
      subscription.topic.subscribe(subscription.callback);
    });
  }

  private notifyConnectionListeners(connected: boolean): void {
    this.connectionListeners.forEach(listener => listener(connected));
  }

  public onConnectionChange(listener: (connected: boolean) => void): () => void {
    this.connectionListeners.push(listener);
    // Return unsubscribe function
    return () => {
      const index = this.connectionListeners.indexOf(listener);
      if (index > -1) {
        this.connectionListeners.splice(index, 1);
      }
    };
  }

  public subscribeTo<T>(topicName: string, messageType: string, callback: (message: T) => void): ROSLIB.Topic {
    // Check if already subscribed
    if (this.topics.has(topicName)) {
      console.warn(`Already subscribed to topic: ${topicName}`);
      return this.topics.get(topicName)!.topic;
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
    });

    topic.subscribe((message: any) => callback(message as T));

    this.topics.set(topicName, {
      topic,
      callback: (message: any) => callback(message as T),
      messageType,
    });

    console.log(`Subscribed to topic: ${topicName} (${messageType})`);
    return topic;
  }

  public unsubscribe(topicName: string): void {
    const subscription = this.topics.get(topicName);
    if (subscription) {
      subscription.topic.unsubscribe();
      this.topics.delete(topicName);
      console.log(`Unsubscribed from topic: ${topicName}`);
    }
  }

  public publish<T>(topicName: string, messageType: string, message: T): void {
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
    });

    topic.publish(message as any);
  }

  public callService<TRequest, TResponse>(
    serviceName: string,
    serviceType: string,
    request: TRequest
  ): Promise<TResponse> {
    let service = this.services.get(serviceName);
    
    if (!service) {
      service = new ROSLIB.Service({
        ros: this.ros,
        name: serviceName,
        serviceType: serviceType,
      });
      this.services.set(serviceName, service);
    }

    return new Promise((resolve, reject) => {
      service!.callService(
        request,
        (response: TResponse) => resolve(response),
        (error: string) => reject(new Error(error))
      );
    });
  }

  public isConnected(): boolean {
    return this.connected;
  }

  public disconnect(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
    }
    this.topics.forEach((subscription) => {
      subscription.topic.unsubscribe();
    });
    this.topics.clear();
    this.services.clear();
    this.ros.close();
  }
}

// Singleton instance
let rosConnectionInstance: ROSConnection | null = null;

export function initializeROSConnection(options: ROSConnectionOptions): ROSConnection {
  if (!rosConnectionInstance) {
    rosConnectionInstance = new ROSConnection(options);
  }
  return rosConnectionInstance;
}

export function getROSConnection(): ROSConnection {
  if (!rosConnectionInstance) {
    throw new Error('ROS connection not initialized. Call initializeROSConnection first.');
  }
  return rosConnectionInstance;
}