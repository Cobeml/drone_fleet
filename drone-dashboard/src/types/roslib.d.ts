// Type declarations for roslib
declare module 'roslib' {
  export interface RosOptions {
    url: string;
  }
  
  export class Ros {
    constructor(options: RosOptions);
    on(event: 'connection' | 'error' | 'close', handler: (event?: any) => void): void;
    connect(url: string): void;
    close(): void;
  }
  
  export interface TopicOptions {
    ros: Ros;
    name: string;
    messageType: string;
  }
  
  export class Topic {
    constructor(options: TopicOptions);
    subscribe(callback: (message: any) => void): void;
    unsubscribe(): void;
    publish(message: any): void;
    on(event: string, handler: (event?: any) => void): void;
  }
  
  export interface ServiceOptions {
    ros: Ros;
    name: string;
    serviceType: string;
  }
  
  export class Service {
    constructor(options: ServiceOptions);
    callService(request: any, onSuccess: (response: any) => void, onError: (error: string) => void): void;
  }
  
  export interface Message {
    [key: string]: any;
  }
}