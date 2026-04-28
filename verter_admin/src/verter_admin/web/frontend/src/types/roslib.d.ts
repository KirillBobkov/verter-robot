/**
 * Type definitions for roslib.js
 * Generated from @types/roslib and extended for our usage
 */

declare global {
  interface Window {
    ROSLIB: typeof ROSLIB;
  }
}

declare namespace ROSLIB {
  export class Ros {
    constructor(options?: RosOptions);
    on(name: 'connection', callback: () => void): void;
    on(name: 'error', callback: (error: Error) => void): void;
    on(name: 'close', callback: () => void): void;
    close(): void;
  }

  export interface RosOptions {
    url?: string;
    transport?: 'websocket' | 'socket.io' | 'tcp';
    groovyCompatibility?: boolean;
  }

  export class Topic<TMsg = any> {
    constructor(options: TopicOptions);
    publish(message: TMsg): void;
    subscribe(callback: (message: TMsg) => void): void;
    unsubscribe(callback: (message: TMsg) => void): void;
    advertise(): void;
    unadvertise(): void;
  }

  export interface TopicOptions {
    ros: Ros;
    name: string;
    messageType: string;
    queue_length?: number;
    compression?: 'none' | 'cbor' | 'png';
    throttle_rate?: number;
  }

  export class Service<TReq = any, TRes = any> {
    constructor(options: ServiceOptions);
    callService(
      request: ServiceRequest<TReq>,
      callback: (response: ServiceResponse<TRes>) => void,
      errorCallback?: (error: any) => void
    ): void;
  }

  export interface ServiceOptions {
    ros: Ros;
    name: string;
    serviceType: string;
  }

  export class ServiceRequest<T> {
    constructor(data: T);
  }

  export class ServiceResponse<T> {
    constructor(data?: T);
    isSuccess(): boolean;
  }

  interface RosOptions {
    url?: string;
    transport?: 'websocket' | 'socket.io' | 'tcp';
    groovyCompatibility?: boolean;
  }

  class Topic<TMsg = any> {
    constructor(options: TopicOptions);
    publish(message: TMsg): void;
    subscribe(callback: (message: TMsg) => void): void;
    unsubscribe(callback: (message: TMsg) => void): void;
    advertise(): void;
    unadvertise(): void;
  }

  interface TopicOptions {
    ros: Ros;
    name: string;
    messageType: string;
    queue_length?: number;
    compression?: 'none' | 'cbor' | 'png';
    throttle_rate?: number;
  }

  class Service<TReq = any, TRes = any> {
    constructor(options: ServiceOptions);
    callService(
      request: ServiceRequest<TReq>,
      callback: (response: ServiceResponse<TRes>) => void,
      errorCallback?: (error: any) => void
    ): void;
  }

  interface ServiceOptions {
    ros: Ros;
    name: string;
    serviceType: string;
  }

  class ServiceRequest<T> {
    constructor(data: T);
  }

  class ServiceResponse<T> {
    constructor(data?: T);
    isSuccess(): boolean;
  }

  class Param {
    constructor(options: ParamOptions);
    get(callback: (value: any) => void): void;
    set(value: any): void;
    delete(): void;
  }

  interface ParamOptions {
    ros: Ros;
    name: string;
  }

  class Action<TGoal = any, TFeedback = any, TResult = any> {
    constructor(options: ActionOptions);
    sendGoal(
      goal: ActionGoal<TGoal>,
      callback?: (feedback: ActionFeedback<TFeedback>) => void,
      timeout?: number
    ): void;
    cancel(): void;
  }

  interface ActionOptions {
    ros: Ros;
    serverName: string;
    actionName: string;
    goalType?: string;
    feedbackType?: string;
    resultType?: string;
  }

  interface ActionGoal<T> {
    goal: T;
  }

  interface ActionFeedback<T> {
    feedback: T;
    status: {
      status: number;
      text: string;
    };
  }

  class TFClient {
    constructor(options: TFClientOptions);
    subscribe(callback: (tf: any) => void): void;
    dispose(): void;
  }

  interface TFClientOptions {
    ros: Ros;
    fixedFrame: string;
    targetFrame: string;
    updateRate?: number;
    averageRate?: number;
    updateOffset?: number;
  }

  namespace Urdf {
    class Mesh {
      constructor(options: UrdfMeshOptions);
    }

    interface UrdfMeshOptions {
      ros: Ros;
      path: string;
    }

    class Model {
      constructor(options: UrdfModelOptions);
      getMeshes(): void;
    }

    interface UrdfModelOptions {
      ros: Ros;
      path: string;
    }
  }

  namespace Powell {
    class Trajectory {
      constructor(options: PowellTrajectoryOptions);
    }
  }

  interface PowellTrajectoryOptions {
    ros: Ros;
  }

  namespace Mujoco {
    class Simulation {
      constructor(options: MujocoSimulationOptions);
    }
  }

  interface MujocoSimulationOptions {
    ros: Ros;
  }

  namespace SBCL {
    class Kinematics {
      constructor(options: SBCLKinematicsOptions);
    }

    interface SBCLKinematicsOptions {
      ros: Ros;
      path: string;
    }
  }

  namespace Stage {
    class Simulation {
      constructor(options: StageSimulationOptions);
    }

    interface StageSimulationOptions {
      ros: Ros;
    }
  }
}

export = ROSLIB;
