import { useEffect, useMemo, useRef } from 'react';
import ROSLIB from 'roslib';
import { useROS } from '../RosContext';

/**
 * useRosApi: headless hook to centralize ROS communications.
 * Exposes a small contract for IK, joint command publishing, and joint state subscription.
 */
export function useRosApi() {
  const { ros, connected } = useROS();
  const ikServiceRef = useRef(null);
  const jointCmdTopicRef = useRef(null);
  const stopTopicRef = useRef(null);

  // Init services/topics when ROS is ready
  useEffect(() => {
    if (!ros || !connected) {
      ikServiceRef.current = null;
      jointCmdTopicRef.current = null;
      return;
    }
    ikServiceRef.current = new ROSLIB.Service({
      ros,
      name: '/compute_ik',
      serviceType: 'moveit_msgs/srv/GetPositionIK',
    });
    jointCmdTopicRef.current = new ROSLIB.Topic({
      ros,
      name: '/joint_group_position_controller/command',
      messageType: 'std_msgs/Float64MultiArray',
    });
    return () => {
      ikServiceRef.current = null;
      jointCmdTopicRef.current = null;
      stopTopicRef.current = null;
    };
  }, [ros, connected]);

  // Compute IK given pose in mm and optional seed joints map
  const computeIK = (poseMm, seedJoints, opts = {}) =>
    new Promise((resolve) => {
      if (!ros || !connected || !ikServiceRef.current) {
        return resolve({ ok: false, error: 'ros_not_ready' });
      }
      const { x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 1 } = poseMm || {};
      const pose = {
        header: { frame_id: 'base_link' },
        pose: {
          position: { x: x / 1000, y: y / 1000, z: z / 1000 },
          orientation: { x: qx, y: qy, z: qz, w: qw },
        },
      };
      const names = [];
      const positions = [];
      if (seedJoints) {
        Object.keys(seedJoints).forEach((n) => {
          if (seedJoints[n] !== undefined) {
            names.push(n);
            positions.push(seedJoints[n]);
          }
        });
      }
      const robot_state = names.length
        ? { joint_state: { name: names, position: positions } }
        : {};
      const req = {
        ik_request: {
          group_name: opts.groupName || 'arm_group',
          pose_stamped: pose,
          ik_link_name: opts.ikLinkName || 'gripper_mid_point',
          timeout: { sec: 0, nanosec: 0 },
          constraints: {},
          robot_state,
          avoid_collisions: opts.avoidCollisions ?? false,
        },
      };
      ikServiceRef.current.callService(req, (res) => {
        if (
          res &&
          res.solution &&
          res.solution.joint_state &&
          res.error_code &&
          res.error_code.val === 1
        ) {
          const solved = {};
          res.solution.joint_state.name.forEach((n, i) => {
            solved[n] = res.solution.joint_state.position[i];
          });
          resolve({ ok: true, joints: solved, raw: res });
        } else {
          resolve({ ok: false, error: 'ik_unreachable', raw: res });
        }
      });
    });

  // Publish a joint group command (Float64MultiArray) from a joints map and order
  const publishJointGroupCommand = (jointOrder, jointsMap) => {
    if (!ros || !connected || !jointCmdTopicRef.current) return false;
    const data = jointOrder.map((n) => (jointsMap[n] !== undefined ? jointsMap[n] : 0));
    const msg = new ROSLIB.Message({ data });
    jointCmdTopicRef.current.publish(msg);
    return true;
  };

  // Publish a stop event to trajectory execution
  const publishStopEvent = () => {
    if (!ros || !connected) return false;
    if (!stopTopicRef.current) {
      stopTopicRef.current = new ROSLIB.Topic({
        ros,
        name: '/trajectory_execution_event',
        messageType: 'std_msgs/String',
      });
    }
    const stopMessage = new ROSLIB.Message({ data: 'stop' });
    stopTopicRef.current.publish(stopMessage);
    return true;
  };

  // Fetch URDF xml string via robot_state_publisher parameters
  const getUrdfXml = () =>
    new Promise((resolve) => {
      if (!ros || !connected) return resolve(null);
      try {
        const srv = new ROSLIB.Service({
          ros,
          name: '/robot_state_publisher/get_parameters',
          serviceType: 'rcl_interfaces/srv/GetParameters',
        });
        const req = new ROSLIB.ServiceRequest({ names: ['robot_description'] });
        srv.callService(req, (result) => {
          const urdfXml = result?.values?.[0]?.string_value || null;
          resolve(urdfXml);
        });
      } catch (_) {
        resolve(null);
      }
    });

  // Subscribe to /joint_states; returns an unsubscribe function
  const subscribeJointStates = (callback) => {
    if (!ros) return () => {};
    const topic = new ROSLIB.Topic({
      ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/msg/JointState',
    });
    topic.subscribe((message) => {
      try {
        callback?.(message);
      } catch (_) {}
    });
    return () => topic.unsubscribe();
  };

  // Publish cartesian goal and subscribe to status/waypoints
  const publishCartesianGoal = (startPose, endPose, opts = {}) =>
    new Promise((resolve) => {
      if (!ros || !connected) {
        return resolve({ ok: false, error: 'ros_not_ready' });
      }

      // Create topics for publishing goal and subscribing to responses
      const cartesianGoalTopic = new ROSLIB.Topic({
        ros,
        name: '/cartesian_goal',
        messageType: 'thor_msgs/CartesianGoal',
      });

      // Status and waypoints topics
      const statusTopic = new ROSLIB.Topic({
        ros,
        name: '/cartesian_goal_status',
        messageType: 'std_msgs/String',
      });

      // Waypoints topic
      const waypointsTopic = new ROSLIB.Topic({
        ros,
        name: '/cartesian_waypoints',
        messageType: 'std_msgs/String',
      });

      let statusSubscription = null;
      let waypointsSubscription = null;
      let timeoutId = null;

      // Cleanup subscriptions and timeout
      const cleanup = () => {
        if (statusSubscription) statusSubscription.unsubscribe();
        if (waypointsSubscription) waypointsSubscription.unsubscribe();
        if (timeoutId) clearTimeout(timeoutId);
      };

      let waypointsData = null;

      // Subscribe to waypoints
      waypointsSubscription = waypointsTopic.subscribe((message) => {
        try {
          const data = JSON.parse(message.data);
          waypointsData = data.waypoints || [];
          console.log('Received waypoints:', waypointsData);
        } catch (e) {
          console.error('Error parsing waypoints:', e);
        }
      });

      // Subscribe to status updates
      statusSubscription = statusTopic.subscribe((message) => {
        try {
          const status = JSON.parse(message.data);
          console.log('Cartesian goal status:', status);

          if (status.status === 'completed') {
            cleanup();
            resolve({
              ok: true,
              waypoints: waypointsData || [],
              fraction: status.fraction || 1.0,
              message: status.message
            });
          } else if (status.status === 'error') {
            cleanup();
            resolve({
              ok: false,
              error: 'cartesian_path_failed',
              message: status.message
            });
          }
          // For 'processing' and 'executing' states, we just wait
        } catch (e) {
          console.error('Error parsing status:', e);
        }
      });

      // Set timeout
      timeoutId = setTimeout(() => {
        cleanup();
        resolve({ ok: false, error: 'timeout' });
      }, 30000); // 30 second timeout

      // Prepare and publish the cartesian goal
      const { x: startX = 0, y: startY = 0, z: startZ = 0, qx: startQx = 0, qy: startQy = 0, qz: startQz = 0, qw: startQw = 1 } = startPose || {};
      const { x: endX = 0, y: endY = 0, z: endZ = 0, qx: endQx = 0, qy: endQy = 0, qz: endQz = 0, qw: endQw = 1 } = endPose || {};

      const goal = new ROSLIB.Message({
        start_pose: {
          position: { x: startX / 1000, y: startY / 1000, z: startZ / 1000 },
          orientation: { x: startQx, y: startQy, z: startQz, w: startQw },
        },
        end_pose: {
          position: { x: endX / 1000, y: endY / 1000, z: endZ / 1000 },
          orientation: { x: endQx, y: endQy, z: endQz, w: endQw },
        },
        gripperbase_to_armgearright: opts.gripperValue || 0.0,
        max_step: opts.maxStep || 0.01,
        jump_threshold: opts.jumpThreshold || 0.0,
        avoid_collisions: opts.avoidCollisions ?? false,
      });

      cartesianGoalTopic.publish(goal);
    });

  // Wait for movement completion by monitoring trajectory execution status
  const waitForMovementCompletion = (timeoutMs = 30000, shouldStopCallback = null) =>
    new Promise((resolve) => {
      if (!ros || !connected) {
        resolve({ success: false, error: 'ROS not connected' });
        return;
      }

      const startTime = Date.now();
      let executionSubscription = null;

      // Subscribe to trajectory execution events
      const executionTopic = new ROSLIB.Topic({
        ros,
        name: '/execute_trajectory/status',
        messageType: 'action_msgs/msg/GoalStatusArray'
      });

      // Helper functions to check timeout and stop conditions
      const checkTimeout = () => {
        if (Date.now() - startTime > timeoutMs) {
          if (executionSubscription) executionSubscription.unsubscribe();
          console.error('[useRosApi] waitForMovementCompletion: timeout after', timeoutMs, 'ms');
          resolve({ success: false, error: 'Movement timeout' });
          return true;
        }
        return false;
      };

      // Check for user-initiated stop
      const checkStop = () => {
        if (shouldStopCallback && shouldStopCallback()) {
          if (executionSubscription) executionSubscription.unsubscribe();
          console.warn('[useRosApi] waitForMovementCompletion: stopped by user via shouldStopCallback');
          resolve({ success: false, error: 'Execution stopped by user' });
          return true;
        }
        return false;
      };

      let hasReceivedGoal = false;

      // Subscribe to execution status
      executionSubscription = executionTopic.subscribe((message) => {
        try {
          if (checkTimeout() || checkStop()) return;

          if (message.status_list && message.status_list.length > 0) {
            const latestStatus = message.status_list[message.status_list.length - 1];
            console.log('[useRosApi] trajectory status update:', latestStatus);
            
            // Goal states: PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3, ABORTED=4, REJECTED=5, PREEMPTING=6, RECALLING=7, RECALLED=8, LOST=9
            // ACTIVE
            if (latestStatus.status === 1) {
              hasReceivedGoal = true;
            }

            // Treat SUCCEEDED as terminal success even if we didn't observe ACTIVE
            if (latestStatus.status === 3) { // SUCCEEDED
              console.log('[useRosApi] Movement completed successfully via ROS trajectory execution (SUCCEEDED)');
              executionSubscription.unsubscribe();
              resolve({ success: true, message: 'Movement completed by ROS' });
            }

            // Treat ABORTED or REJECTED as terminal failure
            if (latestStatus.status === 4 || latestStatus.status === 5) { // ABORTED or REJECTED
              console.warn('[useRosApi] Movement failed via ROS trajectory execution (ABORTED/REJECTED)');
              executionSubscription.unsubscribe();
              resolve({ success: false, error: 'Movement aborted or rejected by ROS' });
            }
          }
        } catch (error) {
          console.error('Error processing trajectory status:', error);
        }
      });

      // Set a timeout as fallback
      setTimeout(() => {
        if (checkTimeout() || checkStop()) return;
        
        // If no trajectory execution detected, assume simple joint command completed quickly
        if (!hasReceivedGoal) {
          console.log('No trajectory execution detected, assuming joint command completed');
          if (executionSubscription) executionSubscription.unsubscribe();
          resolve({ success: true, message: 'Joint command assumed completed' });
        }
      }, 2000); // Wait 2 seconds for trajectory execution to start
    });

  return useMemo(
    () => ({ ros, connected, computeIK, publishJointGroupCommand, subscribeJointStates, getUrdfXml, publishStopEvent, publishCartesianGoal, waitForMovementCompletion }),
    [ros, connected]
  );
}

export default useRosApi;
