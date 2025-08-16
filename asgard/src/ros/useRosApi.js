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

  return useMemo(
    () => ({ connected, computeIK, publishJointGroupCommand, subscribeJointStates, getUrdfXml, publishStopEvent }),
    [connected]
  );
}

export default useRosApi;
