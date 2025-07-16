import React, { useState, useEffect } from 'react';
import Button from '@mui/material/Button';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogContentText from '@mui/material/DialogContentText';
import DialogTitle from '@mui/material/DialogTitle';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';
import { useImperativeHandle, forwardRef } from 'react';

function Poses({ ghostRef, onPreviewJointsChange }, ref) {
  const { ros, connected } = useROS();
  const [poses, setPoses] = useState([]);
  const [isDialogOpen, setIsDialogOpen] = useState(false);
  const [poseToDelete, setPoseToDelete] = useState(null);

  useEffect(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoses(savedPoses);

  }, []);

  const updatePoses = () => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoses(savedPoses);
  };

  useImperativeHandle(ref, () => ({
    updatePoses
  }));

  const handleOpenDialog = (poseName) => {
    setPoseToDelete(poseName);
    setIsDialogOpen(true);
  };

  const handleCloseDialog = () => {
    setIsDialogOpen(false);
    setPoseToDelete(null);
  };

  const handleConfirmDelete = () => {
    const updatedPoses = poses.filter(pose => pose.name !== poseToDelete);
    localStorage.setItem('savedPoses', JSON.stringify(updatedPoses));
    setPoses(updatedPoses);
    setIsDialogOpen(false);
    setPoseToDelete(null);
  };

  const handleMoveToPose = (poseName) => {
    // print log for debugging
    console.log(`Moving to pose: ${poseName}`);

    if (!connected || !ros) {
      console.warn('ROS no está conectado.');
      return;
    }

    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    const pose = savedPoses.find(p => p.name === poseName);

    const topic = new ROSLIB.Topic({
      ros,
      name: '/joint_group_position_controller/command',
      messageType: 'std_msgs/Float64MultiArray',
    });
    const message = new ROSLIB.Message({
      data: Object.values(pose.joints).slice(0, 6),
    });

    topic.publish(message);
  };

  const handleTestOnGhost = (poseName) => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    const pose = savedPoses.find(p => p.name === poseName);

    if (typeof onPreviewJointsChange === 'function') {
      onPreviewJointsChange(pose.joints);
    }
  };

  return (
    <div style={{ padding: '0.5rem' }}>
      {poses.length === 0 ? (
        <p>There are no saved poses to show yet</p>
      ) : (
        <ul style={{ listStyleType: 'none', padding: 0 }}>
          {poses.map((pose) => (
            <li
              key={pose.name}
              style={{ display: 'flex', alignItems: 'center', marginBottom: '0.5rem' }}
            >
              <Button
                variant="contained"
                color="error"
                onClick={() => handleOpenDialog(pose.name)}
                style={{ padding: '0.5rem' }}
              >
                <span className="material-icons">delete</span>
              </Button>
              <span style={{ flex: 1, textAlign: 'left', paddingLeft: '0.5rem' }}>{pose.name}</span>
              <Button
                variant="contained"
                style={{ backgroundColor: '#808080', color: '#fff', padding: '0.5rem', marginRight: '4px' }}
                onClick={() => handleTestOnGhost(pose.name)}
              >
                <span className="material-icons">visibility</span>
              </Button>
              <Button
                variant="contained"
                color="primary"
                onClick={() => handleMoveToPose(pose.name)}
                style={{ padding: '0.5rem' }}
              >
                <span className="material-icons">navigation</span>
              </Button>
            </li>
          ))}
        </ul>
      )}

      {/* Confirmation Dialog */}
      <Dialog open={isDialogOpen} onClose={handleCloseDialog}>
        <DialogTitle>Confirm Deletion</DialogTitle>
        <DialogContent>
          <DialogContentText>
            Are you sure you want to delete the pose "{poseToDelete}"? This action cannot be undone.
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleCloseDialog} color="secondary">
            Cancel
          </Button>
          <Button onClick={handleConfirmDelete} color="error">
            Delete
          </Button>
        </DialogActions>
      </Dialog>
    </div>
  );
}

export default forwardRef(Poses);
