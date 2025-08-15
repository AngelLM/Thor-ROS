import { useEffect, useState } from 'react';
import Button from '@mui/material/Button';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogContentText from '@mui/material/DialogContentText';
import DialogTitle from '@mui/material/DialogTitle';
import { useROS } from '../RosContext';
import ROSLIB from 'roslib';
import { useImperativeHandle, forwardRef } from 'react';

function Poses({ onPreviewJointsChange, poses, setPoses }, ref) {
  const { ros, connected } = useROS();
  const [isDeleteDialogOpen, setIsDeleteDialogOpen] = useState(false);
  const [poseNameToDelete, setPoseNameToDelete] = useState(null);

  useEffect(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoses(savedPoses);

  }, [setPoses]);

  const refreshSavedPoses = () => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoses(savedPoses);
  };

  useImperativeHandle(ref, () => ({
    updatePoses: refreshSavedPoses
  }));

  const openDeleteDialog = (poseName) => {
    setPoseNameToDelete(poseName);
    setIsDeleteDialogOpen(true);
  };

  const closeDeleteDialog = () => {
    setIsDeleteDialogOpen(false);
    setPoseNameToDelete(null);
  };

  const confirmDeletePose = () => {
    const updatedPoses = poses.filter(pose => pose.name !== poseNameToDelete);
    localStorage.setItem('savedPoses', JSON.stringify(updatedPoses));
    setPoses(updatedPoses);
    setIsDeleteDialogOpen(false);
    setPoseNameToDelete(null);
  };

  const publishPoseToController = (poseName) => {

    if (!connected || !ros) {
      console.warn('ROS is not connected.');
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
      data: Object.values(pose.joints),
    });

    topic.publish(message);
  };

  const previewPoseOnGhost = (poseName) => {
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
                onClick={() => openDeleteDialog(pose.name)}
                style={{ padding: '0.5rem' }}
              >
                <span className="material-icons">delete</span>
              </Button>
              <span style={{ flex: 1, textAlign: 'left', paddingLeft: '0.5rem' }}>{pose.name}</span>
              <Button
                variant="contained"
                style={{ backgroundColor: '#808080', color: '#fff', padding: '0.5rem', marginRight: '4px' }}
                onClick={() => previewPoseOnGhost(pose.name)}
              >
                <span className="material-icons">visibility</span>
              </Button>
              <Button
                variant="contained"
                color="primary"
                  onClick={() => publishPoseToController(pose.name)}
                style={{ padding: '0.5rem' }}
              >
                <span className="material-icons">navigation</span>
              </Button>
            </li>
          ))}
        </ul>
      )}

      {/* Confirmation Dialog */}
      <Dialog open={isDeleteDialogOpen} onClose={closeDeleteDialog}>
        <DialogTitle>Confirm Deletion</DialogTitle>
        <DialogContent>
          <DialogContentText>
            Are you sure you want to delete the pose "{poseNameToDelete}"? This action cannot be undone.
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={closeDeleteDialog} color="secondary">
            Cancel
          </Button>
          <Button onClick={confirmDeletePose} color="error">
            Delete
          </Button>
        </DialogActions>
      </Dialog>
    </div>
  );
}

export default forwardRef(Poses);
