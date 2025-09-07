import { useEffect, useState, useImperativeHandle, forwardRef } from 'react';
import Button from '@mui/material/Button';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogContentText from '@mui/material/DialogContentText';
import DialogTitle from '@mui/material/DialogTitle';
import useRosApi from '../ros/useRosApi';

function Poses({ onPreviewJointsChange, poses, setPoses }, ref) {
  const rosApi = useRosApi();
  const [isDeleteDialogOpen, setIsDeleteDialogOpen] = useState(false);
  const [poseNameToDelete, setPoseNameToDelete] = useState(null);
  const jointOrder = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','gripperbase_to_armgearright'];

  // Load saved poses from localStorage on mount
  useEffect(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoses(savedPoses);

  }, [setPoses]);

  // Function to refresh poses from localStorage
  const refreshSavedPoses = () => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoses(savedPoses);
  };

  // Expose refresh function to parent via ref
  useImperativeHandle(ref, () => ({
    updatePoses: refreshSavedPoses
  }));

  // Handle delete dialog
  const openDeleteDialog = (poseName) => {
    setPoseNameToDelete(poseName);
    setIsDeleteDialogOpen(true);
  };

  // Close delete dialog without deleting
  const closeDeleteDialog = () => {
    setIsDeleteDialogOpen(false);
    setPoseNameToDelete(null);
  };

  // Confirm deletion of the pose
  const confirmDeletePose = () => {
    const updatedPoses = poses.filter(pose => pose.name !== poseNameToDelete);
    localStorage.setItem('savedPoses', JSON.stringify(updatedPoses));
    setPoses(updatedPoses);
    setIsDeleteDialogOpen(false);
    setPoseNameToDelete(null);
  };

  // Publish the selected pose to the robot controller
  const publishPoseToController = (poseName) => {
    if (!rosApi.connected) {
      console.warn('ROS is not connected.');
      return;
    }
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    const pose = savedPoses.find(p => p.name === poseName);
    if (!pose) return;
    // Keep behavior: send joints as-is; consumers may handle gripper sign if needed
    rosApi.publishJointGroupCommand(jointOrder, pose.joints);
  };

  // Preview the selected pose on the ghost model
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
