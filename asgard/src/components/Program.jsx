import React, { useState, useEffect } from 'react';
import Button from '@mui/material/Button';
import Select from '@mui/material/Select';
import MenuItem from '@mui/material/MenuItem';
import Autocomplete from '@mui/material/Autocomplete';
import TextField from '@mui/material/TextField';

function Program() {
  const [movements, setMovements] = useState([]);
  const [poseNames, setPoseNames] = useState([]);

  useEffect(() => {
    const savedPoses = JSON.parse(localStorage.getItem('savedPoses')) || [];
    setPoseNames(savedPoses.map(pose => pose.name));
  }, []);

  useEffect(() => {
    const savedProgram = JSON.parse(localStorage.getItem('program')) || [];
    setMovements(savedProgram);
  }, []);

  const saveProgramToLocalStorage = (updatedMovements) => {
    localStorage.setItem('program', JSON.stringify(updatedMovements));
  };

  const handleAddMovement = () => {
    const updatedMovements = [...movements, { pose: '', type: 'Joint' }];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleAddMovementBelow = (index) => {
    const updatedMovements = [
      ...movements.slice(0, index + 1),
      { pose: '', type: 'Joint' },
      ...movements.slice(index + 1)
    ];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleDeleteMovement = (index) => {
    const updatedMovements = movements.filter((_, i) => i !== index);
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleChangeMovement = (index, field, value) => {
    const updatedMovements = movements.map((movement, i) =>
      i === index ? { ...movement, [field]: value } : movement
    );
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleMoveUp = (index) => {
    if (index === 0) return;
    const updatedMovements = [...movements];
    [updatedMovements[index - 1], updatedMovements[index]] = [updatedMovements[index], updatedMovements[index - 1]];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  const handleMoveDown = (index) => {
    if (index === movements.length - 1) return;
    const updatedMovements = [...movements];
    [updatedMovements[index + 1], updatedMovements[index]] = [updatedMovements[index], updatedMovements[index + 1]];
    setMovements(updatedMovements);
    saveProgramToLocalStorage(updatedMovements);
  };

  return (
    <div style={{ padding: '0.25rem' }}>
      {movements.length === 0 ? (
        <div style={{ textAlign: 'center', marginTop: '2rem' }}>
          <p style={{ fontSize: '1.2rem', color: '#555' }}>There are no movements programmed yet</p>
          <Button variant="contained" color="primary" onClick={handleAddMovement}>
            Add Movement
          </Button>
        </div>
      ) : (
        <div style={{ marginTop: '0.25rem' }}>
          {movements.map((movement, index) => (
            <div
              key={index}
              style={{ display: 'flex', alignItems: 'center', marginBottom: '1rem', marginLeft: '0.3rem', marginRight: '0.3rem' }}
            >
              <div style={{ display: 'flex', flexDirection: 'column', marginRight: '0.3rem' }}>
                <Button
                  variant="contained"
                  style={{ backgroundColor: 'green', color: 'white', padding: '0.3rem', minWidth: '36px', marginBottom: '0.2rem' }}
                  onClick={() => handleAddMovementBelow(index)}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>add</span>
                </Button>
                <Button
                  variant="contained"
                  color="error"
                  onClick={() => handleDeleteMovement(index)}
                  style={{ padding: '0.3rem', minWidth: '36px' }}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>delete</span>
                </Button>
              </div>

              <div style={{ display: 'flex', flexDirection: 'column', marginRight: '0.3rem' }}>
                <Button
                  variant="contained"
                  style={{ backgroundColor: 'blue', color: 'white', padding: '0.3rem', minWidth: '36px', marginBottom: '0.2rem' }}
                  onClick={() => handleMoveUp(index)}
                  disabled={index === 0}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>arrow_upward</span>
                </Button>

                <Button
                  variant="contained"
                  style={{ backgroundColor: 'blue', color: 'white', padding: '0.3rem', minWidth: '36px' }}
                  onClick={() => handleMoveDown(index)}
                  disabled={index === movements.length - 1}
                >
                  <span className="material-icons" style={{ fontSize: '20px' }}>arrow_downward</span>
                </Button>
              </div>

              <Autocomplete
                options={poseNames}
                value={movement.pose}
                onChange={(event, newValue) => handleChangeMovement(index, 'pose', newValue)}
                renderInput={(params) => (
                  <TextField {...params} label="Pose" variant="outlined" style={{ marginLeft: '0.5rem', width: '180px', paddingTop: '0', paddingBottom: '0' }} />
                )}
              />

              <Select
                value={movement.type}
                onChange={(e) => handleChangeMovement(index, 'type', e.target.value)}
                style={{ marginLeft: '0.5rem', width: '92px', minWidth: '92px' }}
              >
                <MenuItem value="Joint">Joint</MenuItem>
                <MenuItem value="Linear">Linear</MenuItem>
              </Select>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

export default Program;
