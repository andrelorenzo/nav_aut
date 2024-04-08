import React from 'react';
import { Button, Card, ColorPicker, Switch } from 'antd';
const CoordinateDisplay = ({ coordinates, selectedArea}) => {

  return (
    <div className='App'>
    <div className='coordinates'>
      <Card title='COORDENADAS DEL AREA' styles={{header:{color:'white'}}} style={{width: 450, height:400}} className='cards'>
        <p>Esquina Noreste:</p>
        <p>Latitud: {selectedArea.latne}, Longitud: {selectedArea.lngne}</p>
        <p>Esquina Suroeste:</p>
        <p>Latitud: {selectedArea.latsw}, Longitud: {selectedArea.lngsw}</p>
        <p></p>

  
    </Card>
    <Card title='COORDENADAS DEL MARKER (DD)' styles={{header:{color:'white'}}} className='cards'>
        <p>Latitud: {coordinates.lat}</p>
        <p>Longitud: {coordinates.lng}</p>
    </Card>
  </div>
  </div>

  
  );
};

export default CoordinateDisplay;



