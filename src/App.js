import React from 'react';
import MapComponent from './components/mapa';
import MQTT_Component from './components/mqttConector'
import './App.css'

const App = () => {
  return (
    <div>
      <MQTT_Component />
    </div>
  );
};

export default App;
