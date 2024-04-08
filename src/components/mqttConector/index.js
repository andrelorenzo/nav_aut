import React, { useState ,useEffect} from 'react';
import { Button, Card, Switch } from 'antd';
import mqtt, { connect } from 'mqtt';
import MapComponent from '../mapa';
import MQTTSubscriber from './subscriber';
import { useJsApiLoader } from '@react-google-maps/api';

const App = () => {
  const [client, setClient] = useState(null);
  const [connected, setConnected] = useState(false);
  const [publishbtn,setpublishbtn] = useState('Publish')
  const [publishContent, setPublishContent] = useState('');
  const [FMbtn, setFMbtn] = useState(false);
  const [connecting,setconnecting] = useState(false)
  const [message, setMessage] = useState('');
  const [location, setLocation] = useState(null);
  // const settings = {
  //   username: 'hunter',
  //   password: 'hunter2324',
  //   qos: 2,
  //   will: {
  //     topic: 'WillMsg',
  //     payload: 'Connection Closed abnormally..!',
  //     qos: 0,
  //     retain: false
  //   }
  // };

  var options = {
    protocol: 'ws',
    username: 'hunter',
    password: 'hunter2324',
    // clientId uniquely identifies client
    // choose any string you wish
    clientId: 'mqttjs_' + Math.random().toString(16).substr(2, 8),
    will: {
      topic: 'WillMsg',
      payload: 'Connection Closed abnormally..!',
      qos: 0,
      retain: false
    }
};




  const handleConnect = () => {
    const url = 'ws://hunter:hunter2324@192.168.1.128:8084'
    const mqttClient = mqtt.connect(url,options);
    setClient(mqttClient);
    setconnecting(true);
  };

  useEffect(() => {
    if (client) {
      client.on('connect', () => {
        console.log('Connected to MQTT broker');
        setConnected(true);
        setconnecting(false);
      })
      client.on('error', (err) => {
        console.error('Connection error: ', err)
        client.end()
        setConnected(false);
      })
      client.on('reconnect', () => {
        setconnecting(true);
        
      })
    }
  }, [client])
  
  useEffect(() => {
    if (FMbtn && client) {
      const intervalId = setInterval(() => {
        if (location) {
          const { latitude, longitude } = location.coords;
          const newPosition = {
            lat: latitude,
            lng: longitude,
            type: 'follow',
          };
          const locationMsg = JSON.stringify(newPosition);
          client.publish('desired_pos', locationMsg);
        }
      }, 1000); // Envía la ubicación cada segundo

      return () => clearInterval(intervalId); // Limpia el intervalo al desmontar el componente
    }
  }, [FMbtn, client, location]);

  useEffect(() => {
    if (FMbtn) {
      const watchId = navigator.geolocation.watchPosition(
        position => setLocation(position),
        error => console.error("Error getting geolocation:", error)
      );

      return () => navigator.geolocation.clearWatch(watchId); // Limpia el seguimiento de la ubicación al desmontar el componente
    }
  }, [FMbtn]);



  const handlePublish = () => {
    setpublishbtn('Publishing');
    if (!client) {
      console.error('MQTT client not connected');
      setpublishbtn('Error: not published')
      return;
    }
    setpublishbtn('Publish')
    console.log("enviando mensaje",publishContent)
    client.publish('desired_pos', publishContent);
    
  };

  const handleDisconnect = () => {
    if (client) {
      client.end();
      setConnected(false);
      setClient(null);
    }
  };
  const handleFMactivate = () => {
    
    if(client){
      setFMbtn(true);
      var msg = "true";
      client.publish('follow',msg);
    }
  };
  const handleFMdeactivate = () => {
    if(client){
      setFMbtn(false);
      var msg = "false";
      client.publish('follow',msg);
    }
  }
 
  const handleMessage = (msg) => {
    setMessage(msg);
  };

  return (
    <div className='App'>
      {client && (
        <MQTTSubscriber mqttClient={client} topic="position" onMessage={handleMessage} />
      )}

      <MapComponent pos={message} content={setPublishContent}/> 
      <Card style={{ padding: 30, border: 0}}  styles={{header:{color:'white'}, actions:{background:'#282c34'}}} className= 'cards' title='MQTT CONECTION'
        actions={[     <Button type="primary" loading={connecting} onClick={connected ? handleDisconnect : handleConnect}>
        {connected ? 'Disconnect from Broker' : 'Connect to Broker'}
        </Button>,
        <Button onClick={handlePublish} disabled={!connected}>
        {publishbtn}
        </Button>,
        <Button  disabled={!connected} onClick={FMbtn ? handleFMdeactivate : handleFMactivate}>
        {FMbtn ? 'FOLLOW ME ON' : 'FOLLOW ME OFF'}
        </Button>
 ]}
 >
   <p style={{padding: 10}}>Sending payload: {publishContent.slice(0,165)}....</p>
   
 </Card>
    </div>
  );
};

export default App;
  