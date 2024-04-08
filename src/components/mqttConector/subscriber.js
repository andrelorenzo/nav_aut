import React, { useEffect } from 'react';
import PropTypes from 'prop-types';

const MQTTSubscriber = ({mqttClient, topic, onMessage }) => {
  useEffect(() => {


    mqttClient.on('connect', () => {
      mqttClient.subscribe(topic);
    });

    mqttClient.on('message', (topic, message) => {
      console.log('Mensaje sin parsear:', message.toString());
      let msg = JSON.parse(message)
      onMessage(msg);

    });

    return () => {
        mqttClient.unsubscribe(topic);
        console.log('Desconectado del topic:', topic);
        mqttClient.subscribe(topic);
    };
  }, [mqttClient, topic, onMessage]);

  return null; // No necesitamos renderizar nada en este componente
};


MQTTSubscriber.propTypes = {
    mqttClient: PropTypes.object.isRequired, // Cliente MQTT ya conectado
    topic: PropTypes.string.isRequired, // Tema al que suscribirse
    onMessage: PropTypes.func.isRequired // Función para manejar mensajes recibidos
  };

export default MQTTSubscriber;
