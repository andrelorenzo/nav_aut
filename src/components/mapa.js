import  React, { useState ,useRef, useEffect} from 'react';
import { GoogleMap, Marker, useJsApiLoader, DrawingManager} from '@react-google-maps/api';
import '../App.css';
import CoordinateDisplay from './CoordinateDisplay';
import image from './/robotImage.png'
const libs  = ['drawing','maps'];

const MapComponent = ({content, pos}) => {
  const { isLoaded, } = useJsApiLoader({
    googleMapsApiKey: 'AIzaSyCLlJzJeukwwxa1CQ98iuy8lRLnfUoMLJ0',
     libraries:libs
  });
  const initialMarkerPosition = { lat: 36.71440839196788, lng: -4.478464366197594 };
  //const initialMarkerPosition = { lat: 38.161479, lng: -122.454630 };
  //const initialMarkerPosition = { lat: 36.71440839196788, lng: -4.478464366197594 };
  const initialrectpos={
    latne: 0, lngne: 0,
    latsw: 0, lngsw: 0,
  };
  const initialpoly = [];
  const [markerPosition, setMarkerPosition] = useState(initialMarkerPosition);
  const [selectedArea, setSelectedArea] = useState(initialrectpos);
  const [polygonArea, setpolygonArea] = useState(initialpoly);

  const prevRectangleRef = useRef(null);
  const prevpolygon = useRef(null);
  const prevpolyline = useRef(null);


  const handleContent = (payload) => {

    content(JSON.stringify(payload));
    
  };
  const handleDrawingComplete = (rectangle) => {
    if (prevRectangleRef.current !== null) {
      prevRectangleRef.current.setMap(null); // Borra el rectángulo anterior
    }
    if (prevpolygon.current !== null) {
      prevpolygon.current.setMap(null); // Borra el rectángulo anterior
    }
    prevRectangleRef.current = rectangle; // Almacena el ID del rectángulo actual
    const bounds = rectangle.getBounds();
    const ne = bounds.getNorthEast();
    const sw = bounds.getSouthWest();
    const newArea = {
      latne: ne.lat(), lngne: ne.lng() ,
      latsw: sw.lat(), lngsw: sw.lng() ,
      type: 'rectangle',
    };
    setSelectedArea(newArea);
    handleContent(newArea);

  };


  const handlepolygon = (polygon) => {
    if (prevRectangleRef.current !== null) {
      prevRectangleRef.current.setMap(null);
    }
    if (prevpolygon.current !== null) {
      prevpolygon.current.setMap(null); // Borra el rectángulo anterior
    }
    if (prevpolyline.current !== null) {
      prevpolyline.current.setMap(null); // Borra el rectángulo anterior
    }
    prevpolygon.current = polygon;
    var paths = polygon.getPath();
    var latlngArry = [];
    paths.getArray().forEach(function (value, index, array_x) {
      console.log(" index: " + index + " value: " + value);
      var lati = Number(value.lat());
      var lngi = Number(value.lng());
      var myLatlng = {lat:lati,lng:lngi};
      latlngArry.push(myLatlng);//value
      });
      console.log(latlngArry)
      var area = {
        polygon:latlngArry,
        type:'polygon',
        length:latlngArry.length
      }
      setpolygonArea(area);
      handleContent(area);
  };

  const handleline = (polyline) => {
    if (prevRectangleRef.current !== null) {
      prevRectangleRef.current.setMap(null);
    }
    if (prevpolygon.current !== null) {
      prevpolygon.current.setMap(null);
    }
    if (prevpolyline.current !== null) {
      prevpolyline.current.setMap(null); // Borra el rectángulo anterior
    }
    prevpolyline.current = polyline;
    var paths = polyline.getPath();
    var latlngArray = [];
    paths.forEach((path) => {
      const lati = Number(path.lat());
      const lngi = Number(path.lng());
      const myLatlng = { lat: lati, lng: lngi };
      latlngArray.push(myLatlng);
    });
    var path = {
      line: latlngArray,
      type: "line",
      length:latlngArray.length
    }
    handleContent(path);

  };

 const handleMarkerDragEnd = (event) => {
  if (prevRectangleRef.current !== null) {
    prevRectangleRef.current.setMap(null);
  }
  if (prevpolygon.current !== null) {
    prevpolygon.current.setMap(null);
  }
  if (prevpolyline.current !== null) {
    prevpolyline.current.setMap(null); // Borra el rectángulo anterior
  }
    const newPosition = {
      lat: event.latLng.lat(),
      lng: event.latLng.lng(),
      type: 'marker',
    };
    setMarkerPosition(newPosition);
    handleContent(newPosition);
  };    
    

  return isLoaded ? (
    <>
      <div>
      <h1 className='App-header'>INTERFAZ ROS2 </h1>
      </div>
    <div className='map_cord'>
    <div >
      <GoogleMap
      mapTypeId='satellite'
      tilt={0}
      mapContainerClassName='map-container'
      gestureHandling={'greedy'}
      disableDefaultUI={true}
      center={markerPosition}
      zoom={50}
      >
      {markerPosition && (
        <Marker
          position={markerPosition}
          draggable={true}
          onDragEnd={handleMarkerDragEnd}
        />
       
      )}
      <DrawingManager
          onRectangleComplete={handleDrawingComplete}
          onPolygonComplete={handlepolygon}
          onPolylineComplete={handleline}
          drawingMode={[
            window.google.maps.drawing.OverlayType.RECTANGLE,
            window.google.maps.drawing.OverlayType.POLYGON,
            window.google.maps.drawing.OverlayType.POLYLINE
          ]}
          options={{ drawingControl: true, drawingControlOptions: { position: window.google.maps.ControlPosition.TOP_CENTER }, rectangleOptions: { draggable: false }, polylineOptions: {draggable: false} }}
       />
        {selectedArea && (
          <>
            {/* <Marker position={{ lat: selectedArea.latne, lng: selectedArea.lngne }} />
            <Marker position={{ lat: selectedArea.latsw, lng: selectedArea.lngsw }} /> */}
          </>
        )}
         <Marker 
          icon= {{url: image,scaledSize: {height: 50, width: 50}}}
          // "https://developers.google.com/maps/documentation/javascript/examples/full/images/info-i_maps.png"
          draggable={false}
          position={{ lat: pos.lat, lng: pos.lng }}
         />
      </GoogleMap>
      </div>
  <div >
      <CoordinateDisplay selectedArea={selectedArea} coordinates={markerPosition}></CoordinateDisplay>
  </div>
  </div>
  </>

        
  ) : null;
};

export default MapComponent;
