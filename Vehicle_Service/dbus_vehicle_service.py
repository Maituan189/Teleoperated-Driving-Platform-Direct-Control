import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib
import json
import random
import time
import threading
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('VehicleDBusService')

class VehicleDBusService(dbus.service.Object):
    """
    DBus service for vehicle data communication with Qt
    """
    def __init__(self, bus_name, object_path='/org/example/PythonVehicleService'):
        super().__init__(bus_name, object_path)
        self.counter = 0
        self.qt_data_received = None
        self.data_callback = None
        self.send_callback = None
        
        # Data generation thread
        self.running = False
        self.thread = None
        
        logger.info("Python DBus service initialized")
    
    def register_data_callback(self, callback):
        """Register callback to be called when data is received from Qt"""
        self.data_callback = callback
        
    def register_send_callback(self, callback):
        """Register callback to generate data to send to Qt"""
        self.send_callback = callback
    
    @dbus.service.method('org.example.PythonVehicleService', 
                         in_signature='s', out_signature='b')
    def receiveQtData(self, json_data):
        """Method exposed via D-Bus for Qt to send data"""
        try:
            data = json.loads(json_data)
            
            # logger.info(f"Python received: Steering={data.get('Steering_Angle', 'N/A')}, "
            #            f"Throttle={data.get('Throttle_Value', 'N/A')}, "
            #            f"Brake={data.get('Brake_Value', 'N/A')}, "
            #            f"Gear={data.get('GearShift_Value', 'N/A')}")
            
            self.qt_data_received = data
            
            # Call callback if registered
            if self.data_callback:
                self.data_callback(data)
                
            return True
        except Exception as e:
            logger.error(f"Error processing Qt data: {e}")
            return False

    @dbus.service.method('org.example.PythonVehicleService', 
                         in_signature='', out_signature='s')
    def getVehicleData(self):
        """Method exposed via D-Bus for Qt to request data"""
        try:
            # Use custom data from callback if available
            if self.send_callback:
                data = self.send_callback()
            else:
                # Generate sample data
                self.counter += 1
                data = {
                    "Speed": random.uniform(0, 200),
                    "GPS": [random.uniform(-90, 90), random.uniform(-180, 180)],
                    "Steering angle": random.uniform(-270, 270),
                    "Timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "Counter": self.counter
                }
                
            # logger.info(f"Python sending: Speed={data.get('Speed_Value', 'N/A')}, "
            #            f"GPS={data.get('GPS', 'N/A')}, "
            #            f"Steering_Angle={data.get('Steering_Angle', 'N/A')}, "
            #            f"Roll_Angle={data.get('Roll_Angle', 'N/A')}, "
            #            f"Pitch_Angle={data.get('Pitch_Angle', 'N/A')}, "
            #            f"Yaw_Angle={data.get('Yaw_Angle', 'N/A')}, "
            #            f"Yaw_Rate={data.get('Yaw_Rate', 'N/A')} "
            #            )
                
            return json.dumps(data)
        except Exception as e:
            logger.error(f"Error generating vehicle data: {e}")
            return json.dumps({"error": str(e)})

def create_dbus_service():
    """Create and return a DBus service instance"""
    # Set up the DBus main loop
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    
    # Connect to the session bus
    session_bus = dbus.SessionBus()
    
    # Define the bus name
    bus_name = dbus.service.BusName("org.example.PythonVehicleService", 
                                   session_bus)
    
    # Create service object
    service = VehicleDBusService(bus_name)
    
    logger.info("DBus vehicle service created")
    return service