import requests
from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, MediaStreamTrack
import asyncio
import json
import random
from datetime import datetime, timedelta, timezone
from av import VideoFrame
import time
import numpy as np
import cv2
import os
import pygame
from Vehicle_Service.dbus_vehicle_service import create_dbus_service
import logging
from gi.repository import GLib
import signal
from scipy.interpolate import make_interp_spline

# Configure logging

logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('VehicleInterface')
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

SIGNALING_SERVER_URL = 'http://192.168.0.119:8080'
ID = "answerer01"
FREQUENCY_SEND_DATA = 0.01
FREQUENCY_LATENCY_DATA = 0.1
CODEC_CLOCKRATE = 90000
FPS = 60
# FRAME_SIZE = (640, 480)
# FRAME_SIZE = (1920, 1440)
FRAME_SIZE = (1280, 960)
# FRAME_SIZE = (2560, 1920)
os.makedirs("imgs", exist_ok=True)
peer_connection = RTCPeerConnection()
FPS_Value = 0
# ===================== Vehicle Data Handler =====================#
class VehicleDataHandler:
    """Handles processing of vehicle data"""
    def __init__(self):
        self.last_qt_data = None
        logger.info("Vehicle data handler initialized")
    
    def process_qt_data(self, data):
        """Process data received from Qt"""
        self.last_qt_data = data
        # Process the data (example: apply some transformations)
        # logger.info(f"Processing Qt data: Steering={data.get('Steering_Angle')}, Throttle_Value={data.get('Throttle_Value')}, Brake_Value = {data.get('Brake_Value')}")
        # Add your custom data processing here
        ControlData_Package.set_data(Throttle_Value = data.get('Throttle_Value'))
        ControlData_Package.set_data(Steering_Angle = data.get('Steering_Angle'))
        ControlData_Package.set_data(Brake_Value = data.get('Brake_Value'))
        ControlData_Package.set_data(GearShift_Value = data.get('GearShift_Value'))
        ControlData_Package.set_data(Controller_Mode = data.get('Controller_Mode'))
        ControlData_Package.set_data(Setpoint_Speed = data.get('Setpoint_Speed'))
        
    def generate_vehicle_data(self):
        """Generate data to send to Qt"""
        # Here you can implement your custom data generation
        # This could include sensor reading, simulation, etc.
        # data = {
        #     "Speed_Value": random.uniform(0, 200),
        #     "GPS": [random.uniform(-90, 90), random.uniform(-180, 180)],
        #     "Steering_Angle": random.uniform(-270, 270),
        #     "Roll_Angle": random.uniform(-270, 270),
        #     "Pitch_Angle": random.uniform(-270, 270),
        #     "Yaw_Angle": random.uniform(-270, 270),
        #     "Yaw_Rate": random.uniform(-270, 270),
        #     "Timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        #     "Counter": int(time.time() * 1000) % 10000
        # }

        data = {
            "Speed_Value": FeedBackData_Package.get_Speed_Value(),
            "RPM": FeedBackData_Package.get_RPM_Value(),
            "GPS": [FeedBackData_Package.get_GPS_x(), FeedBackData_Package.get_GPS_y()],
            "Roll_Angle": FeedBackData_Package.get_Roll_Angle(),
            "Pitch_Angle": FeedBackData_Package.get_Pitch_Angle(),
            "Yaw_Angle": FeedBackData_Package.get_Yaw_Angle(),
            "Yaw_Rate": FeedBackData_Package.get_Yaw_Rate(),
            "ACC_Z": FeedBackData_Package.get_ACC_Z(),
            "Displacement": FeedBackData_Package.get_Displacement(),
            "Temperature_Vehicle": FeedBackData_Package.get_Temp(),
            "Current": FeedBackData_Package.get_Current(),
            "Error_Controllers": FeedBackData_Package.get_Error_Controllers(),
            "Battery_Voltage": FeedBackData_Package.get_Battery_Voltage(),
            "Data_Latency":  asyncio.run(Latency_Data.get_Data_Latency()),
            "Video_Latency" : asyncio.run(Latency_Data.get_Video_Latency()), 
            "Jitter" : asyncio.run(Latency_Data.get_Jitter()),
            "FPS" : FeedBackData_Package.get_FPS()
        }

        # data = {
        # "Speed_Value": random.randint(1, 100),
        # "RPM": random.randint(1, 100),
        # "GPS": [random.randint(1, 100), random.randint(1, 100)],
        # "Roll_Angle": random.randint(1, 100),
        # "Pitch_Angle": random.randint(1, 100),
        # "Yaw_Angle": random.randint(1, 100),
        # "Yaw_Rate": random.randint(1, 100),
        # "ACC_Z": random.randint(1, 100),
        # "Displacement": random.randint(1, 100),
        # "Temperature_Vehicle": random.randint(1, 100),
        # "Current": random.randint(1, 20),
        # "Error_Controllers": random.randint(1, 100),
        # "Battery_Voltage": random.randint(23, 25),
        # "Data_Latency": random.randint(1, 100),
        # "Video_Latency": random.randint(1, 100),
        # "Jitter": random.randint(1, 100),
        # "FPS": random.randint(1, 100)
        # }


        # print(f"Generated vehicle data: {data}")
        # You could use last_qt_data to inform this data generation
        # if self.last_qt_data:
        #     # Example: reflect back the steering angle with some modification
        #     data["Steering_Angle"] = self.last_qt_data.get("Steering_Angle", 0) * 0.9
            
        return data

# ===================== Video Receiver =====================#
class VideoReceiver:
    def __init__(self):
        self.track = None
    async def handle_track(self, track):
        print("Inside handle track")
        self.track = track
        frame_count = 0

        pygame.init()
        screen = pygame.display.set_mode(FRAME_SIZE)  
        clock = pygame.time.Clock()
        font = pygame.font.Font(None, 36)
        while True:
            try:
                start = time.perf_counter()
                frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                frame_count += 1
                if isinstance(frame, VideoFrame):
                    # print(f"Frame type: VideoFrame, pts: {frame.pts}, time_base: {frame.time_base}")
                    frame = frame.to_ndarray(format="bgr24")
                elif isinstance(frame, np.ndarray):
                    print(f"Frame type: numpy array")
                else:
                    print(f"Unexpected frame type: {type(frame)}")
                    continue
              
                frame = cv2.resize(frame, FRAME_SIZE, interpolation=cv2.INTER_LINEAR)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = np.fliplr(frame)

                frame = np.ascontiguousarray(frame)
                # Frame đã có từ camera/video
                # Frame đã có từ camera/video
                h, w, _ = frame.shape  # height, width
                color = (255, 255, 255)    
                thickness = 4

                # Góc lái - bạn lấy từ dữ liệu vào
                steering_angle = -(await ControlData_Package.get_SteeringAngle())

                # Scale hệ số cong phù hợp (tùy chỉnh cho đẹp)
                curve_factor = steering_angle * 2   # scale nhỏ cho hợp mắt

                # Số điểm để vẽ đường cong
                num_points = 100
                y_vals = np.linspace(h//2, h, num_points)

                # 2 đường gốc (trái: w//3, phải: 2*w//3)
                base_left = w // 3
                base_right = 2 * w // 3

                # Tính x theo cong cho từng đường
                x_left = base_left + curve_factor * ((h - y_vals) / h)**2  # (h - y_vals) để tính từ dưới lên
                x_right = base_right + curve_factor * ((h - y_vals) / h)**2  # (h - y_vals) để tính từ dưới lên

                # Tạo mảng point (x, y)
                points_left = np.array(list(zip(x_left, y_vals)), dtype=np.int32).reshape((-1, 1, 2))
                points_right = np.array(list(zip(x_right, y_vals)), dtype=np.int32).reshape((-1, 1, 2))
                # Vẽ đường cong trái
                frame = cv2.polylines(frame, [points_left], isClosed=False, color=color, thickness=thickness)
                # Vẽ đường cong phải
                frame = cv2.polylines(frame, [points_right], isClosed=False, color=color, thickness=thickness)

                # Vẽ start và end points cho 2 đường cong
                cv2.circle(frame, (base_left, 0), 5, (0, 0, 255), -1)  # Điểm bắt đầu của đường cong trái
                cv2.circle(frame, (base_left, h), 5, (0, 0, 255), -1)  # Điểm kết thúc của đường cong trái
                cv2.circle(frame, (base_right, 0), 5, (0, 0, 255), -1) # Điểm bắt đầu của đường cong phải
                cv2.circle(frame, (base_right, h), 5, (0, 0, 255), -1) # Điểm kết thúc của đường cong phải

                frame = np.rot90(frame) 
             
                frame = pygame.surfarray.make_surface(frame)
                screen.blit(frame, (0, 0))
                FPS_Value = int(clock.get_fps())
                fps_text = font.render(f"FPS: {FPS_Value}", True, (255, 0, 0))
                await FeedBackData_Package.set_data(FPS = FPS_Value)
                screen.blit(fps_text, (10, 10))
                pygame.display.flip()
                clock.tick(FPS) 

                end = time.perf_counter()
                receive_frame_time = (end - start)*1000
                # print(f"[receive_frame_time]: {receive_frame_time:.3f} ms")
                await Latency_Data.set_data(ReceivedVideoTime = receive_frame_time)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                        break
                
            except asyncio.TimeoutError:
                print("Timeout waiting for frame, continuing...")
            except Exception as e:
                print(f"Error in handle_track: {str(e)}")
                if "Connection" in str(e):
                    break
        print("Exiting handle_track")

# ================================================================#

class JSON_ControlData:
    def __init__(self):
        self.data = {
            "Throttle_Value": 0,
            "Steering_Angle": 0,
            "Brake_Value": 0,
            "GearShift_Value": 0,
            "Clutch_Value": 0,
            "Controller_Mode": 0,
            "Setpoint_Speed": 0,
            "Setpoint_Spacing": 0,
            "Timestamp": 0
        }
    async def __str__(self):
        return json.dumps(self.data)
    
    async def generate_data(self):
        self.data = {
            "Throttle_Value": await self.get_Throttle_Value(),
            "Brake_Value": await self.get_Brake_Value(),
            "GearShift_Value": await self.get_GearShift_Value(),
            "Steering_Angle": await self.get_SteeringAngle(),
            "Clutch_Value": await self.get_Clutch_Value(),
            "Controller_Mode": await self.get_Controller_Mode(),
            "Setpoint_Speed": await self.get_Setpoint_Speed(),
            "Timestamp": await self.get_Timestamp_Value(),
        }
        return self.data
    
    def set_data(self, **kwargs):
        for key, value in kwargs.items():
            if key in self.data:  
                self.data[key] = value

    async def get_Throttle_Value(self):
        return self.data["Throttle_Value"]
    async def get_SteeringAngle(self):        
        return self.data["Steering_Angle"]
    async def get_Brake_Value(self):
        return self.data["Brake_Value"]       
    async def get_GearShift_Value(self):
        return self.data["GearShift_Value"]
    async def get_Clutch_Value(self):
        return self.data["Clutch_Value"]
    async def get_Controller_Mode(self):
        return self.data["Controller_Mode"]
    async def get_Setpoint_Speed(self):
        return self.data["Setpoint_Speed"]
    async def get_Setpoint_Spacing(self):
        return self.data["Setpoint_Spacing"]
    async def get_Timestamp_Value(self):
        return self.data["Timestamp"]

ControlData_Package = JSON_ControlData()
# ================================================================#

# ===================== Feedback Data Handler =====================#
class JSON_FeedBackData:
    def __init__(self):
        self.data = {
            "RPM": 0,   
            "Speed_Value": 0,
            "Error_Controllers": 0,
            "Steering_Angle": 0,
            "GPS_x": 0,
            "GPS_y": 0,
            "Roll_Angle": 0,
            "Filter_Roll_Angle": 0,
            "Pitch_Angle": 0,
            "Yaw_Angle": 0,
            "Yaw_Rate": 0,
            "ACC_Z": 0,
            "Displacement": 0,
            "Current": 0,
            "Battery_Voltage": 0,
            "Filter_Raw_Current": 0,
            "Filter_Battery_Voltage": 0,
            "Temperature_Vehicle": 0,
            "FPS": 0,
        }

    async def __str__(self):
        return json.dumps(self.data)
        
    async def set_data(self, **kwargs):
        for key, value in kwargs.items():
            if key in self.data:  
                self.data[key] = value
                
    def get_Speed_Value(self):
       return self.data["Speed_Value"]
    def get_RPM_Value(self):
       return self.data["RPM"]
    def get_SteeringAngle(self):        
       return self.data["Steering_Angle"]
    def get_GPS_x(self):
       return self.data["GPS_x"]       
    def get_GPS_y(self):
       return self.data["GPS_y"]
    def get_Roll_Angle(self):
       return self.data["Roll_Angle"]
    def get_Pitch_Angle(self):
       return self.data["Pitch_Angle"]
    def get_Yaw_Angle(self):
       return self.data["Yaw_Angle"]
    def get_Yaw_Rate(self):
        return self.data["Yaw_Rate"]
    def get_ACC_Z(self):
        return self.data["ACC_Z"]
    def get_Displacement(self):
        return self.data["Displacement"]
    def get_Temp(self):
        return self.data["Temperature_Vehicle"]
    def get_Current(self):
        return self.data["Current"]
    def get_Battery_Voltage(self):
        return self.data["Battery_Voltage"]
    def get_Error_Controllers(self):
        return self.data["Error_Controllers"]
    def get_FPS(self):
        return self.data["FPS"]
    
FeedBackData_Package = JSON_FeedBackData()
# ================================================================#

# ===================== Latency Monitor =====================
class Latency_Time():
    def __init__(self):
        self.data = {
            "Video_Latency": 0, 
            "Data_Latency": 0,
            "Jitter": 0, 
            "SentVideoTime":0, 
            "ReceivedVideoTime":0,
            "CanBusData_Time": 0
        }

        self.lock = asyncio.Lock()

    async def set_data(self, **kwargs):
        async with self.lock:
            for key, value in kwargs.items():
                if key in self.data:  
                    self.data[key] = value

    async def get_SentVideoTime(self):
        async with self.lock:
            return self.data["SentVideoTime"]

    async def get_ReceivedVideoTime(self):
        async with self.lock:
            return self.data["ReceivedVideoTime"]

    async def get_Data_Latency(self):
        async with self.lock:
            self.data["Data_Latency"] =  self.data["Data_Latency"] + self.data["CanBusData_Time"]     
            return self.data["Data_Latency"] 
        
    async def get_Jitter(self):
        async with self.lock:
            return self.data["Jitter"]
    
    async def get_Video_Latency(self):
        async with self.lock:
            self.data["Video_Latency"] = self.data["SentVideoTime"] + self.data["ReceivedVideoTime"] + self.data["Jitter"]
            return self.data["Video_Latency"]
        
Latency_Data = Latency_Time()
# ================================================================#

# ===================== Monitor Latency =====================
async def latency_task():
    async def latency_measurement(pc: RTCPeerConnection):
        while True:
            stats_report = await pc.getStats()
            for stat in stats_report.values():
                if stat.type == "inbound-rtp" and stat.kind == "video":
                    jitter = (stat.jitter/CODEC_CLOCKRATE)*1000
                    await Latency_Data.set_data (Jitter = jitter)
                    data_latency = await Latency_Data.get_Data_Latency()
                    video_latency = await Latency_Data.get_Video_Latency() 
                    sentvideotime = await Latency_Data.get_SentVideoTime()
                    receivevideotime = await Latency_Data.get_ReceivedVideoTime()
                    # print(f"Jitter: {jitter:.2f} ms || SentVideoTime: {sentvideotime:.2f} ms || ReceivedVideoTime: {receivevideotime:.2f} m|| Video_Latency: {video_latency:.2f} ms || Data_latency: {data_latency:.2f} ms")
                    # print(stat.__dict__)
            await asyncio.sleep(FREQUENCY_LATENCY_DATA)
    asyncio.ensure_future(latency_measurement(peer_connection))
# ================================================================#

# ===================== DBus Service Setup =====================

async def run_dbus_service():
    """Start DBus service with GLib main loop"""
    service = create_dbus_service()
    
    data_handler = VehicleDataHandler()

    service.register_data_callback(data_handler.process_qt_data)
    service.register_send_callback(data_handler.generate_vehicle_data)

    loop = GLib.MainLoop()

    def signal_handler(sig, frame):
        logger.info(f"Received signal {sig}, shutting down...")
        loop.quit()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Starting DBus service main loop")
    await asyncio.to_thread(loop.run)

# ===================== WebRTC Setup =====================
async def run_webrtc():
    print("Starting")
    async def send_pings(channel):
        while True:
            dummy_json = await ControlData_Package.generate_data()
            json_string = json.dumps(dummy_json, indent=4)
            channel.send(json_string)
            # print(await ControlData_Package.get_Speed_Value())
            await asyncio.sleep(FREQUENCY_SEND_DATA)

    video_receiver = VideoReceiver()
    @peer_connection.on("track")
    def on_track(track):
        if isinstance(track, MediaStreamTrack):
            print(f"Receiving {track.kind} track")
            asyncio.ensure_future(video_receiver.handle_track(track))

    @peer_connection.on("datachannel")
    def on_datachannel(channel):
        # print(channel, "-", "created by remote party")
        # print(f"Message from {channel.label}: {message}")
        # channel.send("Hello From Answerer via RTC Datachannel")
        if channel.label == "telemetry":
            @channel.on("message")
            async def on_message(message):
                # print(str(datetime.now()),message)
                data = json.loads(message)
                await FeedBackData_Package.set_data(Speed_Value = data["Speed_Value"])
                await FeedBackData_Package.set_data(RPM = data["RPM"])
                await FeedBackData_Package.set_data(GPS_x = data["GPS_x"])
                await FeedBackData_Package.set_data(GPS_y = data["GPS_y"])
                await FeedBackData_Package.set_data(Roll_Angle = data["Roll_Angle"])
                await FeedBackData_Package.set_data(Pitch_Angle = data["Pitch_Angle"])
                await FeedBackData_Package.set_data(Yaw_Angle = data["Yaw_Angle"])
                await FeedBackData_Package.set_data(Yaw_Rate = data["Yaw_Rate"])
                await FeedBackData_Package.set_data(ACC_Z = data["ACC_Z"])
                await FeedBackData_Package.set_data(Displacement = data["Displacement"])
                await FeedBackData_Package.set_data(Current = data["Current"])
                await FeedBackData_Package.set_data(Battery_Voltage = data["Battery_Voltage"])
                await FeedBackData_Package.set_data(Temperature_Vehicle = data["Temperature_Vehicle"])
                await FeedBackData_Package.set_data(Error_Controllers = data["Error_Controllers"])
                dt1 = datetime.strptime(str(datetime.now()), "%Y-%m-%d %H:%M:%S.%f")
                dt2 = datetime.strptime(data["Timestamp"], "%Y-%m-%d %H:%M:%S.%f")
                delta = abs(dt1 - dt2)
                offset_seconds = delta.total_seconds()
                await Latency_Data.set_data(Data_Latency = offset_seconds*1000)
            asyncio.ensure_future(send_pings(channel))
            
        if channel.label == "latency":
            @channel.on("message")
            async def on_message(message):
                # print(str(datetime.now()),message)
                data = json.loads(message)
                await Latency_Data.set_data(SentVideoTime = data["VideoFrame_Time"])
                await Latency_Data.set_data(CanBusData_Time = data["CanBusData_Time"])
        
    resp = requests.get(SIGNALING_SERVER_URL + "/get_offer")
    
    print(resp.status_code)
    if resp.status_code == 200:
        data = resp.json()
        if data["type"] == "offer":
            rd = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
            await peer_connection.setRemoteDescription(rd)
            await peer_connection.setLocalDescription(await peer_connection.createAnswer())
            message = {"id": ID, "sdp": peer_connection.localDescription.sdp, "type": peer_connection.localDescription.type}
            r = requests.post(SIGNALING_SERVER_URL + '/answer', data=message)
            print(message)
            while True:
                # print("Ready for Stuff")
                await asyncio.sleep(1)

async def main():
    # await asyncio.gather(run_dbus_service(), run_webrtc())
    loop = asyncio.get_running_loop()

    # Trigger shutdown signal
    stop_event = asyncio.Event()

    def shutdown(sig):
        print(f"Received signal {sig}, shutting down...")
        stop_event.set() 
        loop.stop()

    loop.add_signal_handler(signal.SIGINT, lambda: shutdown("SIGINT"))
    loop.add_signal_handler(signal.SIGTERM, lambda: shutdown("SIGTERM"))

    # Run two tasks parallelly
    dbus_task = asyncio.create_task(run_dbus_service())
    webrtc_task = asyncio.create_task(run_webrtc())
    calculate_latency_task = asyncio.create_task(latency_task())
    # monitor_task = asyncio.create_task(monitor())

    await stop_event.wait()  # Wait shutdown signal 
    dbus_task.cancel()       # Cancel task DBus
    webrtc_task.cancel()     # Cancel task WebRTC
    calculate_latency_task.cancel()     # Cancel task WebRTC

if __name__ == "__main__":
    asyncio.run(main())


