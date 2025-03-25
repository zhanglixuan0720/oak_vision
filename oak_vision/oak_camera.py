import depthai as dai
import time
from datetime import timedelta
import cv2
import numpy as np
from dataclasses import dataclass

@dataclass
class OAKFrame:
    frame: np.ndarray # BGR image
    stamp: float # timestamp in seconds
    intrinsics: np.ndarray = None # camera intrinsics

@dataclass
class OAKVIOFrame:
    position: np.ndarray # x, y, z
    orientation: np.ndarray # quaternion(x, y, z, w)
    stamp: float # timestamp in seconds
    


class OAKCamera:
    '''
    OAKCamera class for reading frames from OAK-D-S2 camera
    '''
    boot2utc_time_offset: float = None
    device2boot_time_offset: float = None
    
    def __init__(self, width=640, height=480, fps=30, mask_vio=False):
        self.check_params(width, height, fps)
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = dai.Pipeline()
        if not mask_vio:
            self.set_visual_odometry()
        self.set_color_camera()
        self.set_depth_camera()
        self.set_intrinsics()
        
        self.device = dai.Device(self.pipeline)
        self.device.setTimesync(timedelta(seconds=5), 10, True)
        
        self.start_color_stream()
        self.start_depth_stream()
        if not mask_vio:
            self.start_vio_stream()
        
        self.boot2utc_time_offset = self.sync_boot2utc_time()
        
        
    def check_params(self, width:int, height:int, fps:int):
        if width == 0 or height == 0:
            raise ValueError("Width and height must be greater than 0")
        if fps > 30:
            raise ValueError("Max fps is 30")
    
    def set_visual_odometry(self):
        import spectacularAI
        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline)
    
    def set_color_camera(self):
        color_camera = self.pipeline.create(dai.node.ColorCamera)
        color_xout = self.pipeline.create(dai.node.XLinkOut)
        color_xout.setStreamName("color")
        
        color_camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_camera.setFps(self.fps)
        color_camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)  
        color_camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        with dai.Device() as tmp_dev:
            calib = tmp_dev.readCalibration2()
            lens_pos = calib.getLensPosition(dai.CameraBoardSocket.CAM_A)
            if lens_pos:
                color_camera.initialControl.setManualFocus(lens_pos)
        color_camera.setPreviewSize((self.width, self.height))
        color_camera.preview.link(color_xout.input)
        self.color_camera = color_camera
        self.color_xout = color_xout    
        
    def set_depth_camera(self):
        depth_xout = self.pipeline.create(dai.node.XLinkOut)
        depth_xout.setStreamName("depth")
            
        depth_maip = self.pipeline.create(dai.node.ImageManip)
        depth_maip.initialConfig.setCenterCrop(1.0, self.width/self.height)  
        depth_maip.initialConfig.setResize(self.width, self.height)
        depth_maip.out.link(depth_xout.input)
        
        if hasattr(self, 'vio_pipeline'):
            depth2color_align = self.pipeline.create(dai.node.ImageAlign)
            self.vio_pipeline.stereo.depth.link(depth2color_align.input)
            self.color_camera.video.link(depth2color_align.inputAlignTo)
            depth2color_align.outputAligned.link(depth_maip.inputImage)
            self.depth2color_align = depth2color_align
        else:
            left_mono = self.pipeline.create(dai.node.MonoCamera)
            right_mono = self.pipeline.create(dai.node.MonoCamera)
            stereo = self.pipeline.create(dai.node.StereoDepth)
            
            left_mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            left_mono.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            left_mono.setCamera("left")
            left_mono.setFps(self.fps)

            right_mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            right_mono.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            right_mono.setCamera("right")
            right_mono.setFps(self.fps)
            
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            stereo.setLeftRightCheck(True)  
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  
            
            left_mono.out.link(stereo.left)
            right_mono.out.link(stereo.right)
            stereo.depth.link(depth_maip.inputImage)
            
            self.left_mono = left_mono
            self.right_mono = right_mono
            self.stereo = stereo
        
        self.depth_xout = depth_xout
        self.depth_maip = depth_maip
            
    def set_intrinsics(self):
        with dai.Device() as device:
            calib = device.readCalibration()
            self.intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, (self.width, self.height))
        
    def start_color_stream(self):
        self.color_queue = self.device.getOutputQueue(name="color", maxSize=4, blocking=False)
        
    def start_depth_stream(self):
        self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        
    def start_vio_stream(self):
        self.vio_session = self.vio_pipeline.startSession(self.device)
        
    def read_color_frame(self):
        in_frame = self.color_queue.tryGet()
        if in_frame is not None:
            self.device2boot_time_offset = self.sync_device2boot_time(in_frame.getTimestampDevice(), in_frame.getTimestamp())
            stamp = in_frame.getTimestamp().total_seconds() + self.boot2utc_time_offset
            return OAKFrame(in_frame.getCvFrame(), stamp, self.intrinsics)
    
    def read_depth_frame(self):
        in_frame = self.depth_queue.tryGet()
        if in_frame is not None:
            self.device2boot_time_offset = self.sync_device2boot_time(in_frame.getTimestampDevice(), in_frame.getTimestamp())
            stamp = in_frame.getTimestamp().total_seconds() + self.boot2utc_time_offset
            frame = np.clip(in_frame.getFrame(), 400, 15000)
            return OAKFrame(frame, stamp, self.intrinsics)
        
    def read_vio_frame(self):
        if self.vio_session.hasOutput() and self.device2boot_time_offset is not None:
            vio_out = self.vio_session.getOutput()
            color_vio_out = self.vio_session.getRgbCameraPose(vio_out)
            stamp = color_vio_out.pose.time + self.device2boot_time_offset + self.boot2utc_time_offset
            position = np.array([color_vio_out.pose.position.x, color_vio_out.pose.position.y, color_vio_out.pose.position.z])
            orientation = np.array([color_vio_out.pose.orientation.x, color_vio_out.pose.orientation.y, color_vio_out.pose.orientation.z, color_vio_out.pose.orientation.w])
            return OAKVIOFrame(position, orientation, stamp)
        
    @staticmethod
    def sync_boot2utc_time():
        return time.time() - dai.Clock.now().total_seconds() # usage: utc_time = boot_time + boot2utc_time_offset
    
    @staticmethod
    def sync_device2boot_time(device_time, boot_time):
        return (boot_time - device_time).total_seconds() # usage: boot_time = device_time + device2boot_time_offset  