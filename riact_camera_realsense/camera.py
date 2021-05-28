import sys
import os
import time

import numpy as np
from numbers import Number
import cv2
import json
from pyudev import Context
from enum import Enum, IntEnum

from .window import Window
from .parameter import Parameter, RangeParameter, BoolParameter, OptionParameter

class DataTypes(IntEnum):
    CV_RAW      = -1
    CV_16S      = cv2.CV_16S
    CV_16SC1    = cv2.CV_16SC1
    CV_16SC2    = cv2.CV_16SC2
    CV_16SC3    = cv2.CV_16SC3
    CV_16SC4    = cv2.CV_16SC4
    CV_16U      = cv2.CV_16U
    CV_16UC1    = cv2.CV_16UC1
    CV_16UC2    = cv2.CV_16UC2
    CV_16UC3    = cv2.CV_16UC3
    CV_16UC4    = cv2.CV_16UC4
    CV_32F      = cv2.CV_32F
    CV_32FC1    = cv2.CV_32FC1
    CV_32FC2    = cv2.CV_32FC2
    CV_32FC3    = cv2.CV_32FC3
    CV_32FC4    = cv2.CV_32FC4
    CV_32S      = cv2.CV_32S
    CV_32SC1    = cv2.CV_32SC1
    CV_32SC2    = cv2.CV_32SC2
    CV_32SC3    = cv2.CV_32SC3
    CV_32SC4    = cv2.CV_32SC4
    CV_64F      = cv2.CV_64F
    CV_64FC1    = cv2.CV_64FC1
    CV_64FC2    = cv2.CV_64FC2
    CV_64FC3    = cv2.CV_64FC3
    CV_64FC4    = cv2.CV_64FC4
    CV_8S       = cv2.CV_8S
    CV_8SC1     = cv2.CV_8SC1
    CV_8SC2     = cv2.CV_8SC2
    CV_8SC3     = cv2.CV_8SC3
    CV_8SC4     = cv2.CV_8SC4
    CV_8U       = cv2.CV_8U
    CV_8UC1     = cv2.CV_8UC1
    CV_8UC2     = cv2.CV_8UC2
    CV_8UC3     = cv2.CV_8UC3
    CV_8UC4     = cv2.CV_8UC4


class CameraProperty(Enum):
    FOURCC                  = cv2.CAP_PROP_FOURCC,                  lambda v: int(v) if v<0 else int(v).to_bytes(4, sys.byteorder)
    CODEC_PIXEL_FORMAT      = cv2.CAP_PROP_CODEC_PIXEL_FORMAT,      lambda v: int(v) if v<0 else int(v).to_bytes(4, sys.byteorder)
    DATATYPE                = cv2.CAP_PROP_FORMAT,                  DataTypes
    # VIDEO_MODE            = cv2.CAP_PROP_MODE,                    VideoMode
    BITRATE                 = cv2.CAP_PROP_BITRATE,                 int
    CHANNEL                 = cv2.CAP_PROP_CHANNEL,                 int

    FRAME_COUNT             = cv2.CAP_PROP_FRAME_COUNT,             int
    POS_MSEC                = cv2.CAP_PROP_POS_MSEC,                float
    POS_FRAMES              = cv2.CAP_PROP_POS_FRAMES,              int
    POS_AVI_RATIO           = cv2.CAP_PROP_POS_AVI_RATIO,           float

    FRAME_WIDTH             = cv2.CAP_PROP_FRAME_WIDTH,             int
    FRAME_HEIGHT            = cv2.CAP_PROP_FRAME_HEIGHT,            int
    FPS                     = cv2.CAP_PROP_FPS,                     int

    CONVERT_RGB             = cv2.CAP_PROP_CONVERT_RGB,             bool
    MONOCHROME              = cv2.CAP_PROP_MONOCHROME,              bool
    RECTIFICATION           = cv2.CAP_PROP_RECTIFICATION,           bool

    BRIGHTNESS              = cv2.CAP_PROP_BRIGHTNESS,              int
    CONTRAST                = cv2.CAP_PROP_CONTRAST,                int
    SATURATION              = cv2.CAP_PROP_SATURATION,              int
    HUE                     = cv2.CAP_PROP_HUE,                     int
    GAIN                    = cv2.CAP_PROP_GAIN,                    int
    SHARPNESS               = cv2.CAP_PROP_SHARPNESS,               int
    GAMMA                   = cv2.CAP_PROP_GAMMA,                   int
    TEMPERATURE             = cv2.CAP_PROP_TEMPERATURE,             int
    AUTO_EXPOSURE           = cv2.CAP_PROP_AUTO_EXPOSURE,           int
    EXPOSURE                = cv2.CAP_PROP_EXPOSURE,                int
    AUTO_WB                 = cv2.CAP_PROP_AUTO_WB,                 int
    WB_TEMPERATURE          = cv2.CAP_PROP_WB_TEMPERATURE,          int
    WHITE_BALANCE_RED_V     = cv2.CAP_PROP_WHITE_BALANCE_RED_V,     int
    WHITE_BALANCE_BLUE_U    = cv2.CAP_PROP_WHITE_BALANCE_BLUE_U,    int
    ZOOM                    = cv2.CAP_PROP_ZOOM,                    int
    AUTOFOCUS               = cv2.CAP_PROP_AUTOFOCUS,               bool
    FOCUS                   = cv2.CAP_PROP_FOCUS,                   int

    TRIGGER                 = cv2.CAP_PROP_TRIGGER,                 int
    TRIGGER_DELAY           = cv2.CAP_PROP_TRIGGER_DELAY,           int

    GUID                    = cv2.CAP_PROP_GUID,                    int
    ISO_SPEED               = cv2.CAP_PROP_ISO_SPEED,               int
    BACKLIGHT               = cv2.CAP_PROP_BACKLIGHT,               bool
    PAN                     = cv2.CAP_PROP_PAN,                     int
    TILT                    = cv2.CAP_PROP_TILT,                    int
    ROLL                    = cv2.CAP_PROP_ROLL,                    int
    IRIS                    = cv2.CAP_PROP_IRIS,                    int
    ORIENTATION_AUTO        = cv2.CAP_PROP_ORIENTATION_AUTO,        bool
    ORIENTATION_META        = cv2.CAP_PROP_ORIENTATION_META,        int

    BUFFERSIZE              = cv2.CAP_PROP_BUFFERSIZE,              int
    SAR_NUM                 = cv2.CAP_PROP_SAR_NUM,                 int
    SAR_DEN                 = cv2.CAP_PROP_SAR_DEN,                 int

    # BACKEND                 = cv2.CAP_PROP_BACKEND,                 int
    # SETTINGS                = cv2.CAP_PROP_SETTINGS,                int


class CameraResolution(Enum):
    QQVGA  =  160,  120
    HQVGA  =  240,  160
    QVGA   =  320,  240
    WQVGA1 =  384,  240
    WQVGA2 =  360,  240
    WQVGA3 =  400,  240
    HVGA   =  480,  320
    nHD    =  640,  360
    VGA    =  640,  480
    WVGA1  =  768,  480
    WVGA2  =  720,  480
    WVGA3  =  800,  480
    SVGA   =  800,  600
    WSVGA1 = 1024,  576
    WSVGA2 = 1024,  600
    DVGA   =  960,  640
    XGA    = 1024,  768
    WXGA1  = 1280,  720
    WXGA2  = 1280,  800
    WXGAP  = 1440,  900
    SXGA   = 1280, 1024
    WSXGAP = 1680, 1050
    WUXGA  = 1920, 1200
    QWXGA  = 2048, 1152
    HD     = 1360,  768
    HDP    = 1600,  900
    FHD    = 1920, 1080
    QHD    = 2560, 1440
    UHD    = 3840, 2160

    @classmethod
    def get(cls, resolution):
        try:
            resolution = CameraResolution[resolution]
        except Exception as e:
            pass
        else:
            return resolution

        try:
            resolution = CameraResolution(resolution)
        except Exception as e:
            pass
        else:
            return resolution

        return None



class PropertySupport(IntEnum):
    NONE       = 0
    ALL        = 0
    FALSE      = 0
    READ       = 1
    TRUE       = 1
    READ_WRITE = 3


class CameraDevice:

    @classmethod
    def list(cls):
        context = Context()
        return [CameraDevice(device) for device in context.list_devices() if device.sys_name.startswith('video')]

    @classmethod
    def get(cls, number):
        try:
            cam = CameraDevice.list()[number]
        except Exception:
            raise Exception("Camera device '{}' not available".format(number))
        return cam

    def __init__(self, device):
        self._device = device

    @property
    def name(self):
        return self._device.sys_name

    @property
    def node(self):
        return self._device.device_node

    @property
    def number(self):
        return int(self._device.sys_number)

    @property
    def serial(self):
        return self._device.properties.get('ID_SERIAL')

    @property
    def vendor(self):
        return self._device.properties.get('ID_VENDOR_ENC').encode().decode('unicode-escape')

    @property
    def model(self):
        return self._device.properties.get('ID_MODEL_ENC').encode().decode('unicode-escape')

    @property
    def model_id(self):
        return "{}:{}".format(self._device.properties.get('ID_VENDOR_ID'), self._device.properties.get('ID_MODEL_ID'))

    @property
    def properties(self):
        return {p:v for p,v in self._device.properties.items()}

    def info(self):
        print("{} {} @ '{}' (Model {}) [{:>13}]".format(self.vendor, self.model, self.node, self.model_id, self.serial))

    def details(self):
        print("Device details")
        print("==============")
        for p,v in self.properties.items():
            print("  {:<20} = {}".format(p,v))


class Camera:
    def __init__(self, device = -1):
        if isinstance(device, CameraDevice):
            self._device = device
        else:
            self._device = CameraDevice.get(device)
        self.open()
        self._get_supported_properties()
        self._get_supported_resolutions()


    def open(self):
        self.close()
        self._capture = cv2.VideoCapture(self.number)
        if not self.is_open():
            raise Exception("Camera capture not available")

    def close(self):
        if self.is_open():
            self._capture.release()
            self._capture = None

    def is_open(self):
        return hasattr(self, "_capture") and self._capture is not None and self._capture.isOpened()

    def get_frame(self):
        if not self.is_open(): return None
        [success, frame] = self._capture.read()
        return frame if success else None

    @property
    def name(self):
        return self._device.name

    @property
    def number(self):
        return self._device.number

    @property
    def serial(self):
        return self._device.serial

    @property
    def model(self):
        return self._device.model

    @property
    def resolution(self):
        return self.get_resolution().value

    @property
    def width(self):
        return self.get_resolution().value[0]

    @property
    def height(self):
        return self.get_resolution().value[1]




    def set_property(self, property, value):
        if isinstance(value, Enum):
            value = value.value
        elif isinstance(value, bytes):
            value = int.from_bytes(value, sys.byteorder)
        elif not isinstance(value, float):
            value = int(value)
        # print("Setting property {} = {}".format(property.name, value))
        self._capture.set(property.value[0], value)

    def get_property(self, property):
        value = self._capture.get(property.value[0])
        # print("Getting property {} = {}".format(property.name, value))
        return property.value[1](value)

    def _get_supported_properties(self):
        properties = {}
        for prop in CameraProperty:
            properties[prop] = PropertySupport.NONE
            p = self._capture.get(prop.value[0])
            if isinstance(p, Number) and p < 0:
                continue
            properties[prop] = PropertySupport.READ
            try:
                r = self._capture.set(prop.value[0], p)
            except Exception as e:
                continue
            properties[prop] = PropertySupport.READ_WRITE
        return properties

    def get_supported_properties(self, access=PropertySupport.READ, cached=True):
        if cached and hasattr(self, '_supported_properties'):
            return {p:s for p,s in self._supported_properties.items() if s >= access}
        else:
            self._supported_properties = self._get_supported_properties()
            return self.get_supported_properties(access)


    def set_resolution(self, resolution):
        resolution = CameraResolution.get(resolution)
        if not resolution:
            print("Camera does not support resolution [{}]".format(resolution), file=sys.stderr)

        if resolution not in self.get_supported_resolutions():
            print("Camera does not support {} [{} x {}] resolution".format(resolution.name, *resolution.value), file=sys.stderr)

        self.set_property(CameraProperty.FRAME_WIDTH, resolution.value[0])
        self.set_property(CameraProperty.FRAME_HEIGHT, resolution.value[1])

    def get_resolution(self):
        width = self.get_property(CameraProperty.FRAME_WIDTH)
        height = self.get_property(CameraProperty.FRAME_HEIGHT)
        return CameraResolution((width,height))

    def _get_supported_resolutions(self):
        resolutions = {}
        old_width = self.get_property(CameraProperty.FRAME_WIDTH)
        old_height = self.get_property(CameraProperty.FRAME_HEIGHT)
        for res in CameraResolution:
            self.set_property(CameraProperty.FRAME_WIDTH, res.value[0])
            self.set_property(CameraProperty.FRAME_HEIGHT, res.value[1])
            width = self.get_property(CameraProperty.FRAME_WIDTH)
            height = self.get_property(CameraProperty.FRAME_HEIGHT)
            resolutions[res] = PropertySupport.TRUE if res.value == (width, height) else PropertySupport.FALSE
        self.set_property(CameraProperty.FRAME_WIDTH, old_width)
        self.set_property(CameraProperty.FRAME_HEIGHT, old_height)
        return resolutions

    def get_supported_resolutions(self, support=PropertySupport.TRUE, cached=True):
        if cached and hasattr(self, '_supported_resolutions'):
            return {p:s for p,s in self._supported_resolutions.items() if s == support}
        else:
            self._supported_resolutions = self._get_supported_resolutions()
            return self.get_supported_resolutions(support)

    def get_config(self, support=PropertySupport.READ):
        properties = self.get_supported_properties(support)
        return {p:self.get_property(p) for p in properties}

    def set_config(self, config):
        properties = self.get_supported_properties(PropertySupport.READ_WRITE)
        conf = self.get_config()
        for p in properties:
            if isinstance(conf[p], bytes):
                config[p.name] = config[p.name].encode()
            self.set_property(p, config[p.name])

    def save_config(self):
        filename = os.path.dirname(os.path.abspath(__file__)) + '/config.txt'
        try:
            with open(filename, 'r') as file:
                config = json.load(file)
        except Exception:
            config = {}

        properties = {}
        for k,v in self.get_config().items():
            if isinstance(v, bytes):
                v = v.decode()
            properties[k.name] = v

        config.update({self.serial:properties})
        with open(filename, 'w') as file:
            json.dump(config, file, indent=4)

    def load_config(self):
        filename = os.path.dirname(os.path.abspath(__file__)) + '/config.txt'
        with open(filename, 'r') as file:
            config = json.load(file)
        if self.serial in config:
            self.set_config(config[self.serial])
        else:
            print("No configuration found")

    def print_device(self):
        print("Device")
        print("========")
        self._device.info()
        self._device.details()

    def print_info(self):
        print("Resolutions")
        print("===========")
        for p,a in self.get_supported_resolutions().items():
            print("  {:<20} = {}".format(p.name,p.value))

        print("Properties")
        print("==========")
        for p,a in self.get_supported_properties().items():
            print("  {:<20} = {}".format(p.name,a.name))

    def print_properties(self):
        print("Configuration")
        print("=============")
        for p,v in self.get_config().items():
            print("  {:<20} = {}".format(p.name,v))



class CameraWindow(Window):
    def __init__(self, name, cam):
        super().__init__(name)
        self._cam = cam

        self._params = [
            self.create_control_parameter(CameraProperty.AUTO_WB,           0,    1),
            self.create_control_parameter(CameraProperty.WB_TEMPERATURE, 2800, 6500),
            # self.create_control_parameter(CameraProperty.TEMPERATURE,    2800, 6500),
            self.create_control_parameter(CameraProperty.BRIGHTNESS,      -64,   64),
            self.create_control_parameter(CameraProperty.CONTRAST,          0,   50),
            self.create_control_parameter(CameraProperty.SATURATION,        0,  100),
            self.create_control_parameter(CameraProperty.HUE,            -100,  100),
            self.create_control_parameter(CameraProperty.SHARPNESS,         0,   10),
            self.create_control_parameter(CameraProperty.GAMMA,           100,  300),
            self.create_control_parameter(CameraProperty.AUTO_EXPOSURE,     1,    3),
            self.create_control_parameter(CameraProperty.EXPOSURE,          0, 1000),
        ]

    def create_control_parameter(self, prop, min_value, max_value, scale=1):
        param = RangeParameter(prop.name, self._cam.get_property(prop), min_value, max_value)
        return self.add_control_parameter(param, scale, self._on_param_changed)

    def _on_param_changed(self, tb, param, old_value, new_value):
        prop = CameraProperty[param.name]
        self._cam.set_property(prop, param.value)

    def update_params(self):
        for p in self._params:
            p.value = self._cam.get_property(CameraProperty[p.name])

    def print_params(self):
        for prop in CameraProperty:
            val = self._cam.get_property(prop)
            if not isinstance(val, int) or val >= 0:
                print(prop.name, val)
        print("")

    def _on_key(self, key):

        if key == ord('s'):
            print("Savie camera config")
            self._cam.save_config()

        elif key == ord('l'):
            print("Load camera config")
            self._cam.load_config()
            self.update_params()

        elif key == ord('u'):
            self.update_params()

        elif key == ord('p'):
            self.print_params()

class CameraApp:

    def __init__(self, device=0, resolution=CameraResolution.VGA):

        self.cam = Camera(device)
        self.cam.set_resolution(resolution)
        self.cam.set_property(CameraProperty.FPS, 30)
        self.FPS = self.cam.get_property(CameraProperty.FPS)
        self.win = CameraWindow(self.cam.model, self.cam)

    def run(self):
        while True:
            start = time.time()

            frame = self.cam.get_frame()
            self.win.show(frame)

            if not Window.spin():
                break

            duration = time.time() - start
            time.sleep(max(1./self.FPS - duration, 0))


if __name__ == '__main__':

    devices = CameraDevice.list()
    for dev in devices:
        cam = Camera(dev)
        cam.print_device()
        cam.print_info()
        cam.print_properties()
        cam.close()
        print("\n")

    CameraApp(0).run()
