from .window import Window
from .camera import CameraDevice, Camera
import cv2
import numpy as np
import pyrealsense2 as rs
# import pptk
import os
import time
from datetime import datetime
import json
import argparse
# from pyntcloud import PyntCloud
import open3d as o3d


DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C", "0B64"]

def show_img(window_name, img):
    # while True:
    cv2.imshow(window_name, img) #  cv2.resize(original, (900, 450))
    key_press = cv2.waitKey(1) & 0xFF
    # if key_press == 27:
    #     break

def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = o3d.camera.PinholeCameraIntrinsic(640, 480,
            intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    return out

def time_it(func):
    start = time.time()
    func
    elapsed = time.time() - start
    print("Elapsed saving: {}".format(elapsed))



def find_device_json_input_interface() :
    ctx = rs.context()
    ds5_dev = rs.device()
    devices = ctx.query_devices();
    for dev in devices:
        if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            if dev.supports(rs.camera_info.name):
                print("Found device", dev.get_info(rs.camera_info.name))
            return dev
    raise Exception("No product line device that has json input interface")


def read_count(cwd):
    count_file = os.path.join(cwd, "count.txt")
    if not os.path.exists(count_file):
        return 1
    elif os.stat(count_file).st_size == 0:
        return 1
    else:
        with open(count_file) as f:
            first_line = f.readline()
            print(first_line)
    
        return int(first_line)

def write_count(cwd, count):
    count_file = os.path.join(cwd, "count.txt")
    with open(count_file, 'w+') as f:
         f.write(count)

class RealSenseCamera(Camera):
    
    def __init__(self, device = -1, custom_config_path=None, interval=2, resolution_depth=None, resolution_color=None, clip_dist=5):
        # super().__init__(device)
        
        # self.config.enable_stream(rs.stream.color)
        self.cwd = os.path.dirname(os.path.realpath(__file__))
        self.clip_dist = clip_dist
        self.resolution_depth = resolution_depth
        self.resolution_color = resolution_color
        self.interval = interval
        self.open(custom_config_path)
    
    # Override
    def open(self, custom_config_path):
        ####  Loading JSON Config ####
        if custom_config_path:
            jsonDict = json.load(open(custom_config_path))
            jsonString= str(jsonDict).replace("'", '\"')
            print(jsonString)
            try:
                dev = find_device_json_input_interface()
                print(type(dev))
                ser_dev = rs.serializable_device(dev)  
                # ser_dev.load_json(jsonString)
                # print("loaded json")
                with open(custom_config_path, 'r') as file:
                    f = file.read().strip()       
                    ser_dev.load_json(f)
                    print("loaded json") 

            except Exception as e:
                print(e)
                pass
        ####  Loading JSON Config END####

        self.close()
        self._capture = rs.pipeline()
        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        
        config = rs.config()

        if self.resolution_depth:
            config.enable_stream(rs.stream.depth, self.resolution_depth[0], self.resolution_depth[1])
            config.enable_stream(rs.stream.infrared, self.resolution_depth[0], self.resolution_depth[1])
        else:
            config.enable_stream(rs.stream.depth)
            config.enable_stream(rs.stream.infrared)
            
        if self.resolution_color:
            config.enable_stream(rs.stream.color, self.resolution_color[0], self.resolution_color[1]) # , 320, 240, rs.format.bgr8, 30)         
        else:
            config.enable_stream(rs.stream.color)
            
        
        try:
            self._profile = self._capture.start(config)
        except Exception as e:
            raise Exception("Camera capture not available")

        depth_sensor = self._profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        # self.clipping_distance_in_meters = self.clip_dist # 3 meter
        # self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale
        
    # Override
    def close(self):
        if self.is_open():
            self._capture.stop()
            self._capture = None
            print("Stopped.")
    # Override
    def is_open(self):
        return hasattr(self, "_profile") and self._profile is not None
    # Override
    def get_frame(self):
        if not self.is_open(): return None
        frames = self._capture.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

        return color_image, depth_image, verts
    
    def stream(self):
        while True:
            color_image, depth_image = self.get_frame()
            depth_image_jet = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_image_jet))
            cv2.imshow("image", images)
            key_press = cv2.waitKey(1) & 0xFF
            if key_press == 27:
                break
        self.close()


    def vis(self):
        # Create a pipeline
        pipeline = self._capture

        # Start streaming
        profile = self._profile
        depth_sensor = profile.get_device().first_depth_sensor()

        # Using preset HighAccuracy for recording
        # depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_scale = depth_sensor.get_depth_scale()

        # We will not display the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 3 # 3 meter
        clipping_distance = clipping_distance_in_meters / depth_scale
        # print(depth_scale)
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=self.resolution_depth[0], height=self.resolution_depth[1])

        pcd = o3d.geometry.PointCloud()
        flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

        # Streaming loop
        frame_count = 0
        try:
            while True:

                dt0=datetime.now()

                # Get frameset of color and depth
                frames = pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                intrinsic = o3d.camera.PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = o3d.geometry.Image(np.array(aligned_depth_frame.get_data()))
                color_temp = np.asarray(color_frame.get_data())
                color_image = o3d.geometry.Image(color_temp)

                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        color_image, depth_image, depth_scale=1.0/depth_scale,
                        depth_trunc=self.clipping_distance_in_meters,
                        convert_rgb_to_intensity = False)
                temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                        rgbd_image, intrinsic)
                temp.transform(flip_transform)
                pcd.points = temp.points
                pcd.colors = temp.colors

                if frame_count == 0:
                    vis.add_geometry(pcd)

                vis.update_geometry(pcd)
                vis.poll_events()
                vis.update_renderer()

                process_time = datetime.now() - dt0
                print("FPS: "+str(1/process_time.total_seconds()))
                frame_count += 1

        finally:
            pipeline.stop()
        vis.destroy_window()
            
    def run(self):
        if not os.path.exists(os.path.join(self.cwd, "data")):
            os.makedirs("data")
        count = read_count(self.cwd)
        auto_flag = False
        align_to = rs.stream.color
        align = rs.align(align_to)
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        pcd = o3d.geometry.PointCloud()
        flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        frame_count = 0
        while True:
            count_str = "{0:0=5d}".format(count)
            dt0=datetime.now()
            frames = self._capture.wait_for_frames(timeout_ms=10000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            infra_frame = frames.get_infrared_frame()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            intrinsic = o3d.camera.PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))
            
            # print(color_frame)
            if not depth_frame or not color_frame or not infra_frame or not aligned_depth_frame:
                raise Exception(("Could not get data")) # or continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            infra_image = np.asanyarray(infra_frame.get_data())
            # print("Depth resolution: {}x{}".format(depth_image.shape[1], depth_image.shape[0]))
            # print("Color resolution: {}x{}".format(color_image.shape[1], color_image.shape[0]))
            # print("Infra resolution: {}x{}".format(infra_image.shape[1], infra_image.shape[0]))
            # if color_image.shape[1]!= 640 or depth_image.shape[1] != 640:
            #     raise Exception("L515 Camera must be connected through USB 3.0 in order to acquire 640x480 resolution for both color and depth frames")

            ######### Pointcloud vis ##########
            depth_image_o3d = o3d.geometry.Image(np.array(aligned_depth_frame.get_data()))
            color_temp = np.asarray(color_frame.get_data())
            color_image_o3d = o3d.geometry.Image(color_temp)

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_image_o3d, depth_image_o3d, depth_scale=1.0/self.depth_scale,
                    # depth_trunc=self.clipping_distance_in_meters,
                    convert_rgb_to_intensity = False)
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image, intrinsic)
            temp.transform(flip_transform)
            pcd.points = temp.points
            pcd.colors = temp.colors

            if frame_count == 0:
                vis.add_geometry(pcd)

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            # process_time = datetime.now() - dt0
            # print("FPS: "+str(1/process_time.total_seconds()))
            # frame_count += 1
            ######### Pointcloud vis END ##########

            # Create save_to_ply object
            path = os.path.join("./data/", "ply_" + count_str + ".ply")
            ply = rs.save_to_ply(path)
            
            # Set options to the desired values
            # In this example we'll generate a textual PLY with normals (mesh is already created by default)
            ply.set_option(rs.save_to_ply.option_ply_mesh, False)
            ply.set_option(rs.save_to_ply.option_ply_binary, True)
            ply.set_option(rs.save_to_ply.option_ply_normals, False)
            # ply.set_option(rs.save_to_ply.option_ignore_color, False)

            # Viuslization
            depth_image_jet = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # print("depth_image_jet ", depth_image_jet.shape)
            # print("color", color_image.shape)
            # print("color resize", cv2.resize(color_image, depth_image_jet.shape[:2][::-1]).shape)
            
            images = np.hstack((cv2.resize(color_image, depth_image_jet.shape[:2][::-1], interpolation=cv2.INTER_CUBIC), depth_image_jet))            
            cv2.imshow("image", images)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            # Save
            elif k == 32 or auto_flag:
                if k == 97:
                    auto_flag = False
                    print("-"*50)
                    print("Auto mode Stopped")
                    print("-"*50)
                    continue
                if auto_flag:
                    if time.time() - start < self.interval:
                        continue
                    else:
                        start = time.time()
                        print("Auto saving...")
                print("Saving to " + count_str)
                
                ply.process(depth_frame)
                cv2.imwrite(os.path.join("./data/", "rgb_"  + count_str + ".png"), cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                cv2.imwrite(os.path.join("./data/", "depth_" + count_str + ".png"), depth_image)
                cv2.imwrite(os.path.join("./data/", "infra_" + count_str + ".png"), infra_image)
                print("Done")
                count += 1
                write_count(self.cwd, str(count))
            elif k == 97:
                print("-"*50)
                print("Auto mode started")
                print("-"*50)
                auto_flag = True
                start = time.time()
            elif k == 81:
                pass
            elif k == ord('l'):
                pass
                print("Line mode")

            process_time = datetime.now() - dt0
            print("Process Time: {}".format(process_time))
            # print("FPS: "+str(1/process_time.total_seconds()))
            frame_count += 1



if __name__ == "__main__":

    print("L515 Camera must be connected through USB 3.0 in order to acquire 640x480 resolution for both color and depth frames")
    # Parse input
    parser = argparse.ArgumentParser()
    parser.add_argument("--json_cam_config", help="Path to camera json config file. Default is settings on realsense-viewer")
    parser.add_argument("--interval", type=float, default=2, help="Automatic capture every '{'INTERVAL'}' seconds. Default is 2 seconds")
    parser.add_argument("--resolution_depth", nargs='+', type=int, default=None, help="Resolution of the depth image WxH. Input format is W, H")
    parser.add_argument("--resolution_color", nargs='+', type=int, default=None, help="Resolution of the color image WxH. Input format is W, H")
    
    parser.add_argument("--clipping_distance", type=float, default=None, help="Clipping distance in meters")
    args = parser.parse_args()
    json_cam_config = args.json_cam_config
    interval = args.interval
    resolution_depth = args.resolution_depth
    resolution_color = args.resolution_color
    clipping_distance = args.clipping_distance

    if json_cam_config:
        if not os.path.exists(json_cam_config):
            raise Exception(("JSON path is invalid"))

    cam_id = 0 # 6 is far-most camera, 4 is the one right next to it

    if resolution_depth:
        if len(resolution_depth) != 2:
            raise Exception("Depth resolution format incorrect. Only two int numbers and must be comma separated.")
    if resolution_color:
        if len(resolution_color) != 2:
            raise Exception("Depth resolution format incorrect. Only two int numbers and must be comma separated.")

    cam = RealSenseCamera(cam_id, json_cam_config, 
                          clip_dist=clipping_distance, 
                          interval=interval, 
                          resolution_depth=resolution_depth, 
                          resolution_color=resolution_color)
    # cam.run()
    # cam = RealSenseCamera()
    # cam.vis()
    cam.run()
    