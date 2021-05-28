## Instructions
1. Install [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) latest release (2.45 at the time)
2. Setup a virtualenv
3. `pip install opencv-python pyudev pyrealsense2`
4. Execute the `realsense.py` script. (see Usage section)

## Usage
```
usage: realsense.py [-h] [--json_cam_config JSON_CAM_CONFIG] [--interval INTERVAL] [--depth_res_width DEPTH_RES_WIDTH]
                    [--depth_res_height DEPTH_RES_HEIGHT]
```

```
Keyboard: 
    [space]     Space capture rgb, d, infra, and pointcloud to disk
    [a]         Toggle command. Auto capture every INTERVAL seconds
```

## TODO:
1. Implement a pointcloud streamer along with the RGB-D live stream  

### Notes
  1. Need to hardcode resolution for L515. Depth max is Up to 1024 × 768
      RGB is 1920 × 1080. 
      Must use minimum of both (which is Depth), which is 1024 x 768
      Must connect using USB 3.0
  2. Device Config latches from last-entered settings on realsense-viewer. Otherwise load your json config file
  3. Image name saved is incremented based on the number inside the file count.txt. 
      If it does not exist or if it is empty, count starts from 1.
      It does not matter how many (rgb, depth, infra, andpointcloud **quartets**) files. Numbering is based on 
      the number in count.txt
  4. Resolution is the only setting that does not latch from the realsense-viewer settings. Must be input in 
      terminal
  5. One possible reason for "Exception: Camera capture not available" is that the user input resolution
       is not supported
