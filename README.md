# mjpeg_cam

## Overview

This package captures MJPEG video stream from usb cameras. Unlike the `usb_cam` package, this driver copies the JPEG data to the `CompressedImage` message directly. There is no extra overhead from decoding and re-encoding the image. So, only compressed images are published. This driver is suitable for applications where the images are captured by a low power device (e.g. the Raspberry Pi) and sent to a remote node for further processing. 

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- libv4l-dev (for accessing the usb camera)
    ```$xslt
    sudo apt-get install libv4l-dev
    ```
- v4l-utils (for changing camera settings)
    ```$xslt
    sudo apt-get install v4l-utils
    ```
#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using `catkin_make`.

## Usage

Run the main node with

	roslaunch mjpeg_cam mjpeg_cam_run.launch


## Nodes

### mjpeg_cam

Captures video stream from a usb camera and publishes the images.

#### Published Topics

* **`image/compressed`** (sensor_msgs/CompressedImage, default: "/mjpeg_cam/image/compressed')

#### Parameters

* **`device_name`** (string, default: "/dev/video0")

	The name of the usb camera device.

* **`width`** (int, default: 640)

	The width of the image.
	
* **`height`** (int, default: 480)
    
 	The height of the image.

* **`framerate`** (int, default: 30)
    
 	The framerate of the video.

* **`exposure`** (int, default: 128)
    
 	The exposure of the camera.

* **`brightness`** (int, default: 128)
    
 	The brightness of the image.

* **`autoexposure`** (bool, default: true)
    
 	Turns auto exposure on/off.
 	
The `exposure`, `brightness`, and `autoexposure` settings are dynamically reconfigurable. Simply run `rosrun rqt_reconfigure rqt_reconfigure` and select the node name (default: "mjpeg_cam") to access these settings.


[ROS]: http://www.ros.org