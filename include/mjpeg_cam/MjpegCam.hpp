#pragma once

#include "mjpeg_cam/MjpegCam.hpp"
#include "mjpeg_cam/UsbCamera.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/CompressedImage.h>


namespace mjpeg_cam
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class MjpegCam
{
public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    MjpegCam(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~MjpegCam();

    /*!
     * Enters an event loop to read the camera
     */
    void spin();

    /*!
     * Set parameters that can be dynamically reconfigured
     */
    void setDynamicParams(int exposure, int brightness, bool autoexposure);

private:
    /*!
     * Reads a single frame from the camera and publish to topic.
     */
    bool readAndPublishImage();

    /*!
     * Reads ROS parameters.
     */
    void readParameters();

    /*!
     * Set camera parameters
     */
    bool setCameraParams();

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS Image Publisher
    ros::Publisher imagePub_;

    //! Camera Object
    UsbCamera *cam;
    unsigned int sequence;

    // Parameters
    std::string device_name;
    int width;
    int height;
    int framerate;
    int exposure;
    int brightness;
    bool autoexposure;
};

} /* namespace */
