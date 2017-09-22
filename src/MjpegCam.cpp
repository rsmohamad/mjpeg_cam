#include "mjpeg_cam/MjpegCam.hpp"

namespace mjpeg_cam
{

void clamp(int &val, int min, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
}

MjpegCam::MjpegCam(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle),
      sequence(0)
{
    readParameters();
    imagePub_ = nodeHandle_.advertise<sensor_msgs::CompressedImage>("image/compressed", 1);
    cam = new UsbCamera(device_name, width, height);
    setCameraParams();
    ROS_INFO("Successfully launched node.");
}

MjpegCam::~MjpegCam()
{
    delete cam;
}

bool MjpegCam::readAndPublishImage()
{
    try {
        int length;
        char *image = cam->grab_image(length);
        sensor_msgs::CompressedImage msg;

        msg.header.frame_id = "usb_cam";
        msg.header.seq = sequence++;
        msg.header.stamp = ros::Time::now();
        msg.format = "jpeg";
        msg.data.resize(length);
        std::copy(image, image + length, msg.data.begin());
        imagePub_.publish(msg);

        return true;
    }
    catch (const char *e) {
        std::cout << e << std::endl;
    }

    return false;
}
void MjpegCam::spin()
{
    ros::Rate loop_rate(framerate);
    while (nodeHandle_.ok()) {
        if (!readAndPublishImage())
            ROS_WARN("Could not publish image");

        loop_rate.sleep();
        ros::spinOnce();
    }
}

void MjpegCam::readParameters()
{
    nodeHandle_.param("device_name", device_name, std::string("/dev/video0"));
    nodeHandle_.param("width", width, 640);
    nodeHandle_.param("height", height, 480);
    nodeHandle_.param("framerate", framerate, 30);
    nodeHandle_.param("exposure", exposure, 128);
    nodeHandle_.param("brightness", brightness, 128);
    nodeHandle_.param("autoexposure", autoexposure, true);

    clamp(exposure, 0, 255);
    clamp(brightness, 0, 255);
    clamp(white_balance, 0, 6000);
}

bool MjpegCam::setCameraParams()
{
    if (cam == 0)
        return false;

    cam->set_v4l2_param("brightness", brightness);

    if (!autoexposure)
    {
        cam->set_v4l2_param("exposure_auto", 1);
        cam->set_v4l2_param("exposure_absolute", exposure);
    }

    return true;

}

} /* namespace */
