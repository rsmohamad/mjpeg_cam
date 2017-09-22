#include <ros/ros.h>
#include "mjpeg_cam/MjpegCam.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mjpeg_cam");
    ros::NodeHandle nodeHandle("~");

    mjpeg_cam::MjpegCam mjpegCam(nodeHandle);
    mjpegCam.spin();

    ros::shutdown();
    return 0;
}
