#include <ros/ros.h>
#include "mjpeg_cam/MjpegCam.hpp"

#include <dynamic_reconfigure/server.h>
#include <mjpeg_cam/mjpeg_camConfig.h>

void dynamic_reconfigure_cb(mjpeg_cam::mjpeg_camConfig &config, uint32_t level, mjpeg_cam::MjpegCam *node)
{
    ROS_INFO("Reconfigure Request: \nExposure: %d \nBrightness: %d \nAutoexposure: %s",
             config.exposure,
             config.brightness,
             config.autoexposure?"True":"False");
    node->setDynamicParams(config.exposure, config.brightness, config.autoexposure);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mjpeg_cam");

    dynamic_reconfigure::Server<mjpeg_cam::mjpeg_camConfig> server;
    dynamic_reconfigure::Server<mjpeg_cam::mjpeg_camConfig>::CallbackType cb;

    ros::NodeHandle nodeHandle("~");
    mjpeg_cam::MjpegCam mjpegCam(nodeHandle);

    cb = boost::bind(&dynamic_reconfigure_cb, _1, _2, &mjpegCam);
    server.setCallback(cb);
    mjpegCam.spin();

    ros::shutdown();
    return 0;
}
