#include "mjpeg_cam/MjpegCam.hpp"

namespace mjpeg_cam
{

MjpegCam::MjpegCam(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle),
      sequence(0)
{
    imagePub_ = nodeHandle_.advertise<sensor_msgs::CompressedImage>("/usb_cam/compressed", 10);
    cam = new Webcam("/dev/video0", 1280, 720);
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
    ros::Rate loop_rate(60);
    while (nodeHandle_.ok()) {
        if (!readAndPublishImage())
            ROS_WARN("Could not publish image");

        loop_rate.sleep();
        ros::spinOnce();
    }
}

} /* namespace */
