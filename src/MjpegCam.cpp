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
    imagePub_ = nodeHandle_.advertise<sensor_msgs::CompressedImage>(base_topic+"/compressed", 1);
    infoPub_ = nodeHandle_.advertise<sensor_msgs::CameraInfo>("fake_camera_info", 60);

    cam = new UsbCamera(device_name, width, height);

    try {
        setCameraParams();
    }
    catch (const char * e){
        std::cout << e << std::endl;
    }

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
        sensor_msgs::CameraInfo info_msg;
        msg.header.frame_id = camera_name;
        msg.header.seq = sequence++;
        msg.header.stamp = ros::Time::now();
        msg.format = "jpeg";
        msg.data.resize(length);
        std::copy(image, image + length, msg.data.begin());
        info_msg.header.frame_id = msg.header.frame_id;
        info_msg.header.seq = msg.header.seq;
        info_msg.header.stamp = msg.header.stamp;
        info_msg.width = width;
        info_msg.height = height;
        imagePub_.publish(msg);
        infoPub_.publish(info_msg);
        //std::cout << "Image size in kB: " << length/1000 << std::endl;

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
    nodeHandle_.param("camera_name", camera_name, std::string("mjpeg_camera"));
    nodeHandle_.param("base_topic", base_topic, std::string("image_raw"));

    nodeHandle_.param("exposure", exposure, 128);
    nodeHandle_.param("autoexposure", autoexposure, true);
    nodeHandle_.param("brightness", brightness, 128);
    nodeHandle_.param("gamma", gamma, 100);
    nodeHandle_.param("gain", gain, 0);
    nodeHandle_.param("saturation", saturation, 56);
    nodeHandle_.param("contrast", contrast, 32);
    nodeHandle_.param("auto_white_balance", auto_white_balance, false);
    nodeHandle_.param("white_balance", white_balance, 4600);
}

bool MjpegCam::setCameraParams()
{
    if (cam == 0)
        return false;

    clamp(exposure, 0, 255);
    clamp(brightness, 0, 255);

    cam->set_v4l2_param("brightness", brightness);

    if (autoexposure) {
        cam->set_v4l2_param("exposure_auto", 3);
    }
    else {
        cam->set_v4l2_param("exposure_auto", 1);
        cam->set_v4l2_param("exposure_absolute", exposure);
    }

    if (auto_white_balance) {
      cam->set_v4l2_param("white_balance_temperature_auto", 1);
    }
    else {
      cam->set_v4l2_param("white_balance_temperature_auto", 0);
      cam->set_v4l2_param("white_balance_temperature", white_balance);
    }
    cam->set_v4l2_param("gamma", gamma);
    cam->set_v4l2_param("gain", gain);
    cam->set_v4l2_param("saturation", saturation);
    cam->set_v4l2_param("contrast", contrast);


    return true;
}

void MjpegCam::setDynamicParams(int exposure, int brightness, bool autoexposure, int gamma, int gain, int saturation, int contrast, bool auto_white_balance, int white_balance)
{
    this->exposure = exposure;
    this->brightness = brightness;
    this->autoexposure = autoexposure;
    this->gamma = gamma;
    this->gain = gain;
    this->saturation = saturation;
    this->contrast = contrast;
    this->auto_white_balance = auto_white_balance;
    this->white_balance = white_balance;
    setCameraParams();
}

} /* namespace */
