/**
 * C++ Class to abstract V4l2 API to access usb cameras
 */

#include <string>

struct buffer
{
    void *start;
    size_t size;
};

class UsbCamera
{
public:
    UsbCamera(const std::string &device = "/dev/video0", int width = 640, int height = 480);
    ~UsbCamera();

    char *grab_image(int &len);
    void set_v4l2_param(const std::string &param, int value);

private:
    void init_mmap();
    void open_device();
    void close_device();
    void init_device();
    void uninit_device();
    void start_capturing();
    void stop_capturing();
    bool read_frame();
    void set_v4l2_param(const std::string &param, const std::string &value);

    std::string device;
    int fd;
    struct buffer *buffers;
    size_t n_buffers;
    size_t xres, yres;
    size_t raw_data_len;
    char *raw_data;
    const bool force_format = true;
};




