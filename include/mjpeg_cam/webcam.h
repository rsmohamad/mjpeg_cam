/** Small C++ wrapper around V4L example code to access the webcam
**/

#include <string>
#include <memory> // unique_ptr

#include <sensor_msgs/CompressedImage.h>

struct buffer
{
    void *data;
    size_t size;
};

struct RGBImage
{
    unsigned char *data; // RGB888 <=> RGB24
    size_t width;
    size_t height;
    size_t size; // width * height * 3
};

class Webcam
{

public:
    Webcam(const std::string &device = "/dev/video0",
           int width = 640,
           int height = 480);

    ~Webcam();

    /** Captures and returns a frame from the webcam.
     *
     * The returned object contains a field 'data' with the image data in RGB888
     * format (ie, RGB24), as well as 'width', 'height' and 'size' (equal to
     * width * height * 3)
     *
     * This call blocks until a frame is available or until the provided
     * timeout (in seconds). 
     *
     * Throws a runtime_error if the timeout is reached.
     */
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
    unsigned int n_buffers;

    size_t xres, yres;
    size_t stride;

    char *raw_data;
    size_t raw_data_len;

    const bool force_format = true;
};




