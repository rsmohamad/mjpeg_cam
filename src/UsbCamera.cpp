#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <linux/videodev2.h>

#include <mjpeg_cam/UsbCamera.hpp>
#include <boost/lexical_cast.hpp>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

using namespace std;

static int xioctl(int fh, unsigned long int request, void *arg)
{
    int r;
    do {
        r = ioctl(fh, request, arg);
    }
    while (-1 == r && EINTR == errno);
    return r;
}

UsbCamera::UsbCamera(const string &device, int width, int height)
    :
    device(device),
    xres(width),
    yres(height),
    raw_data(0)
{
    open_device();
    init_device();
    start_capturing();
}

UsbCamera::~UsbCamera()
{
    stop_capturing();
    uninit_device();
    close_device();
}

char *UsbCamera::grab_image(int &len)
{
    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        // Timeout
        tv.tv_sec = 5;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno)
                continue;
            throw runtime_error("select");
        }

        if (0 == r) {
            throw runtime_error(device + ": select timeout");
        }
        if (read_frame()) {
            // Return the address of image
            len = raw_data_len;
            return raw_data;
        }
    }
}

bool UsbCamera::read_frame()
{
    struct v4l2_buffer buf;
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
        switch (errno) {
        case EAGAIN:return false;
        default:throw runtime_error("VIDIOC_DQBUF");
        }

    assert(buf.index < n_buffers);
    raw_data_len = buf.bytesused;
    raw_data = (char *) buffers[buf.index].start;

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        throw runtime_error("VIDIOC_QBUF");

    return true;
}

void UsbCamera::open_device(void)
{
    fd = open(device.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (-1 == fd)
        throw runtime_error(device + ": cannot open! " + to_string(errno) + ": " + strerror(errno));
}

void UsbCamera::init_mmap(void)
{
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
        if (EINVAL == errno)
            throw runtime_error(device + " does not support memory mapping");
        else
            throw runtime_error("VIDIOC_REQBUFS");

    if (req.count < 2)
        throw runtime_error(string("Insufficient buffer memory on ") + device);
    buffers = (buffer *) calloc(req.count, sizeof(*buffers));
    if (!buffers)
        throw runtime_error("Out of memory");

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            throw runtime_error("VIDIOC_QUERYBUF");

        buffers[n_buffers].size = buf.length;
        buffers[n_buffers].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            throw runtime_error("mmap");
    }
}

void UsbCamera::close_device(void)
{
    if (-1 == close(fd))
        throw runtime_error("close");
    fd = -1;
}

void UsbCamera::init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;

    // Check camera capabilities
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
        if (EINVAL == errno)
            throw runtime_error(device + " is no V4L2 device");
        else
            throw runtime_error("VIDIOC_QUERYCAP");

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        throw runtime_error(device + " is no video capture device");


    if (!(cap.capabilities & V4L2_CAP_STREAMING))
        throw runtime_error(device + " does not support streaming i/o");


    CLEAR(cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect;
        xioctl(fd, VIDIOC_S_CROP, &crop);
    }

    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (force_format) {
        fmt.fmt.pix.width = xres;
        fmt.fmt.pix.height = yres;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            throw runtime_error("VIDIOC_S_FMT");

        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
            throw runtime_error("Webcam does not support MJPEG format. Support for more format need to be added!");

        // VIDIO_S_FMT may change width and height
        xres = fmt.fmt.pix.width;
        yres = fmt.fmt.pix.height;
        stride = fmt.fmt.pix.bytesperline;
    }
    else {
        // Revert to default setting
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            throw runtime_error("VIDIOC_G_FMT");
    }
    init_mmap();
}

void UsbCamera::uninit_device(void)
{
    unsigned int i;
    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].start, buffers[i].size))
            throw runtime_error("munmap");
    free(buffers);
}

void UsbCamera::start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;
    for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            throw runtime_error("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        throw runtime_error("VIDIOC_STREAMON");
}

void UsbCamera::stop_capturing(void)
{
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        throw runtime_error("VIDIOC_STREAMOFF");
}

// Taken from usb_cam package
void UsbCamera::set_v4l2_param(const string &param, const string &value)
{
    // build the command
    stringstream ss;
    ss << "v4l2-ctl --device=" << device << " -c " << param << "=" << value << " 2>&1";
    string cmd = ss.str();

    // capture the output
    string output;
    int buffer_size = 256;
    char buffer[buffer_size];
    FILE *stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, buffer_size, stream) != NULL)
                output.append(buffer);
        pclose(stream);
        // any output should be an error
        if (output.length() > 0)
            throw "Failed to set param";
    }
    else
        throw ("usb_cam_node could not run '%s'", cmd.c_str());
}

void UsbCamera::set_v4l2_param(const string &param, int value)
{
    set_v4l2_param(param, boost::lexical_cast<string>(value));
}


