/*

   (c) 2014 Séverin Lemaignan <severin.lemaignan@epfl.ch>
   (c) 2008 Hans de Goede <hdegoede@redhat.com> for yuyv_to_rgb24

 This program is free software; you can redistribute it and/or modify it
 under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2.1 of the License, or (at
 your option) any later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software Foundation,
 Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335  USA

 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <string.h> // strerrno
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <stdexcept>

#include <linux/videodev2.h>

#include <mjpeg_cam/webcam.h>
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

Webcam::Webcam(const string &device, int width, int height)
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

Webcam::~Webcam()
{
    stop_capturing();
    uninit_device();
    close_device();
}

char *Webcam::grab_image(int &len)
{
    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
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
        /* EAGAIN - continue select loop. */
    }
}

bool Webcam::read_frame()
{
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
        case EAGAIN:return false;
        case EIO:
        default:throw runtime_error("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < n_buffers);
    raw_data_len = buf.bytesused;

    if(raw_data)
        delete raw_data;

    raw_data = new char[raw_data_len];
    memcpy(raw_data, buffers[buf.index].data, raw_data_len);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        throw runtime_error("VIDIOC_QBUF");

    return true;
}

void Webcam::open_device(void)
{
    struct stat st;

    if (-1 == stat(device.c_str(), &st)) {
        throw runtime_error(device + ": cannot identify! " + to_string(errno) + ": " + strerror(errno));
    }

    if (!S_ISCHR(st.st_mode)) {
        throw runtime_error(device + " is no device");
    }

    fd = open(device.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd) {
        throw runtime_error(device + ": cannot open! " + to_string(errno) + ": " + strerror(errno));
    }
}

void Webcam::init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            throw runtime_error(device + " does not support memory mapping");
        }
        else {
            throw runtime_error("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        throw runtime_error(string("Insufficient buffer memory on ") + device);
    }

    buffers = (buffer *) calloc(req.count, sizeof(*buffers));

    if (!buffers) {
        throw runtime_error("Out of memory");
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            throw runtime_error("VIDIOC_QUERYBUF");

        buffers[n_buffers].size = buf.length;
        buffers[n_buffers].data =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].data)
            throw runtime_error("mmap");
    }
}

void Webcam::close_device(void)
{
    if (-1 == close(fd))
        throw runtime_error("close");

    fd = -1;
}

void Webcam::init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            throw runtime_error(device + " is no V4L2 device");
        }
        else {
            throw runtime_error("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        throw runtime_error(device + " is no video capture device");
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        throw runtime_error(device + " does not support streaming i/o");
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else {
        /* Errors ignored. */
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
            // note that libv4l2 (look for 'v4l-utils') provides helpers
            // to manage conversions
            throw runtime_error("Webcam does not support MJPEG format. Support for more format need to be added!");

        /* Note VIDIOC_S_FMT may change width and height. */
        xres = fmt.fmt.pix.width;
        yres = fmt.fmt.pix.height;

        stride = fmt.fmt.pix.bytesperline;

    }
    else {
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            throw runtime_error("VIDIOC_G_FMT");
    }

    init_mmap();
}

void Webcam::uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].data, buffers[i].size))
            throw runtime_error("munmap");

    free(buffers);
}

void Webcam::start_capturing(void)
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

void Webcam::stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        throw runtime_error("VIDIOC_STREAMOFF");
}

void Webcam::set_v4l2_param(const string &param, const string &value)
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

void Webcam::set_v4l2_param(const string &param, int value)
{
    set_v4l2_param(param, boost::lexical_cast<string>(value));
}


