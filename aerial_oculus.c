/*
Copyright (c) 2018 <copyright holders>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

//
// Compile with: gcc -o aerial_oculus -O2 aerial_oculus.c -lm
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <linux/fb.h>
#include <linux/kd.h>
#include <assert.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <termios.h>
#include <math.h>
#include <linux/joystick.h>

///--------------------------------------------------------
/// Globals
///--------------------------------------------------------

struct buffer {
    void  * start;
    size_t  length;
};

struct gamepad_state {
    float R_Stick_X;
    float R_Stick_Y;
    char  A;
    char  B;
    char  X;
    char  Y;
    char  BACK;
};

static int display_fd = -1;
static const char * display_name = "/dev/fb0";
static int console_fd = -1;
static const char * console_name = "/dev/tty";
static char * display = 0;
static struct fb_var_screeninfo vinfo;
static struct fb_fix_screeninfo finfo;
static struct fb_var_screeninfo orig_vinfo;
static long int screensize;
static int page_size = 0;
static int cur_page = 0;

static int camera1_fd = -1;
static int camera2_fd = -1;
static const char * camera1_name = "/dev/video0";
static const char * camera2_name = "/dev/video1";
static struct buffer * camera1_buffers;
static struct buffer * camera2_buffers;
static const unsigned int n_buffers = 4;

static const char              *gamepad_name = "/dev/input/js0";
static int                      gamepad_fd = -1;
static struct gamepad_state     gamepad_state;
#define                         BUTTON_A    0
#define                         BUTTON_B    1
#define                         BUTTON_X    2
#define                         BUTTON_Y    3
#define                         BUTTON_BACK 6
static float                    STICK_MAX = 32768.0f;
static float                    STICK_DEADZONE = (float)(6000.0 / 32768.0);

static const char * maestro_name = "/dev/ttyACM0";
static int maestro_fd = -1;
#define SERVO_SPEED  1
#define STARTING_ALT 100
#define WINGSPAN     5
static int oldfp[3] = { 0, 0, STARTING_ALT };
static int fp[3]    = { 0, 0, STARTING_ALT };
static int c1[3]    = { - (WINGSPAN/2), 0, 0 };
static int c2[3]    = { + (WINGSPAN/2), 0, 0 };
static int c1fp[3]  = { + WINGSPAN/2, 0, STARTING_ALT };
static int c2fp[3]  = { - WINGSPAN/2, 0, STARTING_ALT };
static int c1s[3];    // Camera 1 signal vector
static int c2s[3];    // Camera 2 signla vector
static double c1a[3]; // Camera 1 angle vector
static double c2a[3]; // Camera 2 angle vector
#define C1PANCH    3
#define C1TILTCH   4
#define C2PANCH    5
#define C2TILTCH   6

static int capture_count = 0;
static unsigned char BGR_buf[1920*1088*3];

static char error_log[1024];

#define CLEAR(x) memset(&(x), 0, sizeof(x))

///--------------------------------------------------------
/// Function definitions
///--------------------------------------------------------

int main();
static void init_devices();
static void init_display();
static void init_camera(int camera_fd, struct buffer ** buffers);
static void init_maestro();
static void start_capturing(int camera_fd);
static void main_loop();
static void update_gamepad();
static void update_display();
static void update_camera(int camera_fd, struct buffer ** buffers);
static void update_maestro();
static void uninit_devices();
static void stop_capturing(int camera_fd);
static void uninit_camera(int camera_fd, struct buffer ** buffers);
static void uninit_display();

static void save_image();
static  int maestro_set_target(int fd, unsigned char channel, unsigned short target);
static void move_servos(int fd, int c1s[2], int c2s[2]) ;
static void convert_C1_angles(double c1a[2], int c1s[2]);
static void convert_C2_angles(double c2a[2], int c2s[2]);
static void update_FP(int fp[3], int R_X, int R_Y, int speed) ;
static double magnitude(int vec[3]);
static void angles(int vec[3], double ang[2]);
static void convert_pixel(unsigned char Y, unsigned char Cb, unsigned char Cr, unsigned char * BGR);
static void open_device(int * fd, const char * filename, const char * device_type, int flags);
static void errno_exit(const char * s);
static int  xioctl(int fh, int request, void *arg);

///--------------------------------------------------------
/// Function bodies
///--------------------------------------------------------

int main() {
    fprintf(stderr, "Initializing devices...\n");
    init_devices();
    fprintf(stderr, "Initialization successful.\n");

    main_loop();
    fprintf(stderr, "Main loop successfully completed.\n");

    uninit_devices();
    fprintf(stderr, "Shutdown successfully.\n");

    return 0;
}

static void init_devices() {
    open_device(&display_fd, display_name, "display", O_RDWR);
    open_device(&console_fd, console_name, "console", O_WRONLY);
    open_device(&camera1_fd, camera1_name, "camera1", O_RDWR | O_NONBLOCK);
    open_device(&camera2_fd, camera2_name, "camera2", O_RDWR | O_NONBLOCK);
    open_device(&gamepad_fd, gamepad_name, "gamepad", O_RDONLY | O_NONBLOCK );
    open_device(&maestro_fd, maestro_name, "maestro", O_RDWR | O_NOCTTY );

    init_display();
    init_camera(camera1_fd, &camera1_buffers);
    init_camera(camera2_fd, &camera2_buffers);
    init_maestro();

    start_capturing(camera1_fd);
    start_capturing(camera2_fd);
}

static void init_display() {
    if (-1 == xioctl(display_fd, FBIOGET_VSCREENINFO, &orig_vinfo)) {
        errno_exit("Error reading variable information.");
    }

    //Copy the current framebuffer variable info so we can make edits
    memcpy(&vinfo, &orig_vinfo, sizeof(struct fb_var_screeninfo));

    vinfo.xres = 1080;
    vinfo.yres = 1920;
    vinfo.xres_virtual = 1080;
    vinfo.yres_virtual = 1920*2;
    vinfo.xoffset = 0;
    vinfo.yoffset = 0;
    vinfo.bits_per_pixel = 24;
    vinfo.left_margin = 0;
    vinfo.right_margin = 0;
    vinfo.upper_margin = 0;
    vinfo.lower_margin = 0;

    if (-1 == xioctl(display_fd, FBIOPUT_VSCREENINFO, &vinfo)) {
        errno_exit("Error setting variable information.");
    }

    if (-1 == xioctl(display_fd, FBIOGET_FSCREENINFO, &finfo)) {
        errno_exit("Error reading fixed information.");
    }

    page_size = finfo.line_length * vinfo.yres;
    screensize = finfo.smem_len;

    display = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, display_fd, 0);
    if ((int)display == -1) {
        errno_exit("Failed to mmap.");
    }

    if (-1 == ioctl(console_fd, KDSETMODE, KD_GRAPHICS)) {
        errno_exit("Error setting console in graphics mode.");
    }

    //Set the screen to be completely black at the start
    memset(BGR_buf, 0, sizeof(BGR_buf));
}

static void init_camera(int camera_fd, struct buffer ** buffers) {
    struct v4l2_capability cap;
    struct v4l2_format fmt;

    if (-1 == xioctl(camera_fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            sprintf(error_log, "camera%d does not support V4L2", (camera_fd != camera1_fd) + 1);
            errno_exit(error_log);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        sprintf(error_log, "camera%d does not support V4L2 video capture",  (camera_fd != camera1_fd) + 1);
        errno_exit(error_log);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        sprintf(error_log, "camera%d does not support V4L2 streaming i/o",  (camera_fd != camera1_fd) + 1);
        errno_exit(error_log);
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    fmt.fmt.pix.width       = 1920;
    fmt.fmt.pix.height      = 1080;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (-1 == xioctl(camera_fd, VIDIOC_S_FMT, &fmt)) {
        errno_exit("VIDIOC_S_FMT");
    }

    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = n_buffers;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(camera_fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            sprintf(error_log, "camera%d does not support memory mapping",  (camera_fd != camera1_fd) + 1);
            errno_exit(error_log);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        sprintf(error_log, "Insufficient buffer memory on camera%d",  (camera_fd != camera1_fd) + 1);
        errno_exit(error_log);
    }

    *buffers = calloc(req.count, sizeof(struct buffer));

    if (!*buffers) {
        errno_exit("Out of memory");
    }

    for (unsigned int i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        if (-1 == xioctl(camera_fd, VIDIOC_QUERYBUF, &buf)) {
            errno_exit("VIDIOC_QUERYBUF");
        }

        (*buffers)[i].length = buf.length;
        (*buffers)[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, camera_fd, buf.m.offset);

        if (MAP_FAILED == (*buffers)[i].start) {
            errno_exit("mmap");
        }
    }
}

static void init_maestro() {
    struct termios options;

    if (tcgetattr(maestro_fd, &options) == -1) {
        errno_exit("Unable to get maestro attributes");
    }
	
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
	options.c_oflag &= ~(ONLCR | OCRNL);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    if (tcsetattr(maestro_fd, TCSANOW, &options) == -1) {
        errno_exit("Unable to set maestro attributes");
    }
}

static void start_capturing(int camera_fd) {
    for (unsigned int i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(camera_fd, VIDIOC_QBUF, &buf)) {
            errno_exit("VIDIOC_QBUF");
        }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(camera_fd, VIDIOC_STREAMON, &type)) {
        errno_exit("VIDIOC_STREAMON");
    }
}

static void main_loop() {
    int nfds = 0;
    int view_changed = 1;
    int num_frames = 64;

    if (camera1_fd > nfds) nfds = camera1_fd;
    if (camera2_fd > nfds) nfds = camera2_fd;
    if (gamepad_fd > nfds) nfds = gamepad_fd;
    
    for(int i = 0; i < num_frames; ++i) {
        
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        fd_set fds;
        FD_ZERO(&fds);
        if (camera1_fd >= 0) FD_SET(camera1_fd, &fds);
        if (camera2_fd >= 0) FD_SET(camera2_fd, &fds);
        if (gamepad_fd >= 0) FD_SET(gamepad_fd, &fds);
        
        int r = select(nfds+1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno == EINTR) { fprintf(stderr, "Continue called!\n"); continue; }
            else { errno_exit("SELECT"); }
        } else if (r == 0) {
            errno_exit("Select timeout");
        }
        if (FD_ISSET(gamepad_fd, &fds)) {
            update_gamepad();

            // Control logic
            if (gamepad_state.Y) {
                fp[0] = 0.0;
                fp[1] = 0.0;
            }
            if (gamepad_state.A) {
                save_image();
            }
            if (gamepad_state.BACK) return;
        }
        if (FD_ISSET(camera1_fd, &fds)) {
            view_changed = 1;
            update_camera(camera1_fd, &camera1_buffers);
        }
        if (FD_ISSET(camera2_fd, &fds)) {
            view_changed = 1;
            update_camera(camera2_fd, &camera2_buffers);
        }

        if (view_changed) {
            update_display();
            view_changed = 0;
        }

        update_maestro();
    }
}

static void update_gamepad() {
    struct js_event js;
    CLEAR(js);
    read(gamepad_fd, &js, sizeof(js));

    gamepad_state.A = 0;
    gamepad_state.B = 0;
    gamepad_state.X = 0;
    gamepad_state.Y = 0;
    gamepad_state.BACK = 0;

    float tmp;

    if (js.value && (js.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
        switch(js.number) {
            case BUTTON_A: 
                gamepad_state.A = 1;
                break;
            case BUTTON_B: 
                gamepad_state.B = 1;
                break;
            case BUTTON_X: 
                gamepad_state.X = 1;
                break;
            case BUTTON_Y: 
                gamepad_state.Y = 1;
                break;
            case BUTTON_BACK: 
                gamepad_state.BACK = 1;
                break;
        }
    }
    if ((js.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
        switch (js.number) {
            case 3: //R_Stick x-axis
                tmp = js.value / STICK_MAX;
                if (-STICK_DEADZONE < tmp && tmp < STICK_DEADZONE)
                    tmp = 0;
                gamepad_state.R_Stick_X = tmp;
                break;
            case 4: //R_Stick y-axis
                tmp = js.value / STICK_MAX;
                if (-STICK_DEADZONE < tmp && tmp < STICK_DEADZONE)
                    tmp = 0;
                gamepad_state.R_Stick_Y = tmp;
                break;
        }
    }
}

static void update_display() {
    vinfo.yoffset = cur_page * 1920;
    memcpy(display + cur_page * (1088*1920*3), BGR_buf, sizeof(BGR_buf));
    cur_page = !cur_page;
    ioctl(display_fd, FBIOPAN_DISPLAY, &vinfo);
    __u32 dummy = 0;
    ioctl(display_fd, FBIO_WAITFORVSYNC, &dummy);
}

static void update_camera(int camera_fd, struct buffer ** buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(camera_fd, VIDIOC_DQBUF, &buf)) {
        if (errno == EAGAIN) { return; }
        errno_exit("VIDIOC_DQBUF");
    }

    //Camera image comes in at 1920*1080*2
    //Reduce to 1080*960*2
    //Map it onto 1080*960*3
    const int x_offset = (1920-960)/2;
    const unsigned char * start = (unsigned char *)(*buffers)[buf.index].start;
    unsigned char * BGR_loc = BGR_buf + (camera_fd != camera1_fd) * 960*1088*3;
    for (int y = 0; y < 1080; y+=2) {
        for (int x = 0; x < 960; x+=2) {
            const unsigned char * pix = start+(y*1920+(x_offset+x))*2;
            convert_pixel(*(pix+0), *(pix+1), *(pix+3), &BGR_loc[(x*1088+y+0)*3]);
            convert_pixel(*(pix+2), *(pix+1), *(pix+3), &BGR_loc[(x*1088+y+1080)*3]);
            
            pix = start+((y+1)*1920+(x_offset+x))*2;
            convert_pixel(*(pix+0), *(pix+1), *(pix+3), &BGR_loc[(x*1088+y+1)*3]);
            convert_pixel(*(pix+2), *(pix+1), *(pix+3), &BGR_loc[(x*1088+y+1081)*3]);
        }
    }

    if (-1 == xioctl(camera_fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
}

static void update_maestro() {
    oldfp[0] = fp[0];
    oldfp[1] = fp[1];
    oldfp[2] = fp[2];
    int R_X = (gamepad_state.R_Stick_X > 0) - (gamepad_state.R_Stick_X < 0);
    int R_Y = (gamepad_state.R_Stick_Y > 0) - (gamepad_state.R_Stick_Y < 0);
    update_FP(fp, R_X, R_Y, SERVO_SPEED);

    c1fp[0] = fp[0] - c1[0];
    c1fp[1] = fp[1] - c1[1];
    c2fp[0] = fp[0] - c2[0];
    c2fp[1] = fp[1] - c2[1];

    angles(c1fp, c1a);
    angles(c2fp, c2a);		

    convert_C1_angles(c1a, c1s);
    convert_C2_angles(c2a, c2s);

    move_servos(maestro_fd, c1s, c2s);
}

static void uninit_devices() {
    stop_capturing(camera1_fd);
    stop_capturing(camera2_fd);

    uninit_camera(camera1_fd, &camera1_buffers);
    uninit_camera(camera2_fd, &camera2_buffers);
    uninit_display();

    close(gamepad_fd);
    close(maestro_fd);
}

static void stop_capturing(int camera_fd) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (camera_fd != -1 && -1 == xioctl(camera_fd, VIDIOC_STREAMOFF, &type)) {
        fprintf(stderr, "Failed to turn off camera stream. VIDIOC_STREAMOFF FAILED.\n");
    }
}

static void uninit_camera(int camera_fd, struct buffer ** buffers) {
    if (camera_fd != -1) {
        for (unsigned int i = 0; i < n_buffers; ++i) {
            if (-1 == munmap((*buffers)[i].start, (*buffers)[i].length)) {
                fprintf(stderr, "Failed to unmap camera%d's memory mapped buffers.\n", 1+(camera_fd != camera1_fd));
            }
        }
        free(*buffers);
    }
}

static void uninit_display() {
    if (display) {
        munmap(display, screensize);
    }
    if (console_fd >= 0) {
        ioctl(console_fd, KDSETMODE, KD_TEXT);
        close(console_fd);
    }
    if (-1 == xioctl(display_fd, FBIOPUT_VSCREENINFO, &orig_vinfo)) {
        printf("Error re-setting variable information.\n");
    }

    close(display_fd);
}

static void save_image() {
    char namebuf[18];
    sprintf(namebuf, "image-%d.raw", capture_count);
    ++capture_count;
    FILE *fp = fopen(namebuf, "wb");
    fwrite(BGR_buf, sizeof(BGR_buf), 1, fp);
    fflush(fp);
    fclose(fp);
}

//Updates the focal point using the joystick input
void update_FP(int fp[3], int R_X, int R_Y, int speed) {
	fp[0] += R_X * speed;
	fp[1] += R_Y * speed;
	if (fp[1] < 0) {
		fp[1] = 0;
	}
}

// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestro_set_target(int fd, unsigned char channel, unsigned short target) {
	unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1)
	{
        errno_exit("Error writing maestro_set_target");
	}
	return 0;
}

//Moves each servo to their respective position
void move_servos(int fd, int c1s[2], int c2s[2]) {
	maestro_set_target(fd, C1PANCH,  (unsigned short) c1s[1]);
	maestro_set_target(fd, C1TILTCH, (unsigned short) c1s[0]);
	maestro_set_target(fd, C2PANCH,  (unsigned short) c2s[1]);
	maestro_set_target(fd, C2TILTCH, (unsigned short) c2s[0]);
}

//Each servo needs hardcoded values for their respective ranges
//These values were calculated via the maestro control center software
void convert_C1_angles(double c1a[2], int c1s[2]) {
	//tilt
	c1s[0] = round((c1a[0] * (-4216.0/M_PI)) + 5825.0);
	//pan
	c1s[1] = round(((c1a[1] - M_PI) * (4208.0/M_PI)) + 8452.0);
}

void convert_C2_angles(double c2a[2], int c2s[2]) {
	//tilt
	c2s[0] = round((c2a[0] * (5656.0/M_PI)) + 5440.0);
	//pan
	c2s[1] = round(((c2a[1] - M_PI) * (-5308.0/M_PI)) + 3332.0);
}

double magnitude(int vec[3]) {
	return sqrt((vec[0]*vec[0]) + (vec[1]*vec[1]) + (vec[2] * vec[2]));
}

//Takes a R3 vector and returns the angles needed for servo rotation
void angles(int vec[3], double ang[2]) {
	//Special case to avoid dividing by 0 in the acos call
	if (vec[0] == 0 && vec[1] == 0) {
		//directly up
		ang[0] = (M_PI/2.0);
		//directly forward
		ang[1] = (M_PI/2.0);
		return;
	}

	//Angle off the xy-plane
	//acos(vec*vecproj/||vec||*||vecproj||)
	ang[0] = acos((float)(vec[0]*vec[0] + vec[1]*vec[1])/(magnitude(vec)*sqrt((vec[0]*vec[0])+(vec[1]*vec[1]))));
	//Error checking for acos
	if (isnan(ang[0]))
		ang[0] = 0.0;
	
	//Sanity checking, this should never be possible but what the hell lets make sure
	if (ang[0] < 0.0) {
		ang[0] = 0.0;
	} else if (ang[0] > (M_PI/2.0)) {
		ang[0] = (M_PI/2.0);
	}

	//Angle off +x axis
	if (vec[0] == 0) {
		ang[1] = (M_PI/2.0);
	} else if (vec[0] < 0) {
		ang[1] = M_PI + atan((float)vec[1]/(float)vec[0]);
	} else {
		ang[1] = atan((float)vec[1]/(float)vec[0]);
	}

	//Sanity checking
	if (ang[1] < 0.0) {
		ang[1] = 0.0;
	} else if (ang[1] > M_PI) {
		ang[1] = M_PI;
	}
}

static void convert_pixel(unsigned char Y, unsigned char Cb, unsigned char Cr, unsigned char * BGR) {
    int b=(int)(1.16438356164*Y +  2.01723214286*Cb                     - 276.835851272 );
    int g=(int)(1.16438356164*Y - 0.391762290095*Cb - 0.812967647238*Cr + 135.57529499  );
    int r=(int)(1.16438356164*Y                     +  1.59602678571*Cr - 222.921565557 );
    if (b < 0) b = 0; 
    if (b > 255) b = 255;
    if (g < 0) g = 0; 
    if (g > 255) g = 255;
    if (r < 0) r = 0; 
    if (r > 255) r = 255;
    *(BGR+0) = b;
    *(BGR+1) = g;
    *(BGR+2) = r;
}

static void open_device(int * fd, const char * filename, const char * device_type, int flags) {
    struct stat st;
    CLEAR(st);

    if (-1 == stat(filename, &st)) {
        sprintf(error_log, "Cannot find %s.\nExpected location: '%s'.", device_type, filename);
        errno_exit(error_log);
    }

    if (!S_ISCHR(st.st_mode)) {
        sprintf(error_log, "%s is not a device.", filename);
        errno_exit(error_log);
    }

    *fd = open(filename, flags);
    if (*fd == -1) {
        sprintf(error_log, "Cannot open '%s'.", filename);
        errno_exit(error_log);
    }
}

static void errno_exit(const char *s) {
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    uninit_devices();
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg) {
    int r = ioctl(fh, request, arg);
    while (r == -1 && errno == EINTR) r = ioctl(fh, request, arg);
    return r;
}

/*
// Useful for debugging
// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestro_get_position(int fd, unsigned char channel) {
    unsigned char command[] = {0x90, channel};
    if(write(fd, command, sizeof(command)) == -1)
    {
        errno_exit("Error writing maestro_get_position");
    }

    unsigned char response[2];
    if(read(fd,response,2) != 2)
    {
        errno_exit("Error reading maestro_get_position");
    }

    return response[0] + 256*response[1];
}
*/