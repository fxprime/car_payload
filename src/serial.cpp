/* Copyright (C) Thanabdee Bulunseechart, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Thanabdee Bulunseechart <paheyisoicus@gmail.com>, August 2018
 */
#include <ros/ros.h>

#include <car_connect/serial.hpp>
using std::string;
using namespace std;

#define SERIAL_INFO(...) {ROS_INFO(__VA_ARGS__);}
#define SERIAL_DEBUG(...) {/*ROS_INFO(__VA_ARGS__);*/}
#define SERIAL_ERROR(...) {ROS_ERROR(__VA_ARGS__);}


/**


   Returns the file descriptor on success or -1 on error.
*/

int serial::open_port(std::string& port)
{

    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

    SERIAL_INFO("Serial : Openning port %s", port.c_str());
    SERIAL_DEBUG("fd = %d", fd_);
    if (fd_ <0)
    {
        fd_ = -1;
        ROS_ERROR("Serial : Cann't open port %s", port.c_str());
        /* Could not open the port. */
        return (-1);
    }
    else
    {
        // fcntl(fd, F_SETFL, 0);
        fcntl(fd_, F_SETFL, O_NONBLOCK);
    }

    _port = port;

    return (fd_);
}


void serial::close_port()
{

    if(isOpened()){
        SERIAL_INFO("Serial : Closing port");
        close(fd_);
    }
    fd_ = -1;
    _port = "";
}

serial::serial(string& device_name)
        : fd_(-1)
{
    open_port(device_name);
    if(!isOpened())
    {
        close_port();
    }
}

serial::~serial()
{
    close_port();
}

bool serial::isOpened()
{
    return (fd_ >= 0);
}


bool serial::setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    //struct termios options;

    struct termios config;
    if (!isatty(fd_))
    {
        SERIAL_ERROR("Serial : file descriptor %d is NOT a serial port", fd_);
        return false;
    }
    if (tcgetattr(fd_, &config) < 0)
    {
        SERIAL_ERROR("Serial : could not read configuration of fd %d", fd_);
        return false;
    }
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
    config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
    config.c_oflag &= ~ONOEOT;
#endif

    //
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    //tcgetattr(fd_, &options);

    switch (baud)
    {
        case 1200:
            if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
            {
                SERIAL_ERROR("Serial : ERROR: Could not set desired baud rate of %d Baud", baud);
                return false;
            }
            break;
        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
            {
                SERIAL_ERROR("Serial : ERROR: Could not set desired baud rate of %d Baud", baud);
                return false;
            }
            break;
        case 57600:
            if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
            {
                SERIAL_ERROR("Serial : ERROR: Could not set desired baud rate of %d Baud", baud);
                return false;
            }
            break;
        case 115200:
            if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
            {
                SERIAL_ERROR("Serial : ERROR: Could not set desired baud rate of %d Baud", baud);
                return false;
            }
            break;
        case 230400:
            if (cfsetispeed(&config, B230400) < 0 || cfsetospeed(&config, 230400) < 0)
            {
                SERIAL_ERROR("Serial : ERROR: Could not set desired baud rate of %d Baud", baud);
                return false;
            }
            break;

            // These two non-standard (by the 70'ties ) rates are fully supported on
            // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
            {
                SERIAL_ERROR("Serial : ERROR: Could not set desired baud rate of %d Baud", baud);
                return false;
            }
            break;
        case 921600:
            if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
            {
                SERIAL_ERROR("Serial : ERROR: Could not set desired baud rate of %d Baud", baud);
                return false;
            }
            break;
        default:
        SERIAL_ERROR("Serial : ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
            return false;

            break;
    }

    //
    // Finally, apply the configuration
    //
    if (tcsetattr(fd_, TCSAFLUSH, &config) < 0)
    {
        SERIAL_ERROR("Serial : ERROR: could not set configuration of fd %d", fd_);
        return false;
    }
    return true;
}




int serial::Write(const char* data, uint32_t size)
{
    return write(fd_, data, size);
}

int serial::Read(char* data, uint32_t size)
{
    return read(fd_, data, size);
}



// Checks if a byte is available in the serial port
int serial::BytesAvailable() {
    int bytesAvailable;
    ioctl(fd_, FIONREAD, &bytesAvailable);
    // #ifdef AUTOPILOT_TESTING
    // if (bytesAvailable == 0){
    //     //Detected end of file. Rewinding to start.
    //     lseek(fd, 0, SEEK_SET);
    //     ioctl(fd, FIONREAD, &bytesAvailable);
    // }
    // #endif
    return bytesAvailable;
}


bool serial::waitForReadyRead(int ms)
{
    if (!isOpened())
    {
        return false;
    }

    fd_set fileSet;
    FD_ZERO(&fileSet);
    FD_SET(fd_, &fileSet);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = ms * 1000;
    int n = select(fd_ + 1, &fileSet, NULL, &fileSet, &timeout);
    if (!n)
    {
        SERIAL_DEBUG("waitForReadyRead ... select timeout.");
        return false;
    }

    int queue = 0;
    if (n==-1 || ioctl(fd_, FIONREAD, &queue)==-1)
    {
        SERIAL_DEBUG("waitForReadyRead ... no queue.");
        return false;
    }

    return true;
}