 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>  
#include <unistd.h>  
#include <termios.h> 
#include <errno.h>

// --- Functions ---
int open_serial_port(const char* port_name);
int configure_serial_port(int fd);
void read_from_port(int fd);

int main() {
    char port_name[100];
    char command_buffer[100];
    int serial_port_fd;

    printf("--- PC Host for AVR Motor Controller ---\n");
    printf("Enter serial port name (e.g., /dev/ttyUSB0 on Linux, COM3 on Windows): ");
    scanf("%99s", port_name);

    // Open the serial port
    serial_port_fd = open_serial_port(port_name);
    if (serial_port_fd == -1) {
        return 1;
    }

    // Configure the serial port
    if (configure_serial_port(serial_port_fd) == -1) {
        close(serial_port_fd);
        return 1;
    }

    printf("Successfully connected to %s. Waiting for initial message from AVR...\n", port_name);
    
   
    sleep(2);
    read_from_port(serial_port_fd);

    // Main command loop
    while (1) {
        printf("\nEnter target RPM (or 'q' to quit): ");
        scanf("%s", command_buffer);

        if (strcmp(command_buffer, "q") == 0) {
            break;
        }

        // Append a newline character, as the AVR expects this to terminate the command
        strcat(command_buffer, "\n");

        ssize_t bytes_written = write(serial_port_fd, command_buffer, strlen(command_buffer));
        if (bytes_written < 0) {
            perror("Error writing to serial port");
        } else {
            printf("Sent command: %s", command_buffer);
        }
        
        usleep(100000); // Wait 100ms
        
        // Read and display the response
        printf("AVR Response:\n");
        read_from_port(serial_port_fd);
    }

    // Close the port
    close(serial_port_fd);
    printf("\nSerial port closed. Exiting.\n");

    return 0;
}

int open_serial_port(const char* port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        fprintf(stderr, "Error: Unable to open serial port '%s'. %s.\n", port_name, strerror(errno));
    } else {
        
        fcntl(fd, F_SETFL, 0);
        printf("Successfully opened port: %s\n", port_name);
    }
    return fd;
}


int configure_serial_port(int fd) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    // Set Baud Rate
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // Set Control Modes
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;  
    tty.c_cflag |= CS8;     
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // Set Local Modes
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    // Set Input Modes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // Set Output Modes
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // Set blocking read with a 0.5-second timeout
    tty.c_cc[VTIME] = 5;
    tty.c_cc[VMIN] = 0;

    // Apply the attributes
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

void read_from_port(int fd) {
    char read_buf[256];
    memset(read_buf, '\0', sizeof(read_buf));

    int num_bytes = read(fd, &read_buf, sizeof(read_buf) - 1);

    if (num_bytes < 0) {
        // Don't print an error if timeout
        if (errno != EAGAIN) {
             perror("Error reading from serial port");
        }
    } else if (num_bytes > 0) {
        printf("%s", read_buf);
        fflush(stdout);
    }
}
