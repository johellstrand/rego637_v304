#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions        */
#include <stdint.h>
#include <sys/select.h>
#include "rego_funcs.h"

typedef enum {
    RC_read_from_front_panel     = 0,    // response 5 character, 16 bit number
    RC_write_to_front_panel      = 1,    // 1 character, confirm
    RC_read_from_system_register = 2,    // response 5 character, 16 bit number
    RC_write_to_system_register  = 3,    // 1 character, confirm
    RC_read_from_timer_resgister = 4,    // response 5 character, 16 bit number
    RC_write_to_timer_register   = 5,    // 1 character, confirm
    RC_read_from_display         = 0x20, // response 42char text line
    RC_read_last_error_line      = 0x40, // response 42char text line
    RC_read_prev_error_line      = 0x42, // response 42char text line
    RC_read_firmware_version     = 0x75  // response 5 character, 16 bit number
} RegoCommandType;
typedef uint8_t RegoCommandType_u8;


int server_open_serial(const char* port_p)
{
    int fd;/*File Descriptor*/
    
    /*------------------------------- Opening the Serial Port -------------------------------*/
    
    /* Change /dev/ttyUSB0 to the one corresponding to your system */
    
    fd = open( port_p, O_RDWR | O_NOCTTY  );    /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
    /* O_RDWR Read/Write access to serial port           */
    /* O_NOCTTY - No terminal will control the process   */
    /* O_NDELAY -Non Blocking Mode,Does not care about-  */
    /* -the status of DCD line,Open() returns immediatly */
    
    if(fd == -1)
    {
        perror( "open failed" );
        return -1;
    }
    else
        printf("%s Opened Successfully\n", port_p );
    
    
    /*---------- Setting the Attributes of the serial port using termios structure --------- */
    
    struct termios SerialPortSettings;    /* Create the structure                          */
    
    tcgetattr(fd, &SerialPortSettings);    /* Get the current attributes of the Serial port */
    
    cfsetispeed(&SerialPortSettings,B19200); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,B19200); /* Set Write Speed as 9600                       */
    
//    cfmakeraw( &SerialPortSettings );
    
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 20;

    // set vmin and vtime. vtime to 20 and vmin to 1.
    // maybe use cfmakeraw() instaed?
    
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
    {
        close(fd);/* Close the Serial port */
        printf("\n  ERROR ! in Setting attributes");
        fd = -1;
    }
    else
        printf("\n  BaudRate = 19200 \n  StopBits = 1 \n  Parity   = none");
    
    return fd;
}
int open_serial(const char* port_p)
{
	    int fd;/*File Descriptor*/
    
    /*------------------------------- Opening the Serial Port -------------------------------*/
    
    /* Change /dev/ttyUSB0 to the one corresponding to your system */
    
    fd = open( port_p, O_RDWR | O_NOCTTY );    /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
    /* O_RDWR Read/Write access to serial port           */
    /* O_NOCTTY - No terminal will control the process   */
    /* O_NDELAY -Non Blocking Mode,Does not care about-  */
    /* -the status of DCD line,Open() returns immediatly */
    
    if(fd == -1)
    {
        perror( "open failed" );
        return -1;
    }
    else
        printf("%s Opened Successfully\n", port_p );
    
    
    /*---------- Setting the Attributes of the serial port using termios structure --------- */
    
    struct termios SerialPortSettings;    /* Create the structure                          */
    
    tcgetattr(fd, &SerialPortSettings);    /* Get the current attributes of the Serial port */
    
    cfsetispeed(&SerialPortSettings,B19200); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,B19200); /* Set Write Speed as 9600                       */
    
    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;     /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
    
    //    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */
    
    
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
    
    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 20;

    // set vmin and vtime. vtime to 20 and vmin to 1.
    // maybe use cfmakeraw() instaed?
    
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
    {
        close(fd);/* Close the Serial port */
        printf("\n  ERROR ! in Setting attributes");
        fd = -1;
    }
    else
        printf("\n  BaudRate = 19200 \n  StopBits = 1 \n  Parity   = none");
    
    return fd;
}

void close_serial( int fd )
{
    close( fd );
}

void wait_for_response( int fd )
{
    fd_set rfds;
    struct timeval tv;
    int retval;
    
    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    
    /* Wait up to five seconds. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    
    retval = select(1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */
    
    if (retval == -1)
        perror("select()");
    else if (retval)
        fprintf( stderr, "Data is available now.\n");
    /* FD_ISSET(0, &rfds) will be true. */
    else
        fprintf( stderr, "..\n");
}


int serial_read( int fd, uint8_t *buff_p, size_t buffsize)
{
    int bytes_read = read( fd, buff_p, buffsize );
    fprintf( stderr, "read: ");
    for(int i = 0; i < bytes_read; i++, buff_p++ )
    {
        fprintf( stderr, "%02x", *buff_p );
    }
    fprintf( stderr, "\n");
    
    return bytes_read;
}

typedef union
{
    uint8_t raw[9];
    struct
    {
        uint8_t address; // 0x81
        RegoCommandType_u8 command_type;
        uint8_t regNum[3];
        uint8_t value[3];
        uint8_t crc;
    } data;
} RegoRequest;

typedef union
{
    uint8_t raw[5];
    struct
    {
        uint8_t address; // 0x01
        uint8_t value[3];
        uint8_t crc;
    } data;
} Rego5bReply;

static RegoRequest* format_request( RegoRequest* req_p, RegoCommandType ct, int rego_register, int16_t value )
{
    if( sizeof( *req_p ) != 9 ) exit( 1 );
    
    memset( req_p, 0, sizeof( *req_p ) );
    
    req_p->data.address = 0x81;
    req_p->data.command_type = ct;
    
    req_p->data.regNum[0] = (rego_register & 0xC000) >> 14 ;
    req_p->data.regNum[1] = (rego_register & 0x3F80) >> 7 ;
    req_p->data.regNum[2] = rego_register & 0x007F;
    
    req_p->data.value[0] = (value & 0xC000) >> 14 ;
    req_p->data.value[1] = (value & 0x3F80) >> 7 ;
    req_p->data.value[2] = value & 0x007F;
    
    for ( int i = 2; i < 8; i++ )
        req_p->data.crc ^= req_p->raw[ i ];
    
    return req_p;
}

static int send_request( int fd, const RegoRequest* req_p )
{
    fprintf( stderr, "write: ");
    for(int i = 0; i < sizeof( *req_p ); i++ )
    {
        fprintf( stderr, "%02x", req_p->raw[i] );
    }
    fprintf( stderr, "\n");
    
    return write( fd, req_p, sizeof( *req_p ) );
}
static int parse_5byte_reply(const Rego5bReply* rp, int16_t* value )
{
    uint8_t crc = 0;
    
    if( sizeof( *rp ) != 5 ) exit( 1 );
    
    *value = 0;
    if( rp->data.address != 0x01 ) return 0;
    
    for ( int i = 0; i < sizeof( rp->data.value); i++ ) crc ^= rp->data.value[i];
    
    if( crc != rp->data.crc ) return 0;
    
    *value = (rp->data.value[0] << 14) | (rp->data.value[1] << 7) | rp->data.value[2];
    
    if( rp->data.value[0] >= 2 ) (*value) -= 65536;
    
    return 1;
}

static void parse_1byte_reply( uint8_t* rpy_p )
{
    
}
// returns INT16_MIN on error.
int16_t read_system_register( int fd, RegoSystemRegister rr )
{
    RegoRequest req;
    int16_t     returned_value;
    int16_t     value = INT16_MIN;
    uint8_t     response[100];
    
    format_request( &req, RC_read_from_system_register, rr, 0 );
    
    int bytes_written = send_request( fd, &req );
    
    if( bytes_written == sizeof( req ) )
    {
        wait_for_response( fd );
        
        int bytes_read = serial_read( fd, response, sizeof( response ) );
        if( bytes_read == 5 )
            if( parse_5byte_reply( (const Rego5bReply*) response, &returned_value ) ) value = returned_value;
    }
    else
    {
        perror("write failed in read_system_register()");
    }
    
    return value;
}


void write_system_register( int fd, RegoSystemRegister rr, int16_t value )
{
    RegoRequest req;
    uint8_t     response[100];
    
    format_request( &req, RC_write_to_system_register, rr, value );
    
    int bytes_written = send_request( fd, &req );
    
    if( bytes_written == sizeof( req ) )
    {
        wait_for_response( fd );
        
        int bytes_read = serial_read( fd, response, sizeof( response ) );
        if( bytes_read == 1 ) parse_1byte_reply( response );
    }
    else
    {
        perror("write failed in write_system_register()");
    }
}

int16_t read_front_panel( int fd, RegoFrontPanel rr )
{
    RegoRequest req;
    int16_t     returned_value;
    int16_t     value = INT16_MIN;
    uint8_t     response[100];
    
    format_request( &req, RC_read_from_front_panel, rr, 0 );
    
    int bytes_written = send_request( fd, &req );
    
    if( bytes_written == sizeof( req ) )
    {
        wait_for_response( fd );
        
        int bytes_read = serial_read( fd, response, sizeof( response ) );
        if( bytes_read == 5 )
            if( parse_5byte_reply( (const Rego5bReply*) response, &returned_value ) ) value = returned_value;
    }
    else
    {
        perror("write failed in read_front_panel()");
    }

    return value;
}

static int pair_to_int( const uint8_t* p )
{
    return 10* (*(p+1)) + (*(p+3));
}
static time_t parse_datetime( const uint8_t* dt_p )
{
    struct tm dt;
    
    memset( &dt, 0, sizeof( dt ) );
    dt.tm_year = 100 + pair_to_int( dt_p );
    dt_p +=4;
    dt.tm_mon = pair_to_int( dt_p ) - 1;
    dt_p +=4;
    dt.tm_mday = pair_to_int( dt_p );
    dt_p +=6;
    
    dt.tm_hour = pair_to_int( dt_p );
    dt_p +=6;
    dt.tm_min = pair_to_int( dt_p );
    dt_p +=6;
    dt.tm_sec = pair_to_int( dt_p );
    dt_p +=6;
    
    time_t ret = mktime( &dt );
    return ret;
}

static int get_vp_error( int fd, RegoCommandType ct, RegoError* re_p )
{
    RegoRequest req;
    int16_t     value = 0;
    uint8_t     response[100];
    
    format_request( &req, ct, 0, 0 );
    
    int bytes_written = send_request( fd, &req );
    
    if( bytes_written == sizeof( req ) )
    {
        wait_for_response( fd );
        
        int bytes_read = serial_read( fd, response, sizeof( response ) );
        if( bytes_read == 42 )
        {
            value = 1;
            re_p->code = response[2];
            re_p->occurence_time = parse_datetime( response + 3 );
        }
    }
    else
    {
        perror("write failed in get_last_error()");
    }
    return value;
}
int get_last_error( int fd, RegoError* re_p )
{
    return get_vp_error( fd, RC_read_last_error_line, re_p );
}

int get_prev_to_last_error( int fd, RegoError* re_p )
{
    return get_vp_error( fd, RC_read_prev_error_line, re_p );
}


