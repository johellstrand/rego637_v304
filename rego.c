#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions        */
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>
#include <sys/select.h>
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

typedef enum {
    REGO_TYPE_NONE=0,
    REGO_TYPE_TEMP,
    REGO_TYPE_STATUS,
    REGO_TYPE_COUNTER,
    REGO_TYPE_ERROR_LINE
} RegoRegType;

typedef struct
{
    char        name[25];
    uint16_t    reg;
    RegoRegType type;
    RegoCommandType command_type;
    uint16_t    request_len;
    uint16_t    expected_reply_len;
    int16_t     reply_value;
    time_t      time;
    int         updateable;
} RegoCommand;

static RegoCommand rego_commands[] = {
    { "GT1 Radiator",        0x0209, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT2 Out",             0x020A, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT3 Hot water",       0x020B, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT4 Forward",         0x020C, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT5 Room",            0x020D, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT6 Compressor",      0x020E, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT8 Hot fluid out",   0x020F, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT9 Hot fluid in",    0x0210, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT10 Cold fluid in",  0x0211, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT11 Cold fluid out", 0x0212, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT3x Ext hot water",  0x0213, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT1 Target",          0x006E, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT1 On",              0x006F, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT1 Off",             0x0070, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT3 On",              0x0073, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT3 Off",             0x0074, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "GT4 Target",          0x006D, REGO_TYPE_TEMP,       RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "P3 Cold fluid",       0x01FD, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "Compressor",          0x01FE, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "Xtra 3kW",            0x01FF, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "Xtra 6kW",            0x0200, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "P1 Radiator",         0x0203, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "P2 Heat fluid",       0x0204, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "Three-way valve",     0x0205, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "Alarm",               0x0206, REGO_TYPE_STATUS,     RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "Heat-Power",          0x006C, REGO_TYPE_COUNTER,    RC_read_from_system_register, 9, 5, 0, 0, 0 },
    { "Heatcurve",           0x0000, REGO_TYPE_COUNTER,    RC_read_from_system_register, 9, 5, 0, 0, 1 },
    { "Heatcurve fine adj.", 0x0001, REGO_TYPE_COUNTER,    RC_read_from_system_register, 9, 5, 0, 0, 1 },
    { "Last Error",          0x0000, REGO_TYPE_ERROR_LINE, RC_read_last_error_line,      9,42, 0, 0, 0 },
    { "Prev Last Error",     0x0000, REGO_TYPE_ERROR_LINE, RC_read_prev_error_line,      9,42, 0, 0, 0 },
    { "LED Power",           0x0012, REGO_TYPE_STATUS,     RC_read_from_front_panel,     9, 5, 0, 0, 0 },
    { "LED Compressor",      0x0013, REGO_TYPE_STATUS,     RC_read_from_front_panel,     9, 5, 0, 0, 0 },
    { "LED El Tillskott",    0x0014, REGO_TYPE_STATUS,     RC_read_from_front_panel,     9, 5, 0, 0, 0 },
    { "LED Varmvatten",      0x0015, REGO_TYPE_STATUS,     RC_read_from_front_panel,     9, 5, 0, 0, 0 },
    { "LED Alarm",           0x0016, REGO_TYPE_STATUS,     RC_read_from_front_panel,     9, 5, 0, 0, 0 },
    { "Firmware",            0x0000, REGO_TYPE_STATUS,     RC_read_firmware_version,     9, 5, 0, 0, 0 },
    
};




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

// returns bytes written.
static int send_request( int fd, RegoCommand* rc_p )
{
    RegoRequest req;
    
    if( sizeof( req ) != 9 ) exit( 1 );
    
    memset( &req, 0, sizeof( req ) );
    
    req.data.address = 0x81;
    req.data.command_type = rc_p->command_type;
    
    req.data.regNum[0] = (rc_p->reg & 0xC000) >> 14 ;
    req.data.regNum[1] = (rc_p->reg & 0x3F80) >> 7 ;
    req.data.regNum[2] = rc_p->reg & 0x007F;
    
    for ( int i = 2; i < 8; i++ )
        req.data.crc ^= req.raw[ i ];
    
    
    printf("Request: ");
    for(int i = 0; i < sizeof( req ); i++ )
    {
        printf("%02x", req.raw[i] );
    }
    puts("");
    
    rc_p->time = time( NULL );
    return write( fd, &req, sizeof( req ) );
}

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
    
    printf("Reply: ");
    for(int i = 0; i < sizeof( *rp ); i++ )
    {
        printf("%02x", rp->raw[i] );
    }
    puts("");
    
    return 1;
}

typedef union
{
    uint8_t raw[42];
    struct
    {
        uint8_t address; // 0x01
        uint8_t value[3];
        uint8_t crc;
    } data;
} Rego42bReply;

static int parse_42byte_reply(const Rego42bReply* rp, int16_t* value )
{
    //  uint8_t crc = 0;
    
    if( sizeof( *rp ) != 42 ) exit( 1 );
    
    *value = 0;
    if( rp->data.address != 0x01 ) return 0;
    
    /*
     for ( int i = 0; i < sizeof( rp->data.value); i++ ) crc ^= rp->data.value[i];
     
     if( crc != rp->data.crc ) return 0;
     
     *value = (rp->data.value[0] << 14) | (rp->data.value[1] << 7) | rp->data.value[2];
     */
    printf("Reply: ");
    for(int i = 0; i < sizeof( *rp ); i++ )
    {
        printf("%02x", rp->raw[i] );
    }
    puts("");
    
    return 1;
}

static int open_serial(const char* port_p)
{
    int fd;/*File Descriptor*/
    
    /*------------------------------- Opening the Serial Port -------------------------------*/
    
    /* Change /dev/ttyUSB0 to the one corresponding to your system */
    
    fd = open( port_p, O_RDWR | O_NOCTTY | O_NDELAY);    /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
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

int main( int argc, char* argv[] )
{
    int exit_status = EXIT_FAILURE;
    int idx = 1;
    
    if( argc > 1 )
    {
        idx = -1;
        for( int i = 0; idx == -1 && i < sizeof( rego_commands ) / sizeof( rego_commands[0] ); i++ )
        {
            if( !strcmp( argv[1], rego_commands[i].name ) ) idx = i;
        }
        if( idx == -1 )
        {
            printf(" Invalid option %s\n",  argv[1] ) ;
            return exit_status;
        }
        printf( "Continuing with %s, \n", rego_commands[idx].name );
    }
    else
    {
        puts("one of these commands must be entered:");
        for( int i = 0; i < sizeof( rego_commands ) / sizeof( rego_commands[0] ); i++ )
        {
            puts( rego_commands[i].name );
        }
        return exit_status;
    }
    
    int fd = open_serial( "/dev/ttyUSB0" );
    
    puts("");
    
    if( fd >= 0 )
    {
        RegoCommand* rc_p = &rego_commands[idx]; // GT2.
        int bytes_written = send_request( fd, rc_p );
        
        if( bytes_written == rc_p->request_len )
        {
            uint8_t buff[100];
            
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
                    printf("Data is available now.\n");
                /* FD_ISSET(0, &rfds) will be true. */
                else
                    printf("..\n");
            }
            int bytes_read = read( fd, buff, sizeof( buff ) );
            if( bytes_read == rc_p->expected_reply_len )
            {
                if( bytes_read == 5 )
                {
                    if( parse_5byte_reply( (const Rego5bReply*) buff, &rc_p->reply_value ) )
                    {
                        printf("Current value of %s is %d\n", rc_p->name, rc_p->reply_value );
                        exit_status = EXIT_SUCCESS;
                    }
                    else
                        printf("Parse of reply to %s failed\n", rc_p->name );
                }
                else if( bytes_read == 42 )
                {
                    if( parse_42byte_reply( (const Rego42bReply*) buff, &rc_p->reply_value ) )
                    {
                        printf("Current value of %s is %d\n", rc_p->name, rc_p->reply_value );
                        exit_status = EXIT_SUCCESS;
                    }
                    else
                        printf("Parse of reply to %s failed\n", rc_p->name );
                    
                }
                else
                    printf("Unexpectedly only got %d bytes\n", bytes_read );
            }
            else
            {
                printf("Unexpectedly only got %d bytes\n", bytes_read );
                if( bytes_read == -1 ) perror( "read failed");
            }
        }
        close( fd );
    }
    return exit_status;
}
