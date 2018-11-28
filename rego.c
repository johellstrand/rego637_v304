#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions        */
#include <errno.h>   /* ERROR Number Definitions           */


typedef enum {
    RC_read_from_front_panel     = 0,    // response 5 character, 16 bit number
    RC_write_to_front_panel      = 1,    // 1 character, confirm
    RC_read_from_system_register = 2,    // response 5 character, 16 bit number
    RC_write_to_system_register  = 3,    // 1 character, confirm
    RC_read_from_timer_resgister = 4,    // response 5 character, 16 bit number
    RC_write_to_timer_register   = 5,    // 1 character, confirm
    RC_read_from_display         = 0x20, // response 42char text line
    RC_read_last_error_line      = 0x40, // response 42char text line
    RC_read_prev_error_line      = 0x42  // response 42char text line
} RegoCommandType;


typedef uint8_t RegoCommandType_u8;

typedef enum {
    REGO_TYPE_NONE=0,
    REGO_TYPE_TEMP,
    REGO_TYPE_STATUS,
    REGO_TYPE_COUNTER,
    REGO_TYPE_END
} RegoRegType;

typedef struct _tRegoRegisters
{
    char name[25];
    unsigned short regNum_type1;
    unsigned short regNum_type2;
    unsigned short regNum_type3;
    RegoRegType type;
    float lastTemp;
    int lastValue;
    time_t lastSent;
} RegoRegisters;


RegoRegisters g_allRegisters[] = {
    { "GT1 Radiator",           0x0209,	0x020B,	0x020D,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT2 Out",		        0x020A,	0x020C,	0x020E,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT3 Hot water",	        0x020B,	0x020D,	0x020F,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT4 Forward",	        0x020C,	0x020E,	0x0210,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT5 Room",			    0x020D,	0x020F,	0x0211,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT6 Compressor",	        0x020E,	0x0210,	0x0212,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT8 Hot fluid out",      0x020F,	0x0211,	0x0213,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT9 Hot fluid in",		0x0210,	0x0212,	0x0214,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT10 Cold fluid in",		0x0211,	0x0213,	0x0215,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT11 Cold fluid out",	0x0212,	0x0214,	0x0216,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT3x Ext hot water",		0x0213, 0x0215, 0x0217,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "P3 Cold fluid",	    	0x01FD,	0x01FF,	0x0201,	REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "Compressor",				0x01FE,	0x0200,	0x0202,	REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "Add heat 1",     		0x01FF,	0x0201,	0x0203,	REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "Add heat 2",		        0x0200,	0x0202,	0x0204,	REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "P1 Radiator",    		0x0203,	0x0205,	0x0207,	REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "P2 Heat fluid",  		0x0204, 0x0206, 0x0208, REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "Three-way valve",        0x0205, 0x0207, 0x0209, REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "Alarm",                  0x0206, 0x0208, 0x020A, REGO_TYPE_STATUS,       -50.0, -1, 0 },
    { "Operating hours",        0x0046, 0x0048, 0x004A, REGO_TYPE_COUNTER,      -50.0, -1, 0 },
    { "Radiator hours",         0x0048, 0x004A, 0x004C, REGO_TYPE_COUNTER,      -50.0, -1, 0 },
    { "Hot water hours",        0x004A, 0x004C, 0x004E, REGO_TYPE_COUNTER,      -50.0, -1, 0 },
    { "GT1 Target",             0x006E,	0x006E,	0x006E,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT3 Target",             0x002B,	0x002B,	0x002B,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "GT4 Target",             0x006D,	0x006D,	0x006D,	REGO_TYPE_TEMP,         -50.0, -1, 0 },
    { "",                            0,      0,      0, REGO_TYPE_NONE,             0,  0, 0}
};

typedef union
{
    unsigned char raw[9];
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
    char raw[5];
    struct
    {
        uint8_t address; // 0x01
        uint8_t value[3];
        uint8_t crc;
    } data;
} Rego5bReply;


static void format_request( RegoRequest* rp, RegoCommandType ct, unsigned short reg)
{
    memset( rp, 0, sizeof( *rp ) );
    if( sizeof( *rp ) != 9 ) exit( 1 );
    rp->data.address = 0x81;
    rp->data.command_type = ct;
    
    rp->data.regNum[0] = (reg & 0xC000) >> 14 ;
    rp->data.regNum[1] = (reg & 0x3F80) >> 7 ;
    rp->data.regNum[2] = reg & 0x007F;
    
    for ( int i = 2; i < 8; i++ )
        rp->data.crc ^= rp->raw[ i ];
    
    
    printf("Request: ");
    for(int i = 0; i < sizeof( *rp ); i++ )
    {
        printf("%02x", rp->raw[i] );
    }
    puts("");
    
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
    
    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
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
    RegoRequest req;
    
    int fd = open_serial( "/dev/ttyUSB0" );
    
  //  if( fd >= 0 )
    {
        
        format_request( &req, RC_read_from_system_register, 0x0001 ); //GT2 out
        
        int  bytes_written  = 0;      /* Value for storing the number of bytes written to the port */
        
        bytes_written = write( fd, &req, sizeof( req ) ) ;
        
        {
            Rego5bReply rpy;
            int16_t     v;
            
            rpy.data.address = 0x01;
            rpy.data.value[0] = 0;
            rpy.data.value[1] = 0x02;
            rpy.data.value[2] = 0x2c;
            rpy.data.crc = 0;
            
            for ( int i = 0; i < sizeof( rpy.data.value ); i++ )
                rpy.data.crc ^= rpy.data.value[ i ];
            
            
            
            if( parse_5byte_reply( &rpy, &v ) )
                printf("Value is %d\n", v );
        }
        close( fd );
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
