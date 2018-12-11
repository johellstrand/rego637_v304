#include <stdio.h>
#include <stdlib.h>
#include "rego_funcs.h"



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

#include <string.h>
#include <unistd.h>
static Rego5bReply* format_reply( Rego5bReply* req_p, int16_t value )
{
    if( sizeof( *req_p ) != 5 ) exit( 1 );
    
    memset( req_p, 0, sizeof( *req_p ) );
    
    req_p->data.address = 0x01;
    
    req_p->data.value[0] = (value & 0xC000) >> 14 ;
    req_p->data.value[1] = (value & 0x3F80) >> 7 ;
    req_p->data.value[2] = value & 0x007F;
    
    for ( int i = 0; i < 3; i++ )
        req_p->data.crc ^= req_p->data.value[ i ];
    
    return req_p;
}

int serial_read( int fd, uint8_t *buff_p, size_t buffsize);
int server_open_serial(const char* port_p);

int main( int argc, char* argv[] )
{
    int          exit_status = EXIT_FAILURE;
    
#ifdef __APPLE__
    int fd = server_open_serial( "/dev/ptyp3" );
#else
    int fd = -1;
#endif
    
    puts("");
    
    if( fd >= 0 )
    {
        uint8_t buff[100];
        Rego5bReply rpy;
        int num_read = 1;
        while( num_read )
        {
            num_read = serial_read( fd, buff, sizeof( buff ) );
            if( num_read > 0 )
            {
                format_reply( &rpy, 123 );
                if( write( fd, &rpy, sizeof( rpy ) )== sizeof( rpy ) )
                {
                    printf("Sent reply: ");
                    for(int i = 0; i < sizeof( rpy ); i++ )
                    {
                        printf("%02x", rpy.raw[i] );
                    }
                    puts("");
                    
                }
            }
            else
            {
                perror( "read failed" );
                num_read = 0;
            }
        }
        close_serial( fd );
    }
    return exit_status;
}
