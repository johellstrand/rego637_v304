#ifndef REGO_FUNC__H
#define REGO_FUNC__H

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


typedef enum
{
    RR_GT1_radiator       = 0x0209,
    RR_GT2_out            = 0x020A,
    RR_GT3_hotwater       = 0x020B,
    RR_GT4_forward        = 0x020C,
    RR_GT5_room           = 0x020D,
    RR_GT6_compressor     = 0x020E,
    RR_GT8_hotfluidout    = 0x020F,
    RR_GT9_hotfluidin     = 0x0210,
    RR_GT10_coldfluidin   = 0x0211,
    RR_GT11_coldfluidout  = 0x0212,
    RR_GT13_exthotwater   = 0x0213,
    RR_GT1_target         = 0x006E,
    RR_GT1_on             = 0x006F,
    RR_GT1_off            = 0x0070,
    RR_GT3_on             = 0x0073,
    RR_GT3_off            = 0x0074,
    RR_GT4_target         = 0x006D,
    RR_P3_coldfluid       = 0x01FD,
    RR_compressor         = 0x01FE,
    RR_xtra3kw            = 0x01FF,
    RR_xtra6kw            = 0x0200,
    RR_P1_radiator        = 0x0203,
    RR_P2_heatfluid       = 0x0204,
    RR_3WAYvalve          = 0x0205,
    RR_alarm              = 0x0206,
    RR_heatpower          = 0x006C,
    RR_heatcurve          = 0x0000,
    RR_heatcurve_fineadj  = 0x0001,
    RR_lasterror          = 0x0000,
    RR_lasterrorprev      = 0x0000,
    RR_ledpower           = 0x0012,
    RR_ledcompressor      = 0x0013,
    RR_ledextra           = 0x0014,
    RR_ledhotwater        = 0x0015,
    RR_ledalarm           = 0x0016,
    RR_firmware           = 0x0000,
} RegoRegister;

int open_serial(const char* port_p);

void wait_for_response( int fd );

// returns INT16_MIN on error.
int16_t read_system_register( int fd, RegoRegister rr );

void write_system_register( int fd, RegoRegister rr, int16_t value );

int serial_read( int fd, uint8_t *buff_p, size_t buffsize);
#endif
