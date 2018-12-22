#ifndef REGO_FUNC__H
#define REGO_FUNC__H

#include <stdint.h>

int open_serial(const char* port_p);
void close_serial( int fd );


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
} RegoSystemRegister;
#if 0
    RR_lasterror          = 0x0000,
    RR_lasterrorprev      = 0x0000,
    RR_firmware           = 0x0000,
} RegoRegister;
#endif

// returns INT16_MIN on error.
int16_t read_system_register( int fd, RegoSystemRegister rr );
void write_system_register( int fd, RegoSystemRegister rr, int16_t value );


typedef enum
{
    RR_ledpower           = 0x0012,
    RR_ledcompressor      = 0x0013,
    RR_ledextra           = 0x0014,
    RR_ledhotwater        = 0x0015,
    RR_ledalarm           = 0x0016,
} RegoFrontPanel;
int16_t read_front_panel( int fd, RegoFrontPanel rr );


typedef enum
{
    REC_GT1SensorRadiatorReturn = 0,
    REC_GTOutdoorSensor = 1,
    REC_GT3HotWaterSensor = 2,
    REC_GT4MixingValveSensor = 3,
    REC_GT5RoomSensor = 4, // Fasföljdsfel. ?
    REC_GT6CompressorSensor = 5,
    REC_GT8HeatFluidOut = 6, // GT8/GT9 Högt värmebärardelta.
    REC_GT9HeatFluidIn = 7,
    REC_GT10ColdFluidIn = 8,
    REC_GT11ColdFluidOut = 9,
    REC_CompressorCircuitSwitch = 10,
    REC_ElectricalCassette = 11,
    REC_MB2PumpSwith = 12,
    REC_LowPressureSwitch = 13, // LP Pressostat låg.
    REC_HighPressureSwitch = 14,
    REC_GT9HighReturn = 15, // GT9 Hög retur.
    REC_GT8HTFOutMax = 16,
    REC_GT10HTFInUnderLimit = 17,
    REC_GT11HTFOutUnderLimit = 18,
    REC_GT6CompressorSuperhHear = 19,
    REC_3PhaseIncorrect = 20,
    REC_PowerFailure = 21,
    REC_HighDelta = 22
} RegoErrCode;

static char errcode_to_string[23][40] ={
    "GT1SensorRadiatorReturn",
    "GTOutdoorSensor",
    "GT3HotWaterSensor",
    "GT4MixingValveSensor",
    "Fasföljdsfel",
    "GT6CompressorSensor",
    "GT8/GT9 Högt värmebärardelta",
    "GT9HeatFluidIn",
    "GT10ColdFluidIn",
    "GT11ColdFluidOut",
    "CompressorCircuitSwitch",
    "ElectricalCassette",
    "MB2PumpSwith",
    "LP Pressostat låg",
    "HighPressureSwitch",
    "GT9 Hög retur",
    "GT8HTFOutMax",
    "GT10HTFInUnderLimit",
    "GT11HTFOutUnderLimit",
    "GT6CompressorSuperhHear",
    "3PhaseIncorrect",
    "PowerFailure",
    "HighDelta"
};

#include <time.h>
typedef struct
{
    time_t      occurence_time;
    RegoErrCode code;
} RegoError;


// returns 0 on failure.
int get_last_error( int fd, RegoError* re_p );
int get_prev_to_last_error( int fd, RegoError* re_p );

#endif
