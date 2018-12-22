#include <stdio.h>
#include <stdlib.h>
#include "rego_funcs.h"

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


#if 0

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
    
    return 1;
}
#endif

int main( int argc, char* argv[] )
{
    int                exit_status = EXIT_FAILURE;
    int                delta_adjustment = 0;
    int16_t            current_value;
    RegoSystemRegister rr = RR_heatcurve;
    RegoError          re;

    if( argc > 1 ) delta_adjustment = atoi( argv[1] );
    
#ifdef __APPLE__
    int fd = open_serial( "/dev/ttyp3" );
#else
    int fd = open_serial( "/dev/ttyUSB0" );
#endif
    
    puts("");
    
    if( fd >= 0 )
    {
        current_value = read_system_register( fd, rr );
        if( current_value != INT16_MIN )
        {
            exit_status = EXIT_SUCCESS;
            printf( "Current value of heatcurve is %d\n", current_value );
            if( delta_adjustment )
            {
                current_value +=delta_adjustment;
            
                if( current_value >= 0 || current_value <= 100 )
                {
                    write_system_register( fd, rr, current_value );
                    
                    // re-read the new value.
                    current_value = read_system_register( fd, rr );
                    printf( "Current value of heatcurve is now %d\n", current_value );
                }
            }
        }
        
        // show last errors.
        if( get_last_error( fd, &re ) )
        {
            int v = 1;
            char buf[100];
            strftime(buf, sizeof buf, "%FT%TZ", gmtime(&re.occurence_time));
            printf("\t%s last error [%d:%s]\n", buf, re.code, errcode_to_string[re.code] );
            for(int i = 0; i < 3 && v; i++)
            {
                v = get_prev_to_last_error( fd, &re );
                if( v )
                {
                    strftime(buf, sizeof buf, "%FT%TZ", gmtime(&re.occurence_time));
                    printf("\t%s Prev error [%d:%s]\n", buf, re.code, errcode_to_string[re.code] );
                }
            }
        }

        close_serial( fd );
    }
    return exit_status;
}
