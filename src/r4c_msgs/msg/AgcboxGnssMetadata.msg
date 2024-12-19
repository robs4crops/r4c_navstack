std_msgs/Header header

uint8 num_satellites
float64 hdop
float64 pdop
uint8 num_reference_stations
uint16 reference_station_id

uint8 NO_GNSS=0
uint8 GNSS_FIX=1
uint8 DGNSS_FIX=2
uint8 PRECISE_GNSS=3
uint8 RTK_FIXED_INTEGER=4
uint8 RTK_FLOAT=5
uint8 ESTIMATED_DR_MODE=6
uint8 MANUAL_INPUT=7
uint8 SIMULATE_MODE=8
uint8 GNSS_ERROR=14

uint8 gnss_method