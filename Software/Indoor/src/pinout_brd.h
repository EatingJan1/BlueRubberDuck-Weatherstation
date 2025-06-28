#ifndef PINOUT_H
#define PINOUT_H


#define ssid_d "mein"
#define password_d "passwort"

#define SDI 19
#define CLK 18

#define AL_light true
#define AL_ADDR 0x29

#define BME_temp true
#define BME_hum true
#define BME_pres true

#define BME_ADDR 0x76

#define co2_messure false
#define TX_CO2 7
#define RX_CO2 15


#define Button_Pin 4

#define DS_temp true
#define Temp_Pin 5

#define Rain_messure true
#define Rain_detect true
#define Reed1_Pin 3
#define Reed2_Pin 2


#define wind_messure true
#define Trigger_1 20
#define Trigger_2 15
#define Trigger_3 7
#define Trigger_4 1

#define Echo_1 8
#define Echo_2 6
#define Echo_3 14
#define Echo_4 22

#define ledmode 3 //[none, temp, hemidity, co2]
#define WS2812_Pin 20
#define NUM_PIXELS 6


#endif