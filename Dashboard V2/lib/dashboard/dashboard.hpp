/*  Written by Baptiste Savioz
 *  July 2023
 *  
 *  Library made to read and process all GPIO's
 */

#ifndef DASHBOARD_HPP
#define DASHBOARD_HPP

#include <arduino.h>

// Potentiometre (analog input)
#define SPEED_POT_PIN       PIN2_ADC1  //PB06
#define TUNE1_POT_PIN       PIN0_ADC0  //PA02
#define TUNE2_POT_PIN       PIN1_ADC0  //PA03

// Selecteur (digital input)
#define MODE_TEST_PIN       PB03
#define MODE_HARBOR_PIN     PB02
#define MODE_GOING_PIN      PB01
#define MODE_FOIL_1_PIN     PB00
#define MODE_FOIL_2_PIN     PB31
#define MODE_FOIL_3_PIN     PB30

// Switches & button (digital input)
#define HP_SWITCH_PIN       PB09        // Low active
#define ARMING_SWITCH_PIN   PA04
#define DEADMAN_SWITCH_PIN  PA05        // Low active
#define TAKEOFF_BUTTON_PIN  PB05        // Attention C'est le même pin que la blink LED !!! Ne jamais l'utiliser en output ! TakeOff buton : Low active

// LED's (digital output)
#define ARMING_LED_PIN      PB23      
#define HP_LED_PIN          PB22

#define ADC_MAX             1023


enum class flightMode {
	TEST   = 0,
	HARBOR = 1,
	GOING  = 2,
	FOIL1  = 3,
    FOIL2  = 4,
    FOIL3  = 5,
};

class Dashboard
{
    public:
        Dashboard(bool debug); // constructor
        void init();
        void read();
        void updateLED(bool highPowerStatus, bool armingStatus);
        int8_t speedCommand   = 0;
        int8_t tuningCommand1 = 0;
        int8_t tuningCommand2 = 0;
        int8_t HPSwitch       = 0;
        int8_t armingSwitch   = 0;
        int8_t deadManSwitch  = 0;
        int8_t takeOffButton  = 0;
        //int8_t HomingButton   = 0;
        flightMode selector   = flightMode::HARBOR;
        int8_t armingLED      = 0;
        int8_t HPLED          = 0;
 
    private:
        bool _debug;
        int8_t readAndScaleAnalog(int PintoRead);
        int8_t readAndScaleSpeedPot(int PintoRead);

};

#endif // DASHBOARD_HPP