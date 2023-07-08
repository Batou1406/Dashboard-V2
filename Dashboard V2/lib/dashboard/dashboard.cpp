#include <dashboard.hpp>

// constructor
Dashboard::Dashboard(bool debug) {
    _debug = debug;
}


void Dashboard::init(){
    if(_debug){Serial.println("Initialising GPIO's...");}
    
    // selecteur
    pinMode(MODE_TEST_PIN, INPUT_PULLUP); 
    pinMode(MODE_HARBOR_PIN, INPUT_PULLUP);
    pinMode(MODE_GOING_PIN, INPUT_PULLUP);
    pinMode(MODE_FOIL_1_PIN, INPUT_PULLUP);
    pinMode(MODE_FOIL_2_PIN, INPUT_PULLUP);
    pinMode(MODE_FOIL_3_PIN, INPUT_PULLUP);
    
    // Potentiometre
    // TODO : CHECKER L'INFLUENCE DU MODE PULLUP PULLDOWN
    pinMode(SPEED_POT_PIN, INPUT_PULLUP); 
    pinMode(TUNE1_POT_PIN, INPUT); 
    pinMode(TUNE2_POT_PIN, INPUT); 

    // Switches & button   
    pinMode(HP_SWITCH_PIN, INPUT_PULLUP);
    pinMode(ARMING_SWITCH_PIN, INPUT_PULLUP); 
    pinMode(DEADMAN_SWITCH_PIN, INPUT_PULLUP);
    pinMode(TAKEOFF_BUTTON_PIN, INPUT_PULLUP);

    // LED's
    pinMode(ARMING_LED_PIN, OUTPUT);
    pinMode(HP_LED_PIN, OUTPUT);
    digitalWrite(HP_LED_PIN, LOW);
    digitalWrite(ARMING_LED_PIN, LOW);

    if(_debug){Serial.println("Initialising GPIO's done !");}
}


void Dashboard::read(){
    if(_debug){Serial.println("Dashboard read...");}

    // Lecture analogique des potentiomètre, envoie la valeur (0;1023) sur (-100,100)
    speedCommand = readAndScaleAnalog(SPEED_POT_PIN);
    tuningCommand1 = readAndScaleAnalog(TUNE1_POT_PIN);
    tuningCommand2 = readAndScaleAnalog(TUNE2_POT_PIN);
    if(_debug){Serial.print("Speed: ");Serial.print(speedCommand);Serial.print(", tuning1: ");Serial.print(tuningCommand1);Serial.print(", tuning2: ");Serial.println(tuningCommand2);}

    // Selector reading, Attention ! Low Actif
    if(digitalRead(MODE_TEST_PIN) == 0){
        selector = flightMode::TEST;
    }else if(digitalRead(MODE_HARBOR_PIN) == 0){
        selector = flightMode::HARBOR;
    }else if(digitalRead(MODE_GOING_PIN) == 0){
        selector = flightMode::GOING;
    }else if(digitalRead(MODE_FOIL_1_PIN) == 0){
        selector = flightMode::FOIL1;
    }else if(digitalRead(MODE_FOIL_2_PIN) == 0){
        selector = flightMode::FOIL2;
    }else if(digitalRead(MODE_FOIL_2_PIN) == 0){
        selector = flightMode::FOIL3;
    }
    if(_debug){Serial.print("Selector mode : "); Serial.println((int)selector);}

    // Read Switch and buttons
    HPSwitch = digitalRead(HP_SWITCH_PIN);
    armingSwitch = digitalRead(ARMING_SWITCH_PIN);
    deadManSwitch = digitalRead(DEADMAN_SWITCH_PIN);
    takeOffButton = digitalRead(TAKEOFF_BUTTON_PIN);

    if(_debug){Serial.println("Dashboard read finished");}
}


// Read an analogPin (value between 0-1023) and return a value between -100,100 with 10% safety margin
int8_t Dashboard::readAndScaleAnalog(int PintoRead){
    int8_t temp = (   (   ((float)analogRead(SPEED_POT_PIN))/(ADC_MAX >> 1)   ) - 1) * 110;
    return max(-100, min(temp, 100));
}


void Dashboard::updateLED(bool highPowerStatus, bool armingStatus){
    digitalWrite(HP_LED_PIN, highPowerStatus);
    digitalWrite(ARMING_LED_PIN, armingStatus);
}