//------------------------------ Include --------------------------------
#include <Arduino.h>
#include <CAN.h>
#include <encoder.hpp>
#include <dashboard.hpp>


//------------------------------ Define ---------------------------------
// CAN ID
#define BATTERY_PDO_1           0x240 //ID to request HP on or off to the battery
#define PACK_INFO_CAN_ID        0x1C0 //ID sent by batteryV1 with pack infos
#define STATE_MACHINE_CAN_ID    0x1D4 //ID sent by FCU with state machine infos
#define HEARTBEAT_ADDR          0x72A
#define PILOT_COMMAND_01        0x1AA

// Rotary encoder
#define ROTARY_ADDR 0x36
#define ROTARY_INCREMENT_TO_STEERING_WHEEL_DEGREE 3

// Frequency define for lisibility sake
#define HUNDERT_HZ 200
#define ONE_HZ 1000
#define TEN_HZ 100


//----------------------- Functions Declaration -------------------------
void handleHighPower();
void decodeCAN1Callback(int packetLength);
void sendPilotInput();


//-------------------------- Global Variables ---------------------------
bool debug = true;
Encoder rotary(&Wire, ROTARY_ADDR,  ROTARY_BUTTON, debug);
Dashboard dashboard(debug);
int16_t steeringWheelPos = 0; // Position du volant en degree
bool highPowerStatus = false;
bool armingStatus = false;
unsigned long lastTime10Hz = 0;
unsigned long lastTime1Hz = 0;


//------------------------------- Setup ---------------------------------
void setup(){

    // Begin debug communication (USB serial port)
    Serial.begin(9600);
    delay(500);
    if(debug){Serial.println("Starting up...");}

    // Init all the GPIO's
    dashboard.init();

    // start the CAN bus at 1M bps
    if (!can1.begin(1000E3)) {
         if(debug){Serial.println("Starting CAN1 failed");}
        while (42) {
            //TODO : Implementer un watchdog timer
            Serial.println("Starting CAN failed!");
        }
    }else{
        if(debug){ Serial.println("Starting CAN1 Succed");}
         can1.onReceive(decodeCAN1Callback);
    }

    rotary.init();
}


//-------------------------------- Loop ---------------------------------
void loop() {
    // Ten HZ
    if(millis() > lastTime10Hz + TEN_HZ){
        lastTime10Hz = millis();

        dashboard.read();
        dashboard.updateLED(highPowerStatus, armingStatus);
        handleHighPower();

        // Read encoder nutton and value and convert it to steering wheel angle and send it on CAN
        rotary.readRotaryButtonAndResetPos();
        steeringWheelPos = (int16_t)(rotary.getEncoderPosition() * ROTARY_INCREMENT_TO_STEERING_WHEEL_DEGREE);

        // Send dashboard input on the CAN line
        sendPilotInput();
    }

    // One HZ
    if(millis() > lastTime1Hz + ONE_HZ){
        lastTime1Hz = millis();
        
        // Send Heartbeat 
        can1.beginPacket(HEARTBEAT_ADDR);
        can1.write(0x05);
        can1.endPacket();
    }
}


//------------------------------ Functions ------------------------------
// Check if boat HP status is same as HP dashboard switch, send CAN msg if needed
void handleHighPower(){
    if(debug){
        Serial.print("Handling HP ");
        Serial.print("HighPower switch : ");Serial.println(dashboard.HPSwitch);
        Serial.print("HighPower status : ");Serial.println(highPowerStatus);
        Serial.println(" ");
        
    }
    // We are not in the same state as desired
    if(highPowerStatus != dashboard.HPSwitch){
        // To turn on the highPower we must be disarmed with the deadManSwitch enabled
        if(dashboard.HPSwitch && dashboard.deadManSwitch && !dashboard.armingSwitch){
            can1.beginPacket(BATTERY_PDO_1);
            can1.write(1); // Byte 0 : Index pour decoder Byte 1 -> 1 : Gestion du HP
            can1.write(1); // Byte 1 : 1->HP On , 0->HP Off
            can1.endPacket();
        }

        // We want to turn off HP
        if(!dashboard.HPSwitch){
            can1.beginPacket(BATTERY_PDO_1);
            can1.write(1); // Byte 0 : Index pour decoder Byte 1 -> 1 : Gestion du HP
            can1.write(0); // Byte 1 : 1->HP On , 0->HP Off
            can1.endPacket();
        }
    }

}


// Callback function attached to OnReceived from CAN1, Called each time a message is received
void decodeCAN1Callback(int packetLength){
    if(debug){Serial.print("CAN1 message received - ID :");Serial.println(can1.packetId());}

    switch(can1.packetId()){
        case PACK_INFO_CAN_ID: // HP status is in that message 
            if(packetLength != 8){break;} // invalid packet length
            for (int i(0); i < 6; i++) {can1.read();} // dump 6 first byte
            highPowerStatus = (can1.read() >> 6) & 0b1; // Get HP status
            if(debug){Serial.print("HP status: ");Serial.println(highPowerStatus);}
            break;

        case STATE_MACHINE_CAN_ID:
            if(packetLength != 4){break;} // invalid packet length
            for (int i(0); i < 3; i++) {can1.read();} // dump 3 first byte
            armingStatus = (can1.read() == 3); 
            if(debug){Serial.print("Arming status: ");Serial.println(armingStatus);}
            break;

        default:
            break;
    }

    // Debuff if necessary
    while(can1.available()){
        can1.read();
    }
}  

void sendPilotInput(){
    can1.beginPacket(PILOT_COMMAND_01);
    can1.write(dashboard.speedCommand);     //Byte 0
    can1.write(dashboard.tuningCommand1);   //Byte 1
    can1.write(dashboard.tuningCommand2);   //Byte 2
    can1.write( ((int8_t)dashboard.selector & 0b00000111) + (((dashboard.armingSwitch & dashboard.deadManSwitch) << 3) & 0b00001000) ); //Byte 3, Bit 0,1,2 : Selector, Bit 3 : arming state (deadman et Arming)
    can1.write(highByte(steeringWheelPos)); //Byte 4
    can1.write(lowByte(steeringWheelPos));  //Byte 5
    can1.endPacket();

    if(debug){
        Serial.println("Sending CAN message");
        Serial.print("Speed command    : ");Serial.println(dashboard.speedCommand);
        Serial.print("Tuning command 1 : ");Serial.println(dashboard.tuningCommand1);
        Serial.print("Tuning command 2 : ");Serial.println(dashboard.tuningCommand2);
        Serial.print("Steering wheel   : ");Serial.println(steeringWheelPos);
        Serial.print("Arming switch    : ");Serial.println(dashboard.armingSwitch);
        Serial.print("DeadMan switch   : ");Serial.println(dashboard.deadManSwitch);
        Serial.print("HighPower switch : ");Serial.println(dashboard.HPSwitch);
        Serial.print("Selector         : ");Serial.println((int8_t)dashboard.selector);
        Serial.println(" ");
    }
}