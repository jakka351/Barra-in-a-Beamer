// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////     __________________________________________________________________________________________________________________
// /////
// /////                                    __________                                    
// /////                                    \______   \_____ ___________________          
// /////                                     |    |  _/\__  \\_  __ \_  __ \__  \         
// /////                                     |    |   \ / __ \|  | \/|  | \// __ \_       
// /////                                     |______  /(____  /__|   |__|  (____  /       
// /////                                            \/      \/                  \/        
// /////                                    .__                                           
// /////                                    |__| ____   _____                             
// /////                                    |  |/    \  \__  \                            
// /////                                    |  |   |  \  / __ \_                          
// /////                                    |__|___|  / (____  /                          
// /////                                            \/       \/                           
// /////                                    __________                                    
// /////                                    \______   \ ____ _____    _____   ___________ 
// /////                                     |    |  _// __ \\__  \  /     \_/ __ \_  __ \
// /////                                     |    |   \  ___/ / __ \|  Y Y  \  ___/|  | \/
// /////                                     |______  /\___  >____  /__|_|  /\___  >__|   
// /////                                            \/     \/     \/      \/     \/       
// /////
// /////                Barra in a Beamer Arduino Code by Jakka351 -  BMW 730D Cluster driven by Spanish Oak PCM
// /////                                    
// /////     __________________________________________________________________________________________________________________
// /////      |--------------------------------------------------------------------------------------------------------------|
// /////      |          https://github.com/jakka351  | jakka351@outlook.com |      https://facebook.com/testePresent        |
// /////      |--------------------------------------------------------------------------------------------------------------|
// /////      | Copyright (c) 2022/2023 Jack Leighton 0434 645 485                                                           |          
// /////      | All rights reserved.                                                                                         |
// /////      |--------------------------------------------------------------------------------------------------------------|
// /////        Redistribution and use in source and binary forms, with or without modification, are permitted provided that
// /////        the following conditions are met:
// /////        1.    With the express written consent of the copyright holder.
// /////        2.    Redistributions of source code must retain the above copyright notice, this
// /////              list of conditions and the following disclaimer.
// /////        3.    Redistributions in binary form must reproduce the above copyright notice, this
// /////              list of conditions and the following disclaimer in the documentation and/or other
// /////              materials provided with the distribution.
// /////        4.    Neither the name of the organization nor the names of its contributors may be used t
// /////              endorse or promote products derived from this software without specific prior written permission.
// /////      _________________________________________________________________________________________________________________
// /////      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// /////      INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// /////      DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// /////      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// /////      SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// /////      WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// /////      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// /////      _________________________________________________________________________________________________________________
// /////
// /////       This software can only be distributed with my written permission. It is for my own educational purposes and  
// /////       is potentially dangerous to ECU health and safety. Gracias a Gato Blancoford desde las alturas del mar de chelle.                                                        
// /////      _________________________________________________________________________________________________________________
// /////
// /////
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <mcp_can.h>
#include <mcp_can_dfs.h>
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////                     .__      ___.   .__                 
// //// ___  _______ _______|__|____ \_ |__ |  |   ____   ______
// //// \  \/ /\__  \\_  __ \  \__  \ | __ \|  | _/ __ \ /  ___/
// ////  \   /  / __ \|  | \/  |/ __ \| \_\ \  |_\  ___/ \___ \ 
// ////   \_/  (____  /__|  |__(____  /___  /____/\___  >____  >
// ////             \/              \/    \/          \/     \/
// //// 
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// const int SPI_CS_PIN = 9;               // CAN Bus Shield
// const int SPI_CS_PIN = 17;              // CANBed V1
const int SPI_CS_PIN = 3;                  // CANBed M0
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
long unsigned int canId;                         //
unsigned char len = 0;                          //
unsigned char rxBuf[8];                         //
char msgString[128];                            // Array to store serial string
int V_VEH;                                      // Vehicle Speed 
int CHKSM_V_V;                                  // checksum @ 0x1A0 Byte[7] 
int Rpm;                                        // 
int RPM_TEMP_DOM_1;                             // Rpm1 input 
int RPM_TEMP_DOM_2;                             // Rpm2 input
int actualGearPosition;
int engineCoolantTemperature;
int wheelSpeedFrontLeft1;
int wheelSpeedFrontLeft2;
int wheelSpeedFrontRight1;
int wheelSpeedFrontRight2;
int wheelSpeedRearLeft1;
int wheelSpeedRearLeft2;
int wheelSpeedRearRight1;
int wheelSpeedRearRight2;
int odoCount;                                   // 
float odoCountMulti;
bool flagMilLampIlluminated;
bool engineCoolantOverTemperatureWarning;
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////                __                
// ////   ______ _____/  |_ __ ________  
// ////  /  ___// __ \   __\  |  \____ \ 
// ////  \___ \\  ___/|  | |  |  /  |_> >
// //// /____  >\___  >__| |____/|   __/ 
// ////      \/     \/           |__|    
// //// 
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);
    while(!Serial);
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("[ Barra in a Beamer // MCP2515 Initialized. ]");
        delay(100);
    }
    Serial.println("[ Barra in a Beamer //  Error Initializing MCP2515. ]");
    // ////////////////////////////////////////////////////////////////////////////
    //  ________                .__        _________                             
    //  \______ \ _____    _____|  |__    /   _____/_  _  __ ____   ____ ______  
    //   |    |  \\__  \  /  ___/  |  \   \_____  \\ \/ \/ // __ \_/ __ \\____ \ 
    //   |    `   \/ __ \_\___ \|   Y  \  /        \\     /\  ___/\  ___/|  |_> >
    //  /_______  (____  /____  >___|  / /_______  / \/\_/  \___  >\___  >   __/ 
    //          \/     \/     \/     \/          \/             \/     \/|__|    
    // Looks cool, and also tests everything upon power up. 
    // add in fuel gauge and coolant gauge?
    unsigned char dashSweepSpeedo[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char dashSweepTacho[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //unsigned char dashSweepCoolantTemp[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //unsigned char dashSweepFuelGauge[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN.sendMsgBuf(0x1A0, 0, 8, dashSweepSpeedo);  
    CAN.sendMsgBuf(0x1B0, 0, 8, dashSweepTacho);  //tacho
    //CAN.sendMsgBuf(0x1A0, 0, 8, dashSweepCoolantTemp);  
    //CAN.sendMsgBuf(0x1B0, 0, 8, dashSweepFuelGauge);  //tacho
    Serial.println("[ Start Up Function Test // Dash Sweep data Tx. ]");
    delay(100);
    // ////////////////////////////////////////////////////////////////////////////
}
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////  .__                        
// ////  |  |   ____   ____ ______  
// ////  |  |  /  _ \ /  _ \\____ \ 
// ////  |  |_(  <_> |  <_> )  |_> >
// ////  |____/\____/ \____/|   __/ 
// ////                     |__|    
// ////
void loop()
{
    // put your main code here, to run repeatedly:
    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                             .__                                   ___.                                                                              
    // _______   ____   ____  ____ |__|__  __ ____     ____ _____    ____\_ |__  __ __  ______   _____   ____   ______ ___________     ____   ____   ______
    // \_  __ \_/ __ \_/ ___\/ __ \|  \  \/ // __ \  _/ ___\\__  \  /    \| __ \|  |  \/  ___/  /     \_/ __ \ /  ___//  ___/\__  \   / ___\_/ __ \ /  ___/
    //  |  | \/\  ___/\  \__\  ___/|  |\   /\  ___/  \  \___ / __ \|   |  \ \_\ \  |  /\___ \  |  Y Y  \  ___/ \___ \ \___ \  / __ \_/ /_/  >  ___/ \___ \ 
    //  |__|    \___  >\___  >___  >__| \_/  \___  >  \___  >____  /___|  /___  /____//____  > |__|_|  /\___  >____  >____  >(____  /\___  / \___  >____  >
    //              \/     \/    \/              \/       \/     \/     \/    \/           \/        \/     \/     \/     \/      \//_____/      \/     \/     // 
    unsigned char buf[8];
    unsigned char len = 0;
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        unsigned long canId = CAN.getCanId();        
        // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // _______         ________________________  __________                            __                .__         _________                __                .__       _____             .___    .__          
        // \   _  \ ___  __\_____  \   _  \______  \ \______   \______  _  __ ____________/  |_____________  |__| ____   \_   ___ \  ____   _____/  |________  ____ |  |     /     \   ____   __| _/_ __|  |   ____  
        // /  /_\  \\  \/  //  ____/  /_\  \  /    /  |     ___/  _ \ \/ \/ // __ \_  __ \   __\_  __ \__  \ |  |/    \  /    \  \/ /  _ \ /    \   __\_  __ \/  _ \|  |    /  \ /  \ /  _ \ / __ |  |  \  | _/ __ \ 
        // \  \_/   \>    </       \  \_/   \/    /   |    |  (  <_> )     /\  ___/|  | \/|  |  |  | \// __ \|  |   |  \ \     \___(  <_> )   |  \  |  |  | \(  <_> )  |__ /    Y    (  <_> ) /_/ |  |  /  |_\  ___/ 
        //  \_____  /__/\_ \_______ \_____  /____/    |____|   \____/ \/\_/  \___  >__|   |__|  |__|  (____  /__|___|  /  \______  /\____/|___|  /__|  |__|   \____/|____/ \____|__  /\____/\____ |____/|____/\___  >
        //        \/      \/       \/     \/                                     \/                        \/        \/          \/            \/                                  \/            \/               \/ 
        //   _________                        .___   ____    __________                  __________                _____  .__        
        //  /   _____/_____   ____   ____   __| _/  /  _ \   \______   \ _______  _______\______   \ ___________  /     \ |__| ____  
        //  \_____  \\____ \_/ __ \_/ __ \ / __ |   >  _ </\  |       _// __ \  \/ /  ___/|     ___// __ \_  __ \/  \ /  \|  |/    \ 
        //  /        \  |_> >  ___/\  ___// /_/ |  /  <_\ \/  |    |   \  ___/\   /\___ \ |    |   \  ___/|  | \/    Y    \  |   |  \
        // /_______  /   __/ \___  >\___  >____ |  \_____\ \  |____|_  /\___  >\_//____  >|____|    \___  >__|  \____|__  /__|___|  /
        //         \/|__|        \/     \/     \/         \/         \/     \/         \/               \/              \/        \/           
        // powertrainControlModule code notesL
        // CAN ID 0x207 Data Bytes:  [0]EngineRPM [1]EngineRPM [2]EngineSpeedRateOfChange [3]EngineSpeedRateOfChange [4]VehicleSpeed  [5]VehicleSpeed  [6]ThrottlePositionManifold  [7]ThrottlePositionRateOfChange
        // Code for Vehicle Speed and Engine RPM recieved from Barra PCM, then transmitted out to BMW Kombi for speed and tacho display on cluster.
        if (canId == 0x207) 
        {
            //////////////////////////////////////////////////////////////////
            int Valx0 = (int)buf[0]; // ENGINE RPM
            int Valx1 = (int)buf[1]; // ENGINE RPM
            int Valx4 = (int)buf[4]; // VEHICLE SPEED
            int Valx5 = (int)buf[5]; // VEHICLE SPEED
            //////////////////////////////////////////////////////////////////
            float tmpSpeed = (Valx4 + (Valx5 / 255)) * 2;
            if (tmpSpeed != V_VEH)
            {
                V_VEH = (tmpSpeed * 0.1);    // From Barra PCM then calced to KM/H, then multiply KMH * 0.1 to correct for BMW cluster input
                CHKSM_V_V = (V_VEH + 0x40);  // TBA
                Serial.println("[V_VEH]Vehicle Speed km/h: ");
                Serial.println(V_VEH);
                //////////////////////////////////////
                // CAN ID 0x1A0 Byte 0 - Vehicle Speed 
                // V_VEH    0   12  Intel   Unsigned    0.1 0   km/h 
                unsigned char dataSpd[8] = {V_VEH, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CHKSM_V_V};
                CAN.sendMsgBuf(0x1A0, 0, 8, dataSpd);  
                Serial.println("Vehicle Speed data Tx.");
                delay(100);   
            }
            //////////////////////////////////////////////////////////////////
            float tmpRpm = (Valx0 + (Valx1 / 255)) * 2;
            if (tmpRpm != Rpm)
            {
                Rpm = tmpRpm;
                RPM_TEMP_DOM_1 = Rpm;
                RPM_TEMP_DOM_2 = Rpm;
                Serial.println("RPM: ");
                Serial.println(Rpm);
                //CAN ID 0x332 Display RPM range:
                //Type: CAN Standard
                // ID: 0x332
                // DLC: 2
                // Tx method: cycle
                // Cycle time: 5000ms
                // Signal  Start bit   Length  Order   Value type  Factor  Offset  Unit
                // RPM_TEMP_DOM_1  0   8   Intel   Unsigned    50  0   1/min
                // RPM_TEMP_DOM_2  8   8   Intel   Unsigned    50  0   1/min
                // RPM_TEMP_DOM_1, variable engine speed warning field, actual value
                // RPM_TEMP_DOM_2, engine max RPM
                // Message example:
                // 0x332 2 5A 82
                // RPM_TEMP_DOM_1: 0x5A -> 4500 1/min
                // RPM_TEMP_DOM_2: 0x82 -> 6500 1/min
                unsigned char dataRpm[2] = {RPM_TEMP_DOM_1, RPM_TEMP_DOM_2};
                CAN.sendMsgBuf(0x332, 0, 2, dataRpm);
                Serial.println("Engine RPM data Tx.");
                delay(100);
            }
            //////////////////////////////////////////////////////////////////
        }
        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // _______         ________  ________ _______    ___________                              _____          __               .__    ________                     
        // \   _  \ ___  __\_____  \ \_____  \\   _  \   \__    ___/___________    ____   ______ /  _  \   _____/  |_ __ _______  |  |  /  _____/  ____ _____ _______ 
        // /  /_\  \\  \/  //  ____/   _(__  </  /_\  \    |    |  \_  __ \__  \  /    \ /  ___//  /_\  \_/ ___\   __\  |  \__  \ |  | /   \  ____/ __ \\__  \\_  __ \
        // \  \_/   \>    </       \  /       \  \_/   \   |    |   |  | \// __ \|   |  \\___ \/    |    \  \___|  | |  |  // __ \|  |_\    \_\  \  ___/ / __ \|  | \/
        //  \_____  /__/\_ \_______ \/______  /\_____  /   |____|   |__|  (____  /___|  /____  >____|__  /\___  >__| |____/(____  /____/\______  /\___  >____  /__|   
        //        \/      \/       \/       \/       \/                        \/     \/     \/        \/     \/                \/             \/     \/     \/               
        // Powertrain Control Module    Transmission Mode   230 8   Transmission Gear Pos   TargetGearPosition  ActualGearPosition  TorqueConverter             Trans Mode Fault Warn
        // Code for displaying current transmission gear on the cluster (PRNDL)
        if (canId == 0x230)
        {
            int actualGearPosition = (int)buf[3];
            unsigned char sendActualGearPosition[8] = {actualGearPosition, 0, 0, 0, 0, 0, 0, 0}; //
            CAN.sendMsgBuf(0xABC, 0, 2, sendActualGearPosition);
            Serial.println("Transmission Gear data Tx.");
            delay(100);
        }
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  _______            _____ _________________       ________       .___                     __                 _________                      __   
        //  \   _  \ ___  ___ /  |  |\_____  \______  \      \_____  \    __| _/____   _____   _____/  |_  ___________  \_   ___ \  ____  __ __  _____/  |_ 
        //  /  /_\  \\  \/  //   |  |_/  ____/   /    /       /   |   \  / __ |/  _ \ /     \_/ __ \   __\/ __ \_  __ \ /    \  \/ /  _ \|  |  \/    \   __\
        //  \  \_/   \>    </    ^   /       \  /    /       /    |    \/ /_/ (  <_> )  Y Y  \  ___/|  | \  ___/|  | \/ \     \___(  <_> )  |  /   |  \  |  
        //   \_____  /__/\_ \____   |\_______ \/____/        \_______  /\____ |\____/|__|_|  /\___  >__|  \___  >__|     \______  /\____/|____/|___|  /__|  
        //         \/      \/    |__|        \/                      \/      \/            \/     \/          \/                \/                  \/            
        // odometerCount code notes
        // BARRA CAN ID: 0x427 Byte [4] Odometer Count Byte [7] Engine Speed Count
        // Unit:km  Offset:0,Mult:0.000201167, Div:1  FF=Invalid
        // ODOMETER Count output from Barra PCM, may be unneccesary as BMW Kombi does odometer from speed source?
        if (canId == 0x427) 
        {
            int odoCount = (int)buf[4];
            float odoCountMulti = (odoCount * 0.000201167);
            switch(odoCount)
            {
                case 0x00:
                    //Vehicle Stationary
                    Serial.println("Vehicle is Stationary, Odometer count 0.");
                    delay(100);
                    break;
                case 0xFF:
                    //Invalid Data
                    Serial.println("Invalid Odometer Count Data.");
                    delay(100);
                    break;
            }
            if (odoCount > 0 < 0xFF)
            {
                // Valid Data
                Serial.println("Odometer Count: ");
                Serial.println(odoCountMulti);
                delay(100);
            } 
            // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
           
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //    ___________              .__                _________               .__                 __    ___________                                        __                        
            //    \_   _____/ ____    ____ |__| ____   ____   \_   ___ \  ____   ____ |  | _____    _____/  |_  \__    ___/___   _____ ______   ________________ _/  |_ __ _________   ____  
            //     |    __)_ /    \  / ___\|  |/    \_/ __ \  /    \  \/ /  _ \ /  _ \|  | \__  \  /    \   __\   |    |_/ __ \ /     \\____ \_/ __ \_  __ \__  \\   __\  |  \_  __ \_/ __ \ 
            //     |        \   |  \/ /_/  >  |   |  \  ___/  \     \___(  <_> |  <_> )  |__/ __ \|   |  \  |     |    |\  ___/|  Y Y  \  |_> >  ___/|  | \// __ \|  | |  |  /|  | \/\  ___/ 
            //    /_______  /___|  /\___  /|__|___|  /\___  >  \______  /\____/ \____/|____(____  /___|  /__|     |____| \___  >__|_|  /   __/ \___  >__|  (____  /__| |____/ |__|    \___  >
            //            \/     \//_____/         \/     \/          \/                        \/     \/                    \/      \/|__|        \/           \/                        \/ 
            //
            // ID 0x427 byte 0 Engine Coolant Temperature
            // ECT received from Barra PCM and then transmitted to Kombi for display on cluster
            bool engineCoolantOverTemperatureWarning = false;
            int engineCoolantTemperature = (int)buf[0];
            if (engineCoolantTemperature > 0xFF) // need to scale this number to reflect an overtemp condition
            {
                Serial.println("Warning: Engine Coolant Temperature High.");
                engineCoolantOverTemperatureWarning = true;
                delay(100);
            }
            unsigned char sendEngineCoolantTemperature[8] = {engineCoolantTemperature, 0, 0, 0, 0, 0, 0, 0}; //
            CAN.sendMsgBuf(0xABC, 0, 2, sendEngineCoolantTemperature);
            Serial.println("Engine Coolant Temperature Tx data.");
            delay(100);
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //      _____         .__   _____                    __  .__              .___            .___.__               __               .____    .__       .__     __   
            //     /     \ _____  |  |_/ ____\_ __  ____   _____/  |_|__| ____   ____ |   | ____    __| _/|__| ____ _____ _/  |_  ___________|    |   |__| ____ |  |___/  |_ 
            //    /  \ /  \\__  \ |  |\   __\  |  \/    \_/ ___\   __\  |/  _ \ /    \|   |/    \  / __ | |  |/ ___\\__  \\   __\/  _ \_  __ \    |   |  |/ ___\|  |  \   __\
            //   /    Y    \/ __ \|  |_|  | |  |  /   |  \  \___|  | |  (  <_> )   |  \   |   |  \/ /_/ | |  \  \___ / __ \|  | (  <_> )  | \/    |___|  / /_/  >   Y  \  |  
            //   \____|__  (____  /____/__| |____/|___|  /\___  >__| |__|\____/|___|  /___|___|  /\____ | |__|\___  >____  /__|  \____/|__|  |_______ \__\___  /|___|  /__|  
            //           \/     \/                     \/     \/                    \/         \/      \/         \/     \/                          \/ /_____/      \/      
            // 0x427 byte 5 MIL Lamp 
            // Check engine light on flag
            bool flagMilLampIlluminated = false;
            if ((int)buf[5] > 0x00)            
            {
                // send can message to illuminate Kombi check engine light here
                Serial.println("Barra PCM MIL Lamp Illuminated.");
                flagMilLampIlluminated = true;
                delay(100);   
            }
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }
        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // _______            ______________________       _____ __________  _________  __      __.__                  .__      _________                        .___   _____              ___________________     _____   
        // \   _  \ ___  ___ /  |  \______   \   _  \     /  _  \\______   \/   _____/ /  \    /  \  |__   ____   ____ |  |    /   _____/_____   ____   ____   __| _/ _/ ____\___________  \______   \_   ___ \   /     \  
        // /  /_\  \\  \/  //   |  ||    |  _/  /_\  \   /  /_\  \|    |  _/\_____  \  \   \/\/   /  |  \_/ __ \_/ __ \|  |    \_____  \\____ \_/ __ \_/ __ \ / __ |  \   __\/  _ \_  __ \  |     ___/    \  \/  /  \ /  \ 
        // \  \_/   \>    </    ^   /    |   \  \_/   \ /    |    \    |   \/        \  \        /|   Y  \  ___/\  ___/|  |__  /        \  |_> >  ___/\  ___// /_/ |   |  | (  <_> )  | \/  |    |   \     \____/    Y    \
        //  \_____  /__/\_ \____   ||______  /\_____  / \____|__  /______  /_______  /   \__/\  / |___|  /\___  >\___  >____/ /_______  /   __/ \___  >\___  >____ |   |__|  \____/|__|     |____|    \______  /\____|__  /
        //        \/      \/    |__|       \/       \/          \/       \/        \/         \/       \/     \/     \/               \/|__|        \/     \/     \/                                         \/         \/ // 
        // FALCON WHEEL SPEED SIGNAL:
        // Antilock Brake Module   Wheel Speed Sensor  4B0 8   
        // WheelSpeedFrontLeft Units:km/h   Offset:0;Multi:1;Div:100   0xFFFE=Initialization in progress   0xFFFF=Wheel Speed Faulted          
        // WheelSpeedFrontLeft Units:km/h  Offset:0;Multi:1;Div:100  0xFFFE=Initialization in progress  0xFFFF=Wheel Speed Faulted         
        // FrontRightWheelSpeed  Units:km/h  Offset:0;Multi:1;Div:100  0xFFFE=Initialization in progress  0xFFFF=Wheel Speed Faulted   
        // FrontRightWheelSpeed    RearLeftWheelSpeed  RearLeftWheelSpeed  RightRearWheelSpeed RightRearWheelSpeed
        // BMW WHEEL SPEED SIGNAL:
        // CAN ID 0xCE Wheel rotating speed:
        // ID: 0x0CE
        // DLC: 8
        // Tx method: cycle
        // Cycle time: 20ms
        // Signal  Start bit   Length  Order   Value type  Factor  Offset  Unit
        // V_WHL_FLH   0   16  Intel   Signed  0.0625  0   km/h
        // V_WHL_FRH   16  16  Intel   Signed  0.0625  0   km/h
        // V_WHL_RLH   32  16  Intel   Signed  0.0625  0   km/h
        // V_WHL_RRH   48  16  Intel   Signed  0.0625  0   km/h
        // Message example:        // 
        // 0xCE 8 00 00 00 00 00 00 00 00
        // V_WHL_FLH: 0km/h
        // V_WHL_FRH: 0km/h
        // V_WHL_RLH: 0km/h
        // V_WHL_RRH: 0km/h
        // Recieve ABS individual wheel speed signals from BMW ABS module, and re-transmit on ID 0x4B0 for Barra PCM, assuming that Vehicle Speed Source is set to 'ABS via CAN' with PCMTEC
        if (canId == 0x0CE) // BMW WHEEL SPEED ID FROM ABS
        {
            int wheelSpeedFrontLeft1  = (int)buf[0];
            int wheelSpeedFrontLeft2  = (int)buf[1];
            int wheelSpeedFrontRight1 = (int)buf[2];
            int wheelSpeedFrontRight2 = (int)buf[3];
            int wheelSpeedRearLeft1   = (int)buf[4];
            int wheelSpeedRearLeft2   = (int)buf[5];
            int wheelSpeedRearRight1  = (int)buf[6];
            int wheelSpeedRearRight2  = (int)buf[7];
            unsigned char wheelSpeedSignalData[8] = {wheelSpeedFrontLeft1, wheelSpeedFrontLeft2, wheelSpeedFrontRight1, wheelSpeedRearRight2, wheelSpeedRearLeft1, wheelSpeedRearLeft2, wheelSpeedRearRight1, wheelSpeedRearRight2}; //
            CAN.sendMsgBuf(0x4B0, 0, 8, wheelSpeedSignalData);
            Serial.println("ABS Wheel Speed Signal Data Tx on ID 0x4B0 for PCM.");
            delay(100);   
        }
        // Notes on Standalone PCM, disabling PATS for BA/BF via PCMTEC:
        // To run the BA/BF PCM standalone there are two items that must be set up. Firstly the PATS security system must be disabled.
        // First locate auF16527 (PATS Alternate Switch) and auF16595 (PATS Switch). From the factory you will have:
        // auF16527: 0
        // auF16595: 1
        // image.png.f3014942c439fe934569111c8827f79f.png
        // Flip both values. So now you will have:
        // auF16527: 1
        // auF16595: 0
        // Now that the PATS security system is disabled the BCM/ICC will no longer function, you should only do this if you are not utilising the dash cluster in your new vehicle.
        // The next step is to change the speed source. From the factory the VSS (Vehicle Speed Source) would most likely have been set to use the speed from the ABS module or the TCM 
        // sent via CANBUS. We will now change this to use the output shaft speed (OSS) sensor. You'll obviously need to wire an actual speed sensor as well up to Pin B7. Most people mount this on the rear of the manual gearboxes. 
        // image.png.344ca9f3f834e3ee35eba44ba8a6337e.png
        // Now change VSS_Source to Transmission Output Shaft Sensor.
        // image.thumb.png.ea6556a5d1d7e02d6e6559fc97d6f092.png
        // Whilst you are in the VID block editor it is recommended to adjust the final drive ratio and other gear ratios that may have changed.
        // This file will now allow the PCM to run standalone with no other modules.
        // In this case we are going to be taking ABS Speed Source from the BMW and putting sending it to the Barra PCM via CAN on ID 0x4B0
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // __________                              ___________________     _____    ________  .__                                      __  .__         ________          __          
        // \______   \_____ ___________________    \______   \_   ___ \   /     \   \______ \ |__|____     ____   ____   ____  _______/  |_|__| ____   \______ \ _____ _/  |______   
        //  |    |  _/\__  \\_  __ \_  __ \__  \    |     ___/    \  \/  /  \ /  \   |    |  \|  \__  \   / ___\ /    \ /  _ \/  ___/\   __\  |/ ___\   |    |  \\__  \\   __\__  \  
        //  |    |   \ / __ \|  | \/|  | \// __ \_  |    |   \     \____/    Y    \  |    `   \  |/ __ \_/ /_/  >   |  (  <_> )___ \  |  | |  \  \___   |    `   \/ __ \|  |  / __ \_
        //  |______  /(____  /__|   |__|  (____  /  |____|    \______  /\____|__  / /_______  /__(____  /\___  /|___|  /\____/____  > |__| |__|\___  > /_______  (____  /__| (____  /
        //         \/      \/                  \/                    \/         \/          \/        \//_____/      \/           \/               \/          \/     \/          \/          
        // PCM_DiagSig_Rx, Tx ID's 0x7DF, 0x7E0, 0x7E8
        // Print on the serial monitor any diagnostic messsages that are recieved.
        if (canId == 0x7DF)            // OBDII Tester Transmit ID 
        {
            Serial.println("OBD_DiagSig_Rx: ");
            Serial.println(canId, HEX);
            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.println(buf[i], HEX);
                Serial.println("\t");
            }
        }
        if (canId == 0x7E0)            // PCM_DiagSig_Rx (From tester)
        {
            Serial.println("PCM_DiagSig_Rx: ");
            Serial.println(canId, HEX);
            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.println(buf[i], HEX);
                Serial.println("\t");
            }
        }
        if (canId == 0x7E8)            // PCM_DiagSig_Tx (From ECU)
        {
            Serial.println("PCM_DiagSig_Tx: ");
            Serial.println(canId, HEX);
            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.println(buf[i], HEX);
                Serial.println("\t");
            }
        }
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

