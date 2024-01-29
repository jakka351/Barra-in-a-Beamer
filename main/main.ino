e// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
// /////      |          https://github.com/jakka351  | jakka351@outlook.com |      https://facebook.com/testerPresent       |
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
#include "CRC8.h"
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
CRC8 crc8;
const int SPI_CS_PIN                 = 3;                  // CANBed M0
const int SPI_MCP2515_CS_PIN         = 10;                  // CANBed M0
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
MCP_CAN CAN1(SPI_MCP2515_CS_PIN);                                    // Set CS pin
long unsigned int canId;                         //
char mes[8]                          = { 0, 0, 0, 0, 0, 0, 0, 0 };
char msgString[128];                            // Array to store serial string
unsigned char len = 0;                          //
unsigned char rxBuf[8];                         //
uint8_t checksum;             
uint8_t Cnt3FD                       = 0x0;
uint8_t counterDataRpm               = 0x0; // byte [1] counter, counts from 0x10-0x1E over and over for RPM DATA 
uint8_t counterDataSpd               = 0x0; // byte [1] counter, counts from 0x10-0x1E over and over for RPM DATA
uint8_t counterDataEct               = 0x0; // ENGINE COOLANT TEMP COUNTER BYTE 
uint8_t counterDataEctFlag           = 0x0; // ENGINE OVERHEAT MESSAGE COUNTER
uint8_t counterEngineOilPressureFlag = 0x0; // Engine Oil Pressure Warning message counter
uint8_t counterActualGearPos         = 0x0; // byte [1] counter, counts from 0x10-0x1E over and over for RPM DATA 
uint8_t counterAlternatorFailureState= 0x0; // byte [1] counter, counts from 0x10-0x1E over and over for RPM DATA 
uint8_t counterMilLampIlluminated    = 0x0; // byte [1] counter, counts from 0x10-0x1E over and over for RPM DATA 
uint8_t counter08F                   = 0x0;
uint8_t counter0A5                   = 0x0;
uint8_t counter0A6                   = 0x0;
uint8_t counter0A7                   = 0x0;
uint8_t counter0D9                   = 0x0;
uint8_t counter0DC                   = 0x0;
//uint8_t counter0F3                   = 0x0;
uint8_t counter281                   = 0x0;
uint8_t counter2C4                   = 0x0;
uint8_t counter2FC                   = 0x0;
uint8_t counter349                   = 0x0;
uint8_t counter3A0                   = 0x0;
uint8_t counter3BE                   = 0x0;
uint8_t inFuelRange;
uint8_t outFuelRange;
uint8_t backlightBrightness;
uint8_t gear;
uint8_t driveMode;
int V_VEH;                                      // Vehicle Speed 
int CHKSM_V_V;                                  // checksum @ 0x1A0 Byte[7] 
int Rpm;                                        // 
int RPM_TEMP_DOM_1;                             // Rpm1 input 
int RPM_TEMP_DOM_2;                             // Rpm2 input
int actualGearPosition;
int engineTemperature;
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
int fuelPercentage;
float odoCountMulti;
bool flagEngineOilPressureWarning;
bool flagEngineOilPressureWarningFlashing;
bool flagCruiseControlSet;
bool flagAlternatorFailureState;
bool flagEngineOverHeatWarning;
bool flagDiffLockWarning;
bool flagIdleMode;
bool flagAirConCompressor;
bool flagElectronicThrottleControlWarning;
bool flagElectronicThrottleControlWarningFlashing;
bool flagImmobilizerLamp;
bool flagImmobilizerLampFlashing;
bool flagAirConShedLoad;
bool flagDtcLoggingHighSpeedCan;
bool flagMilLampIlluminated;
bool flagEngineCoolantOverTemperatureWarning;
bool handbrake;
bool highBeam;
bool mainBeam;
bool rearFogLight;
bool frontFogLight;
bool doorOpen;
bool espLight;
bool ignition;
bool isCarMini;
bool leftTurningIndicator;
bool rightTurningIndicator;
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
    crc8.begin();
    // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
    while (CAN_OK != CAN.begin(CAN_500KBPS))  // init can bus : baudrate = 500k  8MHZ crystal
    {
        Serial.println("[ Barra in a Beamer //  Error Initializing MCP2515. can0 ]");
        delay(100);
    }
    Serial.println("[ Barra in a Beamer // MCP2515 Initialized can0 up");  
    while (CAN_OK != CAN1.begin(CAN_500KBPS))  // init can bus : baudrate = 500k  8MHZ crystal
    {
        Serial.println("[ Barra in a Beamer //  Error Initializing MCP2515. can1 ]");
        delay(100);
    }
    Serial.println("[ Barra in a Beamer // MCP2515 Initialized can1 up]");
    // ////////////////////////////////////////////////////////////////////////////
    //  ________                .__        _________                             
    //  \______ \ _____    _____|  |__    /   _____/_  _  __ ____   ____ ______  
    //   |    |  \\__  \  /  ___/  |  \   \_____  \\ \/ \/ // __ \_/ __ \\____ \ 
    //   |    `   \/ __ \_\___ \|   Y  \  /        \\     /\  ___/\  ___/|  |_> >
    //  /_______  (____  /____  >___|  / /_______  / \/\_/  \___  >\___  >   __/ 
    //          \/     \/     \/     \/          \/             \/     \/|__|    
    // Looks cool, and also tests everything upon power up. 
    // add in fuel gauge and coolant gauge?
    //    unsigned char dashSweepSpeedo[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //    unsigned char dashSweepTacho[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //    //unsigned char dashSweepCoolantTemp[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //    //unsigned char dashSweepFuelGauge[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //    CAN.sendMsgBuf(0x1A0, 0, 8, dashSweepSpeedo);  
    //    CAN.sendMsgBuf(0x1B0, 0, 8, dashSweepTacho);  //tacho
    //    //CAN.sendMsgBuf(0x1A0, 0, 8, dashSweepCoolantTemp);  
    //    //CAN.sendMsgBuf(0x1B0, 0, 8, dashSweepFuelGauge);  //tacho
    //    Serial.println("[ Start Up Function Test // Dash Sweep data Tx. ]");
    //    delay(100);
    // ////////////////////////////////////////////////////////////////////////////
}
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////  .__                        a
// ////  |  |   ____   ____ ______  
// ////  |  |  /  _ \ /  _ \\____ \ 
// ////  |  |_(  <_> |  <_> )  |_> >
// ////  |____/\____/ \____/|   __/ 
// ////                     |__|    
// ////
void loop()
{
    //////////////////////////////////////////////////////////////
    //CAN IDS EMITTED BY DDE
    //    0x08F
    //    0x0A5
    //    0x0A6
    //    0x0A7
    //    0x0D9
    //    0x0DC
    //    0x0F3
    //    0x281
    //    0x2C4
    //    0x2FC 81 05 05 00 00 00 00
    //    0x349
    //    0x3A0
    //    0x3BE
    unsigned char send08FPre[8] = {0x1A0, counter08F, 0, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send08FPre, 8, 0x70, 1);
    unsigned char send08F[8] = {checksum, counter08F, 0, 0, 0, 0, 0, 0};
    CAN.sendMsgBuf(0x08F, 0, 8, send08F);
    counter08F ++;
    if (counter08F == 0xF)
    {
        counter08F = 0x0;
    }

    unsigned char send0A5Pre[8] = {0x1A0, counter0A5, 0, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send0A5Pre, 8, 0x70, 1);
    unsigned char send0A5[8] = {checksum, counter0A5, 0, 0, 0, 0, 0, 0};
    CAN.sendMsgBuf(0x0A5, 0, 8, send0A5);
    counter0A5 ++;
    if (counter0A5 == 0xF)
    {
        counter0A5 = 0x0;
    }
    
    unsigned char send0A6Pre[8] = {0x0A6, counter0A6, 0, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send0A6Pre, 8, 0x70, 1);
    unsigned char send0A6[8] = {checksum, counter0A6, 0, 0, 0, 0, 0, 0};
    CAN.sendMsgBuf(0x0A6, 0, 8, send0A6);
    counter0A6 ++;
    if (counter0A6 == 0xF)
    {
        counter0A7 = 0x0;
    }
    
    unsigned char send0A7Pre[8] = {0x0A7, counter0A7, 0, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send0A7Pre, 8, 0x70, 1);
    unsigned char send0A7[8] = {checksum, counter0A7, 0, 0, 0, 0, 0, 0};
    CAN1.sendMsgBuf(0x0A7, 0, 7, send0A7);
    counter0A7 ++;
    if (counter0A7 == 0xF)
    {
        counter0A7 = 0x0;
    }
    

    unsigned char send0D9Pre[8] = {0x0D9, counter0D9, 0, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send0D9Pre, 8, 0x70, 1);
    unsigned char send0D9[8] = {checksum, counter0D9, 0, 0, 0, 0, 0, 0};
    CAN1.sendMsgBuf(0x0D9, 0, 8, send0D9);
    counter0D9 ++;
    if (counter0D9 == 0xF)
    {
        counter0D9 = 0x0;
    }
    
    unsigned char send0DCPre[6] = {0x0DC, counter0DC, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send0DCPre, 6, 0x70, 1); /// may need editing
    unsigned char send0DC[6] = {checksum, counter0DC, 0, 0, 0, 0};
    CAN1.sendMsgBuf(0x0DC, 0, 6, send0DC);
    counter0DC ++;
    if (counter0DC == 0xF)
    {
        counter0DC = 0x0;
    }
    
    unsigned char send281Pre[2] = {0x281, counter281};
    checksum = crc8.get_crc8(send281Pre, 2, 0x70, 1); // may need editing
    unsigned char send281[2] = {checksum, counter0D9};
    CAN1.sendMsgBuf(0x281, 0, 2, send281);
    counter281 ++;
    if (counter281 == 0xF)
    {
        counter281 = 0x0;
    }
    
    unsigned char send2C4Pre[8] = {0x2C4, counter2C4, 0, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send2C4Pre, 8, 0x70, 1);
    unsigned char send2C4[8] = {checksum, counter2C4, 0, 0, 0, 0, 0, 0};
    CAN1.sendMsgBuf(0x2C4, 0, 8, send2C4);
    counter2C4 ++;
    if (counter2C4 == 0xF)
    {
        counter2C4 = 0x0;
    }
    
    unsigned char send2FCPre[7] = {0x2FC, counter2FC, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send2FCPre, 7, 0x70, 1);
    unsigned char send2FC[7] = {checksum, counter2FC, 0, 0, 0, 0, 0};
    CAN1.sendMsgBuf(0x2FC, 0, 7, send2FC);
    counter2FC ++;
    if (counter2FC == 0xF)
    {
        counter2FC = 0x0;
    }
    
    unsigned char send349Pre[5] = {0x349, counter349, 0, 0, 0};
    checksum = crc8.get_crc8(send349Pre, 8, 0x70, 1);
    unsigned char send349[5] = {checksum, counter349, 0, 0, 0};
    CAN1.sendMsgBuf(0x349, 0, 5, send349);
    counter349 ++;
    if (counter349 == 0xF)
    {
        counter349 = 0x0;
    }
    
    unsigned char send3A0Pre[8] = {0x3A0, counter3A0, 0, 0, 0, 0, 0, 0};
    checksum = crc8.get_crc8(send3A0Pre, 8, 0x70, 1);
    unsigned char send3A0[8] = {checksum, counter3A0, 0, 0, 0, 0, 0, 0};
    CAN1.sendMsgBuf(0x3A0, 0, 8, send3A0);
    counter3A0 ++;
    if (counter3A0 == 0xF)
    {
        counter3A0 = 0x0;
    }
    
    unsigned char send3BEPre[2] = {0x3BE, counter3BE};
    checksum = crc8.get_crc8(send3BEPre, 2, 0x70, 1);
    unsigned char send3BE[8] = {checksum, counter3BE};
    CAN1.sendMsgBuf(0x3BE, 0, 2, send3BE);
    counter3BE ++;
    if (counter3BE == 0xF)
    {
        counter3BE = 0x0;
    }   
    //delay(100);
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
        Serial.println("CANbus: Barra HighSpeed CAN:");
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
        // Vehicle Speedo readout for BMW Kombi likely comes from ABS
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
                Serial.println("[V_VEH]Vehicle Speed km/h: ");
                Serial.println(V_VEH);
                //////////////////////////////////////
                // CAN ID 0x1A1 Vehicle Speed - ready to be tested on cluster 29/01/2024
                // 
                double bmwSpeed = V_VEH / 64.01;
                uint16_t calculatedSpeed = (double)V_VEH * 64.01; // 
                unsigned char dataSpdPre[5] = {0x1A1, counterDataSpd, lo8(calculatedSpeed), hi8(calculatedSpeed), V_VEH};
                checksum = crc8.get_crc8(dataSpdPre, 8, 0x70, 1);
                unsigned char dataSpd[5] = {checksum, 0xF0|counterDataSpd, lo8(calculatedSpeed), hi8(calculatedSpeed), V_VEH};
                CAN1.sendMsgBuf(0x1A1, 0, 5, dataSpd); 
                counterDataSpd ++;
                if (counterDataSpd == 0xF)
                {
                    counterDataSpd = 0x0;
                }
                Serial.println("Vehicle Speed Data Tx.");
                //delay(100);   
            }
            //////////////////////////////////////////////////////////////////
            float tmpRpm = (Valx0 + (Valx1 / 255)) * 2;
            if (tmpRpm != Rpm)
            {
                Rpm = tmpRpm;
                //RPM_TEMP_DOM_1 = Rpm;
                //RPM_TEMP_DOM_2 = Rpm; // need to convert these to rpmValue for CAN message
                //int rpmValue =  map(rpm, 0, 6900, 0x00, 0x2B);
                calculatedGear = actualGearPosition;
                Serial.println("RPM: ");
                Serial.println(Rpm);             
                unsigned char dataRpmPre[8] = {0xF3, counterDataRpm, Rpm, 0xC0, 0xF0, calculatedGear, 0xFF, 0xFF };
                checksum = crc8.get_crc8(dataRpmPre, 8, 0x70, 1);
                unsigned char dataRpm[8] = {checksum, counterDataRpm, rpmValue, 0xC0, 0xF0, calculatedGear, 0xFF, 0xFF };
                CAN1.sendMsgBuf(0x0F3, 0, 8, dataRpm); 
                counterDataRpm ++;
                if (counterDataRpm == 0xF)
                {
                    counterDataRpm = 0x0;
                }
                Serial.println("Engine RPM Data & Gear Position Data Tx.");
                //delay(100);
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
        // CAN ID 0x230, Byte 0 is Transmission Gear Position
        // Data: 0=Blank  1=Forward+Drive_1  2=Fordwar_Drive_2  3=Forward_Drive_3  4=Forward_Drive_4  5=Forward_Drive_5  6=Fprward_Drive_D  10=Reverse_Drive_R  11=reserved  16=Forward_Drive_6  
        // 17=Forward_Drive_7  18=Forward_Drive_8  19=Forward_Drive_9  F0=Park_P  F1=Neutral_N  FF=Inval
        if (canId == 0x230) // Ready for testing 29/01/2024
        {
            int actualGearPosition = (int)buf[0];
            switch (actualGearPosition)
            {
                case 0x00:
                    Serial.println("Trans Gear: None");
                    break;
                case 0x01:
                    Serial.println("Trans Gear: 1");
                    break;
                case 0x02:
                    Serial.println("Trans Gear: 2");
                    break;
                case 0x3:
                    Serial.println("Trans Gear: 3");
                    break;
                case 0x4:
                    Serial.println("Trans Gear: 4");
                    break;
                case 0x5:
                    Serial.println("Trans Gear: 5");
                    break;
                case 0x6:
                    Serial.println("Trans Gear: Drive");
                    break;
                case 0x10:
                    Serial.println("Trans Gear: Reverse");
                    break;
                case 0x11:
                    Serial.println("Trans Gear: Reserved");
                    break;
                case 0x16:
                    Serial.println("Trans Gear: 6");
                    break;
                case 0x17:
                    Serial.println("Trans Gear: 7");
                    break;
               case 0x18:
                    Serial.println("Trans Gear: 8");
                    break;
               case 0x19:
                    Serial.println("Trans Gear: 9");
                    break;
               case 0xF0:
                    Serial.println("Trans Gear: Park");
                    break;
               case 0xF1:
                    Serial.println("Trans Gear: Neutral");
                    break;
               case 0xFF:
                    Serial.println("Trans Gear: Invalid Data");
                    break;
            }
            // getting rid of this message send as gear position is sent on the same CAN ID as RPM (AS ABOVE)
            //unsigned char sendActualGearPositionPre[8] = {0xABC, counterActualGearPos, actualGearPosition, 0, 0, 0, 0, 0};
            //checksum = crc8.get_crc8(sendActualGearPositionPre, 8, 0x70, 1);    
            //unsigned char sendActualGearPosition[8] = {checksum, counterActualGearPos, actualGearPosition, 0, 0, 0, 0, 0}; //
            //CAN1.sendMsgBuf(0xABC, 0, 8, sendActualGearPosition);
            //counterActualGearPos ++;
            //if (counterActualGearPos == 0xF)
            //{
            //    counterActualGearPos = 0x0;
            //}
            //Serial.println("Trans Gear Position data Tx.");
            //delay(100);
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
                    break;
                case 0xFF:
                    //Invalid Data
                    Serial.println("Invalid Odometer Count Data.");
                    break;
            }
            if (odoCount > 0 < 0xFF)
            {
                // Valid Data
                Serial.println("Odometer Count: ");
                Serial.println(odoCountMulti);
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
            flagEngineCoolantOverTemperatureWarning = false;
            int engineCoolantTemperature = ((int)buf[0] - 40);
            if (engineCoolantTemperature > 0x64) // need to scale this number to reflect an overtemp condition
            {
                Serial.println("Warning: Engine Coolant Temperature High. + 140 Degrees C.");
                flagEngineCoolantOverTemperatureWarning = true;
            }
            if (flagEngineCoolantOverTemperatureWarning == true)
            {
                // Flash Coolant OverHeat + Check Engine Light on Cluster.
                int engineCoolantTemperatureCheckEngineLight = 0x01;
                unsigned char flagEngineCoolantOverTemperatureWarningPre[8] = {0xABC, counterDataEctFlag, engineCoolantTemperatureCheckEngineLight, 0, 0, 0, 0, 0};
                checksum = crc8.get_crc8(flagEngineCoolantOverTemperatureWarningPre, 8, 0x70, 1);
                unsigned char flagEngineCoolantOverTemperatureWarning[8] = {checksum, counterDataEctFlag, engineCoolantTemperatureCheckEngineLight, 0, 0, 0, 0, 0}; //
                CAN1.sendMsgBuf(0xABC, 0, 8, flagEngineCoolantOverTemperatureWarning);
                counterDataEctFlag ++;
                if (counterDataEctFlag == 0xF)
                {
                    counterDataEctFlag = 0x0;
                }
            }
            unsigned char sendEngineCoolantTemperaturePre[8] = {0xABC, counterDataEct, engineCoolantTemperature, 0, 0, 0, 0, 0};
            checksum = crc8.get_crc8(sendEngineCoolantTemperaturePre, 8, 0x70, 1);
            unsigned char sendEngineCoolantTemperature[8] = {checksum, counterDataEct, engineCoolantTemperature, 0, 0, 0, 0, 0}; //
            CAN1.sendMsgBuf(0xABC, 0, 8, sendEngineCoolantTemperature);
            counterDataEct ++;
            if (counterDataEct == 0xF)
            {
                counterDataEct = 0x0;
            }
            Serial.println("Engine Coolant Temperature Tx data.");
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //      _____         .__   _____                    __  .__              .___            .___.__               __               .____    .__       .__     __   
            //     /     \ _____  |  |_/ ____\_ __  ____   _____/  |_|__| ____   ____ |   | ____    __| _/|__| ____ _____ _/  |_  ___________|    |   |__| ____ |  |___/  |_ 
            //    /  \ /  \\__  \ |  |\   __\  |  \/    \_/ ___\   __\  |/  _ \ /    \|   |/    \  / __ | |  |/ ___\\__  \\   __\/  _ \_  __ \    |   |  |/ ___\|  |  \   __\
            //   /    Y    \/ __ \|  |_|  | |  |  /   |  \  \___|  | |  (  <_> )   |  \   |   |  \/ /_/ | |  \  \___ / __ \|  | (  <_> )  | \/    |___|  / /_/  >   Y  \  |  
            //   \____|__  (____  /____/__| |____/|___|  /\___  >__| |__|\____/|___|  /___|___|  /\____ | |__|\___  >____  /__|  \____/|__|  |_______ \__\___  /|___|  /__|  
            //           \/     \/                     \/     \/                    \/         \/      \/         \/     \/                          \/ /_____/      \/      
            // 0x427 bytes 5 & 6  MIL Lamps - set flags as these messages will appear as switches, on and off, not a constant stream of the same data.
            // Byte 5                          
            // Bit 1 40 Oil Light on IC (if not out by 4 secs will beep and flash) 1 0x8 | 0xC | 0xE | 0xF
            // Bit 2 41 Cruise Set on IC 1 0x4 | 0x6 | 0x7 | 0xF
            // Bit 3 42 ALT_FAILURE_STAT 0=Off;1=On 1 0x2 | 0x3 | 0x7 | 0xF
            // Bit 4 43 OilPressureWarning 0=Off;1=On 1 0x1 | 0x3 | 0x7 | 0xF
            // Bit 5 44 EngineOverheat 0=Normal;1=OverheatWarning 0x8
            // Bit 6 45 DiffLockWarning 0=Off;1=Engaged
            // Bit 7 46 IdleMode 0=No;1=Yes
            // Bit 8 47 AC_Compressor 0=Off;1=On
            // Byte 6  
            // Bit 1 48 ETC_Warning  0=telltale_off  1=telltale_on  2=telltale_flash 0x00 | 0x01 | 0x02
            // Bit 2 49 ETC_Warning  0=telltale_off  1=telltale_on  2=telltale_flash 0x00 | 0x01 | 0x02
            // Bit 3 50 ImmobLamp  0=off  1=on  2=flash
            // Bit 4 51 ImmobLamp  0=off  1=on  2=flash
            // Bit 5 52 MIL_Lamp  0=off  1=on  2=flash 0010 0011 0x2 | 0x3
            // Bit 6 53 MIL_Lamp  0=off  1=on  2=flash     
            // Bit 7 54 AC_ShedLoad 0=NotDeployed;1=Deployed
            // Bit 8 55 DTC_Logging_HS 0=DoNotLog;1=LogDTCs 0000 0001
            if ((int)buf[5] | buf[6] == 0x00)
            {
                flagEngineOilPressureWarning = false;
                flagEngineOilPressureWarningFlashing = false;
                flagCruiseControlSet = false;
                flagAlternatorFailureState = false;
                flagEngineOverHeatWarning = false;
                flagDiffLockWarning = false;
                flagIdleMode = false;
                flagAirConCompressor = false;
                flagElectronicThrottleControlWarning = false;
                flagElectronicThrottleControlWarningFlashing = false;
                flagImmobilizerLamp = false;
                flagImmobilizerLampFlashing = false;
                flagAirConShedLoad = false;
                flagDtcLoggingHighSpeedCan  = false;
                flagMilLampIlluminated = false;
            } 
            else if ((int)buf[5] == 0x10)            
            {
                flagEngineOilPressureWarning = true;
                int engineOilPressureWarningLight = 0x01;
                unsigned char engineOilPressureWarningPre[8] = {0xABC, counterEngineOilPressureFlag, engineOilPressureWarningLight, 0, 0, 0, 0, 0};
                checksum = crc8.get_crc8(engineOilPressureWarningPre, 8, 0x70, 1);
                unsigned char engineOilPressureWarning[8] = {checksum, counterEngineOilPressureFlag, engineOilPressureWarningLight, 0, 0, 0, 0, 0}; // MIL LAMP MESSAGE FOR KOMBI 
                CAN1.sendMsgBuf(0xABC, 0, 8, engineOilPressureWarning);
                counterEngineOilPressureFlag ++;
                if (counterEngineOilPressureFlag == 0xF)
                {
                    counterEngineOilPressureFlag = 0x0;
                }
                Serial.println("[Barra PCM] Engine Oil Pressure Warning.");   
            }
            else if ((int)buf[5] == 0x20)            
            {
                flagAlternatorFailureState = true;
                int alternatorFailureStateLight = 0x01;
                unsigned char alternatorFailureStatePre[8] = {0xABC, counterAlternatorFailureState, alternatorFailureStateLight, 0, 0, 0, 0, 0};
                checksum = crc8.get_crc8(alternatorFailureStatePre, 8, 0x70, 1);
                unsigned char alternatorFailureState[8] = {checksum, counterAlternatorFailureState, alternatorFailureStateLight, 0, 0, 0, 0, 0}; // MIL LAMP MESSAGE FOR KOMBI 
                CAN1.sendMsgBuf(0xABC, 0, 8, alternatorFailureState);
                counterAlternatorFailureState ++;
                if (counterAlternatorFailureState == 0xF)
                {
                    counterAlternatorFailureState = 0x0;
                }
                Serial.println("[Barra PCM] Alternator Failure State.");   
            }
            else if ((int)buf[5] == 0x08)        // remove this as it is doubling up and the same function is above //    
            {
                flagEngineOverHeatWarning = true;
                unsigned char engineOverHeatWarning[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // MIL LAMP MESSAGE FOR KOMBI 
                CAN1.sendMsgBuf(0xABC, 0, 8, engineOverHeatWarning);
                Serial.println("[Barra PCM] Engine Over Heat Warning.");   
            }
            else if ((int)buf[6] == 0x02 | 0x03)            
            {
                // send can message to illuminate Kombi check engine light here
                flagMilLampIlluminated = true;
                int milLampIlluminatedFlag = 0x01;
                unsigned char milLampIlluminatedPre[8] = {0xABC, counterMilLampIlluminated, milLampIlluminatedFlag, 0, 0, 0, 0, 0};
                checksum = crc8.get_crc8(milLampIlluminatedPre, 8, 0x70, 1);
                unsigned char milLampIlluminated[8] = {checksum, counterMilLampIlluminated, milLampIlluminatedFlag, 0, 0, 0, 0, 0}; // MIL LAMP MESSAGE FOR KOMBI 
                CAN1.sendMsgBuf(0xABC, 0, 8, milLampIlluminated);
                counterMilLampIlluminated ++;
                if (counterMilLampIlluminated == 0xF)
                {
                    counterMilLampIlluminated = 0x0;
                }

                Serial.println("[Barra PCM] MIL Lamp Illuminated.");   
            }
            //delay(100); // one delay of 100 MS per CAN ID, not individual byte functions....too slow otherwise..lags
            
            // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        // PCM_DiagSig_Rx, Tx ID's 0x7DF, 0x7E0, 0x7E8 TCM 0x7E1, 0x7E9
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
        if (canId == 0x7E1)            // TCM_DiagSig_Rx (From tester)
        {
            Serial.println("TCM_DiagSig_Rx: ");
            Serial.println(canId, HEX);
            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.println(buf[i], HEX);
                Serial.println("\t");
            }
        }
        if (canId == 0x7E9)            // TCM_DiagSig_Tx (From ECU)
        {
            Serial.println("TCM_DiagSig_Tx: ");
            Serial.println(canId, HEX);
            for(int i = 0; i<len; i++)    // print the data
            {
                Serial.println(buf[i], HEX);
                Serial.println("\t");
            }
        }
    }
    if(CAN_MSGAVAIL == CAN1.checkReceive())            // check if data coming
    {
    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        CAN1.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        Serial.println("CANbus: BMW Powertrain CAN:");
        unsigned long canId2= CAN1.getCanId();      
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
        // ASSUMING THAT VEHICLE SPEED SOURCE IS SET TO 'ABS via CAN' with PCMTEC, WE NEED TO PROVIDE THAT SIGNAL FOR THE BARRA PCM VIA THE CANBUS
        // Recieve ABS individual wheel speed signals from BMW ABS module, and re-transmit on ID 0x4B0 for Barra PCM
        if (canId2 == 0x0CE) // BMW WHEEL SPEED ID FROM ABS CAN1
        {
            int wheelSpeedFrontLeft1  = (int)buf[0]; // BMW DATA is * 0.0625
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
            if ( wheelSpeedFrontLeft2 | wheelSpeedRearRight2 | wheelSpeedRearLeft2 | wheelSpeedRearRight2 == 0xFE)
            {
                Serial.println("ABS Wheel Speed Signal INITIALIZATION IN PROGRESS.");                
            } 
            else if (wheelSpeedFrontLeft1 | wheelSpeedFrontLeft2 | wheelSpeedFrontRight1 | wheelSpeedRearRight2 | wheelSpeedRearLeft1 | wheelSpeedRearLeft2 | wheelSpeedRearRight1 | wheelSpeedRearRight2 == 0xFF)
            {
                Serial.println("ABS Wheel Speed Signal ERROR INVALID DATA.");
            }
            //delay(100);   
        }
    }
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
