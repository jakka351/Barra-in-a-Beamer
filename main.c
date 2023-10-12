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
// /////      | Copyright (c) 2022/2023 Jack Leighton                                                                        |          
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
#include <SPI.h>
#include "mcp_can.h"
#define CAN0_INT 2                              // Set INT to pin 2
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////                     .__      ___.   .__                 
// //// ___  _______ _______|__|____ \_ |__ |  |   ____   ______
// //// \  \/ /\__  \\_  __ \  \__  \ | __ \|  | _/ __ \ /  ___/
// ////  \   /  / __ \|  | \/  |/ __ \| \_\ \  |_\  ___/ \___ \ 
// ////   \_/  (____  /__|  |__(____  /___  /____/\___  >____  >
// ////             \/              \/    \/          \/     \/
// //// 
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int V_VEH;                                      // Vehicle Speed 
int CHKSM_V_V;                                  // checksum @ 0x1A0 Byte[7] 
int Rpm;                                        // 
int RPM_TEMP_DOM_1;                             // Rpm1 input 
int RPM_TEMP_DOM_2;                             // Rpm2 input
int odoCount;                                   // 
long unsigned int rxId;                         //
unsigned char len = 0;                          //
unsigned char rxBuf[8];                         //
char msgString[128];                            // Array to store serial string
MCP_CAN CAN0(10);                               // Set CS to pin 10
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
  // put your setup code here, to run once:
  Serial.begin(115200);                                        // start serial @ 115200 Baud rate
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)    // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  {
      Serial.println("Barra in a Beamer // MCP2515 Initialized. ");
  }
  else
  {
      Serial.println("Barra in a Beamer //  Error Initializing MCP2515.");
  }
  CAN0.setMode(MCP_NORMAL);                            // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  Serial.println("Barra in a Beamer // jakka351");
}
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////  .__                        
// ////  |  |   ____   ____ ______  
// ////  |  |  /  _ \ /  _ \\____ \ 
// ////  |  |_(  <_> |  <_> )  |_> >
// ////  |____/\____/ \____/|   __/ 
// ////                     |__|    
// ////
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
    // put your main code here, to run repeatedly:
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                             .__                                   ___.                                                                              
    // _______   ____   ____  ____ |__|__  __ ____     ____ _____    ____\_ |__  __ __  ______   _____   ____   ______ ___________     ____   ____   ______
    // \_  __ \_/ __ \_/ ___\/ __ \|  \  \/ // __ \  _/ ___\\__  \  /    \| __ \|  |  \/  ___/  /     \_/ __ \ /  ___//  ___/\__  \   / ___\_/ __ \ /  ___/
    //  |  | \/\  ___/\  \__\  ___/|  |\   /\  ___/  \  \___ / __ \|   |  \ \_\ \  |  /\___ \  |  Y Y  \  ___/ \___ \ \___ \  / __ \_/ /_/  >  ___/ \___ \ 
    //  |__|    \___  >\___  >___  >__| \_/  \___  >  \___  >____  /___|  /___  /____//____  > |__|_|  /\___  >____  >____  >(____  /\___  / \___  >____  >
    //              \/     \/    \/              \/       \/     \/     \/    \/           \/        \/     \/     \/     \/      \//_____/      \/     \/     // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(!digitalRead(CAN0_INT))                    // If CAN0_INT pin is low, read receive buffer
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);           // print 11 bit ID
        Serial.print(msgString);                       // the actual printer
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
        // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // powertrainControlModule code notesL
        // CAN ID 0x207 Data Bytes:  [0]EngineRPM [1]EngineRPM [2]EngineSpeedRateOfChange [3]EngineSpeedRateOfChange [4]VehicleSpeed  [5]VehicleSpeed  [6]ThrottlePositionManifold  [7]ThrottlePositionRateOfChange
        if (rxId == 0x207) 
        {
            //////////////////////////////////////////////////////////////////
            int Valx0 = (int)rxBuf[0]; // ENGINE RPM
            int Valx1 = (int)rxBuf[1]; // ENGINE RPM
            int Valx4 = (int)rxBuf[4]; // VEHICLE SPEED
            int Valx5 = (int)rxBuf[5]; // VEHICLE SPEED
            //////////////////////////////////////////////////////////////////
            float tmpSpeed = (Valx4 + (Valx5 / 255)) * 2;
            if (tmpSpeed != V_VEH)
            {
                V_VEH = (tmpSpeed * 0.1);    // From Barra PCM then calced to KM/H, then multiply KMH * 0.1 to correct for BMW cluster input
                CHKSM_V_V = (V_VEH + 0x40);  // TBA
                Serial.print("[V_VEH]Vehicle Speed km/h:");
                Serial.println(V_VEH);
                //////////////////////////////////////
                // CAN ID 0x1A0 Byte 0 - Vehicle Speed 
                // V_VEH    0   12  Intel   Unsigned    0.1 0   km/h 
                byte dataSpd[8] = {V_VEH, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CHKSM_V_V};
                byte sndSpd = CAN0.sendMsgBuf(0x1A0, 0, 8, dataSpd);  
                if(sndSpd == CAN_OK)
                {
                    Serial.println("[V_VEH]Message Send success.");
                } 
                else 
                {
                    Serial.println("[V_VEH]Error Sending Message.");
                }
                delay(100);   
            }
            //////////////////////////////////////////////////////////////////
            float tmpRpm = (Valx0 + (Valx1 / 255)) * 2;
            if (tmpRpm != Rpm)
            {
                Rpm = tmpRpm;
                RPM_TEMP_DOM_1 = Rpm;
                RPM_TEMP_DOM_2 = Rpm;
                Serial.print("RPM:");
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
                byte dataRpm[2] = {RPM_TEMP_DOM_1, RPM_TEMP_DOM_2};
                byte sndRpm = CAN0.sendMsgBuf(0x332, 0, 2, dataRpm);
                if(sndRpm == CAN_OK)
                {
                    Serial.println("Message Send success.");
                } 
                else 
                {
                    Serial.println("Error Sending Message.");
                }
                delay(100);
            }
        }
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  _______            _____ _________________       ________       .___                     __                 _________                      __   
        //  \   _  \ ___  ___ /  |  |\_____  \______  \      \_____  \    __| _/____   _____   _____/  |_  ___________  \_   ___ \  ____  __ __  _____/  |_ 
        //  /  /_\  \\  \/  //   |  |_/  ____/   /    /       /   |   \  / __ |/  _ \ /     \_/ __ \   __\/ __ \_  __ \ /    \  \/ /  _ \|  |  \/    \   __\
        //  \  \_/   \>    </    ^   /       \  /    /       /    |    \/ /_/ (  <_> )  Y Y  \  ___/|  | \  ___/|  | \/ \     \___(  <_> )  |  /   |  \  |  
        //   \_____  /__/\_ \____   |\_______ \/____/        \_______  /\____ |\____/|__|_|  /\___  >__|  \___  >__|     \______  /\____/|____/|___|  /__|  
        //         \/      \/    |__|        \/                      \/      \/            \/     \/          \/                \/                  \/            
        // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // odometerCount code notes
        // BARRA CAN ID: 0x427 Byte [4] Odometer Count Byte [7] Engine Speed Count
        // Unit:km  Offset:0,Mult:0.000201167, Div:1  FF=Invalid
        if (rxId == 0x427) 
        {
            int odoCount = (int)rxBuf[4];
            float odoCountMulti = (odoCount * 0.000201167);
            if (odoCount == 0)
            {
                //Vehicle Stationary
                //pass;   
            }
            else if (odoCount > 0 < 0xFF)
            {
                //Valid Data Range
            } 
            else if (odoCount == 0xFF)
            {
                //Invalid Data
            }

        }
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  _______            ______________________  ____________________  _______  ________  .____            /\      /\ .___.____    .____     ____ ___  _____  .____     _______________   _______________.____     
        //  \   _  \ ___  ___ /  |  \_____  \______  \ \______   \______   \ \      \ \______ \ |    |          / /     / / |   |    |   |    |   |    |   \/     \ |    |    \_   _____/\   \ /   /\_   _____/|    |    
        //  /  /_\  \\  \/  //   |  |__(__  <   /    /  |     ___/|       _/ /   |   \ |    |  \|    |         / /     / /  |   |    |   |    |   |    |   /  \ /  \|    |     |    __)_  \   Y   /  |    __)_ |    |    
        //  \  \_/   \>    </    ^   /       \ /    /   |    |    |    |   \/    |    \|    `   \    |___     / /     / /   |   |    |___|    |___|    |  /    Y    \    |___  |        \  \     /   |        \|    |___ 
        //   \_____  /__/\_ \____   /______  //____/    |____|    |____|_  /\____|__  /_______  /_______ \   / /     / /    |___|_______ \_______ \______/\____|__  /_______ \/_______  /   \___/   /_______  /|_______ \
        //         \/      \/    |__|      \/                            \/         \/        \/        \/   \/      \/                 \/       \/               \/        \/        \/                    \/         \/
        // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // PRNDL & illumination level
        // BARRA: CAN ID 0x437:
        //      Byte[0] FuelLevelDamped, Byte[1] FuelLevelInstant, Byte[2] HandbrakeStatus/LowFuelIndicator/FuelSenderFault/ParkLights/MILStrategyOn, Byte[3] ClusterVoltage  
        //      Byte[4]IlluminationLevelLow, Byte[5] IlluminationLevelHigh, Byte[6] PRNDL, Byte[7] MaxFuelLevel 
        // BMW:
        //      CAN ID 0x1D2 Display gearbox data:
        //    Type: CAN Standard
        //    ID: 0x1D2 DLC: 6 Tx method: cycle Cycle time: 200ms
        //    Signal  Start bit   Length  Order   Value type  Factor  Offset  Unit
        //    Message example: 0x1D2 6 E1 0C 8F 7C F0 FF
        if (rxId == 0x437) 
        {

        }

        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //    .___ .__                                      __  .__               
        //   __| _/|__|____     ____   ____   ____  _______/  |_|__| ____   ______
        //  / __ | |  \__  \   / ___\ /    \ /  _ \/  ___/\   __\  |/ ___\ /  ___/
        // / /_/ | |  |/ __ \_/ /_/  >   |  (  <_> )___ \  |  | |  \  \___ \___ \ 
        // \____ | |__(____  /\___  /|___|  /\____/____  > |__| |__|\___  >____  >
        //      \/         \//_____/      \/           \/               \/     \/ 
        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        //
        //
        //
        if (rxId == 0x7DF)            // OBDII Tester Transmit ID 
        {

        }
        if (rxId == 0x7E0)            // PCM_DiagSig_Rx (From tester)
        {

        }
        if (rxId == 0x7E8)            // PCM_DiagSig_Tx (From ECU)
        {

        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if((rxId & 0x40000000) == 0x40000000)          // Determine if message is a remote request frame.
        {    
            sprintf(msgString, "REMOTE REQUEST FRAME");  // print remote request frame
            Serial.print(msgString);
        } 
        else 
        {
            for(byte i = 0; i<len; i++)                  //
            {
                sprintf(msgString, " 0x%.2X", rxBuf[i]);   //
                Serial.print(msgString);                   // print regular can frame
            }
        }
        Serial.println();
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


