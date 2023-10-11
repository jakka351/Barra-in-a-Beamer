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
// /////                                     Barra in a Beamer Arduino Code by Jakka351
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
// /////        4.    Neither the name of the organization nor the names of its contributors may be used to
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
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////                     .__      ___.   .__                 
// //// ___  _______ _______|__|____ \_ |__ |  |   ____   ______
// //// \  \/ /\__  \\_  __ \  \__  \ | __ \|  | _/ __ \ /  ___/
// ////  \   /  / __ \|  | \/  |/ __ \| \_\ \  |_\  ___/ \___ \ 
// ////   \_/  (____  /__|  |__(____  /___  /____/\___  >____  >
// ////             \/              \/    \/          \/     \/
// //// 
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long unsigned int rxId;                         //
unsigned char len = 0;                          //
unsigned char rxBuf[8];                         //
char msgString[128];                            // Array to store serial string
#define CAN0_INT 2                              // Set INT to pin 2
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
      serial.println("Barra in a Beamer // MCP2515 Initialized. ");
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
    if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
        if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
        {
            sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len); // print extended ID
        }
        else
        {
            sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);           // print 11 bit ID
            // test code starts here
            if (rxId == 0x207) // Powertrain Control Module EngineRPM 207 8 EngineRPM EngineRPM EngineSpeedRateOfChange EngineSpeedRateOfChange VehicleSpeed  VehicleSpeed  ThrottlePositionManifold  ThrottlePositionRateOfChange
            {
                //////////////////////////////////////////////////////////////////
                int Valx0 = (int)buf[0]; // ENGINE RPM
                int Valx1 = (int)buf[1]; // ENGINE RPM
                int Valx4 = (int)buf[4]; // VEHICLE SPEED
                int Valx5 = (int)buf[5]; // VEHICLE SPEED
                //////////////////////////////////////////////////////////////////
                float tmpSpeed = (Valx4 + (Valx5 / 255)) * 2;
                if (tmpSpeed != Speed)
                {
                  byte data[8] = {0x00, 0x01, 0x02, 0x03, Speed, 0x06, 0x07};
                  Speed = tmpSpeed;
                  Serial.print("Speed:");
                  Serial.println(Speed);
                  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
                  if(sndStat == CAN_OK)
                  {
                      Serial.println("Message Send success.");
                  } 
                  else 
                  {
                      Serial.println("Error Sending Message.");
                  }
                  delay(100);   
                }
                //////////////////////////////////////////////////////////////////
                float tmpRpm = (Valx0 + (Valx1 / 255)) * 2;
                if (tmpRpm != Rpm)
                {
                  Rpm = tmpRpm;
                  Serial.print("RPM:");
                  Serial.println(Rpm);

                  unsigned char msgRpm[8] = {Rpm, 0, 0, 0, 0, 0, 0, 0}; // need to scale and convert Rpm to match
                  CAN.sendMsgBuf(0xABC, 0, 8, msgSpeed);
                  Serial.println("Engine RPM Signal sent.\r\n");
                }
            }
        }
        Serial.print(msgString); // the actual printer
  
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
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                            .___                                                                             
    //    ______ ____   ____    __| _/   ____ _____    ____     _____   ____   ______ ___________     ____   ____  
    //   /  ___// __ \ /    \  / __ |  _/ ___\\__  \  /    \   /     \_/ __ \ /  ___//  ___/\__  \   / ___\_/ __ \ 
    //   \___ \\  ___/|   |  \/ /_/ |  \  \___ / __ \|   |  \ |  Y Y  \  ___/ \___ \ \___ \  / __ \_/ /_/  >  ___/ 
    //  /____  >\___  >___|  /\____ |   \___  >____  /___|  / |__|_|  /\___  >____  >____  >(____  /\___  / \___  >
    //       \/     \/     \/      \/       \/     \/     \/        \/     \/     \/     \/      \//_____/      \/ 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
    if(sndStat == CAN_OK)
    {
        Serial.println("Message Sent Successfully!");
    } 
    else 
    {
        Serial.println("Error Sending Message...");
    }
    delay(100);                                      // send data per 100ms

}
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


