// /////     __________________________________________________________________________________________________________________
// /////
// /////     ___________           _____    ________           .__                  _________                                    
// /////     \_   _____/  ____    /  _  \   \_____  \  _______ |__|  ____    ____   \_   ___ \   ____    _____    _____    ______
// /////      |    __)   /  _ \  /  /_\  \   /   |   \ \_  __ \|  | /  _ \  /    \  /    \  \/  /  _ \  /     \  /     \  /  ___/
// /////      |     \   (  <_> )/    |    \ /    |    \ |  | \/|  |(  <_> )|   |  \ \     \____(  <_> )|  Y Y  \|  Y Y  \ \___ \ 
// /////      \___  /    \____/ \____|__  / \_______  / |__|   |__| \____/ |___|  /  \______  / \____/ |__|_|  /|__|_|  //____  >
// /////          \/                    \/          \/                          \/          \/               \/       \/      \/ 
// /////
// /////                           ECU CAN INTERFACE - Originally by Mitchell H. Reworked by Jakka351
// /////     __________________________________________________________________________________________________________________
// /////      |--------------------------------------------------------------------------------------------------------------|
// /////      |https://github.com/jakka351/FG-Falcon  | jakka351@outlook.com |  https://github.com/jakka351/FG-Falcon-Hidden |
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
unsigned char len = 0;
unsigned char buf[8];
byte GearPos = 0;
byte StickCode = 0;
byte SportsShift = 0;
int Throttle = 0;
int RPM = 0;
int Rpm = 0;
int Speed = 0;
byte Brakes = 0;
double SteeringAngle = 0;
byte sendprimarycan = 0;
char serialinputbuffer[500];
int serialinputbuffercount = 0;
MCP_CAN CAN(10);
//////////////////////////////////////////////////////////////////////////////
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  memset(&serialinputbuffer[0], 0, sizeof(serialinputbuffer)); //init serial buffer
  //////////////////////////////////////////////////////////////
START_INIT:
  if (CAN_OK == CAN.begin(CAN_500KBPS))                  // init can bus : baudrate = 500k
  {
    Serial.println("HS CAN Shield Startup OK");
  }
  else
  {
    Serial.println("HS CAN Shield Failed Startup");
    Serial.println("HS CAN Shield Retry Startup");
    delay(100);
    goto START_INIT;
  }
}
/////////////////////////////////////////////////////////////////////////////
void loop() {
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    int NodeID = CAN.getCanId();
    if (sendprimarycan == 1)
    {
      Serial.print("CAN:");
      Serial.print(NodeID);
      Serial.print(",");
      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    //////////////////////////
    if (NodeID == 0x85) // YAW RATE SENSOR
    {
      Serial.print("CAN:");
      Serial.print(NodeID);
      Serial.print(",");
      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    ///////////////////////////////////////////////////////
    if (NodeID == 0x90) // STEERING ANGLE SENSOR
    {
      int MultiVal = buf[0];
      int FineVal = buf[1];
      double TotalVal;
      if (MultiVal > 127)
      {
        MultiVal = ((MultiVal - 255) * - 1) * 255;
        FineVal = (FineVal - 255) * - 1;
        TotalVal = (MultiVal + FineVal) * - 1; //Change to Negative
      } else {
        MultiVal = MultiVal * 255;
        TotalVal = MultiVal + FineVal;
      }
      Serial.print("Steering:");
      Serial.println(TotalVal);
    }
    ///////////////////////////////////////////////////////
    if (NodeID == 0x12D) // Powertrain Control Module EngineSpdRateOfChange 12D 8 EngineSpeedRateOfChange AcceleratorPedalPosition  AcceleratorPedalPosition  DSC On/Off Flag EngineSpeed EngineSpeed Cruise Control On BrakeStatus
    {
      //301X3 = Throttle
      // 0x12D byte 2 AceceleratorPedalPosition
      int tmpThrottle = (int)((int)(buf[2]) / 2);
      if (tmpThrottle != Throttle)
      {
        Throttle = tmpThrottle;
        Serial.print("Throttle:");
        Serial.println(Throttle);
      }
      //RPM = ((X5 * 255) + X6) / 4
      int Valx5 = (int)buf[4];
      int Valx6 = (int)buf[5];
      float rawRPM = ((Valx5 * 255) + Valx6) / 4;
      int tmpRPM = (int)rawRPM;
      if (tmpRPM != RPM)
      {
        RPM = tmpRPM;
        Serial.print("RPM:");
        Serial.println(RPM);
      }

      if (Brakes != buf[7])
      {
        Brakes = buf[7];
        if (Brakes == 1)
        {
          Serial.println("Brakes:ON");
        } else {
          Serial.println("Brakes:OFF");
        }
      }
    }
    ///////////////////////////////////////////////////////
    if (NodeID == 0x207) // Powertrain Control Module EngineRPM 207 8 EngineRPM EngineRPM EngineSpeedRateOfChange EngineSpeedRateOfChange VehicleSpeed  VehicleSpeed  ThrottlePositionManifold  ThrottlePositionRateOfChange
    {
      int Valx1 = (int)buf[0]; // VEHICLE SPEED
      int Valx2 = (int)buf[1]; // VEHICLE SPEED
      int Valx5 = (int)buf[4]; // VEHICLE SPEED
      int Valx6 = (int)buf[5]; // VEHICLE SPEED
      float tmpSpeed = (Valx5 + (Valx6 / 255)) * 2;
      if (tmpSpeed != Speed)
      {
        Speed = tmpSpeed;
        Serial.print("Speed:");
        Serial.println(Speed);
      }
      float tmpRpm = (Valx1 + (Valx2 / 255)) * 2;
      if (tmpRpm != Rpm)
      {
        Rpm = tmpRpm;
        Serial.print("RPM:");
        Serial.println(Rpm);      
        unsigned char msgtemp[8] = {2, 1, 15, 0, 0, 0, 0, 0};
        CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
        Serial.println("CAN MESSAGE SENT FOR INTAKE TEMP");
      }
    }
    ///////////////////////////////////////////////////////
    if (NodeID == 0x230)
    {
      int Valx2 = buf[1];
      int tmpGearPos = 0;
      if (Valx2 == 255)
      {
        tmpGearPos = 1;
      }
      else if (Valx2 == 149)
      {
        tmpGearPos = 2;
      }
      else if (Valx2 == 97)
      {
        tmpGearPos = 3;
      }
      else if (Valx2 == 72)
      {
        tmpGearPos = 4;
      }
      else if (Valx2 == 55)
      {
        tmpGearPos = 5;
      }
      else if (Valx2 == 44)
      {
        tmpGearPos = 6;
      }
      if (tmpGearPos != GearPos)
      {
        GearPos = tmpGearPos;
        Serial.print("transgear:");
        Serial.println(GearPos);
      }
    }
    ///////////////////////////////////////////////////////
    if (NodeID == 0x3E9)
    {

      if (StickCode != buf[0])
      {
        StickCode = buf[0];
        Serial.print("gearstickpos:");
        Serial.println(StickCode);
      }
    }
    ///////////////////////////////////////////////////////
    if (NodeID == 0x437)
    {
      Serial.print("fuelstatus:");
      Serial.print(buf[0]);
      Serial.print(",");
      Serial.println(buf[1]);
    }
    ///////////////////////////////////////////////////////
    if (NodeID == 0x4B0)
    {
      int FrontLeft = (buf[0] * 255) + buf[1];
      int FrontRight = (buf[2] * 255) + buf[3];
      int RearLeft = (buf[4] * 255) + buf[5];
      int RearRight = (buf[6] * 255) + buf[7];

      Serial.print("wheelspeed:");
      Serial.print(FrontLeft);
      Serial.print(",");
      Serial.print(FrontRight);
      Serial.print(",");
      Serial.print(RearLeft);
      Serial.print(",");
      Serial.println(RearRight);
    }
    ///////////////////////////////////////////////////////
    //PCM_DiagSig_Tx // OBD CAN ID
    if (NodeID == 0x7E8)
    {
      Serial.print("CAN:");
      Serial.print(NodeID);
      Serial.print(",");
      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    ///////////////////////////////////////////////////////
    //TCM_DiagSig_Tx OBD CAN ID
    if (NodeID == 0x7E9)
    {
      Serial.print("CAN:");
      Serial.print(NodeID);
      Serial.print(",");
      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  ///////////////////////////////////////////////////////
  if (Serial.available())
  {
    //lower than 32 is a command char.
    int byteread = Serial.read();
    if (byteread < 32)
    {
      //13 = command finished, cr.
      if (byteread == 13)
      {
        ///////////////////////////////////////////////////////
        //Action then clear input buffer.
        if (strcmp(serialinputbuffer, "SENDMAINCAN") == 0)
        {
          sendprimarycan = 1;
        }
        else if (strcmp(serialinputbuffer, "STOPMAINCAN") == 0)
        {
          sendprimarycan = 0;
        }
        else if (strcmp(serialinputbuffer, "OBDCOOLANT") == 0)
        {
          ///////////////////////////////////////////////////////
          //obd = additional bytes, current data/freeframe,PID,3-7 unused
          //2 additional bytes = current data and pid
          unsigned char msgtemp[8] = {2, 1, 5, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);

          Serial.println("CAN MESSAGE SENT FOR OBD COOLANT");
        }
        else if (strcmp(serialinputbuffer, "OBDINTAKETEMP") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 15, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR INTAKE TEMP");
        }
        else if (strcmp(serialinputbuffer, "OBDINTAKEPSI") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 11, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR OBD COOLANT");
        }
        else if (strcmp(serialinputbuffer, "OBDAIRTEMP") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 70, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR OBD AMBIENT AIR TEMP");
        }
        else if (strcmp(serialinputbuffer, "OBDECUVOLTAGE") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 66, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR OBD ECU VOLTAGE");
        }
        else if (strcmp(serialinputbuffer, "GetStatus") == 0)
        {
          getstatus();
        }
        memset(&serialinputbuffer[0], 0, sizeof(serialinputbuffer));
        serialinputbuffercount = 0;
        ///////////////////////////////////////////////////////
      }
    }
    else if (byteread < 128)
    {    
      ///////////////////////////////////////////////////////
      //Valid characters.
      serialinputbuffer[serialinputbuffercount] = byteread;
      serialinputbuffercount++;
    }
  }
}
void getstatus()
{
  Serial.print("transgear:");
  Serial.println(GearPos);
  Serial.print("gearstickpos:");
  Serial.println(StickCode);
  ///////////////////////////////////////////////////////
  if (Brakes == 1)
  {
    Serial.println("Brakes:ON");
  } else {
    Serial.println("Brakes:OFF");
  }
}
///////////////////////////////////////////////////////
