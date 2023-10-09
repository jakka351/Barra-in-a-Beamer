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
char serialInputBuffer[500];
int serialInputBuffercount = 0;
int OdoCount = 0;
int Throttle = 0;
int RPM = 0;
int Rpm = 0;
int Speed = 0;
double steeringAngle = 0;
byte sendPrimaryCan = 0;
byte gearPosition = 0;
byte stickCode = 0;
byte serviceBrakes = 0;
byte performanceMode = 0;
MCP_CAN CAN(10);
//////////////////////////////////////////////////////////////////////////////
void setup() 
{
  //////////////////////////////////////////
  // put your setup code here, to run once:
  Serial.begin(115200);
  memset(&serialInputBuffer[0], 0, sizeof(serialInputBuffer)); //init serial buffer
  //////////////////////////////////////////////////////////////
START_INIT:
  if (CAN_OK == CAN.begin(CAN_500KBPS))                  // init can bus : baudrate = 500k
  {
    Serial.println("CAN Interface Up");
  }
  else
  {
    Serial.println("CAN Interface Down");
    Serial.println("CAN Interface Down, Retry");
    delay(125);
    goto START_INIT;
  }
}
/////////////////////////////////////////////////////////////////////////////
// Speedometer Message
// Tachometer Message
// Odometer Count
// Engine Coolant Temp
// Engine Oil Temp
// Warning Lights
// 
void loop() {
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    int ArbitrationIdentifier = CAN.getCanId();
    if (sendPrimaryCan == 1)
    {
      Serial.print("CAN:");
      Serial.print(ArbitrationIdentifier);
      Serial.print(",");
      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    //////////////////////////////////////// DELETE
    if (ArbitrationIdentifier == 0x85) // YAW RATE SENSOR
    {
      Serial.print("CAN:");
      Serial.print(ArbitrationIdentifier);
      Serial.print(",");
      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    //////////////////////////////////////////// DELETE
    if (ArbitrationIdentifier == 0x90) // STEERING ANGLE SENSOR
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
    // Powertrain Control Module  Engine Torque Calculation 0x97  8 IndicatedEngineTorque IndicatedEngineTorque FrictionLoss  FrictionLoss  ActualEngineTorque  ActualEngineTorque  DriverDemandedTorque  DriverDemandedTorque
    // Powertrain Control Module ABSConfigShiftMap FC  8 Traction Control, EBD_PCM 
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (ArbitrationIdentifier == 0x12D) // Powertrain Control Module EngineSpdRateOfChange 12D 8 EngineSpeedRateOfChange AcceleratorPedalPosition  AcceleratorPedalPosition  DSC On/Off Flag EngineSpeed EngineSpeed Cruise Control On BrakeStatus
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

      if (serviceBrakes != buf[7])
      {
        serviceBrakes = buf[7];
        if (serviceBrakes == 1)
        {
          Serial.println("serviceBrakes:ON");
        } else {
          Serial.println("serviceBrakes:OFF");
        }
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (ArbitrationIdentifier == 0x207) // Powertrain Control Module EngineRPM 207 8 EngineRPM EngineRPM EngineSpeedRateOfChange EngineSpeedRateOfChange VehicleSpeed  VehicleSpeed  ThrottlePositionManifold  ThrottlePositionRateOfChange
    {
      int Valx1 = (int)buf[0]; // ENGINE RPM
      int Valx2 = (int)buf[1]; // ENGINE RPM
      int Valx5 = (int)buf[4]; // VEHICLE SPEED
      int Valx6 = (int)buf[5]; // VEHICLE SPEED
      float tmpSpeed = (Valx5 + (Valx6 / 255)) * 2;
      if (tmpSpeed != Speed)
      {
        Speed = tmpSpeed;
        Serial.print("Speed:");
        Serial.println(Speed);
        unsigned char msgSpeed[8] = {Speed, 0, 0, 0, 0, 0, 0, 0}; // need to scale and convert Speed to match
        CAN.sendMsgBuf(0xABC, 0, 8, msgSpeed);
        Serial.println("Engine RPM Signal sent.\r\n");
      }
      float tmpRpm = (Valx1 + (Valx2 / 255)) * 2;
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
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (ArbitrationIdentifier == 0x230) //Powertrain Control Module  Transmission Mode 230 8 Transmission Gear Pos TargetgearPositionition  ActualgearPositionition  TorqueConverter   -  -   -  Trans Mode Fault Warn
    {
      int Valx2 = buf[1];
      int tmpgearPosition = 0;
      if (Valx2 == 255)
      {
        tmpgearPosition = 1;
      }
      else if (Valx2 == 149)
      {
        tmpgearPosition = 2;
      }
      else if (Valx2 == 97)
      {
        tmpgearPosition = 3;
      }
      else if (Valx2 == 72)
      {
        tmpgearPosition = 4;
      }
      else if (Valx2 == 55)
      {
        tmpgearPosition = 5;
      }
      else if (Valx2 == 44)
      {
        tmpgearPosition = 6;
      }
      if (tmpgearPosition != gearPosition)
      {
        gearPosition = tmpgearPosition;
        Serial.print("transgear:");
        Serial.println(gearPosition);
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (ArbitrationIdentifier == 0x3E9) //Transmission Control Module  TrransmissionActualGear 3E9 8 GearActualAndSelected TransmissionShiftMap Normal Mode, Cruise Mode, Hot Mode, Traction Control Mode, Hill/Trailer Towing Mode, Performance Mode  TransmissionOilTemp TCM MIL Status  Performance Mode Indicator  IdleGearboxLoss GeaerPositionTarget TorqueConverterMultiplier
    {
      if (stickCode != buf[0])
      {
        stickCode = buf[0];
        Serial.print("gearstickpos:");
        Serial.println(stickCode);
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (ArbitrationIdentifier == 0x437) //InstrumentCluster  FuelData HandbrakeState 437 8 FuelLevelDamped FuelLevelInstant  HandbrakeStatus/LowFuelIndicator/FuelSenderFault/ParkLights/MILStrategyOn/  ClusterVoltage  IlluminationLevelLow  IlluminationLevelHigh PRNDL?  MaxFuelLevel
    {
      Serial.print("fuelstatus:");
      Serial.print(buf[0]);
      Serial.print(",");
      Serial.println(buf[1]);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (ArbitrationIdentifier == 0x4B0) //Antilock Brake Module  Wheel Speed Sensor  4B0 8 WheelSpeedFrontLeft Units:km/h   Offset:0;Multi:1;Div:100   0xFFFE=Initialization in progress   0xFFFF=Wheel Speed Faulted        WheelSpeedFrontLeft Units:km/h  Offset:0;Multi:1;Div:100  0xFFFE=Initialization in progress  0xFFFF=Wheel Speed Faulted       FrontRightWheelSpeed  Units:km/h  Offset:0;Multi:1;Div:100  0xFFFE=Initialization in progress  0xFFFF=Wheel Speed Faulted   FrontRightWheelSpeed  RearLeftWheelSpeed  RearLeftWheelSpeed  RightRearWheelSpeed RightRearWheelSpeed
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
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Powertrain Control Module  Cruise  425 8 Cruise Control On 0x01  CruiseSetSpeed  CruiseSetSpeed  BoostPressue  BoostPressue  Fuel Level  InstantFuelConsumption  InstantFuelConsumption
    // Powertrain Control Module EngineDtcParameters 427 8 EngineCoolantTemperature  Air Conditioner Pressure  Air Conditioner Pressure  Battery Voltage Odometer Count  MIL Lamp  Comms Fault EngineSpeedCount
    // Powertrain Control Module Cruise  425 8 Cruise Control On 0x01  CruiseSetSpeed  CruiseSetSpeed  BoostPressue  BoostPressue  Fuel Level  InstantFuelConsumption  InstantFuelConsumption
    // Powertrain Control Module EngineDtcParameters 427 8 EngineCoolantTemperature  Air Conditioner Pressure  Air Conditioner Pressure  Battery Voltage Odometer Count  MIL Lamp  Comms Fault EngineSpeedCount
    // Powertrain Control Module Odometer  4C0 8 Odometer  Odometer  Odometer
    ///////////////////////////////////////////////////////
    //PCM_DiagSig_Tx // 
    if (ArbitrationIdentifier == 0x7E8)
    {
      Serial.print("CAN:");
      Serial.print(ArbitrationIdentifier);
      Serial.print(",");
      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    ///////////////////////////////////////////////////////
    //TCM_DiagSig_Tx //
    if (ArbitrationIdentifier == 0x7E9)
    {
      Serial.print("CAN:");
      Serial.print(ArbitrationIdentifier);
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
        if (strcmp(serialInputBuffer, "SENDMAINCAN") == 0)
        {
          sendPrimaryCan = 1;
        }
        else if (strcmp(serialInputBuffer, "STOPMAINCAN") == 0)
        {
          sendPrimaryCan = 0;
        }
        else if (strcmp(serialInputBuffer, "OBDCOOLANT") == 0)
        {
          ///////////////////////////////////////////////////////
          //obd = additional bytes, current data/freeframe,PID,3-7 unused
          //2 additional bytes = current data and pid
          unsigned char msgtemp[8] = {2, 1, 5, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);

          Serial.println("CAN MESSAGE SENT FOR OBD COOLANT");
        }
        else if (strcmp(serialInputBuffer, "OBDINTAKETEMP") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 15, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR INTAKE TEMP");
        }
        else if (strcmp(serialInputBuffer, "OBDINTAKEPSI") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 11, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR OBD COOLANT");
        }
        else if (strcmp(serialInputBuffer, "OBDAIRTEMP") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 70, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR OBD AMBIENT AIR TEMP");
        }
        else if (strcmp(serialInputBuffer, "OBDECUVOLTAGE") == 0)
        {
          unsigned char msgtemp[8] = {2, 1, 66, 0, 0, 0, 0, 0};
          CAN.sendMsgBuf(0x7DF, 0, 8, msgtemp);
          Serial.println("CAN MESSAGE SENT FOR OBD ECU VOLTAGE");
        }
        else if (strcmp(serialInputBuffer, "GetStatus") == 0)
        {
          getstatus();
        }
        memset(&serialInputBuffer[0], 0, sizeof(serialInputBuffer));
        serialInputBuffercount = 0;
        ///////////////////////////////////////////////////////
      }
    }
    else if (byteread < 128)
    {    
      ///////////////////////////////////////////////////////
      //Valid characters.
      serialInputBuffer[serialInputBuffercount] = byteread;
      serialInputBuffercount++;
    }
  }
}
void getstatus()
{
  Serial.print("transgear:");
  Serial.println(gearPosition);
  Serial.print("gearstickpos:");
  Serial.println(stickCode);
  ///////////////////////////////////////////////////////
  if (serviceBrakes == 1)
  {
    Serial.println("serviceBrakes:ON");
  } else {
    Serial.println("serviceBrakes:OFF");
  }
}
///////////////////////////////////////////////////////
