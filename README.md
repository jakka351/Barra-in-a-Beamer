  

<img align="right" src="https://user-images.githubusercontent.com/57064943/166143780-9685fc0f-eeac-4459-9320-abc607407b39.png" height="17%" width="17%"/><img align="right" src="https://pngimg.com/uploads/bmw/bmw_PNG99555.png" height="20%" width="20%"/>

# Barra in a Beamer   [![image](https://img.shields.io/badge/%23-Arduino-lightgrey)](https://arduino.cc/)
<sup>
Code for the project Barra in a Beamer. Installing a Ford Falcon Barra engine into a 2009 BMW F01 7 Series 730D. 
<br/></sup>


![image](https://user-images.githubusercontent.com/57064943/163714778-8598c24a-6ae2-49f6-ba4c-42de94dfa025.png)

<br/>
<img align="right" src="https://raw.githubusercontent.com/jakka351/Barra-in-a-Beamer/main/docs/received_168803406282992.jpeg" height="25%" width="25%"/>


## Checklist Items
- Decode Barra CAN Messages & Convert for BMW Cluster
- Decode BMW DME CAN Messages that need to be emulated
- Malfunction Indicator Lamps from Barra PCM > BMW cluster
- Set Vehicle Speed Source to be `ABS via CAN` in Barra PCM - 6HP26 TCM can provide speed source to PCM.
- Emulate CAN Message for Vehicle Speed Source input on CAN ID 0x4B0 (Emulating the Falcon ABS Module using the BMW ABS Data)
- What is needed to get BMW factory ABS functional?
- Emulate missing DME CAN messages due to removal of DME & TCM modules
- Disable Passive Anti-theft System in Barra PCM 
- Fit a second OBD port for Barra PCM diagnostics (4 GND, 5 GND, 16 12V, 6 CAN H, 14 CAN L, FEPS Pin 13) `Wiring Harness finished`  
- Install interface board into vehicle
- Throttle / Accelerator pedal input into PCM  `BMW Accelerator pedal should function with Barra PCM`
- HVAC and compressor control needs to be thought out `BMW Compressor mounted on Barra with custom mounts, controlled by factory HVAC`
- Brake Status input into PCM  
- Cruise control neede to be thought out > `Auto Electrician`
- BMW shifter to run Ford 6HP26 `Gear Selector should function wired up to Ford 6HP26 Transmission` 
- Engine Start/Stop Button  > `Auto Electrician`
- Tyre/Wheel/final drive needs to be set in PCM
    
## Software Required  

<IMG src="https://camo.githubusercontent.com/c3087133bc5593228778aacb47dd9c5ceccc927fef16a70adc01b5c44717ef0a/68747470733a2f2f666f727363616e2e6f72672f696d616765732f464f525363616e4c69746541707049636f6e526f756e64436f726e6572733134342e706e67" align="right" width="4%" height="4%" />   
<img align="right" src="https://user-images.githubusercontent.com/57064943/160247672-f3568ee7-4d7b-428d-b914-4894a178538a.png" height="15%" width="15%" />     
<img align="right" src="https://cdn.freebiesupply.com/logos/large/2x/arduino-logo-png-transparent.png" height="6%" width="6%" />     
 
- `PCMTEC`
  For programming and configuring the Barra Powertraim Control Module.
- `FORscan`
  For diagnostics and fault finding.
- `Arduino IDE`
  For flashing the microcontroller used as the CAN interface.
- `Sublime Text`
  My preferred text editor for coding.
- `Putty`
  Used to SSH into the Raspberry Pi and and sniff CAN Data
- `SocketCAN`
  Used to pull raw CAN data from the vehicle.
  


## Hardware Required
- Raspberry Pi 4 with PiCAN3 hat
- Longan Labs CANbed development board
- OBD power isolator & various adaptors
- Multimeter
- J2534 Tool

<br/>
<img src="https://user-images.githubusercontent.com/57064943/166506037-a9bc622c-e47f-4263-9ea4-74e6f24acc99.png" align="right"  height="20%" width="20%"/>

## CAN data from the BMW PT-CAN
Pulling Data from the car with a Raspberry Pi + SPI-MCP2515, wired up to PT-CAN_1 at Kombi connector, powered via OBD port
![image](https://github.com/jakka351/Barra-in-a-Beamer/assets/57064943/d7969ea6-c7de-4d09-abc5-f84470116992)
Isolating individual ECUs to determine CAN frame origin
![image](https://github.com/jakka351/Barra-in-a-Beamer/assets/57064943/31ac2721-cd4f-4815-97c4-ac0c5073bd92)

## Wiring Diagrams

DDE:
![image](https://github.com/jakka351/Barra-in-a-Beamer/assets/57064943/cfa4b5eb-1af5-404e-871e-82fe519e42bd)
EGS:
![image](https://github.com/jakka351/Barra-in-a-Beamer/assets/57064943/f73197c8-22e8-4075-9fe0-17d1df38406e)
KOMBI:
![image](https://github.com/jakka351/Barra-in-a-Beamer/assets/57064943/4d377c31-3ac2-4301-b0ed-2087dcacb129)  

  
## Relevant Information
[`FG Falcon Git Repo`](https://github.com/jakka351/FG-Falcon)   
[`Notes on Stand Alone PCM Setup`](https://forum.pcmtec.com/topic/32-howto-run-pcm-standalone-eg-engine-swap-pats-disable-and-speed-source-setup/)    
[`Notes on ABS Module config`](https://forum.pcmtec.com/topic/872-howto-abs-reprogramming/)    
[`BMW Kombi Pin out`](https://www.bimmerfest.com/threads/730d-instrument-cluster-pinout.1459534/#post-13880013)  


## Based Upon 
[`Original ECU HSCAN Interface `](https://github.com/jakka351/FG-Falcon/blob/master/resources/software/arduino/ECU_HS_CAN_Interface.ino)[`by Mitchell H`](https://www.fordforums.com.au/member.php?u=2315299)    



## Documentation  
Relevant BMW and Ford documents are in the Docs folder.  


![image](https://github.com/jakka351/ECU_HS_CAN_INTERFACE/assets/57064943/523446c5-5c71-45b4-9ce7-ada76427206c)


***


