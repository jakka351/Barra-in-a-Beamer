

<img align="right" src="https://www.pngmart.com/files/4/Ford-Logo-PNG-Transparent-Image.png" height="15%" width="15%"/><img align="right" src="https://pngimg.com/uploads/bmw/bmw_PNG99555.png" height="10%" width="10%"/>

# Barra in a Beamer   [![image](https://img.shields.io/badge/%23-Arduino-lightgrey)](https://arduino.cc/)
<sup>
Code for the project Barra in a Beamer. Installing a Ford Falcon Barra engine into a 2009 BMW F01 7 Series 730D. 
<br/></sup>




![image](https://user-images.githubusercontent.com/57064943/163714778-8598c24a-6ae2-49f6-ba4c-42de94dfa025.png)

## Checklist Items
- Decode Barra CAN Messages & Convert for BMW Cluster 
- Decode BMW DME CAN Messages that need to be emulated
- Set Vehicle Speed Source to be 'ABS via CAN'
- Create CAN Message for Vehicle Speed Source input on CAN ID 0x4B0 (Emulating the Falcon ABS Module using the BMW ABS Data)
- Disable Passive Anti-theft System in Barra PCM
  
## Software Used
- PCMTEC
- FORscan
- Arduino IDE
- Sublime Text
- Putty

## Relevant Information
[`FG Falcon Git Repo`](https://github.com/jakka351/FG-Falcon) 
[`Notes on Stand Alone PCM Setup`](https://forum.pcmtec.com/topic/32-howto-run-pcm-standalone-eg-engine-swap-pats-disable-and-speed-source-setup/)  
[`Notes on ABS Module config`](https://forum.pcmtec.com/topic/872-howto-abs-reprogramming/)  
[`BMW Kombi Pin out`]()

## Based Upon 
[`Original ECU HSCAN Interface `](https://github.com/jakka351/FG-Falcon/blob/master/resources/software/arduino/ECU_HS_CAN_Interface.ino)[`by Mitchell H`](https://www.fordforums.com.au/member.php?u=2315299)    



## Documentation  
Relevant BMW and Ford documents are in the Docs folder.  


![image](https://github.com/jakka351/ECU_HS_CAN_INTERFACE/assets/57064943/523446c5-5c71-45b4-9ce7-ada76427206c)

***
