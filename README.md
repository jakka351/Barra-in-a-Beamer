

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
<IMG src="https://camo.githubusercontent.com/c3087133bc5593228778aacb47dd9c5ceccc927fef16a70adc01b5c44717ef0a/68747470733a2f2f666f727363616e2e6f72672f696d616765732f464f525363616e4c69746541707049636f6e526f756e64436f726e6572733134342e706e67" align="right" width="5%" height="5%" />   
 <img align="right" src="https://user-images.githubusercontent.com/57064943/160247672-f3568ee7-4d7b-428d-b914-4894a178538a.png" height="10%" width="10%" />     
 
- `PCMTEC https://pcmtec.com`
- `FORscan https://forscan.org `
- `Arduino IDE  https://arduino.cc`  
- `Sublime Text`  
- `Putty`  

## Relevant Information
[`FG Falcon Git Repo`](https://github.com/jakka351/FG-Falcon)   
[`Notes on Stand Alone PCM Setup`](https://forum.pcmtec.com/topic/32-howto-run-pcm-standalone-eg-engine-swap-pats-disable-and-speed-source-setup/)    
[`Notes on ABS Module config`](https://forum.pcmtec.com/topic/872-howto-abs-reprogramming/)    
[`BMW Kombi Pin out`](https://www.bimmerfest.com/threads/730d-instrument-cluster-pinout.1459534/#post-13880013)  
![image](https://github.com/jakka351/Barra-in-a-Beamer/assets/57064943/4d377c31-3ac2-4301-b0ed-2087dcacb129)  


## Based Upon 
[`Original ECU HSCAN Interface `](https://github.com/jakka351/FG-Falcon/blob/master/resources/software/arduino/ECU_HS_CAN_Interface.ino)[`by Mitchell H`](https://www.fordforums.com.au/member.php?u=2315299)    



## Documentation  
Relevant BMW and Ford documents are in the Docs folder.  


![image](https://github.com/jakka351/ECU_HS_CAN_INTERFACE/assets/57064943/523446c5-5c71-45b4-9ce7-ada76427206c)


***
<img src="https://user-images.githubusercontent.com/57064943/166506037-a9bc622c-e47f-4263-9ea4-74e6f24acc99.png" align="center"  height="20%" width="20%"/>

