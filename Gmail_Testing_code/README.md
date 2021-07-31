## Gmail SMPT server using ESP8266, and a ESP32 has Router of the mesh.
The **.ino** code used for the *ESP8266* which is hosting the SMPT server for the gmail came from:
https://randomnerdtutorials.com/esp8266-nodemcu-send-email-smtp-server-arduino/
**THIS CODE IS NOT OPTOMISED** This was just quickly setup for basic testing... This should be updated in the future and use **Serial Communication** so 
the mac address can be sent and the current number of nodes connected to the mesh can also be sent. 

## The router using the ESP32
This code is very similar from the basic router, the only diffrence is that its checking when the number of nodes increase or decrease.
`<hello>` 
