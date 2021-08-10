## Gmail SMPT server using ESP8266, and a ESP32 has Router of the mesh.
The **.ino** code used for the *ESP8266* which is hosting the SMPT server for the gmail came from:
https://randomnerdtutorials.com/esp8266-nodemcu-send-email-smtp-server-arduino/
**THIS CODE IS NOT OPTOMISED** This was just quickly setup for basic testing... This should be updated in the future and use **Serial Communication** so 
the mac address can be sent and the current number of nodes connected to the mesh can also be sent. 

10/08/2021
**Code is now better**
The code has now included basic serial communiucation. The only conections needed are TX -> RX (To the smpt server). The code just scans through it for a pecific message
such as **&$&** for when a node joins and **$#$** when a node leavs. 
**THIS IS STILL NO OPTOMISED** but is better because now I can sned data values. 

## The router using the ESP32
This code is very similar from the basic router, the only diffrence is that its checking when the number of nodes increase or decrease.


```
int incomingByte = 0; // for incoming serial data
char Mymessage[200];
char id[16], new_id[16]; 
int detected = 0, detected_join =0; 
char nodes_number = 0, layer_number = 0, routing_table_size = 0;  

void setup() {
 Serial.begin(115200);
 Serial.println("Reciving data"); 
}

void loop() {
 Serial.readBytes(Mymessage,200); //Read the serial data and store in var
 for(int i =-1; i <= 199; i++){
 //  Serial.print(Mymessage[i]); //Print data on Serial Monitor
    if(Mymessage[i] == '&' && Mymessage[i+1] == '$' && Mymessage[i+2] == '&'){
      /*
for(int j = i+3; j <=(i+9); j++){ 
Serial.print( Mymessage[j] );       
}    
*/ 
 //Serial.print("   __==node numbers = "); 
 //Serial.println(Mymessage[i+3]); 
nodes_number = Mymessage[i+3]; 
layer_number = Mymessage[i+5]; 
routing_table_size = Mymessage[i+7]; 
Serial.println("A node have left !!!"); 
Serial.printf("Number of nodes = %c, Layer number = %c and routing table size = %c \n ",nodes_number, layer_number, routing_table_size ); 
Mymessage[i] = 0; 


}else if(Mymessage[i] == '#' && Mymessage[i+1] == '$' && Mymessage[i+2] == '#'){
      /*
for(int j = i+3; j <=(i+9); j++){ 
Serial.print( Mymessage[j] );       
}    
*/ 
 //Serial.print("   __==node numbers = "); 
 //Serial.println(Mymessage[i+3]); 
nodes_number = Mymessage[i+3]; 
layer_number = Mymessage[i+5]; 
routing_table_size = Mymessage[i+7]; 
Serial.println("A node have JOINED"); 
Serial.printf("Number of nodes = %c, Layer number = %c and routing table size = %c \n ",nodes_number, layer_number, routing_table_size ); 
Mymessage[i] = 0; 

    }
}
}
```
