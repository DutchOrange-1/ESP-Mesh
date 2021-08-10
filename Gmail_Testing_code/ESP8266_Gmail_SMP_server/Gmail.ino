/*
  Rui Santos
  Complete project details at:
   - ESP32: https://RandomNerdTutorials.com/esp32-send-email-smtp-server-arduino-ide/
   - ESP8266: https://RandomNerdTutorials.com/esp8266-nodemcu-send-email-smtp-server-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Example adapted from: https://github.com/mobizt/ESP-Mail-Client
*/

//To use send Email for Gmail to port 465 (SSL), less secure app option should be enabled. https://myaccount.google.com/lesssecureapps?pli=1
int counter = 0; 
int val = 5; 
//        Button setup
const int pin14 = 14;  
const int pin12 = 12; 
int button14 = 0; 
int button12 = 0; 

int incomingByte = 0; // for incoming serial data
char Mymessage[200];
char id[16], new_id[16]; 
int detected = 0, detected_join =0; 
char nodes_number = 0, layer_number = 0, routing_table_size = 0;  



#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <ESP_Mail_Client.h>

#define WIFI_SSID "Ubuntu_Wifi"
#define WIFI_PASSWORD "Linux2021"

#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "esp.b3d@gmail.com"
#define AUTHOR_PASSWORD "Br@nd0nmac"

/* Recipient's email*/
#define RECIPIENT_EMAIL "bdegreef5@gmail.com"

/* The SMTP Session object used for Email sending */
SMTPSession smtp;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);

void setup(){
pinMode(pin14, INPUT_PULLUP);
pinMode(pin12, INPUT_PULLUP);
delay(50); 
  Serial.begin(115200);
  Serial.println();
  Serial.println("Reciving data"); 
  Serial.print("Connecting to AP");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(200);
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /** Enable the debug via Serial port
   * none debug or 0
   * basic debug or 1
  */
  
}

void loop(){
 //  Serial.println("^");
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
detected = 1; 

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
detected_join =1; 
    }
}

  if(detected == 1){
   Serial.println("\n Sending email now... This is a node disconecting"); 
   for(int i =0; i <= 16; i++){
    Serial.print(id[i]);
   }
    Serial.println(" "); 
detected = 0; 

smtp.debug(1);

  /* Set the callback function to get the sending results */
  smtp.callback(smtpCallback);

  /* Declare the session config data */
  ESP_Mail_Session session;

  /* Set the session config */
  session.server.host_name = SMTP_HOST;
  session.server.port = SMTP_PORT;
  session.login.email = AUTHOR_EMAIL;
  session.login.password = AUTHOR_PASSWORD;
  session.login.user_domain = "";

  /* Declare the message class */
  SMTP_Message message;

  /* Set the message headers */
  message.sender.name = "ESP";
  message.sender.email = AUTHOR_EMAIL;
  message.subject = "ESP system has lost a Node.";
  message.addRecipient("Brandon", RECIPIENT_EMAIL);

  /*Send HTML message*/
  //String htmlMsg = "<div style=\"color:#2f4468;\"><h1>Hello World! This is V2 of my system</h1><p>- Sent from ESP board</p></div> " ;
   //String htmlMsg = String(button14) ;
//  String htmlMsg =("<b>A Node has left </b> \n Total number of nodes :<b>" + String(nodes_number) + "</b>\nThe max layer:<b>" + String(layer_number) + "</b>\nThe routing table size:<b>" + String(routing_table_size)+ "</b>");
 String htmlMsg =("<b><div style=\"color:#FF0000;\"><h1>A Node has Left </b></h1></div>Total number of nodes: <b>" + String(nodes_number) +  " (-1)</b><br>The max layer: <b>" + String(layer_number) + "</b><br>The routing table size: <b>" + String(routing_table_size)+ " (-1)</b>");
  message.html.content = htmlMsg.c_str();
  message.html.content = htmlMsg.c_str();
  message.text.charSet = "us-ascii";
  message.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

 
  if (!smtp.connect(&session))
    return;

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message))
    Serial.println("Error sending Email, " + smtp.errorReason());
  delay(100); 
  Serial.println("Waiting "); 
  Serial.println(counter); 
  //delay(1000); 
  Serial.println("Sending email... A node left "); 
  detected_join = 0; 
Serial.print("Detected_join = ");
Serial.println(detected_join); 
Serial.print("Detected = ");
Serial.println(detected); 


}else if(detected_join == 1){
  Serial.println("\n Sending email now..."); 
   for(int i =0; i <= 16; i++){
    Serial.print(new_id[i]);
   }
   Serial.println(" "); 
   detected_join = 0;
 
    smtp.debug(1);

  // Set the callback function to get the sending results 
  smtp.callback(smtpCallback);

 // Declare the session config data
  ESP_Mail_Session session;

 // Set the session config 
  session.server.host_name = SMTP_HOST;
  session.server.port = SMTP_PORT;
  session.login.email = AUTHOR_EMAIL;
  session.login.password = AUTHOR_PASSWORD;
  session.login.user_domain = "";

 // Declare the message class 
  SMTP_Message message;

  // Set the message headers 
  message.sender.name = "ESP";
  message.sender.email = AUTHOR_EMAIL;
  message.subject = "ESP mesh, A node has joined the system.";
  message.addRecipient("Brandon", RECIPIENT_EMAIL);

 // Send HTML message
  //String htmlMsg = "<div style=\"color:#2f4468;\"><h1>Hello World! This is V2 of my system</h1><p>- Sent from ESP board</p></div> " ;
   //String htmlMsg = String(button14) ;
//Serial.printf("Number of nodes = %c, Layer number = %c and routing table size = %c \n ",nodes_number, layer_number, routing_table_size ); 
   String htmlMsg =("<b><div style=\"color:#00FF00;\"><h1>A Node has Joined </b></h1></div>Total number of nodes : <b>" + String(nodes_number) + " (-1)</b><br>The max layer: <b>" + String(layer_number) + "</b><br>The routing table size: <b>" + String(routing_table_size)+ " (-1)</b>");
  message.html.content = htmlMsg.c_str();
  message.html.content = htmlMsg.c_str();
  message.text.charSet = "us-ascii";
  message.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

 
  if (!smtp.connect(&session))
    return;

  // Start sending Email and close the session 
  if (!MailClient.sendMail(&smtp, &message))
    Serial.println("Error sending Email, " + smtp.errorReason());
  delay(100); 
  Serial.println("Waiting"); 
  Serial.println(counter); 
  //delay(1000); 
  Serial.println("Sending email... A node joined "); 
}
}
//======================================================================================================================================================

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status){
  /* Print the current status */
  Serial.println(status.info());

  /* Print the sending result */
  if (status.success()){
    Serial.println("----------------");
    ESP_MAIL_PRINTF("Message sent success: %d\n", status.completedCount());
    ESP_MAIL_PRINTF("Message sent failled: %d\n", status.failedCount());
    Serial.println("----------------\n");
    struct tm dt;

    for (size_t i = 0; i < smtp.sendingResult.size(); i++){
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);
      time_t ts = (time_t)result.timestamp;
      localtime_r(&ts, &dt);

     // ESP_MAIL_PRINTF("Message No: %d\n", i + 1);
   //   ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
     // ESP_MAIL_PRINTF("Date/Time: %d/%d/%d %d:%d:%d\n", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
     // ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients);
      ESP_MAIL_PRINTF("Subject: %s\n", result.subject);
    }
    Serial.println("----------------\n");
    Serial.print("The Gatway IP is: "); 
   //gateway = WiFi.gatewayIP();
    Serial.print(WiFi.gatewayIP()); 
   
  }
}
