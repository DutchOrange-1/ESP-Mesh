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

#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <ESP_Mail_Client.h>

#define WIFI_SSID "Wifi"
#define WIFI_PASSWORD "Passwords"

#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "Enmail being used"
#define AUTHOR_PASSWORD "Its password"

/* Recipient's email*/
#define RECIPIENT_EMAIL "The email of the person reciving the password"

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
   button14 = digitalRead(pin14);
   button12 = digitalRead(pin12);
   delay(100); 
   Serial.print("Button 14 = ( A device has left)  ");
   Serial.println(button14); 
   Serial.print("Button 12 = ( A device has joined)      ");
   Serial.println(button12); 
counter++; 

if(button14 == 1){ 
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
  message.subject = "ESP system notifications";
  message.addRecipient("Brandon", RECIPIENT_EMAIL);

  /*Send HTML message*/
  //String htmlMsg = "<div style=\"color:#2f4468;\"><h1>Hello World! This is V2 of my system</h1><p>- Sent from ESP board</p></div> " ;
   //String htmlMsg = String(button14) ;
   String htmlMsg =("<div style=\"color:#2f4468;\"><h1>WARNING </h1><p>A node has left the mesh</p></div> ") ;
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
  Serial.println("Waiting 5 seconds"); 
  Serial.println(counter); 
  delay(5000); 
  Serial.println("Sending email... A node left "); 

}
else if(button12 == 1){
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
  message.subject = "ESP system notifications";
  message.addRecipient("Brandon", RECIPIENT_EMAIL);

  /*Send HTML message*/
  //String htmlMsg = "<div style=\"color:#2f4468;\"><h1>Hello World! This is V2 of my system</h1><p>- Sent from ESP board</p></div> " ;
   //String htmlMsg = String(button14) ;
   String htmlMsg =("<div style=\"color:#2f4468;\"><h1>GREAT </h1><p>A node has rejoined / joined the mesh !</p></div> ") ;
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
  Serial.println("Waiting 5 seconds"); 
  Serial.println(counter); 
  delay(5000); 
  Serial.println("Sending email... A node joined "); 
}else{
     Serial.println("No pin is activated");
}
delay(100); 
}

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

      ESP_MAIL_PRINTF("Message No: %d\n", i + 1);
      ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
      ESP_MAIL_PRINTF("Date/Time: %d/%d/%d %d:%d:%d\n", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
      ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients);
      ESP_MAIL_PRINTF("Subject: %s\n", result.subject);
    }
    Serial.println("----------------\n");
  }
}
