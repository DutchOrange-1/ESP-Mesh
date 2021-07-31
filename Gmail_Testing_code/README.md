## Gmail SMPT server using ESP8266, and a ESP32 has Router of the mesh.
The **.ino** code used for the *ESP8266* which is hosting the SMPT server for the gmail came from:
https://randomnerdtutorials.com/esp8266-nodemcu-send-email-smtp-server-arduino/
**THIS CODE IS NOT OPTOMISED** This was just quickly setup for basic testing... This should be updated in the future and use **Serial Communication** so 
the mac address can be sent and the current number of nodes connected to the mesh can also be sent. 

## The router using the ESP32
This code is very similar from the basic router, the only diffrence is that its checking when the number of nodes increase or decrease.


```
int number_of_nodes = esp_mesh_get_total_node_num() ; 
       // printf("Number of Nodes = %d \n",number_of_nodes); 
        vTaskDelay(500 / portTICK_RATE_MS); 
        if (number_of_nodes < prevoius_amount_of_nodes)     
        {
            printf("A node has left the system, \nSending email to admin... \n");
            prevoius_amount_of_nodes = number_of_nodes; 
            
            gpio_set_level(NODE_DISCONECTING_, 1);
            vTaskDelay(5000 / portTICK_RATE_MS); 
            gpio_set_level(NODE_DISCONECTING_, 0);

        }else if(number_of_nodes > prevoius_amount_of_nodes){
            printf("A node has joined the system, \n Sending email to admin...\n");
            prevoius_amount_of_nodes = number_of_nodes; 
           
            gpio_set_level(NODE_CONECCTING_, 1);
            vTaskDelay(5000 / portTICK_RATE_MS); 
            gpio_set_level(NODE_CONECCTING_, 0);

        }
```
