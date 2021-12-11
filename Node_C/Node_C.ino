#include <SPI.h>
#include <LoRa.h>
#define PACKET_LENGTH     100
#define NODE_A          1  
#define NODE_B          2
#define NODE_C          3
#define NODE_D          4

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(14);
}

void loop() {
  // try to parse packet  
  int packetSize = LoRa.parsePacket();
  char buf[50];
  if (packetSize) {
    // read packet
    int i=0;
    while (LoRa.available()) {
      buf[i] = (char)LoRa.read();
     //Serial.print((char)LoRa.read());
     i++;
    }
    
    int k = i;
    String body ="";
    for(i= 0;i<k;i++){    
      body = body+buf[i];
    //Serial.print(buf[i]);
    }
   Serial.print("Received data: ");
   Serial.println(body);
   if(buf[0] == '1'){
    String data = "";
    body = body.substring(2);
    /*
        delay(500);        
        data = String(NODE_C)+String(NODE_A);        
        data += body;
        LoRa.beginPacket();
        Serial.println(data);
        LoRa.print(data);          
        LoRa.endPacket();        
        Serial.println("Sent data to A");
        */
        delay(1000);
        data = "";
        data = String(NODE_C)+String(NODE_D);
        data += body;
        LoRa.beginPacket();
        Serial.println(data);
        LoRa.print(data);              
        LoRa.endPacket();
        Serial.println("Sent data to D");
   }
  } 
}
String convertToString(char* a, int size) 
{ 
    int i; 
    String s = ""; 
    for (i = 0; i < size; i++) { 
        s = s + a[i]; 
    } 
    return s; 
} 
