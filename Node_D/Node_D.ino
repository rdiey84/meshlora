#include <SPI.h>
#include <RH_RF95.h>
#include <LoRa.h>
#define RFM95_CS     10         //@ PB2:SS
#define RFM95_RST     7
#define RFM95_INT     2        //@ interrupt 0
#define RF95_FREQ   915.0

#define PACKET_LENGTH     100

#define NODE_B          2
#define NODE_C          3
#define NODE_D          4
    
RH_RF95 rf95(RFM95_CS, RFM95_INT);


//Recive Data from NODE A...............................
float humidity=0.0;    //Stores humidity value
float temperature=0.0; //Stores temperature value

long concentrationPM25 = 0;
long concentrationPM10 = 0;
float mq7Value = 0.0; 
float sensorValue = 0.0;
float mq135Value = 0.0;
//.......................................................


void setup() 
{     
   
    Serial.begin(9600);
  while (!Serial);
 
    Serial.println("Node D received....");
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  } 
    Serial.println("\n\n");
}
  
 String copyBuf; 
 uint8_t copyLen;
void loop()
{
  int packetSize = LoRa.parsePacket();
   if (packetSize)
  { 
    Serial.println("\n\n");    
    char buf[PACKET_LENGTH];
    
  int i=0;
    while(LoRa.available()){ 
    buf[i] =(char)LoRa.read();
    //Serial.print((char)LoRa.read());
    i++;
  }
  copyLen=i;
  String data = "";
  for(i=0;i<copyLen;i++){
    data += buf[i];
    //Serial.print(buf[i]);
  }  
    Serial.print("Recive Data: ");             
    Serial.println(data);
    copyBuf = data;  
      if(buf[1] == '4')   analyzeResult();         
      else Serial.print("\tData error..");     
   Serial.println("\tRSSI: " + String(LoRa.packetRssi()));
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
void analyzeResult() {
    
  String h="";
    String t="";    
    String c1=""; //@ PM25
    String c2=""; //@ PM10 
    String mq7="";
    String s="";
    String mq135="";
    String packet="";
    
    int hashCount = 0;
    String rec = copyBuf;//convertToString(copyBuf,copyLen);
    //decoding
    Serial.println("Decoding..");
    Serial.println(rec);
    for( int x=2; x<copyLen+1; x++){
     
      if(rec.substring(x-1,x) == "#"){
          hashCount ++;      
      }
      else if( hashCount == 1)  {   h      += rec.substring(x-1,x);      }
      else if( hashCount == 2)  {   t      += rec.substring(x-1,x);      }
      else if( hashCount == 3)  {   c1     += rec.substring(x-1,x);      }
      else if( hashCount == 4)  {   c2     += rec.substring(x-1,x);      }    
      else if( hashCount == 5)  {   mq7    += rec.substring(x-1,x);      }
      else if( hashCount == 6)  {   s      += rec.substring(x-1,x);      }
      else if( hashCount == 7)  {   mq135  += rec.substring(x-1,x);      }
      else if( hashCount == 8)   {   packet += rec.substring(x-1,x);      }
    }
/*
  humidity = h.toFloat();
  temperature = t.toFloat(); 
  concentrationPM25 = c1.toFloat();
  concentrationPM10 = c2.toFloat();    
  mq7Value = mq7.toFloat();
  lpg = l.toInt();
  co = c.toInt();
  smoke = s.toInt();
  mq135Value = mq135.toFloat();
 */    
  //DHT Sensor ..............................
  Serial.println("\tRead DHT22 Sensor ");
  Serial.print("\t\tHumidity    : "); Serial.print(h); Serial.println("\t%");
  Serial.print("\t\tTemperature : "); Serial.print(t); Serial.println("\t°C");  

  //DSM501A Sensor ..............................
  Serial.println("\tRead DSM501A Sensor ");
  //Serial.print("\t\tPM25    : "); Serial.print(ppmvPM25); Serial.println("\tmg/m3");
  //Serial.print("\t\tPM10    : "); Serial.print(ppmvPM10); Serial.println("\tmg/m3");
  Serial.print("\t\tPM25    : "); Serial.print(c1); Serial.println("\tpcs/0.01cf");
  //Serial.print("\t\tPM10    : "); Serial.print(c2); Serial.println("\tpcs/0.01cf");
  //Serial.print("\t\tAir Quality Status: "); Serial.println(airQualityStatus);
 
  //MQ7 Sensor ..............................
  Serial.println("\tRead MQ7 Sensor ");
  Serial.print("\t\tMQ7 Value : "); Serial.print(mq7); Serial.println("\tmg/m3");

  //MQ2 Sensor ..............................
  Serial.println("\tRead MQ2 Sensor ");
  Serial.print("\t\tGas Concentration : "); Serial.print(s); Serial.println("\t(Smoke = Gas Concentration > 300)"); Serial.println("");

  /*if(sensorValue > 300)
  {
    Serial.print("\t\t\t| SMOKE DETECTED!");
  }  
  else
  {
     Serial.print("\t\t\t| No Detectable Smoke!");
  }*/
 
  
  //MQ135 Sensor ..............................
  Serial.println("\tRead MQ135 Sensor ");
  Serial.print("\t\tMQ135 Value : "); Serial.print(mq135); Serial.println("\tmg/m3");
  //packet test
  Serial.print("\tPacketID :");Serial.println(packet);
}
