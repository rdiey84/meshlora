#include <SPI.h>
#include <RH_RF95.h>
#include <LoRa.h>
#include <DHT.h>
#include "MQ7.h"
#include "MQ135.h"

#define RFM95_CS     10         //@ PB2:SS
#define RFM95_RST     7
#define RFM95_INT     2        //@ interrupt 0
#define RF95_FREQ   915.0

#define PACKET_LENGTH     80

#define NODE_A          1  
#define NODE_B          2
#define NODE_C          3
#define NODE_D          4

    
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//DHT Sensor ................................................................
#define DHTPIN 4            // what pin we're connected to
#define DHTTYPE DHT22       // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);   // Initialize DHT sensor for normal 16mhz Arduino

//for send
float humidity=0.0;    //Stores humidity value
float temperature=0.0; //Stores temperature value


//DSM501A Air quality Sensor ................................................
#define DUST_SENSOR_DIGITAL_PIN_PM10  5
#define DUST_SENSOR_DIGITAL_PIN_PM25  6

unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 5000;//sample 10s ;
unsigned long lowpulseoccupancy;
float ratio = 0.0;
float ppmvPM10 = 0.0;
float ppmvPM25 = 0.0;
float lastDustPM10 = 0.0;
float lastDustPM25 = 0.0;
long concentrationPM10 = 0;
long concentrationPM25 = 0;
String airQualityStatus = "";

//MQ7 Sensor .................................................................
MQ7 mq7(A0,5.0);
float mq7Value = 0.0; 

//MQ2 Sensor..................................................................
#define MQ2pin (A4)
float sensorValue;  //variable to store sensor value

//MQ135 Sensor ...............................................................
#define ANALOGPIN A2    //  Define Analog PIN on Arduino Board
#define RZERO 206.85    //  Define RZERO Calibration Value
MQ135 gasSensor = MQ135(ANALOGPIN);
//for send
float mq135Value = 0.0;


void setup()
{ 
  
  //DHT Sensor Setup
  dht.begin();

  //DSM501A Air Quality Sensor
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM10,INPUT);
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM25,INPUT);  

 
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
 LoRa.setTxPower(14);//@ no RFO, from +5 to 14

 Serial.println();
}
uint16_t packetNumber=0;

void loop()
{
 
  readSensors();
  packetNumber++;
  if(packetNumber > 60000) packetNumber = 0;
 String dataPacket= "";
 dataPacket = String(NODE_A)+String(NODE_D);
 dataPacket += '#'+String(humidity)+'#'+String(temperature)\
      +'#'+String(concentrationPM25)+'#'+String(concentrationPM10)\
      +'#'+String(mq7Value)\
      +'#'+String(sensorValue)\
      +'#'+String(mq135Value)\
      +'#'+String(packetNumber);
 // send packet
  LoRa.beginPacket();
  LoRa.print(dataPacket);
  Serial.println("Sent data..");
  LoRa.endPacket();
  long i=millis(); 
  while(millis()-i> 3000){
      
      // receive packet  
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        // received a packet
        Serial.print("Received packet:");
    
        // read packet
        while (LoRa.available()) {
          Serial.print((char)LoRa.read());
        }
        Serial.println();
      }
  }

    delay(2000);
  
}


void readSensors() {
  Serial.println("Read Sensors...."); 

  //DHT Sensor ..............................
  Serial.println("\tRead DHT22 Sensor ");
  read_Dht_Sensor();
  Serial.print("\t\tHumidity    : "); Serial.print(humidity); Serial.println("\t%");
  Serial.print("\t\tTemperature : "); Serial.print(temperature); Serial.println("\t°C");  

  //DSM501A Sensor ..............................
  Serial.println("\tRead DSM501A Sensor ");
  read_DSM501A_Sensor();
  //Serial.print("\t\tPM25    : "); Serial.print(ppmvPM25); Serial.println("\tmg/m3");
  //Serial.print("\t\tPM10    : "); Serial.print(ppmvPM10); Serial.println("\tmg/m3");
  Serial.print("\t\tPM25    : "); Serial.print(concentrationPM25); Serial.println("\tpcs/0.01cf");
  Serial.print("\t\tPM10    : "); Serial.print(concentrationPM10); Serial.println("\tpcs/0.01cf");
  //Serial.print("\t\tAir Quality Status: "); Serial.println(airQualityStatus);
 
  //MQ7 Sensor ..............................
  Serial.println("\tRead MQ7 Sensor ");
  read_mq7_Sensor();  
  Serial.print("\t\tMQ7 Value : "); Serial.print(mq7Value); Serial.println("\tmg/m3");

  //MQ2 Sensor ..............................
  Serial.println("\tRead MQ2 Sensor ");
  read_mq2_Sensor();
  Serial.print("\t\tSensor Value: "); Serial.print(sensorValue); Serial.println("\t(Gas Concentration)");
  if(sensorValue > 300)
  {
    Serial.print("\t\t\t| SMOKE DETECTED!");
  }
  else
  {
    Serial.print("\t\t\t| No Detectable Smoke!");
  }
  
  Serial.println("");

  //MQ135 Sensor ..............................
  Serial.println("\tRead MQ135 Sensor ");
  read_mq135_Sensor();
  Serial.print("\t\tMQ135 Value : "); Serial.print(mq135Value); Serial.println("\tmg/m3");
 
}



void read_Dht_Sensor(){
  float h = dht.readHumidity();  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    //Failed to read from DHT sensor!
     humidity=0;    
     temperature=0; 
  } else {
    //Sensor Readed.
     humidity=h;    
     temperature=t; 
  }
}

long getPM(int DUST_SENSOR_DIGITAL_PIN) {

  starttime = millis();

  while (1) {  
    duration = pulseIn(DUST_SENSOR_DIGITAL_PIN, LOW);
    lowpulseoccupancy += duration;
          
    if ((millis()-starttime) > sampletime_ms)
    {
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100    
      long concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve    
      lowpulseoccupancy = 0;
      return(concentration);    
    }
  }  
}


void read_DSM501A_Sensor(){

    //get PM 2.5 density of particles over 2.5 μm.
    concentrationPM25=(long)getPM(DUST_SENSOR_DIGITAL_PIN_PM25);
    Serial.print("PM25: ");
    Serial.println(concentrationPM25);
    Serial.print("\n");
    //ppmv=mg/m3 * (0.08205*Tmp)/Molecular_mass
    //0.08205   = Universal gas constant in atm·m3/(kmol·K)
    ppmvPM25=(float)(((concentrationPM25*0.0283168)/100) *  ((0.08205*temperature)/0.01))/1000;

   //get PM 1.0 - density of particles over 1 μm.
    concentrationPM10=getPM(DUST_SENSOR_DIGITAL_PIN_PM10);
//    Serial.print("PM10: ");
//    Serial.println(concentrationPM10);
//    Serial.print("\n");
    //ppmv=mg/m3 * (0.08205*Tmp)/Molecular_mass
    //0.08205   = Universal gas constant in atm·m3/(kmol·K)
    //ppmvPM10=(float)(((concentrationPM10*0.0283168/100) *  (0.08205*temperature)/0.01))/1000;

 }
void read_mq7_Sensor(){
  //@ as PPM
    mq7Value = mq7.getPPM();
}
void read_mq2_Sensor(){
sensorValue = analogRead(MQ2pin); // read analog input pin 0
  
}
bool read_mq135_Sensor(){
  //@ as PPM
    mq135Value = gasSensor.getPPM(); 

}
