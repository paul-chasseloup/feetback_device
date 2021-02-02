#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
BluetoothSerial ESP_BT; //Object for Bluetooth

//initialisation pin capteurs
#define SENSOR1 39
#define SENSOR2 34
#define SENSOR3 35
#define SENSOR4 32
#define SENSOR5 33
#define LEDG 18
#define LEDB 21
#define LEDR 22

// message pour transmission bluetooth 
int message[3];

//initialisaion valeur sensor 
float sensorVolt1 = 0.0; 
float sensorVolt2 = 0.0;
float sensorVolt3 = 0.0;
float sensorVolt4 = 0.0;
float sensorVolt5 = 0.0;
float sensorValue1 = 0.0;
float sensorValue2 = 0.0;
float sensorValue3 = 0.0;
float sensorValue4 = 0.0;
float sensorValue5 = 0.0;
 
void setup() 
{
  // Init Array message bluetooth
  message[0] = 0;
  message[1] = 0;
  message[2] = 0;

  // Init LEDS et SENSORS
  pinMode(SENSOR1,INPUT); 
  pinMode(SENSOR2,INPUT);
  pinMode(SENSOR3,INPUT);
  pinMode(SENSOR4,INPUT);
  pinMode(SENSOR5,INPUT);
  pinMode(LEDR,OUTPUT);
  pinMode(LEDB,OUTPUT);
  pinMode(LEDG,OUTPUT);
   
  // Bluetooth
  Serial.begin(9600); //Start Serial monitor in 9600
  ESP_BT.begin("ESP32_Feetback"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");
}

void loop() 
{
  // Allumer la LED verte   
  digitalWrite(LEDG,HIGH);
  
  //Check if we receive anything from Bluetooth
  if(ESP_BT.available()) 
  {
    // Reading serial bluetooth
    Serial.println("Message entrant:");
    for(int i=0; i<3; i++)
    {
      int incoming = ESP_BT.read(); //Read what we recevive
      Serial.println(incoming); // afficher sur le serial
      message[i] = incoming; // stock les valeurs reçues dans message
    }

    // Message "Start" (1) received : start to collect data
    if (message[0] == 49) // 49 -> 1 en ASCII
        {
          // while message "Stop" (0) not received : continue data collecting
          while(message[0]!= 48) // 48 -> 0 en ASCII
          {
            // Allumer LED bleue et éteindre LED Verte
            digitalWrite(LEDG,LOW);
            digitalWrite(LEDB,HIGH);
            
            // Collecting data
            Serial.println("Collecting data...");
            // Lire les valeurs analogiques retournées par les sensors
            sensorValue1 = analogRead(SENSOR1);
            sensorValue2 = analogRead(SENSOR2);
            sensorValue3 = analogRead(SENSOR3);
            sensorValue4 = analogRead(SENSOR4);
            sensorValue5 = analogRead(SENSOR5);
            // Calculer les tensions correspondantes
            sensorVolt1 = sensorValue1 ;
            sensorVolt2 = sensorValue2 ;
            sensorVolt3 = sensorValue3 ;
            sensorVolt4 = sensorValue4 ;
            sensorVolt5 = sensorValue5 ;
            
            // Print on serial bluetooth
            ESP_BT.print("Sensor 1 :");
            ESP_BT.println(sensorVolt1);
            ESP_BT.print("Sensor 2 :");
            ESP_BT.println(sensorVolt2);
            ESP_BT.print("Sensor 3 :");
            ESP_BT.println(sensorVolt3);
            ESP_BT.print("Sensor 4 :"); 
            ESP_BT.println(sensorVolt4);
            ESP_BT.print("Sensor 5 :");
            ESP_BT.println(sensorVolt5);
            ESP_BT.println(" ");
            // Print on serial arduino
            Serial.println(sensorVolt1);
            Serial.println(sensorVolt2);
            Serial.println(sensorVolt3);
            Serial.println(sensorVolt4);
            Serial.println(sensorVolt5);

            // Fréquence d'envoie des données
            delay(1000);

            //Check if we receive anything from Bluetooth pour le stop
            if (ESP_BT.available()) 
            {
               // Reading serial bluetooth
               for(int i=0; i<3; i++)
               {
                 int incoming = ESP_BT.read(); //Read what we recevive
                 Serial.println(incoming);
                 message[i] = incoming; // Stocker dans message
               }
            }
         } 
   
          // Collecting data over
          ESP_BT.println("Data collected !");
          Serial.println("Data collected !");
       }

       // Eteindre la led bleue et allumer la led
       digitalWrite(LEDG,HIGH);
       digitalWrite(LEDB,LOW);
    }
 }
