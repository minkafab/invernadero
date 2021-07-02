#include <Arduino.h>
#include <uFire_SHT20.h>
#define is_up  25 //inductive sensor UP input
#define is_down  26 //inductive sensor DOWN input
#define ac1 34 //AC input 1
#define ac2 35 //AC input 2
#define ac3 32 //AC input 3
#define ac4 33 //AC input 4


#define stepPin 23
#define dirPin 4
#define enablePin 13

uFire_SHT20 sht20;
uint8_t cont=0;
//int led = 23;




void setup() {
  pinMode (ac1, INPUT_PULLUP);
  pinMode (ac2, INPUT_PULLUP);
  pinMode (ac3, INPUT_PULLUP);
  pinMode (ac4, INPUT_PULLUP);
  pinMode (is_up, INPUT);
  pinMode (is_down, INPUT);

  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enablePin,OUTPUT);


  Serial.begin(115200);
  Serial.println("HOLA MUNDO");

  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enablePin,OUTPUT);

  Wire.begin();
  sht20.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000);
  //cont++;

  /*sht20.measure_all();
  Serial.println((String)sht20.tempC + "°C");
  Serial.println((String)sht20.dew_pointC + "°C dew point");
  Serial.println((String)sht20.RH + " %RH");
  Serial.println((String)sht20.vpd() + " kPa VPD");
  Serial.println();*/
  //if(digitalRead(is_up)) Serial.println("inductivo UP en alto");
  //else Serial.println("inductivo UP en bajo");
  //if(digitalRead(is_down)) Serial.println("inductivo DOWN en alto");
  //else Serial.println("inductivo DOWN en bajo");
  //if(digitalRead(ac1) == LOW || digitalRead(ac2) == LOW || digitalRead(ac3) == LOW || digitalRead(ac4) == LOW) digitalWrite(led, HIGH);
  /*if(digitalRead(ac1) == LOW) {
    Serial.println("SIN AC1");
    digitalWrite(led, LOW);
    }
  else { 
    Serial.println("CON AC1");
  digitalWrite(led, HIGH);
  }
  if(digitalRead(ac2) == LOW) {
    Serial.println("\tSIN AC2");
    digitalWrite(led, LOW);
    }
  else { 
    Serial.println("\tCON AC2");
  digitalWrite(led, HIGH);
  }
  if(digitalRead(ac3) == LOW) {
    Serial.println("\t\tSIN AC3");
    digitalWrite(led, LOW);
    }
  else { 
    Serial.println("\t\tCON AC3");
  digitalWrite(led, HIGH);
  }
  if(digitalRead(ac4) == LOW) {
    Serial.println("\t\t\tSIN AC4");
    digitalWrite(led, LOW);
    }
  else { 
    Serial.println("\t\t\tCON AC4");
  digitalWrite(led, HIGH);
  }
  delay(500);*/

digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  digitalWrite(enablePin, LOW);
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 10000; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(200); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(200); 
  }
  delay(50); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 10000; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(200);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(200);
  }
  delay(50);
  digitalWrite(enablePin, HIGH);



}

