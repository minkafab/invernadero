#include <Arduino.h>
#include <uFire_SHT20.h>

#define is_up 25   //inductive sensor UP input
#define is_down 26 //inductive sensor DOWN input

#define ac1 34 //AC input 1
#define ac2 35 //AC input 2
#define ac3 32 //AC input 3
#define ac4 33 //AC input 4

#define sw 27 //Switch

#define pul 23 //stepper pulse pin
#define dir 19 //stepper dir pin
#define ena 13 //stepper enable pin

#define ev1 16 //Electrovalve Relay output 1
#define ev2 4  //Electrovalve Relay output 2
#define ev3 17 //Electrovalve Relay output 3
#define ev4 18 //Electrovalve Relay output 4

uFire_SHT20 sht20; //SHT20 Humidity and Temperature Sensor i2C interface
uint8_t cont = 0;

float humidity = 0.0;
float temperature = 0.0;
float _humidity_over_setp = 0.0;
float _humidity_under_setp = 0.0;

long debouncing_time = 100; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

bool ac1_state = false;
bool ac2_state = false;
bool ac3_state = false;
bool last_ac1_state = false;
bool last_ac2_state = false;
bool last_ac3_state = false;


void do_electrovalve_action(uint8_t electrovalve, bool action)
{
  uint8_t electrovalve_num = electrovalve;
  //if(humidity > _humidity_over_setp){
    switch (electrovalve_num)
    {
      case 1:
        digitalWrite(ev1, action);
        break;
      case 2:
        digitalWrite(ev2, action);
        break;
      case 3:
        digitalWrite(ev3, action);
        break;
      case 4:
        digitalWrite(ev4, action);
        break;
      default:
      break;
    }
  //}
}

void eval_ac_inputs(){
  if(last_ac1_state != ac1_state){
    Serial.println("AC1: " + (String)ac1_state);
    do_electrovalve_action(1,ac1_state);
    last_ac1_state = ac1_state;
  }
  else{
    if((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(ac1) != ac1_state) {
    ac1_state = digitalRead(ac1);
    last_micros = micros();
  }}
  if(last_ac2_state != ac2_state){
    Serial.println("\tAC2: " + (String)ac2_state);
    do_electrovalve_action(2,ac2_state);
    last_ac2_state = ac2_state;
  }
  else{
    if((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(ac2) != ac2_state) {
    ac2_state = digitalRead(ac2);
    last_micros = micros();
  }}
  if(last_ac3_state != ac3_state){
    Serial.println("\t\tAC3: " + (String)ac3_state);
    do_electrovalve_action(3,ac3_state);
    last_ac3_state = ac3_state;
  }
  else{
    if((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(ac3) != ac3_state) {
    ac3_state = digitalRead(ac3);
    last_micros = micros();
  }}
}

void setup()
{
  pinMode(ac1, INPUT);
  pinMode(ac2, INPUT);
  pinMode(ac3, INPUT);
  pinMode(ac4, INPUT);

  pinMode(ev1, OUTPUT);
  pinMode(ev2, OUTPUT);
  pinMode(ev3, OUTPUT);
  pinMode(ev4, OUTPUT);
  digitalWrite(ev1, HIGH);
  digitalWrite(ev2, HIGH);
  digitalWrite(ev3, HIGH);
  digitalWrite(ev4, HIGH);

  //attachInterrupt(ac1, ac1int, CHANGE);
  //attachInterrupt(ac2, ac2int, CHANGE);
  //attachInterrupt(ac3, ac3int, CHANGE);
  //attachInterrupt(ac4, ac4int, CHANGE);

  ac1_state = digitalRead(ac1);
  ac2_state = digitalRead(ac2);
  ac3_state = digitalRead(ac3);
  last_ac1_state = ac1_state;
  last_ac2_state = ac2_state;
  last_ac3_state = ac3_state;

  pinMode(is_up, INPUT);
  pinMode(is_down, INPUT);

  pinMode(pul, OUTPUT); 
  pinMode(dir, OUTPUT);
  pinMode(ena, OUTPUT);

  Serial.begin(115200);
  Serial.println("HOLA MUNDO");

  Wire.begin();
  sht20.begin();
}

void loop()
{
  eval_ac_inputs();

  // put your main code here, to run repeatedly:
  //delay(1000);
  //cont++;

  /*
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

  /*digitalWrite(dir, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(ena, LOW);
  // Makes 200 pulses for making one full cycle rotation
  for (int x = 0; x < 10000; x++)
  {
    digitalWrite(pul, HIGH);
    delayMicroseconds(200);
    digitalWrite(pul, LOW);
    delayMicroseconds(200);
  }
  delay(50); // One second delay

  digitalWrite(dir, LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for (int x = 0; x < 10000; x++)
  {
    digitalWrite(pul, HIGH);
    delayMicroseconds(200);
    digitalWrite(pul, LOW);
    delayMicroseconds(200);
  }
  delay(50);
  digitalWrite(ena, HIGH);*/
}

void subirCortina()
{
}

void bajarCortina()
{
}



void leerSHT20()
{
  sht20.measure_all();
  Serial.println((String)sht20.tempC + "°C");
  Serial.println((String)sht20.dew_pointC + "°C dew point");
  Serial.println((String)sht20.RH + " %RH");
  Serial.println((String)sht20.vpd() + " kPa VPD");
  Serial.println();
}