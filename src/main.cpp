#include <Arduino.h>
#include <uFire_SHT20.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

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

#define led 5 //Led for blink on Wifi State
#define total_steps 105000 //total de pulsos para subir o bajar

const char* ssid = "MINKAFAB";
const char* password = "minka20194586";

uFire_SHT20 sht20; //SHT20 Humidity and Temperature Sensor i2C interface
uint8_t cont = 0;

float humidity = 0.0;
float temperature = 0.0;
int _min_humidity = 0;
int _max_humidity = 0;
int _min_temperature = 0;
int _max_temperature = 0;

long debouncing_time = 100;   //Debouncing Time in Milliseconds
long homing_max_time = 70 * 1000; //tiempo en milisegundos
long manual_mode_timeout = 2 * 3600 * 1000;//tiempo maximo despues de un modo manual antes de pasar a modo automatico
bool time_back_manual_mode = false;
volatile unsigned long last_micros;
volatile unsigned long last_millis;
unsigned long actual_steps = 0;

bool ac1_state = false;
bool ac2_state = false;
bool ac3_state = false;
bool ac4_state = false;
bool up_state = false;
bool down_state = false;
bool last_ac1_state = false;
bool last_ac2_state = false;
bool last_ac3_state = false;
bool last_ac4_state = false;
bool last_up_state = false;
bool last_down_state = false;
int8_t screen_state = -1; //-1: undefined -- 1: closed -- 0: opened
bool controller_mode = true; // TRUE = automatico

char mqtt_server[20] = "m2mlight.com";
char max_humidity[4] = "100";
char min_humidity[4] = "50";
char max_temperature[4] = "30";
char min_temperature[4] = "20";
const char usertopic[20] = "/test";
const char replyusertopic[20] = "/test/reply/";
const char sensorusertopic[20] = "/test/sensor/";
const char ev1_apikey[20] = "3r5A1njcru";
const char ev2_apikey[20] = "qC4O1njcrv";
const char ev3_apikey[20] = "NjV91njcrw";
const char ev4_apikey[20] = "vc1q1njcrx";
const char hum_apikey[15] = "VCWG1njcrs";
const char temp_apikey[15] = "sxdg1njcrt";
const char screen_apikey[15] = "YiEI1njcry";
const char auto_man_apikey[15] = "ayMs1njcrz";

//flag for saving data
bool shouldSaveConfig = false;
WiFiClient wifiClient;
PubSubClient client(wifiClient);
char reply[15];

struct t
{
  uint32_t tStart;
  uint32_t tTimeout;
};
//Tasks and their Schedules.
t t_verify = {0, 60 * 1000};              //Run every x miliseconds
t t_electrovalves_state = {0, 13 * 1000}; //Run every x miliseconds
t t_verify_screen = {0, 30 * 1000};

void send_ev_states()
{
  char mess[20];
  mess[0] = '\0';
  if (WiFi.isConnected())
  {
    if (digitalRead(ev1) == false)
    {
      strcat(mess, ev1_apikey);
      strcat(mess, "&1");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    else if (digitalRead(ev1) == true)
    {
      strcat(mess, ev1_apikey);
      strcat(mess, "&0");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    mess[0] = '\0';
    if (digitalRead(ev2) == false)
    {
      strcat(mess, ev2_apikey);
      strcat(mess, "&1");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    else if (digitalRead(ev2) == true)
    {
      strcat(mess, ev2_apikey);
      strcat(mess, "&0");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    mess[0] = '\0';
    if (digitalRead(ev3) == false)
    {
      strcat(mess, ev3_apikey);
      strcat(mess, "&1");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    else if (digitalRead(ev3) == true)
    {
      strcat(mess, ev3_apikey);
      strcat(mess, "&0");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    mess[0] = '\0';
    if (digitalRead(ev4) == false)
    {
      strcat(mess, ev4_apikey);
      strcat(mess, "&1");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    else if (digitalRead(ev4) == true)
    {
      strcat(mess, ev4_apikey);
      strcat(mess, "&0");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    mess[0] = '\0';
    if (controller_mode)
    {
      strcat(mess, auto_man_apikey);
      strcat(mess, "&1");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    else
    {
      strcat(mess, auto_man_apikey);
      strcat(mess, "&0");
      mess[12] = '\0';
      client.publish(sensorusertopic, mess);
    }
    mess[0] = '\0';
  }
}

void do_electrovalve_action(uint8_t electrovalve, bool action)
{
  uint8_t electrovalve_num = electrovalve;
  bool control_action = true;

  if (humidity < _max_humidity && temperature > _min_temperature && action == false && controller_mode)
  {
    control_action = false;
  }
  switch (electrovalve_num)
  {
  case 1:
    digitalWrite(ev1, (action || control_action));
    break;
  case 2:
    digitalWrite(ev2, (action || control_action));
    break;
  case 3:
    digitalWrite(ev3, (action || control_action));
    break;
  case 4:
    digitalWrite(ev4, (action || control_action));
    break;
  default:
    break;
  }
  send_ev_states();
}

void eval_ac_inputs()
{
  if (last_ac1_state != ac1_state)
  {
    Serial.println("AC1: " + (String)!ac1_state);
    do_electrovalve_action(1, ac1_state);
    last_ac1_state = ac1_state;
  }
  else
  {
    if ((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(ac1) != ac1_state)
    {
      ac1_state = digitalRead(ac1);
      last_micros = micros();
    }
  }
  if (last_ac2_state != ac2_state)
  {
    Serial.println("\tAC2: " + (String)!ac2_state);
    do_electrovalve_action(2, ac2_state);
    last_ac2_state = ac2_state;
  }
  else
  {
    if ((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(ac2) != ac2_state)
    {
      ac2_state = digitalRead(ac2);
      last_micros = micros();
    }
  }
  if (last_ac3_state != ac3_state)
  {
    Serial.println("\t\tAC3: " + (String)!ac3_state);
    do_electrovalve_action(3, ac3_state);
    last_ac3_state = ac3_state;
  }
  else
  {
    if ((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(ac3) != ac3_state)
    {
      ac3_state = digitalRead(ac3);
      last_micros = micros();
    }
  }
  if (last_ac4_state != ac4_state)
  {
    Serial.println("\t\t\tAC4: " + (String)!ac4_state);
    do_electrovalve_action(4, ac4_state);
    last_ac4_state = ac4_state;
  }
  else
  {
    if ((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(ac4) != ac4_state)
    {
      ac4_state = digitalRead(ac4);
      last_micros = micros();
    }
  }
  if (last_up_state != up_state)
  {
    Serial.println("UP: " + (String)!up_state);
    last_up_state = up_state;
    /*if (!up_state)
    {
      screen_state = 0;
    }*/
  }
  else
  {
    if ((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(is_up) != up_state)
    {
      up_state = digitalRead(is_up);
      last_micros = micros();
    }
  }
  if (last_down_state != down_state)
  {
    Serial.println("DOWN: " + (String)!down_state);
    last_down_state = down_state;
    /*if (!down_state)
    {
      screen_state = 1;
    }*/
  }
  else
  {
    if ((long)(micros() - last_micros) >= debouncing_time * 1000 && digitalRead(is_down) != down_state)
    {
      down_state = digitalRead(is_down);
      last_micros = micros();
    }
  }
  if (!up_state && !down_state)
  {
    screen_state = -1;
  }
  else
  {
    if (actual_steps > 100000)
    {
      screen_state = 1;
    }
    else if (actual_steps<50)
    {
      screen_state = 0;
    }
  }
}

void openScreen()
{
  eval_ac_inputs();
  if (up_state == LOW)
  {
    digitalWrite(ena, HIGH);
    actual_steps = 0;
  }
  else
  {
    last_millis = millis();
    Serial.print(F("ABRIENDO CORTINA ."));
    digitalWrite(dir, HIGH);
    digitalWrite(ena, LOW);
    eval_ac_inputs();
    while ((millis() - last_millis) <= homing_max_time && up_state == HIGH && actual_steps > 1)
    {
      eval_ac_inputs();
      digitalWrite(pul, HIGH);
      delayMicroseconds(250);
      digitalWrite(pul, LOW);
      delayMicroseconds(250);
      actual_steps--;
      if(actual_steps<0) actual_steps=0;
    }
    digitalWrite(ena, HIGH);
    Serial.println("OK");
  }
}

void closeScreen()
{
  eval_ac_inputs();
  if (down_state == LOW)
  {
    digitalWrite(ena, HIGH);
    actual_steps = total_steps;
  }
  else
  {
    last_millis = millis();
    Serial.print(F("CERRANDO CORTINA ."));
    digitalWrite(dir, LOW);
    digitalWrite(ena, LOW);
    while ((millis() - last_millis) <= homing_max_time && down_state == HIGH && (total_steps - actual_steps) > 10)
    {
      eval_ac_inputs();
      digitalWrite(pul, HIGH);
      delayMicroseconds(250);
      digitalWrite(pul, LOW);
      delayMicroseconds(250);
      actual_steps ++;
    }
    digitalWrite(ena, HIGH);
    Serial.println("OK");
  }
}

bool tCheck(struct t *t)
{
  if (millis() >= t->tStart + t->tTimeout)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void tRun(struct t *t)
{
  t->tStart = millis();
}

void save_conf()
{
  Serial.println(F("saving config"));
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json["max_humidity"] = max_humidity;
  json["min_humidity"] = min_humidity;
  json["max_temperature"] = max_temperature;
  json["min_temperature"] = min_temperature;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println(F("failed to open config file for writing"));
  }

  json.prettyPrintTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

void callback(char *topico, byte *payload, unsigned int length)
{
  char *min;
  char *max;
  char *ret;
  char *rit;
  char min_willbeint[6];
  char max_willbeint[6];
  char e_willbeint[6];
  uint8_t type = 0;
  int new_min = 0;
  int new_max = 0;
  int new_e = 0;

  //sens3144&o=I&a=jkrl1njcrk&t=0&s=1&e=50&u=&v=
  Serial.print(F("Message arrived ["));
  Serial.print(topico);
  Serial.print(F("] "));
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if (strcmp((char *)payload, "sens") > 4)
  {
    max = strrchr((char*)payload, 'h');
    max[6] = '\0';
    Serial.printf("String after |h=| is - |%s|\n", max);
    uint8_t max_fin = strchr(max, '&') - max;
    min = strrchr((char*)payload, 'l');
    min[6] = '\0';
    Serial.printf("String after |l=| is - |%s|\n", min);
    uint8_t min_fin = strchr(min, '&') - min;
    Serial.printf("min_fin: %d max_fin: %d\n", min_fin, max_fin);
    ret = strrchr((char *)payload, 'e');
    ret[6] = '\0';
    Serial.printf("String after |e=| is - |%s|\n", ret);
    uint8_t fin = strchr(ret, '&') - ret;
    // verificar APIKEY
    Serial.println("---------------------------------------------------");
    strncpy(reply, (char *)payload, 12);
    rit = strchr((char *)payload, 'a');
    rit[12] = '\0';
    Serial.printf("String after |a=| is - |%s|\n", rit);
    rit += 2;

    if (strcmp(rit, (char *)temp_apikey) == 0)
    {
      Serial.println(F("TEMPERATURA"));
      type = 1;
    }
    else if (strcmp(rit, (char *)hum_apikey) == 0)
    {
      Serial.println(F("HUMEDAD"));
      type = 2;
    }
    else if (strcmp(rit, (char *)auto_man_apikey) == 0)
    {
      Serial.println(F("CAMBIO DE MODO"));
      type = 3;
      
    }
    else if(strcmp(rit, (char *)ev1_apikey) == 0)
    {
      Serial.println(F("EV1-MODO MANUAL"));
      type = 11;
    }
    else if(strcmp(rit, (char *)ev2_apikey) == 0)
    {
      Serial.println(F("EV2-MODO MANUAL"));
      type = 12;
    }
    else if(strcmp(rit, (char *)ev3_apikey) == 0)
    {
      Serial.println(F("EV3-MODO MANUAL"));
      type = 13;
    }
    else if(strcmp(rit, (char *)ev4_apikey) == 0)
    {
      Serial.println(F("EV4-MODO MANUAL"));
      type = 14;
    }
    else if(strcmp(rit, (char *)screen_apikey) == 0)
    {
      Serial.println(F("CORTINA-MODO MANUAL"));
      type = 15;
    }
    Serial.println("---------------------------------------------------"); 

    if (fin > 5 || fin <= 2)
    {
      //valor mayor a 3 digitos no peude ser anadido
      Serial.println(F("error en el valor setp"));
      client.publish(usertopic, "noVALsetp");
      type=0;
    }
    else
    {
      reply[9] = 'O';
      reply[10] = 'K';
      reply[11] = '\0';
      Serial.println(reply);
      client.publish(replyusertopic, reply);
      for (uint8_t i = 2; i < fin; i++)
      {
        e_willbeint[i - 2] = ret[i];
      }
      e_willbeint[fin - 2] = '\0';
      Serial.println(e_willbeint); //valor listo a ser convertido en entero
      sscanf(e_willbeint, "%d", &new_e);
    }

    if (min_fin > 5 || min_fin <= 2 || max_fin > 5 || max_fin <= 2)
    {
      Serial.println(F("error en el valor setp"));
      client.publish(usertopic, "noVALsetp");
      if(fin <= 1) type=0;
    }
    else
    {
      reply[9] = 'O';
      reply[10] = 'K';
      reply[11] = '\0';
      Serial.println(reply);
      client.publish(replyusertopic, reply);
      for (uint8_t i = 2; i < max_fin; i++)
      {
        max_willbeint[i - 2] = max[i];
        Serial.printf("%c",max[i]);
      }
      Serial.println();
      max_willbeint[max_fin - 2] = '\0';
      //Serial.println(max_willbeint); //valor listo a ser convertido en entero
      sscanf(max_willbeint, "%d", &new_max);
      Serial.println(new_max);
      for (uint8_t i = 2; i < min_fin; i++)
      {
        min_willbeint[i - 2] = min[i];
        Serial.printf("%c",min[i]);
      }
      Serial.println();
      min_willbeint[min_fin - 2] = '\0';
      //Serial.println(min_willbeint); //valor listo a ser convertido en entero
      sscanf(min_willbeint, "%d", &new_min);
      Serial.println(new_min);
      
    }  
    Serial.println("---------------------------------------------------"); 
    

    switch (type){
        case 1:
          Serial.println(F("actualizacion para temperatura"));
          strncpy(min_temperature, min_willbeint, 3);
          strncpy(max_temperature, max_willbeint, 3);
          _min_temperature = new_min;
          _max_temperature = new_max;
          save_conf();
          break;
        case 2:
          Serial.println(F("actualizacion para humedad"));
          strncpy(min_humidity, min_willbeint, 3);
          strncpy(max_humidity, max_willbeint, 3);
          _min_humidity = new_min;
          _max_humidity = new_max;
          save_conf();
          break;
        case 3:
          Serial.println(F("CAMBIO DE MODO"));
          if(new_e >= 1){
            controller_mode = true;
            time_back_manual_mode = false;
            manual_mode_timeout = 2 * 3600 * 1000;
            digitalWrite(ev1,HIGH);
            digitalWrite(ev2,HIGH);
            digitalWrite(ev3,HIGH);
            digitalWrite(ev4,HIGH);
            send_ev_states();
          }
          else if(new_e <=0){
            controller_mode = false;
            time_back_manual_mode = true;
            manual_mode_timeout = 2 * 3600 * 1000;
            digitalWrite(ev1,HIGH);
            digitalWrite(ev2,HIGH);
            digitalWrite(ev3,HIGH);
            digitalWrite(ev4,HIGH);
            send_ev_states();
          }
          break;
          
        case 11:
          Serial.println(F("EV1 - UPDATE"));
          controller_mode = false; // activar modo manual = controller_mode = false
          time_back_manual_mode = true; //
          manual_mode_timeout = 2 * 3600 * 1000;
          if(new_e == 11) digitalWrite(ev1,LOW);
          else if(new_e == 0) digitalWrite(ev1,HIGH);
          send_ev_states();
          break;
        case 12:
          Serial.println(F("EV2 - UPDATE"));
          controller_mode = false; // activar modo manual = controller_mode = false
          time_back_manual_mode = true; //
          manual_mode_timeout = 2 * 3600 * 1000;
          if(new_e == 11) digitalWrite(ev2,LOW);
          else if(new_e == 0) digitalWrite(ev2,HIGH);
          send_ev_states();
          break;
        case 13:
          Serial.println(F("EV3 - UPDATE"));
          controller_mode = false; // activar modo manual = controller_mode = false
          time_back_manual_mode = true; //
          manual_mode_timeout = 2 * 3600 * 1000;
          if(new_e == 11) digitalWrite(ev3,LOW);
          else if(new_e == 0) digitalWrite(ev3,HIGH);
          send_ev_states();
          break;
        case 14:
          Serial.println(F("EV4 - UPDATE"));
          controller_mode = false; // activar modo manual = controller_mode = false
          time_back_manual_mode = true; //
          manual_mode_timeout = 2 * 3600 * 1000;
          if(new_e == 11) digitalWrite(ev4,LOW);
          else if(new_e == 0) digitalWrite(ev4,HIGH);
          send_ev_states();
          break;
        case 15:
          Serial.println(F("RECIBIDO - CORTINA - MODO MANUAL"));
          controller_mode = false; // activar modo manual = controller_mode = false
          time_back_manual_mode = true; //
          manual_mode_timeout = 2 * 3600 * 1000;
          if(new_e == 11 && screen_state != -1) {
            closeScreen();
            }
          else if(new_e == 0){
             openScreen();
             }
          send_ev_states();
          break;
        default:
          break;
      }
      return;

  }
    
}

void reconnect()
{
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect("esp32client_1234", "mqtt", "m2mlight12"))
    {
      Serial.println(F("connected"));
      // Once connected, publish an announcement...
      //client.publish(topic,"hello world");
      // ... and resubscribe
      client.subscribe(usertopic);
    }
    else
    {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
    }
}

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println(F("Should save config"));
  shouldSaveConfig = true;
}

void setupSpiffs()
{
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println(F("mounted file system"));
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println(F("reading config file"));
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println(F("opened config file"));
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success())
        {
          Serial.println(F("\nparsed json"));

          strcpy(min_humidity, json["min_humidity"]);
          strcpy(max_humidity, json["max_humidity"]);
          strcpy(min_temperature, json["min_temperature"]);
          strcpy(max_temperature, json["max_temperature"]);
        }
        else
        {
          Serial.println(F("failed to load json config"));
        }
      }
    }
  }
  else
  {
    Serial.println(F("failed to mount FS"));
  }
}

void readSHT20()
{
  char mess[20];
  mess[0] = '\0';
  char number[10];
  sht20.measure_all();
  humidity = sht20.RH;
  temperature = sht20.tempC;
  if (temperature > 128.0)
  {
    //sensor desconectado
    if (WiFi.isConnected())
      client.publish(sensorusertopic, "sensor desconectado");
  }
  else
  {
    //sensor correcto - enviar data
    if (WiFi.isConnected())
    {
      dtostrf(humidity, 4, 2, number);
      strcat(mess, hum_apikey);
      strcat(mess, "&");
      strcat(mess, number);
      mess[16] = '\0';
      Serial.println(mess);
      client.publish(sensorusertopic, mess);
      mess[0] = '\0';
      dtostrf(temperature, 4, 2, number);
      strcat(mess, temp_apikey);
      strcat(mess, "&");
      strcat(mess, number);
      mess[16] = '\0';
      Serial.println(mess);
      client.publish(sensorusertopic, mess);
      Serial.println((String)sht20.tempC + "°C");
      //Serial.println((String)sht20.dew_pointC + "°C dew point");
      Serial.println((String)sht20.RH + " %RH");
      //Serial.println((String)sht20.vpd() + " kPa VPD");
      Serial.println();
    }
  }
}

void setup()
{
  pinMode(ac1, INPUT_PULLUP);
  pinMode(ac2, INPUT_PULLUP);
  pinMode(ac3, INPUT_PULLUP);
  pinMode(ac4, INPUT_PULLUP);

  pinMode(ev1, OUTPUT);
  pinMode(ev2, OUTPUT);
  pinMode(ev3, OUTPUT);
  pinMode(ev4, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  digitalWrite(ev1, HIGH);
  digitalWrite(ev2, HIGH);
  digitalWrite(ev3, HIGH);
  digitalWrite(ev4, HIGH);

  ac1_state = digitalRead(ac1);
  ac2_state = digitalRead(ac2);
  ac3_state = digitalRead(ac3);
  ac4_state = digitalRead(ac4);
  last_ac1_state = ac1_state;
  last_ac2_state = ac2_state;
  last_ac3_state = ac3_state;
  last_ac4_state = ac4_state;
  last_up_state = up_state;
  last_down_state = down_state;

  pinMode(is_up, INPUT_PULLUP);
  pinMode(is_down, INPUT_PULLUP);

  pinMode(pul, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(ena, OUTPUT);
  digitalWrite(ena,HIGH);

  pinMode(sw, INPUT);

  Serial.begin(115200);
  Serial.println(F("BOOTING ESP32"));

  Wire.begin();
  sht20.begin();

  setupSpiffs();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(100);
  }

   ArduinoOTA.setPort(3232);
   ArduinoOTA.setHostname("invernadero");

  MDNS.begin("invernadero");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(F("Failed to connect"));
    digitalWrite(led, LOW);
  }
  else
  {
    //if you get here you have connected to the WiFi
    Serial.println(F("connected...yeey :)"));
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(led, HIGH);
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    reconnect();
  }

  sscanf(min_humidity, "%d", &_min_humidity);
  sscanf(max_humidity, "%d", &_max_humidity);
  sscanf(min_temperature, "%d", &_min_temperature);
  sscanf(max_temperature, "%d", &_max_temperature);

  Serial.println(F("local ip"));
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());

  readSHT20();
  last_millis = millis();
  digitalWrite(dir, HIGH);
  digitalWrite(ena, LOW);
  eval_ac_inputs();
  while ((millis() - last_millis) <= homing_max_time && up_state == HIGH)
  {
    eval_ac_inputs();
    digitalWrite(pul, HIGH);
    delayMicroseconds(250);
    digitalWrite(pul, LOW);
    delayMicroseconds(250);
  }
  if(up_state == LOW){
    actual_steps = 0;
  }

  Serial.println(F("+++++++++ CONTROLLER ACTUAL CONFIG +++++++++"));
  Serial.printf("Temperature min: %d  max: %d \n", _min_temperature, _max_temperature);
  Serial.printf("Humidity min: %d max: %d \n", _min_humidity, _max_humidity);
  Serial.println(F("+++++++++            END            +++++++++"));
}

void loop()
{
  ArduinoOTA.handle();
  eval_ac_inputs();
  if (tCheck(&t_verify))
  {
    if ((WiFi.status() != WL_CONNECTED)){
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      delay(10);
    }
    readSHT20();
    if(time_back_manual_mode){
      manual_mode_timeout = manual_mode_timeout - t_verify.tTimeout;
      Serial.println(manual_mode_timeout);
      if(manual_mode_timeout < t_verify.tTimeout) {
        controller_mode = true;
        time_back_manual_mode = false;
        manual_mode_timeout = 2 * 3600 * 1000;
        Serial.println("Se acabo el tiempo, volviendo a modo automatico");
        }
    }
    tRun(&t_verify);
  }

  if (tCheck(&t_verify_screen))
  {
    Serial.println("Chequeando cortina");
    if (temperature < _min_temperature && controller_mode && screen_state != -1)
    {
      closeScreen();
    }
    if (temperature > _max_temperature && controller_mode)
    {
      openScreen();
    }
    if(humidity > _max_humidity){
      digitalWrite(ev1,HIGH);
      digitalWrite(ev2,HIGH);
      digitalWrite(ev3,HIGH);
      digitalWrite(ev4,HIGH);
      send_ev_states();
    }

    eval_ac_inputs();
    char number[5];
    char mess[20];
    mess[0] = '\0';
    strcat(mess, screen_apikey);
    strcat(mess, "&");
    dtostrf(screen_state, 1, 1, number);
    strcat(mess, number); 
    mess[13] = '\0';
    if (WiFi.isConnected()){
      Serial.println("publicando estado de cortina");
      client.publish(sensorusertopic, mess);
    }
    mess[0] = '\0';
    tRun(&t_verify_screen);
  }

  if (tCheck(&t_electrovalves_state))
  {
    send_ev_states();
    Serial.println("enviando EV states");
    tRun(&t_electrovalves_state);
  }

  if (digitalRead(sw) == LOW)
  {
    volatile unsigned long counter = millis();
    volatile unsigned long acum = 0;
    while (digitalRead(sw) == LOW && millis() - counter < 8 * 1000)
    {
      acum++;
      delay(10);
    }
    //Serial.println("SWITCH DADO");
    Serial.println(acum);
    if (acum > 350)
    {
      if(controller_mode == true){
        Serial.println(F("Apertura de cortina, modo manual activado"));
        controller_mode = false;
        digitalWrite(ev1,HIGH);
        digitalWrite(ev2,HIGH);
        digitalWrite(ev3,HIGH);
        digitalWrite(ev4,HIGH);
        openScreen();
        time_back_manual_mode = true;
        send_ev_states();
        //comienzo de cuenta atras de manual_mode_timeout
      }else if(controller_mode == false){
        controller_mode = true;
        time_back_manual_mode = false;
        Serial.println(F("modo manual DESACTIVADO"));
        digitalWrite(ev1,HIGH);
        digitalWrite(ev2,HIGH);
        digitalWrite(ev3,HIGH);
        digitalWrite(ev4,HIGH);
        send_ev_states();
        //closeScreen(); //no necesariamente necesita cerrarse.
      }
    }
    if (acum > 750)
    {
      Serial.println(F("RESET DONE"));
      ESP.restart();
    }
  }

  

  if (client.state() < 0 && WiFi.status() == WL_CONNECTED)
  {
      Serial.println(F("conectando mqtt"));
      client.setServer(mqtt_server, 1883);
      client.connect("esp32client_1234", "mqtt", "m2mlight12");
      client.subscribe(usertopic);
      client.setCallback(callback);
  }
  if (WiFi.isConnected())
    client.loop();
}