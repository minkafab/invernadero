#include <Arduino.h>
#include <uFire_SHT20.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

WiFiManager wm;

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
int _humidity_setp = 0;
int _temperature_setp = 0;

long debouncing_time = 100; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

bool ac1_state = false;
bool ac2_state = false;
bool ac3_state = false;
bool ac4_state = false;
bool last_ac1_state = false;
bool last_ac2_state = false;
bool last_ac3_state = false;
bool last_ac4_state = false;

char mqtt_server[20] = "m2mlight.com";
char humidity_setp[4] = "200";
char temperature_setp[4] = "45";
const char usertopic[20] = "/3x1Z1njcje";
const char replyusertopic[20] = "/3x1Z1njcje/reply/";
const char hum_apikey[15] = "VCWG1njcrs";
const char temp_apikey[15] = "sxdg1njcrt";

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
t t_verify = {0, 30 * 1000}; //Run every x miliseconds

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

void save_conf(){
  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json["humidity_setp"] = humidity_setp;
  json["temperature_setp"] = temperature_setp;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }

  json.prettyPrintTo(Serial);
  json.printTo(configFile);
  configFile.close();
}


void callback(char *topico, byte *payload, unsigned int length)
{
  char *ret;
  char *rit;
  char willbeint[6];
  uint8_t type = 0;
  int new_setpoint = 0;
  
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
    //Serial.println(F("Esl payload es mayor que 4"));
    ret = strrchr((char *)payload, 'e');
    ret[7] = '\0';
    Serial.printf("String after |e=| is - |%s|\n",ret);
    uint8_t fin = strchr(ret, '&') - ret;
    //Serial.println("FIN es: " + (String)fin);
    strncpy(reply,(char*)payload,12);
    //reply = (char*)payload;
    Serial.println("---------------------------------------------------");
    rit = strchr((char *)payload, 'a');
    rit[12] = '\0';
    Serial.printf("String after |a=| is - |%s|\n",rit);
    rit += 2;
    if(strcmp(rit,(char*)temp_apikey)==0){
      Serial.println("TEMPERATURA");
      type = 1;
    }
    if(strcmp(rit,(char*)hum_apikey)==0){
      Serial.println("HUMEDAD");
      type = 2;
    }
    Serial.println("---------------------------------------------------");
    if (fin > 5 || fin < 2)
    {
      //valor mayor a 3 digitos no peude ser anadido
      Serial.println(F("error en el valor setp"));
      client.publish(usertopic,"noVALsetp");
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
        willbeint[i - 2] = ret[i];
      }
      willbeint[fin - 2] = '\0';
      Serial.println(willbeint); //valor listo a ser convertido en entero
      sscanf(willbeint,"%d",&new_setpoint);
      if(type == 1){
        Serial.println("actualizacion para temperatura");
        //Serial.println(temperature_setp);
        strncpy(temperature_setp,willbeint,3);
        //Serial.println(temperature_setp);
        _temperature_setp = new_setpoint;
      }
      if(type == 2){
        Serial.println("actualizacion para humedad");
        //Serial.println(temperature_setp);
        strncpy(humidity_setp,willbeint,3);
        //Serial.println(temperature_setp);
        _humidity_setp = new_setpoint;
      }
      save_conf();
    }
  }
}


void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient", "mqtt", "m2mlight12"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish(topic,"hello world");
      // ... and resubscribe
      client.subscribe(usertopic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  }
}

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setupSpiffs()
{
  //clean FS, for testing
  // SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success())
        {
          Serial.println("\nparsed json");

          strcpy(humidity_setp, json["humidity_setp"]);
          strcpy(temperature_setp, json["temperature_setp"]);

          // if(json["ip"]) {
          //   Serial.println("setting custom ip from config");
          //   strcpy(static_ip, json["ip"]);
          //   strcpy(static_gw, json["gateway"]);
          //   strcpy(static_sn, json["subnet"]);
          //   Serial.println(static_ip);
          // } else {
          //   Serial.println("no custom ip in config");
          // }
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
  //end read
}

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

void eval_ac_inputs()
{
  if (last_ac1_state != ac1_state)
  {
    Serial.println("AC1: " + (String)!ac1_state);
    do_electrovalve_action(1, ac1_state);
    last_ac1_state = ac1_state;
    Serial.println(humidity_setp);
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
}

void readSHT20()
{
  sht20.measure_all();
  Serial.println((String)sht20.tempC + "°C");
  Serial.println((String)sht20.dew_pointC + "°C dew point");
  Serial.println((String)sht20.RH + " %RH");
  Serial.println((String)sht20.vpd() + " kPa VPD");
  Serial.println();
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

  ac1_state = digitalRead(ac1);
  ac2_state = digitalRead(ac2);
  ac3_state = digitalRead(ac3);
  ac4_state = digitalRead(ac4);
  last_ac1_state = ac1_state;
  last_ac2_state = ac2_state;
  last_ac3_state = ac3_state;
  last_ac4_state = ac4_state;

  pinMode(is_up, INPUT);
  pinMode(is_down, INPUT);

  pinMode(pul, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(ena, OUTPUT);

  pinMode(sw, INPUT);

  Serial.begin(115200);
  Serial.println("HOLA MUNDO");

  Wire.begin();
  sht20.begin();

  setupSpiffs();

  WiFi.mode(WIFI_STA);
  wm.setSaveConfigCallback(saveConfigCallback);
  WiFiManagerParameter custom_humidity_setp("humidity_setp", "humidity_setp", humidity_setp, 3);
  WiFiManagerParameter custom_temperature_setp("temperature_setp", "temperature_setp", temperature_setp, 3);

  //add all your parameters here
  wm.addParameter(&custom_humidity_setp);
  wm.addParameter(&custom_temperature_setp);

  //reset settings - wipe credentials for testing
  //wm.resetSettings();

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result

  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  res = wm.autoConnect("AutoConnectAP", "password"); // password protected ap

  if (!res)
  {
    Serial.println("Failed to connect");
    //ledcAttachPin(ev4, 0);
    //ledcSetup(0, 1000, 4);
    //ledcWrite(0, 4);
    // ESP.restart();
  }
  else
  {
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
  }

  strcpy(humidity_setp, custom_humidity_setp.getValue());
  strcpy(temperature_setp, custom_temperature_setp.getValue());
  sscanf(humidity_setp,"%d",&_humidity_setp);
  sscanf(temperature_setp,"%d",&_temperature_setp);

  if (shouldSaveConfig)
  {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["humidity_setp"] = humidity_setp;
    json["temperature_setp"] = temperature_setp;

    // json["ip"]          = WiFi.localIP().toString();
    // json["gateway"]     = WiFi.gatewayIP().toString();
    // json["subnet"]      = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("failed to open config file for writing");
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
    shouldSaveConfig = false;
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  reconnect();

  Serial.println("+++++++++ CONTROLLER ACTUAL CONFIG +++++++++");
  Serial.printf("Temperature set point: %d \n",_temperature_setp);
  Serial.printf("Humidity set point: %d \n",_humidity_setp);
  Serial.println("+++++++++            END            +++++++++");
}

void loop()
{
  eval_ac_inputs();
  if (tCheck(&t_verify)) {
      readSHT20();
      tRun(&t_verify);
    }
  if (digitalRead(sw) == LOW)
  {
    volatile unsigned long counter = millis();
    volatile unsigned long acum = 0;
    while (digitalRead(sw) == LOW && millis() - counter < 5 * 1000)
    {
      acum++;
    }
    //Serial.println("SWITCH DADO");
    Serial.println(acum);
    if (acum > 3000000)
    {
      Serial.println("RESET DONE");
      wm.resetSettings();
      ESP.restart();
    }
  }

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

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