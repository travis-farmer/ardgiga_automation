#include <dhtnew.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = WSSID;        // your network SSID (name)
char pass[] = WPSWD;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

#define RELAY_HEAT 30
#define RELAY_COOL 31
#define RELAY_FAN 32
#define RELAY_LIGHTS 2

#define SWITCH_LIGHTS 22


DHTNEW mySensor(48);


// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xBE };
IPAddress server(192, 168, 1, 100);
IPAddress ip(192, 168, 0, 29);
IPAddress myDns(192, 168, 0, 1);

WiFiClient wClient;
PubSubClient client(wClient);

long lastReconnectAttempt = 0;

int setTemp = 0;
String setMode = "";
float tempF = 0.00;
float humidityRH = 0.00;
float thermostatHysteresis = 3.00;
unsigned long lastTimer = 0UL;
bool stateHeating = false;
bool stateCooling = false;
bool stateFan = false;
bool stateCool = false;
bool stateLights1 = false;
bool stateComp1 = false;
bool stateLights1SW = false;
bool stateComp1SW = false;
bool state_act_lights1 = false;
bool state_act_comp1 = false;
unsigned long coolTimer = 0UL;
String setLights1 = "";
String setComp1 = "";

boolean reconnect() {
  if (client.connect("ArduinoRoom")) {
    client.subscribe("roomhvac/mode/set");
    client.subscribe("roomhvac/temperature/set");
    client.subscribe("room/switch/lights1/set");

  }
  return client.connected();
}

void callback(char* topic, byte* payload, unsigned int length) {
  String tmpTopic = topic;
  char tmpStr[length+1];
  for (int x=0; x<length; x++) {
    tmpStr[x] = (char)payload[x]; // payload is a stream, so we need to chop it by length.
  }
  tmpStr[length] = 0x00; // terminate the char string with a null

  if (tmpTopic == "roomhvac/mode/set") {setMode = tmpStr; }
  else if (tmpTopic == "roomhvac/temperature/set") { setTemp = atoi(tmpStr); }
  else if (tmpTopic == "room/switch/lights1/set") { setLights1 = tmpStr; }

}

void setup() {
  client.setServer(server, 1883);
  client.setCallback(callback);

 // check for the WiFi module:
  if (WiFi.status() == WL_NO_SHIELD) {
    //Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    //Serial.print("Attempting to connect to SSID: ");
    //Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 3 seconds for connection:
    delay(3000);
  }
  delay(1500);
  lastReconnectAttempt = 0;

  /*for (int i=30; i<= 46; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH); // Relays are active with a LOW signal
  }*/

  for (int i = 30; i <= 34; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i,HIGH);
  }
  pinMode(RELAY_LIGHTS,OUTPUT);
  digitalWrite(RELAY_LIGHTS,LOW);

  pinMode(SWITCH_LIGHTS,INPUT_PULLUP);

}

void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    client.loop();
  }

  if (millis() - lastTimer >= 500) {
    /** \brief setup current values
     *
     *
     */
    lastTimer = millis();
    int chk = mySensor.read();
    humidityRH = mySensor.getHumidity();
    tempF = ((mySensor.getTemperature() * 1.80) + 32.00);



    /** \brief handle switches
     *
     *
     */
    if (setLights1 == "ON") {
        stateLights1 = true;
    } else if (setLights1 == "OFF") {
        stateLights1 = false;
    }
    if (digitalRead(SWITCH_LIGHTS) == HIGH) {stateLights1SW = true;}
    else {stateLights1SW = false;}
    if (stateLights1 == stateLights1SW) {
        digitalWrite(RELAY_LIGHTS,HIGH);
        state_act_lights1 = true;
    } else {
        digitalWrite(RELAY_LIGHTS,LOW);
        state_act_lights1 = false;
    }


    /** \brief handle thermostat
     *
     *
     */
    if (setMode == "off") {
        stateHeating = false;
        stateCooling = false;
    } else if (setMode == "heat") {
        stateCooling = false;
        if (tempF >= setTemp) {stateHeating = false;}
        else if (tempF <= setTemp - thermostatHysteresis) {stateHeating = true;}
    } else if (setMode == "cool") {
        stateHeating = false;
        if (tempF <= setTemp) {stateCooling = false;}
        else if (tempF >= setTemp + thermostatHysteresis) {stateCooling = true;}
    } else if (setMode == "auto") {
        if (tempF >= setTemp + thermostatHysteresis && stateCooling == false) {
            stateCooling = true;
            stateHeating = false;
        } else if (tempF <= setTemp - thermostatHysteresis && stateHeating == false) {
            stateCooling = false;
            stateHeating = true;
        } else if ((tempF >= setTemp && stateHeating == true) || (tempF <= setTemp && stateCooling == true)) {
            stateCooling = false;
            stateHeating = false;
        }

    }
    if (stateHeating == true) {digitalWrite(RELAY_HEAT, LOW);}
    else {digitalWrite(RELAY_HEAT, HIGH);}
    if (stateCooling == true && stateFan == false && stateCool == false) { digitalWrite(RELAY_FAN, LOW); stateFan = true; }
    else if (stateCooling == true && millis() - coolTimer >= 30000 && stateFan == true && stateCool == false) {stateCool = true; coolTimer = millis(); digitalWrite(RELAY_COOL, LOW);}
    else if(stateCooling == false) {digitalWrite(RELAY_COOL, HIGH); digitalWrite(RELAY_FAN, HIGH); stateFan = false; stateCool = false;}

    /** \brief setup and send values and states
     *
     *
     */
    char sz[32];
    String strAction = "";
    if (stateHeating == true) {strAction = "heating";}
    else if (stateCooling == true) {strAction = "cooling";}
    else {strAction = "off";}
    strAction.toCharArray(sz, 32);
    client.publish("roomhvac/action",sz);
    dtostrf(tempF, 4, 2, sz);
    client.publish("roomhvac/temperature/current",sz);
    dtostrf(humidityRH, 4, 2, sz);
    client.publish("roomhvac/humidity/current",sz);
    if (state_act_lights1 == true) {
        client.publish("room/switch/lights1","ON");
    } else {
        client.publish("room/switch/lights1","OFF");
    }
  
    if (state_act_lights1 == true) {
        client.publish("room/switch/lights1/state","ON");
    } else {
        client.publish("room/switch/lights1/state","OFF");
    }
 
  }
}
