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

#define RELAY_HEAT 32
#define RELAY_COOL 33
#define RELAY_FAN 34
#define RELAY_LIGHTS 22



DHTNEW mySensor(48);


// Update these with values suitable for your hardware/network.
IPAddress server(192, 168, 1, 100);

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
bool state_act_lights1 = false;
unsigned long coolTimer = 0UL;
String setLights1 = "";
String setComp1 = "";

boolean reconnect() {
  if (client.connect("ArduinoRoom")) {
    client.subscribe("room/hvac/mode/set");
    client.subscribe("room/hvac/temperature/set");
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

  if (tmpTopic == "room/hvac/mode/set") {setMode = tmpStr; }
  else if (tmpTopic == "room/hvac/temperature/set") { setTemp = atoi(tmpStr); }
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

  for (int i = 22; i <= 34; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i,HIGH);
  }

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
        digitalWrite(RELAY_LIGHTS,HIGH);
        state_act_lights1 = true;
    } else if (setLights1 == "OFF") {
        digitalWrite(RELAY_LIGHTS,LOW);
        state_act_lights1 = false;
    }

    /*if (state_act_lights1 == true) {
        client.publish("room/switch/lights1","ON");
    } else {
        client.publish("room/switch/lights1","OFF");
    }*/

    if (state_act_lights1 == true) {
        client.publish("room/switch/lights1/state","ON");
    } else {
        client.publish("room/switch/lights1/state","OFF");
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

    char sz[32];
    String strAction = "";
    if (stateHeating == true) {strAction = "heating";}
    else if (stateCooling == true) {strAction = "cooling";}
    else {strAction = "off";}
    strAction.toCharArray(sz, 32);
    client.publish("room/hvac/action",sz);
    sprintf(sz, "%6.2f", tempF);
    client.publish("room/hvac/temperature/current",sz);
    sprintf(sz, "%6.2f", humidityRH);
    client.publish("room/hvac/humidity/current",sz);


  }
}
