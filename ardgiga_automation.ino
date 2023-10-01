#include <dhtnew.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "arduino_secrets.h"
#include <ModbusRTUMaster.h>
#include "Nextion.h"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = WSSID;        // your network SSID (name)
char pass[] = WPSWD;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

#define RELAY_HEAT 32
#define RELAY_COOL 33
#define RELAY_FAN 34
#define RELAY_LIGHTS1 22
#define RELAY_LIGHTS2 23
#define RELAY_LIGHTS3 24

#define MODBUS_DE 2
#define MODBUS_BAUD 9600
ModbusRTUMaster modbus(Serial1, MODBUS_DE); // serial port, driver enable pin for rs-485 (optional)

DHTNEW mySensor(48);

NexText nexTempF = NexText(0, 2, "tempf");
NexButton nexB0P0 = NexButton(0, 6, "b0p0");
NexButton nexB1P0 = NexButton(0, 7, "b1p0");
NexButton nexB2P0 = NexButton(0, 8, "b2p0");

NexTouch *nex_listen_list[] =
{
    &nexB0P0,
    &nexB1P0,
    &nexB2P0,
    NULL
};


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
bool stateLights1 = false;
bool stateLights1SW = true;
bool state_act_lights1 = false;
bool stateLights2 = false;
bool stateLights2SW = true;
bool state_act_lights2 = false;
bool stateLights3 = false;
bool stateLights3SW = true;
bool state_act_lights3 = false;
unsigned long coolTimer = 0UL;
bool setLights1 = false;
bool setLights2 = false;
bool setLights3 = false;

// Modbus
bool coils[2];
bool discreteInputs[2];
uint16_t holdingRegisters[2];
uint16_t inputRegisters[3];

void nexB0P0PopCallback(void *ptr)
{
    if (stateLights1SW == true)
    {
        stateLights1SW = false;
        nexB0P0.setText("Turn ON");
    }
    else
    {
        stateLights1SW = true;
        nexB0P0.setText("Turn OFF");
    }
}
void nexB1P0PopCallback(void *ptr)
{
    if (stateLights2SW == true)
    {
        stateLights2SW = false;
        nexB1P0.setText("Turn ON");
    }
    else
    {
        stateLights2SW = true;
        nexB1P0.setText("Turn OFF");
    }
}
void nexB2P0PopCallback(void *ptr)
{
    if (stateLights3SW == true)
    {
        stateLights3SW = false;
        nexB2P0.setText("Turn ON");
    }
    else
    {
        stateLights3SW = true;
        nexB2P0.setText("Turn OFF");
    }
}

boolean reconnect()
{
    if (client.connect("ArduinoRoom"))
    {
        client.subscribe("room/hvac/mode/set");
        client.subscribe("room/hvac/temperature/set");
        client.subscribe("room/switch/lights1/set");
        client.subscribe("room/switch/lights2/set");
        client.subscribe("room/switch/lights3/set");

    }
    return client.connected();
}

bool switch2Bool(String switch_state)
{
    if (switch_state == "ON")
    {
        return true;
    }
    else
    {
        return false;
    }
}

void callback(char* topic, byte* payload, unsigned int length)
{
    String tmpTopic = topic;
    char tmpStr[length+1];
    for (int x=0; x<length; x++)
    {
        tmpStr[x] = (char)payload[x]; // payload is a stream, so we need to chop it by length.
    }
    tmpStr[length] = 0x00; // terminate the char string with a null

    if (tmpTopic == "room/hvac/mode/set")
    {
        setMode = tmpStr;
    }
    else if (tmpTopic == "room/hvac/temperature/set")
    {
        setTemp = atoi(tmpStr);
    }
    else if (tmpTopic == "room/switch/lights1/set")
    {
        setLights1 = switch2Bool(tmpStr);
    }else if (tmpTopic == "room/switch/lights2/set")
    {
        setLights2 = switch2Bool(tmpStr);
    }else if (tmpTopic == "room/switch/lights3/set")
    {
        setLights3 = switch2Bool(tmpStr);
    }

}



void setup()
{
    nexInit();
    client.setServer(server, 1883);
    client.setCallback(callback);

    modbus.begin(MODBUS_BAUD); // baud rate, config (optional)

// check for the WiFi module:
    if (WiFi.status() == WL_NO_SHIELD)
    {
        //Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true);
    }

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED)
    {
        //Serial.print("Attempting to connect to SSID: ");
        //Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);

        // wait 3 seconds for connection:
        delay(3000);
    }
    delay(1500);
    lastReconnectAttempt = 0;

    for (int i = 22; i <= 34; i++)
    {
        pinMode(i, OUTPUT);
        digitalWrite(i,HIGH);
    }



    /* Register the pop event callback function of the current button component. */
    nexB0P0.attachPop(nexB0P0PopCallback, &nexB0P0);
    nexB1P0.attachPop(nexB1P0PopCallback, &nexB1P0);
    nexB2P0.attachPop(nexB2P0PopCallback, &nexB2P0);

}

void loop()
{
    nexLoop(nex_listen_list);
    if (!client.connected())
    {
        long now = millis();
        if (now - lastReconnectAttempt > 5000)
        {
            lastReconnectAttempt = now;
            // Attempt to reconnect
            if (reconnect())
            {
                lastReconnectAttempt = 0;
            }
        }
    }
    else
    {
        // Client connected

        client.loop();
    }

    if (millis() - lastTimer >= 500)
    {
        /** \brief setup current values
         *
         *
         */
        lastTimer = millis();
        int chk = mySensor.read();
        humidityRH = mySensor.getHumidity();
        tempF = ((mySensor.getTemperature() * 1.80) + 32.00);

        modbus.readDiscreteInputs(1, 0, discreteInputs, 3);

        /** \brief handle switches
         *
         *
         */

        if (setLights1 == true)
        {
            stateLights1 = true;
        }
        else
        {
            stateLights1 = false;
        }
        /*if (discreteInputs[0] == false)
        {
            stateLights1SW = true;
        }
        else
        {
            stateLights1SW = false;
        }*/
        if (stateLights1 == stateLights1SW)
        {
            digitalWrite(RELAY_LIGHTS1,HIGH);
            state_act_lights1 = true;
        }
        else
        {
            digitalWrite(RELAY_LIGHTS1,LOW);
            state_act_lights1 = false;
        }

        if (state_act_lights1 == true)
        {
            client.publish("room/switch/lights1","ON");
            nexB0P0.setText("Turn OFF");
        }
        else
        {
            client.publish("room/switch/lights1","OFF");
            nexB0P0.setText("Turn ON");
        }


        if (setLights2 == true)
        {
            stateLights2 = true;
        }
        else
        {
            stateLights2 = false;
        }
        /*if (discreteInputs[1] == false)
        {
            stateLights2SW = true;
        }
        else
        {
            stateLights2SW = false;
        }*/
        if (stateLights2 == stateLights2SW)
        {
            digitalWrite(RELAY_LIGHTS2,HIGH);
            state_act_lights2 = true;
        }
        else
        {
            digitalWrite(RELAY_LIGHTS2,LOW);
            state_act_lights2 = false;
        }

        if (state_act_lights2 == true)
        {
            client.publish("room/switch/lights2","ON");
            nexB1P0.setText("Turn OFF");
        }
        else
        {
            client.publish("room/switch/lights2","OFF");
            nexB1P0.setText("Turn ON");
        }


        if (setLights3 == true)
        {
            stateLights3 = true;
        }
        else
        {
            stateLights3 = false;
        }
        /*if (discreteInputs[2] == false)
        {
            stateLights3SW = true;
        }
        else
        {
            stateLights3SW = false;
        }*/
        if (stateLights3 == stateLights3SW)
        {
            digitalWrite(RELAY_LIGHTS3,HIGH);
            state_act_lights3 = true;
        }
        else
        {
            digitalWrite(RELAY_LIGHTS3,LOW);
            state_act_lights3 = false;
        }

        if (state_act_lights3 == true)
        {
            client.publish("room/switch/lights3","ON");
            nexB2P0.setText("Turn OFF");
        }
        else
        {
            client.publish("room/switch/lights3","OFF");
            nexB2P0.setText("Turn ON");
        }

        /** \brief handle thermostat
         *
         *
         */
        if (setMode == "off")
        {
            stateHeating = false;
            stateCooling = false;
        }
        else if (setMode == "heat")
        {
            stateCooling = false;
            if (tempF >= setTemp)
            {
                stateHeating = false;
            }
            else if (tempF <= setTemp - thermostatHysteresis)
            {
                stateHeating = true;
            }
        }
        else if (setMode == "cool")
        {
            stateHeating = false;
            if (tempF <= setTemp)
            {
                stateCooling = false;
            }
            else if (tempF >= setTemp + thermostatHysteresis)
            {
                stateCooling = true;
            }
        }
        else if (setMode == "auto")
        {
            if (tempF >= setTemp + thermostatHysteresis && stateCooling == false)
            {
                stateCooling = true;
                stateHeating = false;
            }
            else if (tempF <= setTemp - thermostatHysteresis && stateHeating == false)
            {
                stateCooling = false;
                stateHeating = true;
            }
            else if ((tempF >= setTemp && stateHeating == true) || (tempF <= setTemp && stateCooling == true))
            {
                stateCooling = false;
                stateHeating = false;
            }

        }
        if (stateHeating == true)
        {
            digitalWrite(RELAY_HEAT, LOW);
        }
        else
        {
            digitalWrite(RELAY_HEAT, HIGH);
        }
        if (stateCooling == true && stateFan == false && stateCool == false)
        {
            digitalWrite(RELAY_FAN, LOW);
            stateFan = true;
        }
        else if (stateCooling == true && millis() - coolTimer >= 30000 && stateFan == true && stateCool == false)
        {
            stateCool = true;
            coolTimer = millis();
            digitalWrite(RELAY_COOL, LOW);
        }
        else if(stateCooling == false)
        {
            digitalWrite(RELAY_COOL, HIGH);
            digitalWrite(RELAY_FAN, HIGH);
            stateFan = false;
            stateCool = false;
        }

        char sz[32];
        String strAction = "";
        if (stateHeating == true)
        {
            strAction = "heating";
        }
        else if (stateCooling == true)
        {
            strAction = "cooling";
        }
        else
        {
            strAction = "off";
        }
        strAction.toCharArray(sz, 32);
        client.publish("room/hvac/action",sz);
        sprintf(sz, "%6.2f", tempF);
        nexTempF.setText(sz);
        client.publish("room/hvac/temperature/current",sz);
        sprintf(sz, "%6.2f", humidityRH);
        client.publish("room/hvac/humidity/current",sz);


    }
}
