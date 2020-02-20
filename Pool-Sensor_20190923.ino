// Wemos D1 board, connected to a battery box and a DS18B20 temperature sensor
//
// For temperature reading
// Libraries needed:
// * OneWire
// * DallasTemperature
//
// Pinout: https://wiki.wemos.cc/products:d1:d1_mini
// D0 = GPIO16 --> Connect D0 to RST for Deep Sleep-Wakeup
#include <OneWire.h>
#include <DallasTemperature.h>
const char* ssid = "JoE20151208";
const char* password = "JoE_Berlin";
#define DEVICENAME "joe_berlin/poolsensor"
#define TOPIC1 DEVICENAME "/temperature"
#define TOPIC2 DEVICENAME "/voltage"
#define ONLINETOPIC DEVICENAME "/online"
#define MQTTSERVER IPAddress(5, 196, 95, 208) // test.mosquitto.org = 37.187.106.16 neu 5.196.95.208
const int sleepTimeS = 900; // Reduce this value for debugging. Increase if you want more battery life
// For One-Wire
#define VCCPIN D7
#define ONE_WIRE_BUS D6
#define GNDPIN D5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempC;

//For ADC-Measurement
//ADC_MODE(ADC_VCC);
unsigned int raw=0;
float volt=0.0;
String Ubatt = "";

// For WLAN & MQTT
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
AsyncMqttClient mqttClient;
uint16_t packetId1Pub;
bool packet1Ack = false;
bool ready = false;
char *ftoa( double f, char *a, int precision)
{
long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
char *ret = a;
long heiltal = (long)f;
itoa(heiltal, a, 10);
while (*a != '\0') a++;
*a++ = '.';
long desimal = abs((long)((f - heiltal) * p[precision]));
itoa(desimal, a, 10);
return ret;
}

void onMqttPublish(uint16_t packetId) {
Serial.println("** Publish acknowledged **");
Serial.print(" packetId: ");
Serial.println(packetId);
// TODO: Only when both packages were acknowledged...
if (packetId == packetId1Pub) {
packet1Ack = true;
}
if (packet1Ack) {
ready = true;
}
}
void onMqttConnect(bool sessionPresent) {
char buf[7];
packetId1Pub = mqttClient.publish(TOPIC1, 1, true, ftoa(tempC, buf, 2));
Serial.print("TOPIC: ");
Serial.println(TOPIC1);
packetId1Pub = mqttClient.publish(TOPIC2, 1, true, ftoa(volt, buf, 2));
Serial.print("TOPIC: ");
Serial.println(TOPIC2);
}
void setup() {
pinMode(GNDPIN, OUTPUT);
pinMode(VCCPIN, OUTPUT);
digitalWrite(GNDPIN, LOW);
digitalWrite(VCCPIN, HIGH);
Serial.begin(115200);
Serial.println("ESP-Temperature-Reader-and-MQTT-Poster-via-WiFi - 20190923");
Serial.print("Refresh: ");
Serial.print(sleepTimeS);
Serial.println("s");

// Betriebsspannung auslesen
pinMode(A0, INPUT);
raw = analogRead(A0);
volt=raw/1024.0;
volt=volt*4.2;
Ubatt = String(volt, 3);
Serial.println("Ubatt: " + Ubatt + "V");

// Start up the sensors library
delay(1000); // Delay eingeführt, weil sonst nur der 1te Wert nach Reset einwandfrei, danach immer +85grdC gelesen wurde
sensors.begin();
}
void loop() {
// Send the command to get temperature readings
Serial.println("Requesting Temperature");
sensors.requestTemperatures();
// You can have more than one DS18B20 on the same bus.
// 0 refers to the first IC on the wire
Serial.print("Requesting Temperature from Device 0: ");
tempC = sensors.getTempCByIndex(0);
Serial.print(tempC);
Serial.println("grdC");
Serial.println("Connecting to WIFI");
// Connect to WiFi
WiFi.begin(ssid, password);
int timeout = 0;
while (WiFi.status() != WL_CONNECTED) {
timeout++;
if (timeout>20) {
// WIFI isn't available after 10 seconds -> abort mission, mission's a failure
initiateDeepSleep();
}
delay(500);
Serial.print(".");
}
Serial.println("");
Serial.println("WiFi connected");
// Print the IP address
Serial.println(WiFi.localIP());
// Publish result to MQTT
mqttClient.onConnect(onMqttConnect);
mqttClient.onPublish(onMqttPublish);
mqttClient.setServer(MQTTSERVER, 1883);
mqttClient.setKeepAlive(5).setCleanSession(false).setWill(ONLINETOPIC, 2, true, "no"); // .setCredentials("user", "pass").setClientId(DEVICENAME);
//setKeepAlive(5) in seconds
//setWill(const char* topic, uint8_t qos, bool retain, const char* payload = nullptr, size_t length = 0)
//setWill(ONLINETOPIC, 2, true, "no")
//        topic
//                     qos=2 In der höchsten Stufe 2 garantiert der Broker sogar exactly-once: die Nachricht wird also genau einmal abgelegt, nicht öfter und nicht weniger.
Serial.println("Connecting to MQTT...");
Serial.print("MQTTSERVER: ");
Serial.println(MQTTSERVER);
mqttClient.connect();
timeout = 0;
while (!ready) {
delay(250);
timeout++;
if (timeout > 40)
{
// MQTT isn't available after 10 seconds -> abort mission, mission's a failure
initiateDeepSleep();
}
Serial.print(".");
}
Serial.println("");
initiateDeepSleep();
}
void initiateDeepSleep()
{
ESP.deepSleep(sleepTimeS * 1000000);
delay(100);
}
