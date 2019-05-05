#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define DEVICE_NAME "mySens-01" //This name is used for both WIFI and MQTT. Must be unique.
#define MQTT_SERVER_IP "192.168.0.111" //Address of the MQTT Broker.
#define QCONNECT //Accelerates wifi-connection after deep sleep. 
#define CONNECT_TIMEOUT 150 //Number of attempts before wifi connection times out.
#define INTERVAL 5e6 //Duration of sleep cycles 
#define WIFI_SSID "Eine Scheibe Brot"
#define WIFI_PASSWORD "T3m$7gHQ3Z"
IPAddress IP(192, 168, 0, 33); //Static ip
IPAddress GATEWAY(192, 168, 0, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress DNS(192, 168, 0, 1);

long bootTime; //Used to calculate code execution time
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(D2, DHT22);

/* Struct rtcData: used to store wifi parameters inside the rtc memory during deep sleep.
    This is necessary for wifi quick connect. Must be a multiple of 4 bytes*/
struct {
  uint32_t crc;
  uint8_t wifi_channel;
  uint8_t bSSID[6];
  uint8_t reserved ;
} rtcData;

/*Taken from the RTCUserMemory example. This function calculates a crc32 ckecksum for given data.
   Used to ensure dada validity. The Data must be alligned for this function to work.
   Arguments:
 * *data -> pointer to starting address of the data
   length -> lenght of the data */
uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while ( length-- ) {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if ( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if ( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

//This function stores the current wifi channel and bSSID in the rtc memory.
void storeRTC() {
  rtcData.wifi_channel = WiFi.channel();
  memcpy( rtcData.bSSID, WiFi.BSSID(), 6 );
  rtcData.crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );
}

// Connect to the wifi network
void setup_wifi() {
  bootTime = millis();
  int tries = 0;
  bool rtcOK = false;
  WiFi.mode(WIFI_STA);
  WiFi.hostname(DEVICE_NAME);
  WiFi.config(IP, GATEWAY, SUBNET);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
#ifdef QCONNECT
  Serial.println("(quick connect)");
  if ( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) )
  {
    //first 4 bytes are skipped because they contain the checksum itself
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc ) {
      rtcOK = true;
    }
  }
  if ( rtcOK ) {
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD, rtcData.wifi_channel, rtcData.bSSID, true);
  }
  else {
    // If the RTC data was not valid, we attempt a regular slower connection
    Serial.println("crc checksum not correct! Attempting regular connect");
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
  }
#else
  Serial.println("(regular connect)");
  WiFi.begin( WIFI_SSID, WIFI_PASSWORD);
#endif
  while (WiFi.status() != WL_CONNECTED && tries++ < CONNECT_TIMEOUT )
  {
    Serial.print(".");
    delay(50);
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("failed to connect!");
#ifdef QCONNECT
    Serial.println("attempting regular connect");
    //Discard previous wifi parameters and try a normal connect.
    WiFi.disconnect();
    delay( 100 );
    WiFi.forceSleepBegin();
    delay( 10 );
    WiFi.forceSleepWake();
    delay( 10 );
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
    tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries++ < CONNECT_TIMEOUT)
    {
      Serial.print(".");
      delay(50);
    }
#endif
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("going to sleep to try again later");
      WiFi.disconnect();
      //If no wifi connection could be made go to sleep for 3s and try again.
      ESP.deepSleep(3e6);
    }
  }
  Serial.println("");
  Serial.println("connection took: " + String(millis() - bootTime) + String("ms"));
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print(" gateway: ");
  Serial.print(WiFi.gatewayIP());
  Serial.print(" subnet: ");
  Serial.println(WiFi.subnetMask());
  storeRTC();
}

bool send_values_MQTT()
{
  int attempts = 0;
  char msg[50];  //buffer for MQTT messages
  dht.begin();
  client.setServer(MQTT_SERVER_IP, 1883);
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection:");
    if (client.connect(DEVICE_NAME)) {
      Serial.println("connected");
      client.loop();
    } else {
      if (attempts++ > 5)
      {
        return false;
      }
      Serial.print(".");
      delay(20);
    }
  }
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  Serial.println("temp: " + String(temperature));
  Serial.println("hum: " + String(humidity));
  dtostrf(temperature, 6, 2, msg);
  dtostrf(humidity, 6, 2, msg);
  return client.publish(DEVICE_NAME"/DHT22/temperature", msg, true) & client.publish(DEVICE_NAME"/DHT22/humidity", msg, true);
}

void setup() {
  Serial.begin(115000);
  setup_wifi();
  if (!send_values_MQTT())
  {
    Serial.println("MQTT connection failed");
  }
  delay(50);
  client.disconnect();
  Serial.println("going to sleep after " + String(millis() - bootTime) + String("ms"));
  ESP.deepSleep(INTERVAL);
  delay(1000);
}

void loop() {}
