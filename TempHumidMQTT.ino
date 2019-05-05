#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define DEVICE_NAME "mySens-01"
#define MQTT_SERVER_IP "192.168.0.111"
#define QCONNECT
#define CONNECT_TIMEOUT 150
#define INTERVAL 5e6
#define WIFI_SSID "Eine Scheibe Brot"
#define WIFI_PASSWORD "T3m$7gHQ3Z"
IPAddress IP(192, 168, 0, 33);
IPAddress GATEWAY(192, 168, 0, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress DNS(192, 168, 0, 1);


char msg[50];
float tmp = 0;
float humidity = 0;
long bootTime;
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(D2, DHT22);


struct {
  uint32_t crc32;   // 4 bytes
  uint8_t wifi_channel;  // 1 byte,   5 in total
  uint8_t bSSID[6]; // 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
} rtcData;

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
void storeRTC() {
  rtcData.wifi_channel = WiFi.channel();
  memcpy( rtcData.bSSID, WiFi.BSSID(), 6 );
  rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );
}

void setup_wifi() {
  int tries = 0;
  bool rtcValid = false;
  WiFi.mode(WIFI_STA);
  WiFi.hostname(DEVICE_NAME);
  WiFi.config(IP, GATEWAY, SUBNET);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

#ifdef QCONNECT
  Serial.println("(quick connect)");
  if ( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc32 ) {
      rtcValid = true;
    }
  }
  if ( rtcValid ) {
    // The RTC data was good, make a quick connection
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD, rtcData.wifi_channel, rtcData.bSSID, true);
  }
  else {
    // The RTC data was not valid, so make a regular connection
    Serial.println("crc invalid! Attempting regular connect");
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
  tmp = dht.readTemperature();
  humidity = dht.readHumidity();
  Serial.println("temp: " + String(tmp));
  Serial.println("hum: " + String(humidity));
  dtostrf(tmp, 6, 2, msg);
  dtostrf(humidity, 6, 2, msg);
  return client.publish("homie/"DEVICE_NAME"/DHT22/temperature", msg, true) & client.publish("homie/"DEVICE_NAME"/DHT22/humidity", msg, true);
}

void setup() {
  bootTime = millis();
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
