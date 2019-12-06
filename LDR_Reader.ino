#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ArduinoHttpClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

#define AIO_USERNAME "YOUR_AIO_USERNAME"
#define AIO_KEY "YOUR_AIO_KEY"
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883

#define LDR_PIN A0
#define ADC_REF_VOLTAGE 3.3
#define MAX_ADC_READING 1023
#define REF_RESISTANCE 9740
#define RESISTOR_ADJUST 0.68
#define LUX_ADJUST 1.5
#define LUX_CALC_SCALAR 6896474446
#define LUX_CALC_EXPONENT -1.85948

int LDR_Reading;
float Resistor_Voltage;
float LDR_Voltage;
float LDR_Resistance;
float LDR_Lux;

WiFiClient client;

// Connect to Wi-Fi and Adafruit IO handel 
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish LUX = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/energy-harvesting.lux");


void setup(void) {

  Serial.begin(115200);
  pinMode(A0,OUTPUT);
  while (!Serial);
  startWIFI();
}

void loop(void) {

   if (WiFi.status() != WL_CONNECTED) {
    startWIFI();
    //Reconectar ao Adafruit
    if (! mqtt.ping()) {
      if (! mqtt.connected())
        connect();
    }
   }
    
    LDR_Reading = analogRead(LDR_PIN);
    Resistor_Voltage = (((float)LDR_Reading)*(ADC_REF_VOLTAGE/MAX_ADC_READING))*RESISTOR_ADJUST;
    LDR_Voltage = ADC_REF_VOLTAGE - Resistor_Voltage;
    LDR_Resistance = ((LDR_Voltage/Resistor_Voltage)*REF_RESISTANCE);
    LDR_Lux =  LUX_CALC_SCALAR * pow(LDR_Resistance, LUX_CALC_EXPONENT);
    // LDR_Lux = (((2500/LDR_Voltage) - 500)/REF_RESISTANCE)*LUX_ADJUST;
    LUX.publish(LDR_Lux);
    
    Serial.println("READING VALUES");
    Serial.print("Resistor Voltage : "); Serial.print(Resistor_Voltage); Serial.println(" volts");
    Serial.print("LDR Raw Data : "); Serial.println(LDR_Reading);
   //  Serial.print("LDR Voltage : "); Serial.print(LDR_Voltage); Serial.println(" volts");
   //  Serial.print("LDR Resistance : "); Serial.print(LDR_Resistance); Serial.println(" Ohms");
    Serial.print("LDR Illuminance: "); Serial.print(LDR_Lux); Serial.println(" lux");
    
    delay(2000);
}

void startWIFI() {

  Serial.printf("\nIniciando a conexão... '%s'\n");
  WiFi.begin(WIFI_SSID,  WIFI_PASSWORD);
  while (WiFi.status() == WL_DISCONNECTED) {
    delay(200);
    Serial.print(".");
  }

  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    Serial.printf("\nConectado ao SSID:  '%s'\n", WiFi.SSID().c_str());
  } else {
    Serial.printf("\nNão foi possível conectar ao WI-FI. state='%d'", status);
  }

  //Conecta ao Adafruit IO
  Serial.print("\nIniciando a conexão ao Adafruit... '%s'\n");
  connect();
}

void connect() {

  Serial.print(F("Conectando ao Adafruit IO... "));

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {

    switch (ret) {
      case 1: Serial.println(F("Erro de Protocolo")); break;
      case 2: Serial.println(F("ID Rejeitado")); break;
      case 3: Serial.println(F("Servidor Indisponível")); break;
      case 4: Serial.println(F("Erro de usuário/senha")); break;
      case 5: Serial.println(F("Não autorizado")); break;
      case 6: Serial.println(F("Falha de subscribe")); break;
      default: Serial.println(F("Falha de conexão")); break;
    }

    if (ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    yield();

  }

  Serial.println(F("Adafruit IO Connected!"));
}
