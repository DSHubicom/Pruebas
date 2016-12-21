#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define LED_GPIO 2 //LED interno
#define HUMEDAD_GPIO A0 //Nivel de humedad (moisture sensor)

/************************* Parámetros de conexión red WiFi WiFi *********************************/
#define WLAN_SSID       "Orange-152A"
#define WLAN_PASS       "3EE529C4"

/************************* Parámatros de conexión Servidor MQTT (e.g., Adafruit.io) *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   //8883 para SSL
#define AIO_USERNAME    "mlinaje"
#define AIO_KEY         "167931566c6f53a9e2be8f7f153cef99500bd08b"

/************ Variables para cliente WiFi y cliente MQTT ******************/
// Crea un cliente ESP8266
WiFiClient client; //usar: WiFiClientSecure client; para cliente SSL

// Crea el cliente MQTT
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Canales (Feeds) ***************************************/
// La ruta cambia según el servidor MQTT usado, para Adafruit.io: <username>/feeds/<feedname>
// Nos subscribiremos al canal "canalonoff"
Adafruit_MQTT_Subscribe canalonoff = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");

// Publicaremos en un canal el valor del sensor de humedad, el canal se llamara "canalhumedad"
Adafruit_MQTT_Publish canalhumedad = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humedad");

// Variable para el calculo de humedad para su envío
unsigned int porcentajeHumedad;

//Variable para el control de errores de envío al publicar
unsigned int errorPublicacion;

/***************************************************************/
void setup() {
  Serial.begin(115200);

  //Configuramos los pines como entradas o salidas
  pinMode(HUMEDAD_GPIO, INPUT);
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, HIGH); //encendemos el LED interno al comienzo

  //Conectamos a la WiFi
  WIFI_connect();

  // Nos subscribimos al canal MQTT canalonoff
  mqtt.subscribe(&canalonoff);
}


void loop() {
  // MQTT_connect() sirve tanto para la primera conexión como para en caso de producirse una desconexión, volver a conectar (por eso se llama en cada iteracción del loop)
  MQTT_connect();

  //ESP8266 soporta comunicación bidireccional, pero no simultaneamente: si está enviando no puede recibir y si está recibiendo no puede enviar
  //así que en este caso primero nos ponderemos a escuchar (los canales a los que estemos subscritos que nos interesen) y después a publicar (en los canales correspondientes)

  //****** SUBSCRIPCION
  //Vemos si hay mensajes por leer en los canales que nos interesan, como pueden ser varios, y en este caso sólo nos interesa el canalonoff,
  //filtramos los mensajes con "if (subscription == &canalonoff)"
  //En caso de publicarse algo en el tiempo que estamos pendientes de las subscripciones (50000ms en este caso) lo leeríamos.
  //Como ejemplo, si el mensaje recibido es ON, encendemos el LED, en caso contrario lo apagamos
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &canalonoff) {
      if (strcmp((char *)canalonoff.lastread, "ON"))
        digitalWrite(LED_GPIO, HIGH);
      else //si no es ON es OFF
        digitalWrite(LED_GPIO, LOW);
    }
  }

  //****** PUBLICACION
  //Calculo el valor a enviar (pasándolo a %)
  Serial.println(analogRead(HUMEDAD_GPIO));
  porcentajeHumedad = analogRead(HUMEDAD_GPIO) * 100 / 1024; //Los valores leidos por el sensor van de 0 a 1024, los transformamos de 0 a 100%

  //Si no hay error de publicación, la función devuelve 0, sino el código de error correspondiente (sólo interesante para debug)
  if (! (errorPublicacion = canalhumedad.publish(porcentajeHumedad))) {
    Serial.print("Error de publicación (error ");
    Serial.print(errorPublicacion);
    Serial.println(")");
  }

  delay(10000);
}


// Funcción para conectar inicialmente y reconectar cuando se haya perdido la conexión al servidor MQTT
void MQTT_connect() {
  int8_t ret;

  if (!mqtt.connected()) {

    Serial.println("Conectando al servidor MQTT... ");

    uint8_t intentos = 3;
    while ((ret = mqtt.connect()) != 0) { // connect devuelve 0 cuando se ha conectado correctamente y el código de error correspondiente en caso contrario
      Serial.println(mqtt.connectErrorString(ret)); // (sólo interesante para debug)
      Serial.println("Reintentando dentro de 3 segundos...");
      mqtt.disconnect();
      delay(3000);  // esperar 3 segundos
      if (! intentos--) { //decrementamos el número de intentos hasta que sea cero
        while (1);  // El ESP8266 no soporta los while(1) sin código dentro, se resetea automáticamente, así que estamos forzando un reset
          //Si no quisieramos que se resetease, dentro del while(1) habría que incluir al menos una instrucción, por ejemplo delay(1); o yield();
      }
    }
    Serial.println("MQTT conectado");
  }
}

void WIFI_connect() {
  // Poner el módulo WiFi en modo station (este modo permite de serie el "light sleep" para consumir menos
  // y desconectar de cualquier red a la que pudiese estar previamente conectado
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(2000);

  // Conectamos a la WiFi
  Serial.println("Conectando a la red WiFi");

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) { //Nos quedamos esperando hasta que conecte
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado.");
}


/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution

  Modificaciones de programación estructurada, traducción y añadidos por Marino Linaje
 ****************************************************/

