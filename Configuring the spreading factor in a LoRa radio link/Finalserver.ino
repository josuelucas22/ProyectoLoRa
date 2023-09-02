// Código para servidor TTN----Módulo TTGO LoRa32
#include <LoRa.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <U8glib.h>
#include <Arduino_LoRaWAN.h>
//Librería para comunicar con y dibujar en la pantalla OLED integrada
#include <SSD1306.h>
//Debemos definir los pines que se utilizarán por el módulo LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define AOUT_PIN 36   // Definir Pin de salida del TTGO LoRa32 para la lectura del sensor de humedad del suelo.
#define LEDPIN 2

//Definimos los pines necesarios para conectar con pantalla OLED
#define ANCHOPANTALLA 128 // El ancho de la pantalla en pixeles es de 128px
#define ALTOPANTALLA 64 // El ancho de la pantalla en pixeles es de 64px
#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
#define LEDPIN 2
const int Value_dry = 3770; //Límite del Valor del sensor cuando no estaba en presencia de agua (seco)
const int Value_wet = 1540; //Límite del valor del sensor cuando estaba en agua
int SensorValue = 0;
int MoisturePercent=0;
unsigned int counter = 0;
SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

// Datos para el servidor TTN
static const u1_t PROGMEM APPEUI[8]={ 0x38, 0xD1, 0x42, 0x48, 0xEB, 0x40, 0x76, 0xCA };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x04, 0xFF, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x81, 0xA3, 0xC8, 0x57, 0x77, 0x3A, 0xE5, 0x4A, 0xFC, 0xF3, 0x06, 0xD9, 0xB1, 0xA4, 0x19, 0x28 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[2] ={0x68}; 
static osjob_t sendjob;

// Programar la transmisión cada tantos segundos, en este caso, está programado cada 20 segundos
const unsigned TX_INTERVAL = 20;

// Asignación de pines para TTGO LoRa32 V1:
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }

            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            display.clear();
            display.drawString (0, 0, "EV_TXCOMPLETE event!");
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Programar la próxima transmisión
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // datos recibidos en la ranura de ping
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}
// Funcion humedad , para la toma de datos del sensor capacitivo
void humedad(){

 float SensorValue = analogRead(AOUT_PIN); // lee el valor de la entrada analogica del sensor
 float MoisturePercent=map(SensorValue, Value_dry, Value_wet, 0, 100);// funcion map para colocar rango de 1-100% de humedad
 if (MoisturePercent>100){ // evitar mas del 100% de humedad
 MoisturePercent=100;
 }
 if(MoisturePercent<0){ // evitar menos del 0% de humedad
 MoisturePercent=0;
 }
 float humedad = MoisturePercent* 1.0;
 mydata[1] = humedad * 2; // envia el valor de humedad en la posicion 1 de la cadena
 Serial.print(humedad);
 Serial.print(F("%"));
 Serial.println("");
 delay(250);

}

void do_send(osjob_t* j)
{
  // Verificar si no hay un trabajo TX / RX actual en ejecución
 if (LMIC.opmode & OP_TXRXPEND) {
 Serial.println(F("OP_TXRXPEND, not sending"));
 } else {
 humedad();
 Serial.print("LMIC.freq:");
 Serial.println(LMIC.freq);
 //Prepara la transmisión de datos para el siguiente momento
 LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
 digitalWrite(LEDPIN, HIGH);
 display.clear();
 display.drawString (0, 0, "Envío de paquete de enlace ascendente...");
 display.drawString (0, 20, String (++counter));
 String humidityS = "Porcentaje_Humedad: " + String(mydata[1]/2) + "%";
 display.drawString (0, 30,humidityS);
 display.display ();
 Serial.println(F("Packet queued"));
 // Establecer la velocidad de datos y la potencia de transmisión para el enlace ascendente (SF7,SF8,SF9 y SF10) 
 LMIC_setDrTxpow(DR_SF10,14); 
 }
 // La siguiente transmisión está programada después del evento TX_COMPLETE.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));
    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);

    // Configurar y restablecer la OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);

    display.init ();
    display.flipScreenVertically ();
    display.setFont (ArialMT_Plain_10);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    display.drawString (0, 0, "Empezando....");
    display.display ();
    // Iniciar LMIC 
    os_init();
    // Restablecer el estado MAC. Las transferencias de sesión y de datos pendientes se descartarán.
    LMIC_reset();
    
    LMIC_setAdrMode (0); 
    LMIC.dn2Dr = DR_SF9; 
 
    // Iniciar trabajo (el envío también inicia automáticamente OTAA)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

