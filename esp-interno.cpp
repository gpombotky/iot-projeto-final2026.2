/*
 * ============================================================================
 * PROJETO FINAL IoT - MONITORAMENTO INTELIGENTE DE SALAS
 * ============================================================================
 * Arquivo: esp-interno.cpp
 * Dispositivo: Heltec WiFi LoRa 32 (Nó Interno Receptor, RFID, DHT & Climatização)
 * 
 * Baseado ESTRITAMENTE em:
 * - modularizacao/comunicacao-lora-receptor.cpp
 * - modularizacao/cracha-rfid.cpp
 * - modularizacao/sensor-temp-umidade.cpp
 * ============================================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "HT_SSD1306Wire.h"

// Opcional: Se for usar WiFi/MQTT com Node-RED, ative a flag abaixo e coloque seus dados
#define USAR_WIFI_MQTT                              false
#if USAR_WIFI_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
const char* WIFI_SSID     = "SEU_WIFI_SSID";
const char* WIFI_PASSWORD = "SUA_WIFI_SENHA";
const char* MQTT_BROKER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
#endif

// ---------------------------------------------------------------------------
// 1. Configurações LoRa Receptor (idênticas a comunicacao-lora-receptor.cpp)
// ---------------------------------------------------------------------------
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30        // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
int16_t txNumber;
int16_t rssi, rxSize;
bool lora_idle = true;
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

// ---------------------------------------------------------------------------
// 2. Configurações RFID RC522 (idênticas a cracha-rfid.cpp)
// ---------------------------------------------------------------------------
#define SS_PIN   7
#define RST_PIN  3
#define SCK_PIN  6
#define MISO_PIN 4
#define MOSI_PIN 5

// Inicializa o leitor RFID
MFRC522 rfid(SS_PIN, RST_PIN);

// Guarda ate 20 crachas (exatamente como em cracha-rfid.cpp)
byte sala[20][10];
byte tamanhoUidSala[20];
int qtdPessoas = 0;
unsigned long ultimoAviso = 0;
unsigned long ultimaChecagemRFID = 0;

// ---------------------------------------------------------------------------
// 3. Sensor DHT Interno (idêntico a sensor-temp-umidade.cpp)
// ---------------------------------------------------------------------------
#define DHTTYPE    DHT11
#define DHTPIN     19
DHT_Unified dht(DHTPIN, DHTTYPE);

// ---------------------------------------------------------------------------
// 4. Display OLED Onboard da Heltec
// ---------------------------------------------------------------------------
#ifdef SDA_OLED
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#else
static SSD1306Wire display(0x3c, 500000, 17, 18, GEOMETRY_128_64, 21);
#endif

// Variáveis de temperatura
float tempExterna = 0.0;
bool tempExternaRecebida = false;
float tempInterna = 0.0;
unsigned long ultimaLeituraDHT = 0;
unsigned long ultimoUpdateDisplay = 0;

// ---------------------------------------------------------------------------
// Função: Cálculo da Temperatura Ideal do Ar Condicionado
// ---------------------------------------------------------------------------
float calcularTemperaturaIdeal(float tInt, float tExt, int pessoas) {
    if (pessoas == 0) {
        return 25.0; // Modo economia quando sala vazia
    }
    // Base de conforto térmico (NR-17): 23 °C
    float tIdeal = 23.0;

    // Compensação por ocupação: -0.5 °C por pessoa adicional
    tIdeal -= (pessoas - 1) * 0.5;

    // Compensação de calor externo recebido via LoRa
    if (tempExternaRecebida) {
        if (tExt >= 30.0) tIdeal -= 1.0;
        else if (tExt < 20.0) tIdeal += 1.0;
    }

    // Limites operacionais de conforto
    if (tIdeal < 18.0) tIdeal = 18.0;
    if (tIdeal > 26.0) tIdeal = 26.0;
    return tIdeal;
}

// ---------------------------------------------------------------------------
// Função: Atualização do Display OLED da Heltec
// ---------------------------------------------------------------------------
void atualizarDisplayOLED() {
    float arIdeal = calcularTemperaturaIdeal(tempInterna, tempExterna, qtdPessoas);

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "=== MONITOR DE SALA ===");

    char buffer[32];
    sprintf(buffer, "Pessoas na sala: %d", qtdPessoas);
    display.drawString(0, 15, buffer);

    if (tempInterna > 0.0) {
        sprintf(buffer, "Temp Int: %.1f C", tempInterna);
    } else {
        sprintf(buffer, "Temp Int: Lendo...");
    }
    display.drawString(0, 27, buffer);

    if (tempExternaRecebida) {
        sprintf(buffer, "Temp Ext: %.1f C (LoRa)", tempExterna);
    } else {
        sprintf(buffer, "Temp Ext: Aguardando...");
    }
    display.drawString(0, 39, buffer);

    sprintf(buffer, "Ar Ideal: %.1f C", arIdeal);
    display.drawString(0, 51, buffer);

    display.display();
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1500); // Espera o Monitor Serial reconectar apos o upload (como em cracha-rfid.cpp)

    // 1. Inicializa MCU Heltec (OBRIGATÓRIO ser a primeira chamada)
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    // 2. Inicializa Display OLED integrado
    #ifdef Vext
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW); // Liga alimentação do OLED na Heltec
    delay(50);
    #endif
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.clear();
    display.drawString(0, 0, "Iniciando ESP Interno...");
    display.display();

    // 3. Força reset físico no pino RST do RC522 para não travar (de cracha-rfid.cpp)
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW);
    delay(50);
    digitalWrite(RST_PIN, HIGH);
    delay(50);

    // 4. Inicializa SPI com os pinos escolhidos para o RFID
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    rfid.PCD_Init();
    delay(50);

    Serial.println("================================");
    Serial.println("        RFID RC522 INICIADO");
    Serial.println("================================");
    Serial.println("Aproxime um cartao ou tag...");

    // 5. Restaura os pinos do SPI para o módulo LoRa integrado da Heltec
    #ifdef SCK
    SPI.begin(SCK, MISO, MOSI, SS);
    #else
    SPI.begin(9, 11, 10, 8); // Pinos padrão Heltec V3
    #endif

    // 6. Inicializa o rádio LoRa Receptor (idêntico a comunicacao-lora-receptor.cpp)
    txNumber = 0;
    rssi = 0;
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Serial.println("LoRa Receptor configurado em 915 MHz.");

    // 7. Inicializa sensor DHT interno
    dht.begin();
    Serial.println("DHT interno inicializado no pino 19.");

    #if USAR_WIFI_MQTT
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    #endif

    atualizarDisplayOLED();
}

// ---------------------------------------------------------------------------
// Rotina de Leitura de RFID (com restauração de SPI para coexistência com LoRa)
// ---------------------------------------------------------------------------
void processarRFID() {
    // Comuta o barramento SPI para os pinos do RFID (6, 4, 5, 7)
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

    // Verifica se existe um novo cartao
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        // Restaura imediatamente para os pinos do LoRa
        #ifdef SCK
        SPI.begin(SCK, MISO, MOSI, SS);
        #else
        SPI.begin(9, 11, 10, 8);
        #endif
        return;
    }

    // --- CÓDIGO EXATO DE cracha-rfid.cpp ---
    Serial.print("Cartao detectado! UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] < 0x10) {
            Serial.print("0");
        }
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    MFRC522::PICC_Type tipo = rfid.PICC_GetType(rfid.uid.sak);
    Serial.print("Tipo: ");
    Serial.println(rfid.PICC_GetTypeName(tipo));

    // --- CONTROLE DE ENTRADA E SAIDA DA SALA (cracha-rfid.cpp) ---
    int pos = -1;
    for (int i = 0; i < qtdPessoas; i++) {
        if (tamanhoUidSala[i] == rfid.uid.size) {
            bool igual = true;
            for (byte b = 0; b < rfid.uid.size; b++) {
                if (sala[i][b] != rfid.uid.uidByte[b]) {
                    igual = false;
                    break;
                }
            }
            if (igual) {
                pos = i;
                break;
            }
        }
    }

    if (pos == -1) {
        if (qtdPessoas < 20) {
            tamanhoUidSala[qtdPessoas] = rfid.uid.size;
            for (byte b = 0; b < rfid.uid.size; b++) {
                sala[qtdPessoas][b] = rfid.uid.uidByte[b];
            }
            qtdPessoas++;
            Serial.println(">> Status: ENTROU na sala!");
        }
    } else {
        for (int i = pos; i < qtdPessoas - 1; i++) {
            tamanhoUidSala[i] = tamanhoUidSala[i + 1];
            for (byte b = 0; b < tamanhoUidSala[i]; b++) {
                sala[i][b] = sala[i + 1][b];
            }
        }
        qtdPessoas--;
        Serial.println("<< Status: SAIU da sala!");
    }

    Serial.print("Total na sala: ");
    Serial.println(qtdPessoas);

    for (int i = 0; i < qtdPessoas; i++) {
        Serial.print("  [Pessoa ");
        Serial.print(i + 1);
        Serial.print("] UID: ");
        for (byte b = 0; b < tamanhoUidSala[i]; b++) {
            if (sala[i][b] < 0x10) {
                Serial.print("0");
            }
            Serial.print(sala[i][b], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println("-------------------------------");

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

    // Restaura o barramento SPI para o LoRa da Heltec
    #ifdef SCK
    SPI.begin(SCK, MISO, MOSI, SS);
    #else
    SPI.begin(9, 11, 10, 8);
    #endif

    atualizarDisplayOLED();
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop()
{
    // 1. Lógica LoRa Receptor (idêntica a comunicacao-lora-receptor.cpp)
    if(lora_idle)
    {
        lora_idle = false;
        Radio.Rx(0);
    }
    Radio.IrqProcess( );

    // 2. Lógica RFID (amostragem periódica a cada 100ms)
    if (millis() - ultimaChecagemRFID >= 100) {
        ultimaChecagemRFID = millis();
        processarRFID();
    }

    // 3. Print periódico a cada 3 segundos (de cracha-rfid.cpp)
    if (millis() - ultimoAviso > 3000) {
        ultimoAviso = millis();
        Serial.print("[Sistema rodando] Pessoas dentro da sala agora: ");
        Serial.println(qtdPessoas);
        Serial.printf("[Status Climatizacao] Temp Int: %.1f C | Temp Ext: %.1f C | Ar Ideal: %.1f C\n",
                      tempInterna, tempExterna, calcularTemperaturaIdeal(tempInterna, tempExterna, qtdPessoas));
    }

    // 4. Leitura periódica do sensor DHT interno (a cada 2,5 segundos)
    if (millis() - ultimaLeituraDHT >= 2500) {
        ultimaLeituraDHT = millis();
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        if (!isnan(event.temperature)) {
            tempInterna = event.temperature;
        }
        atualizarDisplayOLED();
    }

    // 5. Atualização visual do Display OLED a cada 1 segundo
    if (millis() - ultimoUpdateDisplay >= 1000) {
        ultimoUpdateDisplay = millis();
        atualizarDisplayOLED();
    }

    #if USAR_WIFI_MQTT
    if (WiFi.status() == WL_CONNECTED) {
        if (!mqttClient.connected()) {
            mqttClient.connect("ESP32_Interno_Sala");
        } else {
            mqttClient.loop();
            // Publica dados da sala no broker MQTT
            char buf[64];
            sprintf(buf, "%d", qtdPessoas);
            mqttClient.publish("sala/pessoas", buf);
            sprintf(buf, "%.1f", calcularTemperaturaIdeal(tempInterna, tempExterna, qtdPessoas));
            mqttClient.publish("sala/temperatura_indicada", buf);
            sprintf(buf, "%.1f", tempInterna);
            mqttClient.publish("sala/temperatura_interna", buf);
            sprintf(buf, "%.1f", tempExterna);
            mqttClient.publish("sala/temperatura_externa", buf);
        }
    }
    #endif
}

// ---------------------------------------------------------------------------
// Callback de Recepção LoRa (idêntico a comunicacao-lora-receptor.cpp)
// ---------------------------------------------------------------------------
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rxSize = size;
    memcpy(rxpacket, payload, size );
    rxpacket[size] = '\0';
    Radio.Sleep( );
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);

    // Converte a string de temperatura externa recebida para float
    float valorRecebido = atof(rxpacket);
    if (valorRecebido != 0.0 || strcmp(rxpacket, "0.0") == 0 || strcmp(rxpacket, "0.00") == 0) {
        tempExterna = valorRecebido;
        tempExternaRecebida = true;
        Serial.printf("[LoRa] Temperatura externa atualizada: %.2f *C\n", tempExterna);
    }

    lora_idle = true;
    atualizarDisplayOLED();
}
