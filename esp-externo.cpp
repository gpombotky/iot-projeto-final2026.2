/*
 * ============================================================================
 * PROJETO FINAL IoT - MONITORAMENTO INTELIGENTE DE SALAS
 * ============================================================================
 * Arquivo: esp-externo.cpp
 * Dispositivo: Heltec WiFi LoRa 32 (Nó Externo Transmissor)
 * 
 * Baseado ESTRITAMENTE em:
 * - modularizacao/comunicacao-lora-transmissor.cpp
 * - modularizacao/sensor-temp-umidade.cpp
 * ============================================================================
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// ---------------------------------------------------------------------------
// Configurações LoRa (idênticas a comunicacao-lora-transmissor.cpp)
// ---------------------------------------------------------------------------
#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5         // dBm

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

bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

// ---------------------------------------------------------------------------
// Configurações do Sensor DHT (idênticas a sensor-temp-umidade.cpp)
// ---------------------------------------------------------------------------
#define DHTTYPE    DHT11                           // Sensor DHT11 (ou DHT22)
#define DHTPIN     19                              // Pino Data conectado no GPIO 19
DHT_Unified dht(DHTPIN, DHTTYPE);

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Inicialização da MCU da Heltec (primeira chamada obrigatória)
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    // Inicializa o sensor DHT
    dht.begin();
    Serial.println("========================================");
    Serial.println("   HELTEC LORA - TRANSMISSOR EXTERNO    ");
    Serial.println("========================================");
    Serial.println("Sensor DHT11 inicializado no pino 19.");

    // Configuração dos eventos de rádio LoRa
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Serial.println("LoRa Transmissor configurado em 915 MHz.");
}

void loop()
{
    if(lora_idle == true)
    {
        delay(1000);

        // Leitura da Temperatura pelo sensor DHT
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        float temp = event.temperature;

        // Se houver falha de leitura ou sensor desconectado, envia 0.00 para NUNCA travar a transmissão LoRa
        if (isnan(temp)) {
            Serial.println("[DHT] Falha na leitura da temperatura! (Verifique conexao no pino 19)");
            temp = 0.0;
        } else {
            Serial.print("[DHT] Temperatura externa lida: ");
            Serial.print(temp);
            Serial.println(" *C");
        }

        // Formata o pacote com a temperatura para envio
        sprintf(txpacket, "%.2f", temp);
   
        Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));

        Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); // Envia o pacote LoRa
        lora_idle = false;
    }
    Radio.IrqProcess( );
}

void OnTxDone( void )
{
    Serial.println("TX done......");
    lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}
