/**
 * ============================================================================
 * PROJETO FINAL IoT - MONITORAMENTO INTELIGENTE DE SALAS DE ESCRITÓRIO
 * ============================================================================
 * Arquivo: esp-externo.cpp
 * Dispositivo: Heltec WiFi LoRa 32 (ESP32 LoRa - Nó Externo Transmissor)
 * 
 * Funcionalidades:
 * 1. Leitura periódica de temperatura e umidade com sensor DHT (DHT11/DHT22).
 * 2. Transmissão dos dados meteorológicos externos via rádio LoRa (915 MHz)
 *    para o ESP32 interno da sala.
 * 3. Exibição das medições e status de envio no display OLED onboard (SSD1306).
 * 4. Monitoramento contínuo via Serial (115200 bps).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "LoRaWan_APP.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "HT_SSD1306Wire.h"

// ---------------------------------------------------------------------------
// Configurações do Rádio LoRa (915 MHz - Padrão Brasil / Heltec)
// ---------------------------------------------------------------------------
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Preâmbulo padrão
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define BUFFER_SIZE                                 64
char txpacket[BUFFER_SIZE];
bool lora_idle = true;
static RadioEvents_t RadioEvents;

// ---------------------------------------------------------------------------
// Configurações do Sensor de Temperatura e Umidade DHT
// ---------------------------------------------------------------------------
#define DHTPIN                                      19        // Pino conectado ao Data do sensor DHT
#define DHTTYPE                                     DHT11     // Defina DHT11 ou DHT22 conforme o sensor utilizado
DHT_Unified dht(DHTPIN, DHTTYPE);

// ---------------------------------------------------------------------------
// Configurações do Display OLED Onboard (SSD1306 - Heltec)
// ---------------------------------------------------------------------------
#ifdef SDA_OLED
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#else
// Fallback para pinos comuns Heltec V3 (SDA=17, SCL=18, RST=21)
static SSD1306Wire display(0x3c, 500000, 17, 18, GEOMETRY_128_64, 21);
#endif

// ---------------------------------------------------------------------------
// Variáveis Globais de Controle e Temporização
// ---------------------------------------------------------------------------
unsigned long ultimoEnvio = 0;
const unsigned long INTERVALO_ENVIO_MS = 5000; // Intervalo de envio via LoRa (5 segundos)

uint32_t totalPacotesEnviados = 0;
float ultimaTemperatura = 0.0;
float ultimaUmidade = 0.0;
bool leituraValida = false;

// ---------------------------------------------------------------------------
// Protótipos de Funções
// ---------------------------------------------------------------------------
void OnTxDone(void);
void OnTxTimeout(void);
void atualizarDisplayExterno(float temp, float umid, bool sucesso, uint32_t total);

// ---------------------------------------------------------------------------
// Callbacks do Rádio LoRa
// ---------------------------------------------------------------------------
void OnTxDone(void) {
  Serial.println("[LoRa TX] Pacote transmitido com sucesso (TX Done)!");
  lora_idle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("[LoRa TX] Timeout na transmissão (TX Timeout)!");
  lora_idle = true;
}

// ---------------------------------------------------------------------------
// Atualização Visual no Display OLED
// ---------------------------------------------------------------------------
void atualizarDisplayExterno(float temp, float umid, bool sucesso, uint32_t total) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "=== ESP LORA EXTERNO ===");

  char buffer[32];
  if (sucesso) {
    sprintf(buffer, "Temp Ext: %.1f C", temp);
    display.drawString(0, 16, buffer);

    sprintf(buffer, "Umidade:  %.1f %%", umid);
    display.drawString(0, 28, buffer);
  } else {
    display.drawString(0, 20, "Erro lendo sensor DHT!");
  }

  sprintf(buffer, "Pacotes TX: %u", total);
  display.drawString(0, 42, buffer);

  display.drawString(0, 54, lora_idle ? "Status: LoRa Pronto" : "Status: Enviando...");
  display.display();
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=========================================");
  Serial.println("     ESP32 LORA - NÓ EXTERNO INICIADO    ");
  Serial.println("=========================================");

  // Habilita alimentação externa dos periféricos/OLED na placa Heltec
  #ifdef Vext
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); // LOW liga o trilho de alimentação Vext
  delay(100);
  #endif

  // Inicializa o Display OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "Iniciando Nó Externo...");
  display.drawString(0, 16, "Sensor: DHT no Pino 19");
  display.drawString(0, 28, "LoRa: 915 MHz");
  display.display();

  // Inicializa o sensor DHT
  dht.begin();
  Serial.println("[DHT] Sensor de temperatura e umidade inicializado no pino 19.");

  // Inicializa a MCU da Heltec e o Rádio LoRa
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Serial.println("[LoRa] Rádio LoRa configurado com sucesso a 915 MHz.");
  atualizarDisplayExterno(0.0, 0.0, false, 0);
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
  // Transmissão periódica não bloqueante
  if (lora_idle && (millis() - ultimoEnvio >= INTERVALO_ENVIO_MS)) {
    ultimoEnvio = millis();

    sensors_event_t eventTemp, eventUmid;
    dht.temperature().getEvent(&eventTemp);
    dht.humidity().getEvent(&eventUmid);

    if (isnan(eventTemp.temperature) || isnan(eventUmid.relative_humidity)) {
      Serial.println("[DHT] Erro ao ler dados do sensor DHT!");
      leituraValida = false;
      atualizarDisplayExterno(0.0, 0.0, false, totalPacotesEnviados);
    } else {
      ultimaTemperatura = eventTemp.temperature;
      ultimaUmidade = eventUmid.relative_humidity;
      leituraValida = true;
      totalPacotesEnviados++;

      // Formata a carga útil para transmissão via LoRa
      // Formato chave-valor: TEMP:XX.XX;UMID:YY.YY
      snprintf(txpacket, BUFFER_SIZE, "TEMP:%.2f;UMID:%.2f", ultimaTemperatura, ultimaUmidade);

      Serial.printf("[DHT] Leitura externa -> Temp: %.2f *C | Umid: %.2f %%\n", ultimaTemperatura, ultimaUmidade);
      Serial.printf("[LoRa TX] Enviando pacote #%u: \"%s\" (tam: %d bytes)\n", totalPacotesEnviados, txpacket, strlen(txpacket));

      Radio.Send((uint8_t *)txpacket, strlen(txpacket));
      lora_idle = false;

      atualizarDisplayExterno(ultimaTemperatura, ultimaUmidade, true, totalPacotesEnviados);
    }
  }

  // Processa as interrupções de hardware do rádio LoRa
  Radio.IrqProcess();
}
