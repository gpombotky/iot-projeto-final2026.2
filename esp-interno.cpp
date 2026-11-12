/**
 * ============================================================================
 * PROJETO FINAL IoT - MONITORAMENTO INTELIGENTE DE SALAS DE ESCRITÓRIO
 * ============================================================================
 * Arquivo: esp-interno.cpp
 * Dispositivo: Heltec WiFi LoRa 32 (ESP32 LoRa - Nó Interno Receptor & Climatização)
 * 
 * Funcionalidades:
 * 1. Controle de acesso e presença via RFID (RC522):
 *    - Leitura das tags/crachás dos ocupantes via SPI.
 *    - Gerenciamento de entrada e saída por meio de um vetor dinâmico em memória.
 *    - Contagem precisa de quantas pessoas estão dentro da sala.
 * 2. Leitura da temperatura e umidade interna da sala através de sensor DHT.
 * 3. Recepção dos dados de temperatura externa enviados pelo Nó Externo via LoRa (915 MHz).
 * 4. Algoritmo inteligente de climatização:
 *    - Calcula a temperatura ideal do ar-condicionado considerando:
 *      * Quantidade de pessoas na sala (carga térmica humana ~100W/ocupante);
 *      * Temperatura externa (carga térmica ambiental);
 *      * Temperatura interna atual;
 *      * Parâmetros de conforto térmico (NR-17 / ASHRAE 55).
 * 5. Exibição consolidada no display OLED onboard da Heltec (SSD1306).
 * 6. Logs e relatórios periódicos no Monitor Serial (115200 bps).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <vector>
#include "LoRaWan_APP.h"
#include <MFRC522.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "HT_SSD1306Wire.h"

// ---------------------------------------------------------------------------
// Pinos do Módulo RFID RC522 (Interface SPI)
// Mapeamento testado para Heltec ESP32 sem conflito com pinos internos de LoRa/OLED
// ---------------------------------------------------------------------------
#define SS_PIN                                      7
#define RST_PIN                                     3
#define SCK_PIN                                     6
#define MISO_PIN                                    4
#define MOSI_PIN                                    5

MFRC522 rfid(SS_PIN, RST_PIN);

// ---------------------------------------------------------------------------
// Vetor Dinâmico de Ocupação da Sala (Controle de Presença por Crachá RFID)
// ---------------------------------------------------------------------------
std::vector<String> salaOcupantes;

// ---------------------------------------------------------------------------
// Configurações do Sensor de Temperatura e Umidade DHT Interno
// ---------------------------------------------------------------------------
#define DHTPIN                                      19        // Pino do sensor DHT interno
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
// Configurações do Rádio LoRa (Receptor - 915 MHz)
// ---------------------------------------------------------------------------
#define RF_FREQUENCY                                915000000 // Hz
#define LORA_BANDWIDTH                              0         // [0: 125 kHz]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5]
#define LORA_PREAMBLE_LENGTH                        8         // Preâmbulo
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define BUFFER_SIZE                                 64
char rxpacket[BUFFER_SIZE];
int16_t rxRssi = 0;
int16_t rxSize = 0;
bool lora_idle = true;
volatile bool novoPacoteLoRa = false;
static RadioEvents_t RadioEvents;

// ---------------------------------------------------------------------------
// Variáveis de Estado Térmico e de Controle
// ---------------------------------------------------------------------------
float tempInterna = -999.0;
float umidInterna = -999.0;
float tempExterna = -999.0;
float umidExterna = -999.0;
bool tempExternaRecebida = false;
unsigned long ultimaRecepcaoLoRa = 0;

// ---------------------------------------------------------------------------
// Protótipos de Funções
// ---------------------------------------------------------------------------
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void processarPacoteLoRa();
void processarRFID();
void lerSensorInterno();
float calcularTemperaturaIdeal(float tInterna, float tExterna, int pessoas);
void atualizarDisplay();
void imprimirStatusPeriodico();

// ---------------------------------------------------------------------------
// Callback de Recepção LoRa
// ---------------------------------------------------------------------------
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  rxRssi = rssi;
  rxSize = size;
  uint16_t len = (size < BUFFER_SIZE) ? size : (BUFFER_SIZE - 1);
  memcpy(rxpacket, payload, len);
  rxpacket[len] = '\0';

  Radio.Sleep();
  novoPacoteLoRa = true;
  lora_idle = true;
}

// ---------------------------------------------------------------------------
// Parser de Dados Recebidos via LoRa
// ---------------------------------------------------------------------------
void processarPacoteLoRa() {
  if (!novoPacoteLoRa) return;
  novoPacoteLoRa = false;

  Serial.printf("\n[LoRa RX] Pacote recebido: \"%s\" (RSSI: %d dBm, Tamanho: %d bytes)\n", rxpacket, rxRssi, rxSize);

  // Formato esperado padrão: "TEMP:XX.XX;UMID:YY.YY"
  char *pTemp = strstr(rxpacket, "TEMP:");
  if (pTemp != NULL) {
    tempExterna = atof(pTemp + 5);
    tempExternaRecebida = true;
    ultimaRecepcaoLoRa = millis();

    char *pUmid = strstr(rxpacket, "UMID:");
    if (pUmid != NULL) {
      umidExterna = atof(pUmid + 5);
    }
    Serial.printf("[LoRa RX] -> Temp Externa atualizada: %.2f *C | Umid: %.2f %%\n", tempExterna, umidExterna);
  } else {
    // Fallback: se o pacote contiver diretamente o valor numérico (ex: "27.50")
    float val = atof(rxpacket);
    if (val != 0.0 || strcmp(rxpacket, "0") == 0 || strcmp(rxpacket, "0.0") == 0) {
      tempExterna = val;
      tempExternaRecebida = true;
      ultimaRecepcaoLoRa = millis();
      Serial.printf("[LoRa RX] -> Temp Externa (valor direto): %.2f *C\n", tempExterna);
    }
  }

  // Recalcula e atualiza display com o novo dado externo
  atualizarDisplay();
}

// ---------------------------------------------------------------------------
// Controle de Ocupação por RFID com Vetor Dinâmico
// ---------------------------------------------------------------------------
void processarRFID() {
  // Verifica presença de novo cartão
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  // Converte UID lido em String Hexadecimal formatada
  String uidStr = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      uidStr += "0";
    }
    uidStr += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) {
      uidStr += " ";
    }
  }
  uidStr.toUpperCase();

  // Debounce para evitar múltiplas leituras se o cartão permanecer encostado
  static String ultimoUidLido = "";
  static unsigned long tempoUltimaLeitura = 0;
  if (uidStr == ultimoUidLido && (millis() - tempoUltimaLeitura < 1500)) {
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }
  ultimoUidLido = uidStr;
  tempoUltimaLeitura = millis();

  // Busca se o UID já está presente no vetor dinâmico
  int indexEncontrado = -1;
  for (size_t i = 0; i < salaOcupantes.size(); i++) {
    if (salaOcupantes[i] == uidStr) {
      indexEncontrado = (int)i;
      break;
    }
  }

  Serial.println("\n----------------------------------------");
  Serial.print("Cartao detectado! UID: ");
  Serial.println(uidStr);

  // Se NÃO estava na sala: adiciona ao vetor dinâmico (ENTROU)
  if (indexEncontrado == -1) {
    salaOcupantes.push_back(uidStr);
    Serial.println(">> Status: ENTROU na sala!");
  } 
  // Se JÁ estava na sala: remove do vetor dinâmico (SAIU)
  else {
    salaOcupantes.erase(salaOcupantes.begin() + indexEncontrado);
    Serial.println("<< Status: SAIU da sala!");
  }

  // Exibe a lista dinâmica atual de ocupantes da sala
  Serial.print("Total de pessoas na sala: ");
  Serial.println(salaOcupantes.size());
  for (size_t i = 0; i < salaOcupantes.size(); i++) {
    Serial.printf("  [Pessoa %d] UID: %s\n", (int)(i + 1), salaOcupantes[i].c_str());
  }

  float setpointIdeal = calcularTemperaturaIdeal(tempInterna, tempExterna, salaOcupantes.size());
  Serial.printf(">> Temperatura ideal recalculada: %.1f *C\n", setpointIdeal);
  Serial.println("----------------------------------------");

  // Finaliza sessão com a tag RFID atual
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // Atualiza imediatamente o display com a nova contagem de pessoas
  atualizarDisplay();
}

// ---------------------------------------------------------------------------
// Leitura do Sensor DHT Interno
// ---------------------------------------------------------------------------
void lerSensorInterno() {
  static unsigned long ultimaLeituraDHT = 0;
  // O sensor DHT necessita de intervalo mínimo de ~2s entre amostras
  if (millis() - ultimaLeituraDHT < 2500) {
    return;
  }
  ultimaLeituraDHT = millis();

  sensors_event_t eventTemp, eventUmid;
  dht.temperature().getEvent(&eventTemp);
  dht.humidity().getEvent(&eventUmid);

  if (!isnan(eventTemp.temperature)) {
    tempInterna = eventTemp.temperature;
  } else {
    Serial.println("[DHT Interno] Falha na leitura da temperatura!");
  }

  if (!isnan(eventUmid.relative_humidity)) {
    umidInterna = eventUmid.relative_humidity;
  }

  atualizarDisplay();
}

// ---------------------------------------------------------------------------
// Algoritmo de Cálculo da Temperatura Ideal do Ar-Condicionado
// ---------------------------------------------------------------------------
/**
 * Calcula a temperatura ideal para o ar-condicionado baseando-se em:
 * 1. Ocupação (cada pessoa gera ~100W de calor sensível/latente).
 * 2. Carga térmica externa (temperatura externa recebida via LoRa).
 * 3. Temperatura interna medida em tempo real.
 * 4. Padrões de ergonomia e conforto térmico (NR-17 / ASHRAE 55).
 */
float calcularTemperaturaIdeal(float tInterna, float tExterna, int pessoas) {
  // Caso 1: Sala vazia (0 pessoas) -> Modo econômico / stand-by
  if (pessoas == 0) {
    return 25.0; // Economia de energia em salas sem ocupação
  }

  // Caso 2: Temperatura base recomendada para ambientes de trabalho de escritório (NR-17: 20°C a 23°C)
  float tIdeal = 23.0;

  // Ajuste por pessoas:
  // Para cada pessoa adicional além da primeira, reduz 0.5°C para compensar o calor metabólico
  tIdeal -= (pessoas - 1) * 0.5;

  // Ajuste pela temperatura externa (carga térmica transferida pelas janelas/paredes):
  if (tempExternaRecebida) {
    if (tExterna >= 32.0) {
      tIdeal -= 1.0; // Dia muito quente: demanda maior potência frigorífica
    } else if (tExterna >= 28.0) {
      tIdeal -= 0.5; // Dia quente moderado
    } else if (tExterna < 20.0) {
      tIdeal += 1.0; // Clima ameno/frio externo: ameniza a refrigeração interna
    }
  }

  // Ajuste fino por temperatura interna atual:
  // Se a sala já estiver abafada (> 26.5°C), diminui meio grau adicional para acelerar o resfriamento
  if (tInterna > -50.0 && tInterna >= 26.5) {
    tIdeal -= 0.5;
  }

  // Limites de segurança e conforto operacional (evita choque térmico ou congelamento)
  if (tIdeal < 18.0) tIdeal = 18.0;
  if (tIdeal > 26.0) tIdeal = 26.0;

  return tIdeal;
}

// ---------------------------------------------------------------------------
// Atualização Visual do Display OLED Heltec
// ---------------------------------------------------------------------------
void atualizarDisplay() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // Linha 0: Cabeçalho
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "=== MONITOR DE SALA ===");

  char buffer[32];

  // Linha 1: Contagem de pessoas via vetor dinâmico
  sprintf(buffer, "Pessoas na sala: %d", (int)salaOcupantes.size());
  display.drawString(0, 13, buffer);

  // Linha 2: Temperatura Interna medida
  if (tempInterna > -50.0) {
    sprintf(buffer, "Temp Int: %.1f C", tempInterna);
  } else {
    sprintf(buffer, "Temp Int: Lendo...");
  }
  display.drawString(0, 25, buffer);

  // Linha 3: Temperatura Externa recebida via LoRa
  if (tempExternaRecebida) {
    sprintf(buffer, "Temp Ext: %.1f C", tempExterna);
  } else {
    sprintf(buffer, "Temp Ext: Aguardando...");
  }
  display.drawString(0, 37, buffer);

  // Linha 4: Temperatura Ideal Calculada para o Ar-Condicionado
  float setpoint = calcularTemperaturaIdeal(tempInterna, tempExterna, salaOcupantes.size());
  if (salaOcupantes.empty()) {
    sprintf(buffer, "Ar Ideal: %.1f C (Eco)", setpoint);
  } else {
    sprintf(buffer, "Ar Ideal: %.1f C", setpoint);
  }
  display.drawString(0, 50, buffer);

  display.display();
}

// ---------------------------------------------------------------------------
// Relatório Periódico no Monitor Serial
// ---------------------------------------------------------------------------
void imprimirStatusPeriodico() {
  static unsigned long ultimoAviso = 0;
  if (millis() - ultimoAviso < 4000) {
    return;
  }
  ultimoAviso = millis();

  float setpoint = calcularTemperaturaIdeal(tempInterna, tempExterna, salaOcupantes.size());

  Serial.println("=========================================");
  Serial.printf("[STATUS] Pessoas na sala: %d\n", (int)salaOcupantes.size());
  if (tempInterna > -50.0) {
    Serial.printf("[STATUS] Temp Interna:    %.1f *C (Umid: %.1f %%)\n", tempInterna, umidInterna);
  } else {
    Serial.println("[STATUS] Temp Interna:    Lendo sensor...");
  }

  if (tempExternaRecebida) {
    Serial.printf("[STATUS] Temp Externa:    %.1f *C (LoRa RSSI: %d dBm)\n", tempExterna, rxRssi);
  } else {
    Serial.println("[STATUS] Temp Externa:    Aguardando transmissao LoRa...");
  }

  Serial.printf("[STATUS] >> AR IDEAL:     %.1f *C <<\n", setpoint);
  Serial.println("=========================================");
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("=========================================");
  Serial.println("     ESP32 LORA - NÓ INTERNO INICIADO    ");
  Serial.println("=========================================");

  // Habilita trilho de energia Vext para acionar display OLED na placa Heltec
  #ifdef Vext
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  #endif

  // Inicialização do Display OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "Iniciando Nó Interno...");
  display.drawString(0, 16, "RFID RC522: SPI (3,4,5,6,7)");
  display.drawString(0, 28, "DHT Interno: Pino 19");
  display.drawString(0, 40, "LoRa RX: 915 MHz");
  display.display();

  // Reset físico do RC522 para garantir inicialização limpa
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(50);
  digitalWrite(RST_PIN, HIGH);
  delay(50);

  // Inicializa SPI dedicado para o RC522 com pinagem compatível com Heltec ESP32
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  rfid.PCD_Init();
  delay(50);
  Serial.println("[RFID] Leitor RC522 inicializado.");

  // Inicializa sensor DHT interno
  dht.begin();
  Serial.println("[DHT] Sensor interno inicializado no pino 19.");

  // Inicializa MCU e rádio LoRa em modo recepção contínua
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.println("[LoRa] Receptor LoRa configurado a 915 MHz.");

  atualizarDisplay();
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
  // 1. Processamento de interrupções de hardware do rádio LoRa
  Radio.IrqProcess();

  // 2. Coloca o rádio no estado de escuta contínua (RX) quando estiver ocioso
  if (lora_idle) {
    lora_idle = false;
    Radio.Rx(0);
  }

  // 3. Processamento de novo pacote LoRa recebido do ambiente externo
  processarPacoteLoRa();

  // 4. Verificação de cartões/crachás RFID na porta da sala
  processarRFID();

  // 5. Amostragem periódica da temperatura e umidade interna da sala
  lerSensorInterno();

  // 6. Impressão periódica do status consolidado no Monitor Serial
  imprimirStatusPeriodico();
}
