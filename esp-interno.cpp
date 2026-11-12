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
 * 5. Comunicação MQTT via biblioteca PubSubClient (knolleary/pubsubclient):
 *    - Conexão e reconexão WiFi e MQTT de forma 100% não bloqueante (sem travar RFID/LoRa);
 *    - Publicação em tópicos individuais:
 *      * sala/pessoas (número de pessoas na sala)
 *      * sala/temperatura_indicada (temperatura recomendada para o ar-condicionado)
 *      * sala/temperatura_interna (temperatura medida na sala)
 *      * sala/temperatura_externa (temperatura externa recebida via LoRa)
 *    - Publicação consolidada em formato JSON no tópico:
 *      * sala/telemetria (ideal para integração direta com dashboards Node-RED)
 * 6. Exibição consolidada no display OLED onboard da Heltec (SSD1306) com status WiFi/MQTT.
 * 7. Logs e relatórios periódicos no Monitor Serial (115200 bps).
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <vector>
#include <WiFi.h>
#include <PubSubClient.h>
#include "LoRaWan_APP.h"
#include <MFRC522.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "HT_SSD1306Wire.h"

// ---------------------------------------------------------------------------
// Configurações de Rede WiFi e Broker MQTT (PubSubClient)
// ---------------------------------------------------------------------------
// Substitua pelas credenciais da sua rede WiFi
const char* WIFI_SSID       = "SEU_WIFI_SSID";
const char* WIFI_PASSWORD   = "SUA_WIFI_SENHA";

// Configurações do Broker MQTT (ex.: HiveMQ público, Mosquitto local ou Node-RED)
const char* MQTT_BROKER     = "broker.hivemq.com"; 
const int   MQTT_PORT       = 1883;
const char* MQTT_USER       = "";                  // Usuário (opcional, deixe "" se não usar)
const char* MQTT_PASSWORD   = "";                  // Senha (opcional, deixe "" se não usar)
const char* MQTT_CLIENT_PREFIX = "ESP32_Heltec_Sala";

// Tópicos MQTT para publicação das variáveis solicitadas
#define TOPICO_PESSOAS           "sala/pessoas"
#define TOPICO_TEMP_INDICADA     "sala/temperatura_indicada"
#define TOPICO_TEMP_INTERNA      "sala/temperatura_interna"
#define TOPICO_TEMP_EXTERNA      "sala/temperatura_externa"
#define TOPICO_TELEMETRIA_JSON   "sala/telemetria"

// Instâncias de rede WiFi e cliente MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Controle de tempo para reconexões e publicações MQTT não bloqueantes
unsigned long ultimaTentativaWiFi = 0;
unsigned long ultimaTentativaMQTT = 0;
unsigned long ultimaPublicacaoMQTT = 0;
const unsigned long INTERVALO_PUBLICACAO_MQTT = 5000; // Publica a cada 5 segundos se conectado

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

void inicializarWiFi();
void gerenciarConexaoWiFiEMqtt();
void publicarDadosMQTT();

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

  // Recalcula, atualiza display e publica dados no broker MQTT
  atualizarDisplay();
  publicarDadosMQTT();
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

  // Atualiza imediatamente o display e publica no MQTT com a nova contagem de pessoas
  atualizarDisplay();
  publicarDadosMQTT();
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
// Gerenciamento Não Bloqueante de WiFi e MQTT (PubSubClient)
// ---------------------------------------------------------------------------
void inicializarWiFi() {
  Serial.print("[WiFi] Conectando a rede: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Aguarda até 3 segundos pela conexão inicial para não bloquear setup caso sem sinal
  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - inicio < 3000)) {
    delay(200);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Conectado! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Conexao inicial em andamento em segundo plano...");
  }

  // Configura servidor do broker MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

void gerenciarConexaoWiFiEMqtt() {
  // 1. Manutenção do WiFi (não bloqueante)
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long agora = millis();
    if (agora - ultimaTentativaWiFi > 10000) {
      ultimaTentativaWiFi = agora;
      Serial.println("[WiFi] Tentando reconectar ao WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
    }
    return; // Se não tem WiFi, não tenta MQTT ainda
  }

  // 2. Manutenção do MQTT (não bloqueante)
  if (!mqttClient.connected()) {
    unsigned long agora = millis();
    if (agora - ultimaTentativaMQTT > 5000) {
      ultimaTentativaMQTT = agora;
      Serial.print("[MQTT] Tentando conectar ao broker MQTT: ");
      Serial.print(MQTT_BROKER);
      Serial.print("...");

      // Cria um Client ID único com base no MAC para evitar conflito de conexões
      String clientId = String(MQTT_CLIENT_PREFIX) + "_" + String(random(0xffff), HEX);

      bool conectado = false;
      if (strlen(MQTT_USER) > 0) {
        conectado = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD);
      } else {
        conectado = mqttClient.connect(clientId.c_str());
      }

      if (conectado) {
        Serial.println(" Conectado com sucesso!");
        // Publica imediatamente após reconexão para atualizar o broker/Node-RED
        publicarDadosMQTT();
      } else {
        Serial.printf(" Falha (rc=%d). Nova tentativa em 5 segundos.\n", mqttClient.state());
      }
    }
  } else {
    // Processa pacotes de entrada e mantém o keep-alive do MQTT ativo
    mqttClient.loop();
  }
}

// ---------------------------------------------------------------------------
// Publicação dos Dados no Broker MQTT (PubSubClient)
// ---------------------------------------------------------------------------
void publicarDadosMQTT() {
  if (!mqttClient.connected()) {
    return; // Não publica se não estiver conectado ao broker
  }

  int pessoas = (int)salaOcupantes.size();
  float tempIndicada = calcularTemperaturaIdeal(tempInterna, tempExterna, pessoas);

  // 1. Publicação do número de pessoas na sala
  char bufPessoas[12];
  snprintf(bufPessoas, sizeof(bufPessoas), "%d", pessoas);
  mqttClient.publish(TOPICO_PESSOAS, bufPessoas, true);

  // 2. Publicação da temperatura indicada para o ar-condicionado
  char bufTempIndicada[16];
  snprintf(bufTempIndicada, sizeof(bufTempIndicada), "%.1f", tempIndicada);
  mqttClient.publish(TOPICO_TEMP_INDICADA, bufTempIndicada, true);

  // 3. Publicação da temperatura interna
  char bufTempInterna[16];
  if (tempInterna > -50.0) {
    snprintf(bufTempInterna, sizeof(bufTempInterna), "%.1f", tempInterna);
  } else {
    snprintf(bufTempInterna, sizeof(bufTempInterna), "null");
  }
  mqttClient.publish(TOPICO_TEMP_INTERNA, bufTempInterna, true);

  // 4. Publicação da temperatura externa
  char bufTempExterna[16];
  if (tempExternaRecebida) {
    snprintf(bufTempExterna, sizeof(bufTempExterna), "%.1f", tempExterna);
  } else {
    snprintf(bufTempExterna, sizeof(bufTempExterna), "null");
  }
  mqttClient.publish(TOPICO_TEMP_EXTERNA, bufTempExterna, true);

  // 5. Publicação consolidada em JSON (ideal para dashboards no Node-RED)
  char payloadJson[192];
  snprintf(payloadJson, sizeof(payloadJson),
           "{\"pessoas\":%d,\"temperatura_indicada\":%.1f,\"temperatura_interna\":%.1f,\"temperatura_externa\":%.1f}",
           pessoas,
           tempIndicada,
           (tempInterna > -50.0 ? tempInterna : 0.0),
           (tempExternaRecebida ? tempExterna : 0.0));
  mqttClient.publish(TOPICO_TELEMETRIA_JSON, payloadJson, true);

  Serial.println("\n[MQTT] Dados publicados nos topicos:");
  Serial.printf("  - %s: %s\n", TOPICO_PESSOAS, bufPessoas);
  Serial.printf("  - %s: %s *C\n", TOPICO_TEMP_INDICADA, bufTempIndicada);
  Serial.printf("  - %s: %s *C\n", TOPICO_TEMP_INTERNA, bufTempInterna);
  Serial.printf("  - %s: %s *C\n", TOPICO_TEMP_EXTERNA, bufTempExterna);
  Serial.printf("  - %s: %s\n", TOPICO_TELEMETRIA_JSON, payloadJson);
  Serial.println("----------------------------------------");
}

// ---------------------------------------------------------------------------
// Atualização Visual do Display OLED Heltec
// ---------------------------------------------------------------------------
void atualizarDisplay() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // Linha 0: Cabeçalho com indicador de conectividade MQTT
  display.setFont(ArialMT_Plain_10);
  if (mqttClient.connected()) {
    display.drawString(0, 0, "SALA IoT  [MQTT: OK]");
  } else if (WiFi.status() == WL_CONNECTED) {
    display.drawString(0, 0, "SALA IoT  [WiFi: OK]");
  } else {
    display.drawString(0, 0, "=== MONITOR DE SALA ===");
  }

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
  Serial.printf("[STATUS] WiFi: %s | MQTT: %s\n", 
                (WiFi.status() == WL_CONNECTED ? "CONECTADO" : "DESCONECTADO"),
                (mqttClient.connected() ? "CONECTADO" : "DESCONECTADO"));
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
  display.drawString(0, 16, "RFID: SPI (3,4,5,6,7)");
  display.drawString(0, 28, "DHT Interno: Pino 19");
  display.drawString(0, 40, "LoRa: 915 MHz | MQTT");
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

  // Inicialização não bloqueante da conexão WiFi e MQTT
  inicializarWiFi();

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

  // 6. Gerenciamento de conexões de rede WiFi e broker MQTT (PubSubClient)
  gerenciarConexaoWiFiEMqtt();

  // 7. Publicação periódica no broker MQTT
  if (millis() - ultimaPublicacaoMQTT >= INTERVALO_PUBLICACAO_MQTT) {
    ultimaPublicacaoMQTT = millis();
    publicarDadosMQTT();
  }

  // 8. Impressão periódica do status consolidado no Monitor Serial
  imprimirStatusPeriodico();
}
