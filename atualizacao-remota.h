/*
 * ============================================================================
 * ATUALIZAÇÃO REMOTA DE FIRMWARE (OTA) - nó interno
 * ============================================================================
 * Fluxo:
 *  1. O Node-RED publica "atualizar" em sala/ota/comando.
 *  2. A placa lê versao.txt da última Release do GitHub (HTTPS validado).
 *  3. Se a versão for maior, baixa firmware-interno.bin e grava na outra
 *     partição OTA; o firmware atual fica intacto até a gravação terminar.
 *  4. Reinicia. Se o novo firmware não ficar 30 s conectado ao MQTT em até
 *     3 minutos, a placa volta sozinha à versão anterior (rollback).
 *
 * A URL é fixa no código: como o broker MQTT é público, o comando nunca pode
 * escolher de onde baixar o firmware.
 * ============================================================================
 */
#pragma once

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <esp_ota_ops.h>

// Aumente a cada Release (deve ser igual à tag, sem o "v")
#define FIRMWARE_VERSAO       "1.0.1"
#define OTA_URL_BASE          "https://github.com/gpombotky/iot-projeto-final2026.2/releases"
#define OTA_ARQUIVO           "firmware-interno.bin"
#define OTA_TOPICO_COMANDO    "sala/ota/comando"
#define OTA_TOPICO_STATUS     "sala/ota/status"
#define OTA_TEMPO_ESTAVEL_MS  30000UL
#define OTA_TEMPO_LIMITE_MS   180000UL

extern PubSubClient mqttClient;
extern byte sala[20][10];
extern byte tamanhoUidSala[20];
extern int qtdPessoas;
void mostrarOTA(const char* texto);  // OLED, definida em esp-interno.cpp

// Autoridades certificadoras raiz usadas pelo GitHub
static const char OTA_CERTIFICADOS[] = R"PEM(
-----BEGIN CERTIFICATE-----
MIICjzCCAhWgAwIBAgIQXIuZxVqUxdJxVt7NiYDMJjAKBggqhkjOPQQDAzCBiDEL
MAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNl
eSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMT
JVVTRVJUcnVzdCBFQ0MgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAwMjAx
MDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNVBAgT
Ck5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVUaGUg
VVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBFQ0MgQ2VydGlm
aWNhdGlvbiBBdXRob3JpdHkwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAQarFRaqflo
I+d61SRvU8Za2EurxtW20eZzca7dnNYMYf3boIkDuAUU7FfO7l0/4iGzzvfUinng
o4N+LZfQYcTxmdwlkWOrfzCjtHDix6EznPO/LlxTsV+zfTJ/ijTjeXmjQjBAMB0G
A1UdDgQWBBQ64QmG1M8ZwpZ2dEl23OA1xmNjmjAOBgNVHQ8BAf8EBAMCAQYwDwYD
VR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAwNoADBlAjA2Z6EWCNzklwBBHU6+4WMB
zzuqQhFkoJ2UOQIReVx7Hfpkue4WQrO/isIJxOzksU0CMQDpKmFHjFJKS04YcPbW
RNZu9YO6bVi9JNlWSOrvxKJGgYhqOkbRqZtNyWHa0V1Xahg=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIF3jCCA8agAwIBAgIQAf1tMPyjylGoG7xkDjUDLTANBgkqhkiG9w0BAQwFADCB
iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl
cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV
BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAw
MjAxMDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNV
BAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVU
aGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBSU0EgQ2Vy
dGlmaWNhdGlvbiBBdXRob3JpdHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIK
AoICAQCAEmUXNg7D2wiz0KxXDXbtzSfTTK1Qg2HiqiBNCS1kCdzOiZ/MPans9s/B
3PHTsdZ7NygRK0faOca8Ohm0X6a9fZ2jY0K2dvKpOyuR+OJv0OwWIJAJPuLodMkY
tJHUYmTbf6MG8YgYapAiPLz+E/CHFHv25B+O1ORRxhFnRghRy4YUVD+8M/5+bJz/
Fp0YvVGONaanZshyZ9shZrHUm3gDwFA66Mzw3LyeTP6vBZY1H1dat//O+T23LLb2
VN3I5xI6Ta5MirdcmrS3ID3KfyI0rn47aGYBROcBTkZTmzNg95S+UzeQc0PzMsNT
79uq/nROacdrjGCT3sTHDN/hMq7MkztReJVni+49Vv4M0GkPGw/zJSZrM233bkf6
c0Plfg6lZrEpfDKEY1WJxA3Bk1QwGROs0303p+tdOmw1XNtB1xLaqUkL39iAigmT
Yo61Zs8liM2EuLE/pDkP2QKe6xJMlXzzawWpXhaDzLhn4ugTncxbgtNMs+1b/97l
c6wjOy0AvzVVdAlJ2ElYGn+SNuZRkg7zJn0cTRe8yexDJtC/QV9AqURE9JnnV4ee
UB9XVKg+/XRjL7FQZQnmWEIuQxpMtPAlR1n6BB6T1CZGSlCBst6+eLf8ZxXhyVeE
Hg9j1uliutZfVS7qXMYoCAQlObgOK6nyTJccBz8NUvXt7y+CDwIDAQABo0IwQDAd
BgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rIDZsswDgYDVR0PAQH/BAQDAgEGMA8G
A1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAFzUfA3P9wF9QZllDHPF
Up/L+M+ZBn8b2kMVn54CVVeWFPFSPCeHlCjtHzoBN6J2/FNQwISbxmtOuowhT6KO
VWKR82kV2LyI48SqC/3vqOlLVSoGIG1VeCkZ7l8wXEskEVX/JJpuXior7gtNn3/3
ATiUFJVDBwn7YKnuHKsSjKCaXqeYalltiz8I+8jRRa8YFWSQEg9zKC7F4iRO/Fjs
8PRF/iKz6y+O0tlFYQXBl2+odnKPi4w2r78NBc5xjeambx9spnFixdjQg3IM8WcR
iQycE0xyNN+81XHfqnHd4blsjDwSXWXavVcStkNr/+XeTWYRUc+ZruwXtuhxkYze
Sf7dNXGiFSeUHM9h4ya7b6NnJSFd5t0dCy5oGzuCr+yDZ4XUmFF0sbmZgIn/f3gZ
XHlKYC6SQK5MNyosycdiyA5d9zZbyuAlJQG03RoHnHcAP9Dc1ew91Pq7P8yF1m9/
qS3fuQL39ZeatTXaw2ewh0qpKJ4jjv9cJ2vhsE/zB+4ALtRZh8tSQZXq9EfX7mRB
VXyNWQKV3WKdwrnuWih0hKWbt5DHDAff9Yk2dDLWKMGwsAvgnEzDHNb842m1R0aB
L6KCq9NjRHDEjf8tM7qtj3u1cIiuPhnPQCjY/MiQu12ZIvVS5ljFH4gxQ+6IHdfG
jjxDah2nGN59PRbxYvnKkKj9
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)PEM";

// Impede o core Arduino de aprovar o novo firmware automaticamente no boot
extern "C" bool verifyRollbackLater() { return true; }

static volatile bool otaSolicitada = false;
static bool otaPendente = false;         // firmware novo ainda em teste
static unsigned long otaMqttDesde = 0;   // início da conexão MQTT contínua

void otaStatus(const char* texto) {
    Serial.printf("[OTA] %s\n", texto);
    mostrarOTA(texto);
    if (mqttClient.connected()) mqttClient.publish(OTA_TOPICO_STATUS, texto, true);
}

// --- Ocupação da sala: preservada durante o reinício da atualização ---------
void otaSalvarOcupacao() {
    Preferences p;
    p.begin("ota", false);
    p.putInt("qtd", qtdPessoas);
    p.putBytes("sala", sala, sizeof(sala));
    p.putBytes("tam", tamanhoUidSala, sizeof(tamanhoUidSala));
    p.end();
}

void otaApagarOcupacao() {
    Preferences p;
    p.begin("ota", false);
    p.clear();
    p.end();
}


void otaRestaurarOcupacao() {
    Preferences p;
    p.begin("ota", false);

    int qtd = p.getInt("qtd", 0);

    if (qtd >= 0 && qtd <= 20 &&
        p.getBytesLength("sala") == sizeof(sala) &&
        p.getBytesLength("tam") == sizeof(tamanhoUidSala)) {

        p.getBytes("sala", sala, sizeof(sala));
        p.getBytes("tam", tamanhoUidSala, sizeof(tamanhoUidSala));

        qtdPessoas = qtd;

        Serial.printf("[OTA] Ocupacao restaurada: %d pessoa(s)\n", qtd);
    }

    p.end();
}

// --- Wi-Fi: credenciais na flash, fora do .bin publicado --------------------
// Se ssid estiver preenchido (gravação por USB), salva. Se vazio (.bin da
// Release), usa as credenciais salvas anteriormente na placa.
void otaIniciarWiFi(const char* ssid, const char* senha) {
    Preferences p;
    p.begin("wifi", false);
    if (strlen(ssid) > 0) {
        p.putString("ssid", ssid);
        p.putString("senha", senha);
    }
    String s = p.getString("ssid", "");
    String k = p.getString("senha", "");
    p.end();

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    if (s.length() == 0) {
        Serial.println("[WiFi] Sem credenciais: grave uma vez por USB com SSID e senha.");
        return;
    }
    WiFi.begin(s.c_str(), k.c_str());
}

void otaCallbackMQTT(char* topico, byte* dados, unsigned int tamanho) {
    if (strcmp(topico, OTA_TOPICO_COMANDO) == 0 &&
        tamanho == 9 && memcmp(dados, "atualizar", 9) == 0) {
        otaSolicitada = true;  // executa no loop, fora do callback
    }
}

void otaSetup() {
    esp_ota_img_states_t estado;
    otaPendente = esp_ota_get_state_partition(esp_ota_get_running_partition(), &estado) == ESP_OK &&
                  estado == ESP_OTA_IMG_PENDING_VERIFY;
    Serial.printf("[OTA] Firmware %s%s\n", FIRMWARE_VERSAO, otaPendente ? " (em teste)" : "");
    otaRestaurarOcupacao();
    if (!otaPendente) otaApagarOcupacao();
    mqttClient.setCallback(otaCallbackMQTT);
}

// Chamar a cada conexão MQTT
void otaAoConectar() {
    mqttClient.subscribe(OTA_TOPICO_COMANDO);
    char msg[48];
    snprintf(msg, sizeof(msg), "Versao %s%s", FIRMWARE_VERSAO, otaPendente ? " em teste" : "");
    otaStatus(msg);
}

static bool otaLerVersao(const char* texto, int v[3]) {
    char extra;
    return sscanf(texto, "%d.%d.%d%c", &v[0], &v[1], &v[2], &extra) == 3
        && v[0] >= 0 && v[1] >= 0 && v[2] >= 0;
}

static bool otaVersaoMaior(const int nova[3], const int atual[3]) {
    for (int i = 0; i < 3; i++) {
        if (nova[i] != atual[i]) return nova[i] > atual[i];
    }
    return false;
}

void otaExecutar() {
    char msg[96];
    int nova[3], atual[3];
    otaStatus("Verificando versao");

    {
        WiFiClientSecure cliente;
        cliente.setCACert(OTA_CERTIFICADOS);
        HTTPClient http;
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.setTimeout(15000);
        if (!http.begin(cliente, OTA_URL_BASE "/latest/download/versao.txt")) {
            otaStatus("Erro: URL invalida");
            return;
        }
        int codigo = http.GET();
        if (codigo != HTTP_CODE_OK) {
            snprintf(msg, sizeof(msg), "Erro ao ler versao (%d)", codigo);
            http.end();
            otaStatus(msg);
            return;
        }
        String texto = http.getString();
        http.end();
        texto.trim();
        if (!otaLerVersao(texto.c_str(), nova) || !otaLerVersao(FIRMWARE_VERSAO, atual)) {
            otaStatus("Erro: versao invalida");
            return;
        }
    }

    if (!otaVersaoMaior(nova, atual)) {
        otaStatus("Ja atualizado: " FIRMWARE_VERSAO);
        return;
    }

    snprintf(msg, sizeof(msg), "Baixando %d.%d.%d", nova[0], nova[1], nova[2]);
    otaStatus(msg);

    char url[160];
    snprintf(url, sizeof(url), OTA_URL_BASE "/download/v%d.%d.%d/" OTA_ARQUIVO, nova[0], nova[1], nova[2]);

    WiFiClientSecure cliente;
    cliente.setCACert(OTA_CERTIFICADOS);
    httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    httpUpdate.rebootOnUpdate(false);
    httpUpdate.onProgress([](int feito, int total) {
        static int ultimo = -1;
        int pct = total > 0 ? (int)((int64_t)feito * 100 / total) : 0;
        if (pct / 10 != ultimo) {
            ultimo = pct / 10;
            char t[24];
            snprintf(t, sizeof(t), "Gravando %d%%", pct);
            Serial.printf("[OTA] %s\n", t);
            mostrarOTA(t);
        }
    });

    if (httpUpdate.update(cliente, url) == HTTP_UPDATE_OK) {
        otaSalvarOcupacao();
        otaStatus("Gravado. Reiniciando");
        delay(1000);
        ESP.restart();
    }
    snprintf(msg, sizeof(msg), "Falha: %s", httpUpdate.getLastErrorString().c_str());
    otaStatus(msg);  // o firmware atual continua rodando
}

// Chamar no loop. Retorna true se tentou atualizar (o rádio deve ser rearmado).

bool otaLoop() {
    if (otaPendente) {

        // Primeiro verifica se o prazo de inicialização expirou.
        if (millis() >= OTA_TEMPO_LIMITE_MS) {
            Serial.println("[OTA] Sem MQTT estavel: voltando a versao anterior");
            otaSalvarOcupacao();
            esp_ota_mark_app_invalid_rollback_and_reboot();
            return false;
        }

        // Verifica a estabilidade da conexão MQTT.
        if (!mqttClient.connected()) {
            otaMqttDesde = 0;
        }
        else if (otaMqttDesde == 0) {
            otaMqttDesde = millis();
        }
        else if (millis() - otaMqttDesde >= OTA_TEMPO_ESTAVEL_MS) {

            esp_err_t resultado = esp_ota_mark_app_valid_cancel_rollback();

            if (resultado == ESP_OK) {
                otaPendente = false;
                otaApagarOcupacao();
                otaStatus("Versao " FIRMWARE_VERSAO " confirmada");
                Serial.println("[OTA] Firmware confirmado com sucesso");
            }
            else {
                Serial.printf("[OTA] Erro ao confirmar firmware: %d\n", (int)resultado);
                otaStatus("Erro ao confirmar firmware");
                otaMqttDesde = millis();
            }
        }
    }

    if (!otaSolicitada) return false;

    otaSolicitada = false;

    if (otaPendente) {
        otaStatus("Aguarde: versao em teste");
        return false;
    }

    otaExecutar();
    return true;
}
