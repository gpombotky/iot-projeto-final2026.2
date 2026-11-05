#include <SPI.h>
#include <MFRC522.h>

// Pinos do RFID RC522
#define SS_PIN   7
#define RST_PIN  3
#define SCK_PIN  6
#define MISO_PIN 4
#define MOSI_PIN 5

// Inicializa o leitor RFID
MFRC522 rfid(SS_PIN, RST_PIN);

// Guarda ate 20 crachas (cada cracha tem ate 10 bytes + 1 byte pro tamanho)
byte sala[20][10];
byte tamanhoUidSala[20];
int qtdPessoas = 0;

unsigned long ultimoAviso = 0;

void setup() {
  Serial.begin(115200);
  delay(1500); // Espera o Monitor Serial reconectar apos o upload

  // Forca um reset fisico no pino RST do RC522 para nao travar apos upload
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(50);
  digitalWrite(RST_PIN, HIGH);
  delay(50);

  // Inicializa SPI com os pinos escolhidos
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  // Inicializa o RC522
  rfid.PCD_Init();
  delay(50);

  Serial.println("================================");
  Serial.println("        RFID RC522 INICIADO");
  Serial.println("================================");
  Serial.println("Aproxime um cartao ou tag...");
}

void loop() {
  // Printa a cada 3 segundos para voce ter certeza que o Monitor Serial esta funcionando
  if (millis() - ultimoAviso > 3000) {
    ultimoAviso = millis();
    Serial.print("[Sistema rodando] Pessoas dentro da sala agora: ");
    Serial.println(qtdPessoas);
  }

  // Verifica se existe um novo cartao
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  // Tenta ler o cartao
  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  // Exibe o UID (exatamente igual ao seu original)
  Serial.print("Cartao detectado! UID: ");

  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      Serial.print("0");
    }

    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  // Exibe o tipo de cartao (exatamente igual ao seu original)
  MFRC522::PICC_Type tipo = rfid.PICC_GetType(rfid.uid.sak);

  Serial.print("Tipo: ");
  Serial.println(rfid.PICC_GetTypeName(tipo));

  // --- CONTROLE DE ENTRADA E SAIDA DA SALA ---
  int pos = -1;

  // 1. Verifica se esse cracha ja esta dentro da sala
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

  // 2. Se pos == -1, nao estava na sala (ENTROU). Senao, ja estava (SAIU).
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

  // 3. Mostra a lista de quem esta na sala
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
  // -------------------------------------------

  Serial.println("-------------------------------");

  // Finaliza comunicacao com o cartao
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  delay(500);
}