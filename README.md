# Monitoramento Inteligente de Salas de Escritório (IoT)

Sistema de monitoramento ambiental e de ocupação para salas de escritório utilizando **ESP32 LoRa (Heltec)**, sensores de temperatura/umidade (**DHT11/DHT22**), controle de acesso por **RFID (RC522)** e dashboard em nuvem via **Node-RED**.

## Arquitetura do Sistema

1. **Nó Externo (ESP LoRa Transmissor)**
   - Mede a **temperatura externa** através do sensor DHT.
   - Envia periodicamente os dados de temperatura via rádio **LoRa (915 MHz)** para os nós localizados dentro das salas de escritório.

2. **Nós Internos — Salas de Escritório (ESP LoRa Receptor + RFID + Sensor Interno)**
   - Recebem a **temperatura externa** enviada pelo ESP LoRa externo.
   - Medem a **temperatura interna** da sala.
   - Controlam a entrada e saída de pessoas via leitura de crachás **RFID (MFRC522)**, armazenando os UIDs presentes em uma lista dinâmica em memória e exibindo continuamente no **Monitor Serial** a quantidade atual de pessoas na sala.
   - Enviam para a **nuvem** os dados consolidados da sala:
     - Quantidade de pessoas na sala
     - Temperatura externa
     - Temperatura interna

3. **Nuvem e Interface (Node-RED)**
   - Recebe e armazena o histórico de ocupação e temperaturas de cada sala.
   - Disponibiliza uma interface gráfica construída em **Node-RED** para visualização em tempo real.

## Estrutura do Repositório

### Firmwares Integrados (Prototipagem Final)

Na raiz do repositório encontram-se os códigos completos e integrados de cada nó:

- [`esp-externo.cpp`](./esp-externo.cpp): Firmware do nó externo. Realiza leituras periódicas do sensor DHT11/DHT22 (pino 19), transmite pacotes com dados meteorológicos via LoRa (915 MHz) e exibe o status no display OLED onboard da placa Heltec ESP32.
- [`esp-interno.cpp`](./esp-interno.cpp): Firmware do nó interno. Realiza:
  - Controle de ocupação por crachá RFID RC522 via SPI com **vetor dinâmico** (`std::vector<String>`), gerenciando entradas e saídas de pessoas na sala;
  - Medição de temperatura/umidade interna com sensor DHT (pino 19);
  - Recepção contínua da temperatura externa transmitida via rádio LoRa (915 MHz);
  - Cálculo inteligente da **temperatura ideal para o ar-condicionado** considerando carga térmica humana, temperatura externa e padrões de ergonomia (NR-17);
  - Exibição de todas as informações em tempo real no display OLED Heltec (128x64) e Monitor Serial.

### Módulos Separados (Base)

A pasta [`modularizacao/`](./modularizacao) contém os módulos base utilizados individualmente nos testes de bancada:

- [`comunicacao-lora-transmissor.cpp`](./modularizacao/comunicacao-lora-transmissor.cpp): Envio de pacotes via LoRa (915 MHz) na placa Heltec ESP32.
- [`comunicacao-lora-receptor.cpp`](./modularizacao/comunicacao-lora-receptor.cpp): Recepção de pacotes via LoRa (915 MHz).
- [`cracha-rfid.cpp`](./modularizacao/cracha-rfid.cpp): Leitura de tags RFID via SPI (MFRC522), controle dinâmico de entrada/saída de pessoas na sala e impressão periódica da lotação no Monitor Serial (`115200 bps`).
- [`sensor-temp-umidade.cpp`](./modularizacao/sensor-temp-umidade.cpp): Leitura de temperatura e umidade com o sensor DHT11/DHT22.
