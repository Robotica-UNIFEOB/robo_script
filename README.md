# Robo Script — Vespa Rocket + RoboARM

Firmware para controle web do robô **Vespa** (RoboCore) e do braço robótico **RoboARM**, via Wi-Fi e WebSocket. Interface embutida com joystick virtual e sliders para os servomotores.

Baseado no sketch `teste.ino` (v1.2 — 20/01/2025), Copyright 2025 RoboCore.

## Funcionalidades

- **Joystick virtual** — direção e velocidade dos motores de tração (0–100%, ângulo 0–360°)
- **Braço robótico** — 4 servos controlados por sliders na interface web:
  - S1 — Garra
  - S2 — Altura
  - S3 — Distância (alcance)
  - S4 — Base
- **Monitor de bateria** — tensão enviada ao cliente a cada 5 s; alerta visual e LED piscando abaixo de 7 V
- **Um cliente por vez** — segunda conexão recebe página “ocupado”
- **Segurança** — motores param e servos voltam à posição salva quando não há cliente conectado

## Hardware

| Item | Detalhe |
|------|---------|
| Placa | ESP32 (pacote de placas v2.0.x ou v3.0.x) |
| Robô | Vespa Rocket (biblioteca `RoboCore_Vespa`) |
| LED status | GPIO 15 |
| Servos | 4× (pulsos 500–2500 µs) nos pinos `VESPA_SERVO_S1` … `S4` |

## Dependências (Arduino Library Manager / links)

| Biblioteca | Versão indicada no código |
|------------|---------------------------|
| [ArduinoJson](https://arduinojson.org) | 7.3.0 |
| [AsyncTCP](https://github.com/ESP32Async/AsyncTCP) | 3.3.5 |
| [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer) | 3.7.1 |
| RoboCore_Vespa | 1.3.0 |

Também é necessário o pacote de placas **ESP32** da Espressif (Arduino IDE ou PlatformIO).

## Rede Wi-Fi

O robô cria um **Access Point** com SSID dinâmico:

- Formato: `Vespa-xxxxx` (últimos dígitos derivados do MAC da placa)
- Senha padrão: `robocore`
- IP típico: exibido no Serial Monitor após o boot (geralmente `192.168.4.1`)

Conecte o celular ou PC à rede do robô e abra no navegador o IP do AP (porta 80).

## Protocolo WebSocket

Endpoint: `ws://<IP-do-robô>/ws`

### Controle dos motores (joystick)

```json
{ "velocidade": 0, "angulo": 0 }
```

- `velocidade`: 0–100 (%)
- `angulo`: 0–360 (graus)

### Controle de um servo

```json
{ "servo": 1, "posicao": 120 }
```

- `servo`: 1 (garra) a 4 (base)
- `posicao`: 0–180 (graus; S3 e S4 são invertidos no firmware)

### Telemetria (servidor → cliente)

```json
{ "vbat": 8400 }
```

- `vbat`: tensão da bateria em **mV**

## Posições iniciais dos servos

| Servo | Função | Inicial |
|-------|--------|---------|
| 1 | Garra | 120° |
| 2 | Altura | 140° |
| 3 | Distância | 90° |
| 4 | Base | 90° |

## Como usar

1. Instale as bibliotecas listadas acima.
2. Abra `teste.ino` na Arduino IDE (ou renomeie a pasta do sketch para coincidir com o nome do `.ino`, se necessário).
3. Selecione a placa **ESP32** e a porta serial correta.
4. Faça o upload e abra o **Serial Monitor** em **115200 baud**.
5. Conecte-se à rede `Vespa-xxxxx` e acesse o IP no navegador.

## Estrutura do repositório

```
robo_script/
├── teste.ino      # Firmware principal (HTML + lógica embutidos)
├── README.md
└── .gitignore
```

## Licença

Este programa é software livre sob a [GNU Lesser General Public License](https://www.gnu.org/licenses/) (LGPL v3 ou posterior), conforme o cabeçalho do sketch original da RoboCore.
