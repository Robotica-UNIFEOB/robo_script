/*******************************************************************************
* Controle web de um robo e um braco robotico com a Vespa por WebSocket
* (v1.2 - 20/01/2025)
*
* Copyright 2025 RoboCore.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version (<https://www.gnu.org/licenses/>).
*
* Versoes
* v1.2
*   - Atualizacao dos limites dos servomotores.
* v1.1
*   - Atualizacao do SSID dinamico para o pacote de placas v3.0.x.
* 
*******************************************************************************/

// --------------------------------------------------
// Bibliotecas
   
#include <esp_arduino_version.h> // Versao do pacote de placas ESP32 (até v3.0.x)

#include <WiFi.h>
#include <ArduinoJson.h> // https://arduinojson.org (v7.3.0)
#include <AsyncTCP.h> // https://github.com/ESP32Async/AsyncTCP (v3.3.5)
#include <ESPAsyncWebServer.h> // https://github.com/ESP32Async/ESPAsyncWebServer (v3.7.1)

#include <RoboCore_Vespa.h> // (v1.3.0)

// --------------------------------------------------

struct servo_angulos_t {
  uint8_t min; // angulo minimo [0;180]
  uint8_t max; // angulo maximo [0;180]
  uint8_t atual; // posicao atual
};

// --------------------------------------------------
// Variaveis

// web server assincrono na porta 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// LED
const uint8_t PIN_LED = 15;

// JSON aliases
const char *ALIAS_ANGULO = "angulo";
const char *ALIAS_POSICAO = "posicao";
const char *ALIAS_SERVO = "servo";
const char *ALIAS_VELOCIDADE = "velocidade";
const char *ALIAS_VBAT = "vbat";

// variaveis da Vespa
VespaMotors motores;
VespaServo servos[4];
const uint16_t SERVO_MAX = 2500;
const uint16_t SERVO_MIN = 500;
enum Motor { Base = 0 , Alcance , Elevacao , Garra };
servo_angulos_t sangulos[4] = { { 0, 180, 120 },
                                { 0, 180, 140 },
                                { 0, 180, 90 },
                                { 0, 180, 90 } };
VespaBattery vbat;
uint8_t vbat_critico = 0xFF;
const uint32_t INTERVALO_ATUALIZACAO_VBAT = 5000; // [ms]
const uint32_t INTERVALO_ATUALIZACAO_DESCONEXAO = 100; // [ms]
const uint32_t INTERVALO_LED_VBAT_HIGH = 1000; // [ms]
const uint32_t INTERVALO_LED_VBAT_LOW = 500; // [ms]
uint32_t timeout_vbat, timeout_desconexao, timeout_led_vbat;
bool habilitar_reset_motores = true;

// --------------------------------------------------
// Pagina web principal

const char html_index[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>RoboCore Joystick + RoboARM</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, minimum-scale=1.0, maximum-scale=1.0, user-scalable=0">
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Oswald:wght@400;600&display=swap');

        *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

        :root {
            --bg:        #0e0f11;
            --panel:     #161820;
            --border:    #2a2d38;
            --accent:    #f0be00;
            --green:     #39ff6a;
            --red:       #ff3b3b;
            --orange:    #ff8c00;
            --text:      #c8cad4;
            --dim:       #5a5d6e;
            --mono:      'Share Tech Mono', monospace;
            --sans:      'Oswald', sans-serif;
        }

        html, body {
            width: 100%; height: 100%;
            background: var(--bg);
            color: var(--text);
            font-family: var(--mono);
            overflow: hidden;
            -webkit-user-select: none;
            user-select: none;
            overscroll-behavior: none;
        }

        /* ── HEADER ── */
        #header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            background: #000;
            border-bottom: 1px solid var(--border);
            padding: 6px 14px;
            height: 44px;
            gap: 12px;
        }
        #header svg { height: 22px; }

        /* battery */
        #bat-wrap {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 13px;
            color: var(--text);
            white-space: nowrap;
        }
        #vbat-val { color: var(--accent); font-weight: bold; }
        .bat-shell {
            width: 38px; height: 16px;
            border: 2px solid var(--dim);
            border-radius: 3px;
            position: relative;
            padding: 2px;
        }
        .bat-shell::after {
            content: '';
            position: absolute;
            right: -5px; top: 50%;
            transform: translateY(-50%);
            width: 3px; height: 8px;
            background: var(--dim);
            border-radius: 0 2px 2px 0;
        }
        #bat-fill {
            height: 100%;
            border-radius: 1px;
            background: var(--green);
            transition: width .5s, background .5s;
        }

        /* ws status */
        #ws-dot {
            width: 9px; height: 9px;
            border-radius: 50%;
            background: var(--red);
            box-shadow: 0 0 6px var(--red);
            transition: background .3s, box-shadow .3s;
            flex-shrink: 0;
        }
        #ws-dot.on { background: var(--green); box-shadow: 0 0 8px var(--green); }
        #ws-label { font-size: 11px; color: var(--dim); }

        /* ── DEBUG BAR ── */
        #debug-bar {
            background: var(--panel);
            border-bottom: 1px solid var(--border);
            padding: 4px 14px;
            display: flex;
            flex-wrap: wrap;
            gap: 6px 20px;
            font-size: 11px;
            color: var(--dim);
        }
        .dbg { display: flex; gap: 5px; align-items: center; }
        .dbg-lbl { color: var(--dim); text-transform: uppercase; font-size: 10px; }
        .dbg-val { color: var(--accent); min-width: 32px; }
        .dbg-val.ok  { color: var(--green); }
        .dbg-val.err { color: var(--red); }
        #dbg-last-msg {
            color: var(--dim);
            font-size: 10px;
            flex: 1;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }

        /* ── LOW BATTERY BANNER ── */
        #bat-warn {
            display: none;
            position: absolute;
            top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(255,59,59,.08);
            border: 2px solid var(--red);
            z-index: 20;
            pointer-events: none;
            animation: pulse-border 1s infinite;
        }
        #bat-warn-msg {
            display: none;
            position: absolute;
            top: 44px; left: 50%;
            transform: translateX(-50%);
            background: var(--red);
            color: #fff;
            font-family: var(--sans);
            font-size: 14px;
            padding: 4px 18px;
            border-radius: 0 0 6px 6px;
            z-index: 21;
            letter-spacing: 1px;
        }
        @keyframes pulse-border { 0%,100%{opacity:1} 50%{opacity:.3} }

        /* ── MAIN LAYOUT ── */
        #main {
            display: flex;
            align-items: center;
            justify-content: space-evenly;
            flex-wrap: wrap;
            gap: 20px;
            padding: 16px;
            height: calc(100% - 44px - 32px); /* header + debug */
        }

        /* ── JOYSTICK ── */
        #canvas_joystick { border: none; cursor: crosshair; }

        /* ── ARM PANEL ── */
        #arm-panel {
            background: var(--panel);
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 16px 20px;
            display: flex;
            flex-direction: column;
            gap: 14px;
            min-width: 260px;
            max-width: 340px;
        }
        #arm-panel h2 {
            font-family: var(--sans);
            font-size: 12px;
            letter-spacing: 3px;
            color: var(--dim);
            text-transform: uppercase;
            border-bottom: 1px solid var(--border);
            padding-bottom: 8px;
        }

        .ctrl-row {
            display: flex;
            flex-direction: column;
            gap: 5px;
        }
        .ctrl-label {
            display: flex;
            justify-content: space-between;
            align-items: center;
            font-size: 11px;
        }
        .ctrl-name {
            color: var(--dim);
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .ctrl-val {
            font-size: 13px;
            color: var(--accent);
            min-width: 36px;
            text-align: right;
        }

        /* sliders */
        input[type=range] {
            -webkit-appearance: none;
            appearance: none;
            width: 100%;
            height: 36px;
            background: transparent;
            cursor: pointer;
            outline: none;
        }
        input[type=range]::-webkit-slider-runnable-track {
            height: 6px;
            background: var(--border);
            border-radius: 3px;
        }
        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 22px;
            height: 22px;
            border-radius: 50%;
            background: var(--accent);
            margin-top: -8px;
            box-shadow: 0 0 8px rgba(240,190,0,.5);
            transition: box-shadow .15s;
        }
        input[type=range]:active::-webkit-slider-thumb {
            box-shadow: 0 0 16px rgba(240,190,0,.9);
        }
        input[type=range]::-moz-range-track {
            height: 6px;
            background: var(--border);
            border-radius: 3px;
        }
        input[type=range]::-moz-range-thumb {
            width: 22px;
            height: 22px;
            border-radius: 50%;
            background: var(--accent);
            border: none;
            box-shadow: 0 0 8px rgba(240,190,0,.5);
        }

        /* garra dupla */
        .garra-wrap {
            display: flex;
            gap: 8px;
        }
        .garra-wrap input { flex: 1; }

        /* slider vertical para altura e distância */
        .vert-group {
            display: flex;
            gap: 16px;
            align-items: center;
        }
        .vert-ctrl {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 6px;
            flex: 1;
        }
        .vert-ctrl .ctrl-name { text-align: center; }
        .vert-ctrl .ctrl-val  { text-align: center; }
        .vert-slider-wrap {
            width: 36px;
            height: 130px;
            position: relative;
            overflow: visible;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .vert-slider-wrap input[type=range] {
            width: 130px;
            height: 36px;
            transform: rotate(-90deg);
            position: absolute;
        }
    </style>
</head>
<body>

<!-- LOW BATTERY -->
<div id="bat-warn"></div>
<div id="bat-warn-msg">⚠ BATERIA FRACA</div>

<!-- HEADER -->
<div id="header">
    <div style="display:flex;align-items:center;gap:10px;">
        <div id="ws-dot"></div>
        <span id="ws-label">DESCONECTADO</span>
    </div>

    <svg version="1.0" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1420 225" xml:space="preserve">
        <g fill="#f0be00">
            <path fill-rule="evenodd" clip-rule="evenodd" d="M175.7,94.9c0,11.7-6.2,20.2-18.5,25.6c-9.8,4-21,5.9-33.8,5.9H84.5l78.8,33.5c4.2,1.8,6.3,4.6,6.3,8.2c0,5.3-4.6,8-13.9,8c-3.7,0-6.6-0.3-8.7-0.8l-107.6-46v38.4c0,5.6-2.8,8.4-8.3,8.4c-5.9,0-9.3-0.3-10.2-0.7c-3.4-1.1-5.1-3.6-5.1-7.7V54.8c0-5.6,2.8-8.3,8.3-8.3h95c14.7,0,26.8,1.9,36.3,5.7c13.5,5.5,20.3,14.5,20.3,27V94.9L175.7,94.9z M152.1,95.1V78.7c0-8.7-11.1-13-33.1-13H39.4v41.5h83.8C142.4,107.2,152.1,103.2,152.1,95.1L152.1,95.1z M343.9,141.2c0,23.2-20,34.7-60,34.7h-36.8c-16.2,0-29-2.1-38.3-6.2c-14.3-5.8-21.5-15.3-21.5-28.5V81.6c0-23.4,19.9-35.1,59.8-35.1h36.8c18.6,0,31.8,1.6,39.4,4.9c13.7,5.3,20.6,15.4,20.6,30.2V141.2L343.9,141.2z M320.3,140.6V82.1c0-10.9-11.9-16.4-35.8-16.4H247c-24.1,0-36.1,5.4-36.1,16.4v58.5c0,10.6,12,15.8,36.1,15.8h37.5C308.4,156.5,320.3,151.2,320.3,140.6L320.3,140.6z M512.2,141.5c0,12.4-5.9,21.6-17.6,27.5c-9.1,4.6-20.5,6.9-34.2,6.9H360.2c-5.5,0-8.3-2.8-8.3-8.4V54.8c0-5.5,2.8-8.3,8.3-8.3H459c8.8,0,16.9,0.7,24.2,2.1c7.9,1.9,14.5,5.4,20,10.5c6,5.7,9,12.4,9,19.9c0,7.4-0.3,12.7-0.8,16c-0.1,0.6-0.6,1.7-1.4,3.2c-0.8,1.5-1.9,3.3-3.2,5.6c-2.6,4.5-3.9,7-3.9,7.5c0,0.4,1.3,2.6,3.9,6.7c1.3,2.1,2.4,3.8,3.2,5.2c0.8,1.4,1.2,2.5,1.4,3.3c0.3,1.1,0.5,2.9,0.6,5.4C512.1,134.4,512.2,137.6,512.2,141.5L512.2,141.5z M489.2,90.1c0-2.4-0.2-5.9-0.7-10.8c-0.7-9-10.9-13.5-30.6-13.5H376v36.3h82.8C479.1,102.1,489.2,98.1,489.2,90.1L489.2,90.1z M488.5,134.9c0-8.9-9.8-13.4-29.2-13.4h-83.8v35h82.2c19,0,29.3-4.1,30.6-12.2c0.1-0.4,0.1-1.4,0.2-3C488.5,139.7,488.5,137.6,488.5,134.9L488.5,134.9z M676.8,141.2c0,23.2-20,34.7-60,34.7H580c-16.2,0-29-2.1-38.3-6.2c-14.3-5.8-21.5-15.3-21.5-28.5V81.6c0-23.4,19.9-35.1,59.8-35.1h36.8c18.6,0,31.8,1.6,39.4,4.9c13.7,5.3,20.6,15.4,20.6,30.2V141.2L676.8,141.2z M653.1,140.6V82.1c0-10.9-11.9-16.4-35.8-16.4h-37.4c-24.1,0-36.1,5.4-36.1,16.4v58.5c0,10.6,12,15.8,36.1,15.8h37.5C641.2,156.5,653.1,151.2,653.1,140.6z"/>
            <polygon fill-rule="evenodd" clip-rule="evenodd" points="724,11.6 765,11.6 726,104.8 779.6,76.9 750.7,189.8 765,182.8 733.6,230.4 722,172.8 732.9,184 744.9,111.7 689.2,140.7"/>
            <path fill-rule="evenodd" clip-rule="evenodd" d="M935.5,57.4c0,5.6-2.8,8.4-8.3,8.4h-84c-20.4,0-30.6,4.1-30.6,12.2v64.1c0,9.6,11.3,14.4,33.9,14.4h80.2c5.8,0,8.6,3.2,8.6,9.7c0,6.5-2.9,9.8-8.6,9.8h-80.2c-10.8,0-18.6-0.4-23.4-1.3c-5.2-1-10.6-2.9-16.1-5.7c-6.4-3.3-10.8-6.8-13.4-10.4c-3-4.1-4.6-9.5-4.6-16.1V78.1c0-8.2,3.7-15.1,11.1-20.8c6.2-4.8,13.8-7.9,22.7-9.6c2.3-0.4,5-0.6,8.3-0.8c3.3-0.2,7.1-0.3,11.6-0.3h84.5C932.7,46.5,935.5,50.2,935.5,57.4L935.5,57.4z M1101.8,141.2c0,23.2-20,34.7-60,34.7H1005c-16.2,0-29-2.1-38.3-6.2c-14.3-5.8-21.5-15.3-21.5-28.5V81.6c0-23.4,19.9-35.1,59.8-35.1h36.8c18.6,0,31.8,1.6,39.4,4.9c13.7,5.3,20.6,15.4,20.6,30.2V141.2L1101.8,141.2z M1078.1,140.6V82.1c0-10.9-11.9-16.4-35.8-16.4h-37.4c-24.1,0-36.1,5.4-36.1,16.4v58.5c0,10.6,12,15.8,36.1,15.8h37.5C1066.2,156.5,1078.1,151.2,1078.1,140.6L1078.1,140.6z M1267.8,94.9c0,11.7-6.2,20.2-18.5,25.6c-9.8,4-21,5.9-33.8,5.9h-38.9l78.8,33.5c4.2,1.8,6.3,4.6,6.3,8.2c0,5.3-4.6,8-13.9,8c-3.7,0-6.6-0.3-8.7-0.8l-107.6-46v38.4c0,5.6-2.8,8.4-8.3,8.4c-5.9,0-9.3-0.3-10.2-0.7c-3.4-1.1-5.1-3.6-5.1-7.7V54.8c0-5.6,2.8-8.3,8.3-8.3h95c14.7,0,26.8,1.9,36.3,5.7c13.5,5.5,20.3,14.5,20.3,27V94.9L1267.8,94.9z M1244.2,95.1V78.7c0-8.7-11.1-13-33.1-13h-79.6v41.5h83.8C1234.6,107.2,1244.2,103.2,1244.2,95.1L1244.2,95.1z M1416.5,111.2c0,6.9-2.9,10.3-8.8,10.3h-113.6v21.3c0,9.1,10.1,13.7,30.3,13.7h83.3c5.5,0,8.3,2.8,8.3,8.4c0,7.4-2.8,11.1-8.3,11.1h-84.4c-12.3,0-23.6-2.2-33.9-6.6c-12.6-5.7-18.9-14.3-18.9-25.7V79.5c0-6.2,2.1-11.8,6.3-16.8c9.1-10.8,25.3-16.1,48.6-16.1h23.1c5.4,0,8.2,3.6,8.2,10.8c0,5.6-2.8,8.4-8.2,8.4h-3.4h-2.1h-1.2c-0.8,0-1.2,0-1.3-0.1c-1.8-0.1-3.3-0.1-4.6-0.1c-1.3,0-2.3-0.1-3.2-0.1c-25.9,0-38.8,4.7-38.8,14v22.6h113.9C1413.6,102.1,1416.5,105.1,1416.5,111.2z"/>
        </g>
    </svg>

    <div id="bat-wrap">
        <span id="vbat-val">--.-</span><span style="color:var(--dim)"> V</span>
        <div class="bat-shell"><div id="bat-fill" style="width:0%"></div></div>
    </div>
</div>

<!-- DEBUG BAR -->
<div id="debug-bar">
    <div class="dbg"><span class="dbg-lbl">VEL</span><span class="dbg-val" id="dbg-vel">0</span><span class="dbg-lbl">%</span></div>
    <div class="dbg"><span class="dbg-lbl">ANG</span><span class="dbg-val" id="dbg-ang">0</span><span class="dbg-lbl">°</span></div>
    <div class="dbg"><span class="dbg-lbl">S1-GARRA</span><span class="dbg-val" id="dbg-s1">120</span></div>
    <div class="dbg"><span class="dbg-lbl">S2-ALTURA</span><span class="dbg-val" id="dbg-s2">140</span></div>
    <div class="dbg"><span class="dbg-lbl">S3-DIST</span><span class="dbg-val" id="dbg-s3">90</span></div>
    <div class="dbg"><span class="dbg-lbl">S4-BASE</span><span class="dbg-val" id="dbg-s4">90</span></div>
    <div class="dbg"><span class="dbg-lbl">TX</span><span class="dbg-val ok" id="dbg-tx">0</span></div>
    <div class="dbg"><span class="dbg-lbl">ERR</span><span class="dbg-val" id="dbg-err">0</span></div>
    <span id="dbg-last-msg">aguardando conexão...</span>
</div>

<!-- MAIN -->
<div id="main">
    <canvas id="canvas_joystick"></canvas>

    <div id="arm-panel">
        <h2>Controle do Braço</h2>

        <!-- GARRA -->
        <div class="ctrl-row">
            <div class="ctrl-label">
                <span class="ctrl-name">🦾 Garra (S1)</span>
                <span class="ctrl-val" id="lbl-s1">120</span>
            </div>
            <div class="garra-wrap">
                <input type="range" min="100" max="160" value="120" id="s1"  title="Garra - controle A">
                <input type="range" min="100" max="160" value="120" id="s1c" title="Garra - controle B (espelho)">
            </div>
            <div style="display:flex;justify-content:space-between;font-size:10px;color:var(--dim);padding:0 2px;">
                <span>FECHADA</span><span>ABERTA</span>
            </div>
        </div>

        <!-- ALTURA + DISTANCIA lado a lado vertical -->
        <div class="vert-group">
            <div class="vert-ctrl">
                <span class="ctrl-name">Altura (S2)</span>
                <div class="vert-slider-wrap">
                    <input type="range" min="100" max="145" value="140" id="s2">
                </div>
                <span class="ctrl-val" id="lbl-s2">140</span>
            </div>

            <div style="width:1px;height:130px;background:var(--border);flex-shrink:0;"></div>

            <div class="vert-ctrl">
                <span class="ctrl-name">Distância (S3)</span>
                <div class="vert-slider-wrap">
                    <input type="range" min="0" max="90" value="90" id="s3">
                </div>
                <span class="ctrl-val" id="lbl-s3">90</span>
            </div>
        </div>

        <!-- BASE -->
        <div class="ctrl-row">
            <div class="ctrl-label">
                <span class="ctrl-name">🔄 Base (S4)</span>
                <span class="ctrl-val" id="lbl-s4">90</span>
            </div>
            <input type="range" min="0" max="180" value="90" id="s4">
            <div style="display:flex;justify-content:space-between;font-size:10px;color:var(--dim);padding:0 2px;">
                <span>◀ ESQ</span><span>DIR ▶</span>
            </div>
        </div>
    </div>
</div>

<script>
    // ── WebSocket ──────────────────────────────────────────────────────────
    var txCount = 0, errCount = 0;
    var connection = new WebSocket(`ws://${window.location.hostname}/ws`);

    function setWsStatus(on) {
        var dot = document.getElementById('ws-dot');
        var lbl = document.getElementById('ws-label');
        if (on) { dot.classList.add('on'); lbl.textContent = 'CONECTADO'; }
        else     { dot.classList.remove('on'); lbl.textContent = 'DESCONECTADO'; }
    }

    connection.onopen = function () {
        console.log('[WS] Conexão aberta com ' + window.location.hostname);
        setWsStatus(true);
        dbgMsg('WS conectado em ' + window.location.hostname);
    };

    connection.onerror = function (error) {
        console.error('[WS] Erro:', error);
        setWsStatus(false);
        errCount++;
        document.getElementById('dbg-err').textContent = errCount;
        document.getElementById('dbg-err').classList.add('err');
        dbgMsg('ERRO WebSocket: ' + JSON.stringify(error));
        alert('WebSocket Error: ' + error);
    };

    connection.onclose = function (e) {
        console.warn('[WS] Conexão fechada. Código:', e.code, 'Razão:', e.reason);
        setWsStatus(false);
        dbgMsg('WS fechado | código=' + e.code);
    };

    connection.onmessage = function (e) {
        console.log('[WS] Recebido:', e.data);
        dbgMsg('RX: ' + e.data);
        try {
            var data = JSON.parse(e.data);
            if (data["vbat"] !== undefined) {
                var mv = data["vbat"];
                var v  = (mv / 1000).toFixed(1);
                document.getElementById('vbat-val').textContent = v;
                var pct = Math.min(100, Math.max(2, Math.round(mv * 100 / 8400)));
                var fill = document.getElementById('bat-fill');
                fill.style.width = pct + '%';
                if (pct < 20) {
                    fill.style.background = 'var(--red)';
                    document.getElementById('bat-warn').style.display = 'block';
                    document.getElementById('bat-warn-msg').style.display = 'block';
                } else if (pct < 50) {
                    fill.style.background = 'var(--orange)';
                    document.getElementById('bat-warn').style.display = 'none';
                    document.getElementById('bat-warn-msg').style.display = 'none';
                } else {
                    fill.style.background = 'var(--green)';
                    document.getElementById('bat-warn').style.display = 'none';
                    document.getElementById('bat-warn-msg').style.display = 'none';
                }
                console.log('[BAT] ' + v + ' V (' + pct + '%)');
            }
        } catch(ex) {
            console.error('[WS] Falha ao parsear mensagem:', ex, '| raw:', e.data);
            errCount++;
            document.getElementById('dbg-err').textContent = errCount;
        }
    };

    function dbgMsg(msg) {
        document.getElementById('dbg-last-msg').textContent = msg;
    }

    function send_slider(servo_id, value) {
        var payload = { servo: servo_id, posicao: parseInt(value) };
        var json = JSON.stringify(payload);
        console.log('[SLIDER] Enviando servo=' + servo_id + ' posicao=' + value + ' | JSON:', json);
        if (connection.readyState === WebSocket.OPEN) {
            connection.send(json);
            txCount++;
            document.getElementById('dbg-tx').textContent = txCount;
            dbgMsg('TX: ' + json);
        } else {
            console.warn('[SLIDER] WS não está aberto. readyState=' + connection.readyState);
            errCount++;
            document.getElementById('dbg-err').textContent = errCount;
            dbgMsg('ERRO TX: WS não aberto (state=' + connection.readyState + ')');
        }
    }

    function send_joystick(speed, angle) {
        var payload = { velocidade: speed, angulo: angle };
        var json = JSON.stringify(payload);
        console.log('[JOYSTICK] vel=' + speed + ' ang=' + angle + ' | JSON:', json);
        if (connection.readyState === WebSocket.OPEN) {
            connection.send(json);
            txCount++;
            document.getElementById('dbg-tx').textContent = txCount;
            dbgMsg('TX: ' + json);
        } else {
            console.warn('[JOYSTICK] WS não está aberto. readyState=' + connection.readyState);
            errCount++;
            document.getElementById('dbg-err').textContent = errCount;
        }
    }

    // ── Sliders (ARM) ──────────────────────────────────────────────────────
    function sliderMirror(from, to) {
        var fEl = document.getElementById(from);
        var tEl = document.getElementById(to);
        tEl.value = parseInt(tEl.max) - (parseInt(fEl.value) - parseInt(fEl.min));
    }

    document.getElementById('s1').addEventListener('input', function(e) {
        sliderMirror('s1', 's1c');
        var v = parseInt(e.target.value);
        document.getElementById('lbl-s1').textContent = v;
        document.getElementById('dbg-s1').textContent = v;
        console.log('[GARRA] s1 -> posicao=' + v);
        send_slider(1, v);
    });

    document.getElementById('s1c').addEventListener('input', function(e) {
        sliderMirror('s1c', 's1');
        var v = parseInt(document.getElementById('s1').value);
        document.getElementById('lbl-s1').textContent = v;
        document.getElementById('dbg-s1').textContent = v;
        console.log('[GARRA] s1c -> posicao=' + v);
        send_slider(1, v);
    });

    document.getElementById('s2').addEventListener('input', function(e) {
        var v = parseInt(e.target.value);
        document.getElementById('lbl-s2').textContent = v;
        document.getElementById('dbg-s2').textContent = v;
        console.log('[ALTURA] s2 -> posicao=' + v);
        send_slider(2, v);
    });

    document.getElementById('s3').addEventListener('input', function(e) {
        var v = parseInt(e.target.value);
        document.getElementById('lbl-s3').textContent = v;
        document.getElementById('dbg-s3').textContent = v;
        console.log('[DISTANCIA] s3 -> posicao=' + v);
        send_slider(3, v);
    });

    document.getElementById('s4').addEventListener('input', function(e) {
        var v = parseInt(e.target.value);
        document.getElementById('lbl-s4').textContent = v;
        document.getElementById('dbg-s4').textContent = v;
        console.log('[BASE] s4 -> posicao=' + v);
        send_slider(4, v);
    });

    // ── Joystick ───────────────────────────────────────────────────────────
    var canvas_joystick, ctx_joystick;
    var paint = false;
    var movimento = 0;
    var last_update = 0;
    var width, height, radius;
    var origin_joystick = { x: 0, y: 0 };
    const radius_factor = 7;

    window.addEventListener('load', function() {
        canvas_joystick = document.getElementById('canvas_joystick');
        ctx_joystick = canvas_joystick.getContext('2d');
        resize();
        canvas_joystick.addEventListener('mousedown',  startDrawing);
        canvas_joystick.addEventListener('mouseup',    stopDrawing);
        canvas_joystick.addEventListener('mousemove',  Draw);
        canvas_joystick.addEventListener('touchstart', startDrawing, { passive: true });
        canvas_joystick.addEventListener('touchend',   stopDrawing);
        canvas_joystick.addEventListener('touchcancel',stopDrawing);
        canvas_joystick.addEventListener('touchmove',  Draw, { passive: true });
        window.addEventListener('resize', resize);
    });

    function resize() {
        if (window.innerWidth > window.innerHeight) {
            width = (window.innerWidth / 2) - 40;
        } else {
            width = window.innerWidth - 40;
        }
        radius = width * 0.04;
        height = radius * radius_factor * 2 + 100;
        canvas_joystick.width  = width;
        canvas_joystick.height = height;
        origin_joystick.x = width / 2;
        origin_joystick.y = height / 2;
        joystick(origin_joystick.x, origin_joystick.y);
    }

    function joystick_background() {
        ctx_joystick.clearRect(0, 0, canvas_joystick.width, canvas_joystick.height);

        // anel externo
        ctx_joystick.beginPath();
        ctx_joystick.arc(origin_joystick.x, origin_joystick.y, radius * radius_factor, 0, Math.PI * 2);
        ctx_joystick.fillStyle = '#1a1c22';
        ctx_joystick.fill();
        ctx_joystick.strokeStyle = '#2a2d38';
        ctx_joystick.lineWidth = 2;
        ctx_joystick.stroke();

        // anel interno guia
        ctx_joystick.beginPath();
        ctx_joystick.arc(origin_joystick.x, origin_joystick.y, radius * radius_factor * 0.6, 0, Math.PI * 2);
        ctx_joystick.strokeStyle = '#252830';
        ctx_joystick.lineWidth = 1;
        ctx_joystick.stroke();

        // cruza
        ctx_joystick.strokeStyle = '#252830';
        ctx_joystick.lineWidth = 1;
        ctx_joystick.beginPath();
        ctx_joystick.moveTo(origin_joystick.x, origin_joystick.y - radius * radius_factor);
        ctx_joystick.lineTo(origin_joystick.x, origin_joystick.y + radius * radius_factor);
        ctx_joystick.stroke();
        ctx_joystick.beginPath();
        ctx_joystick.moveTo(origin_joystick.x - radius * radius_factor, origin_joystick.y);
        ctx_joystick.lineTo(origin_joystick.x + radius * radius_factor, origin_joystick.y);
        ctx_joystick.stroke();

        // setas
        var r = radius * radius_factor;
        var arrowOff = 18;
        ctx_joystick.fillStyle = '#f0be00';

        function arrow(pts) {
            ctx_joystick.beginPath();
            ctx_joystick.moveTo(pts[0][0], pts[0][1]);
            for (var i = 1; i < pts.length; i++) ctx_joystick.lineTo(pts[i][0], pts[i][1]);
            ctx_joystick.closePath();
            ctx_joystick.fill();
        }

        var cx = origin_joystick.x, cy = origin_joystick.y;
        arrow([[cx, cy-r-arrowOff], [cx+14, cy-r-arrowOff+18], [cx-14, cy-r-arrowOff+18]]);
        arrow([[cx, cy+r+arrowOff], [cx+14, cy+r+arrowOff-18], [cx-14, cy+r+arrowOff-18]]);
        arrow([[cx-r-arrowOff, cy], [cx-r-arrowOff+18, cy+14], [cx-r-arrowOff+18, cy-14]]);
        arrow([[cx+r+arrowOff, cy], [cx+r+arrowOff-18, cy+14], [cx+r+arrowOff-18, cy-14]]);
    }

    function joystick(x, y) {
        joystick_background();
        // sombra
        ctx_joystick.shadowColor = 'rgba(240,190,0,0.35)';
        ctx_joystick.shadowBlur = 20;
        ctx_joystick.beginPath();
        ctx_joystick.arc(x, y, radius * 3, 0, Math.PI * 2);
        ctx_joystick.fillStyle = '#f0be00';
        ctx_joystick.fill();
        ctx_joystick.shadowBlur = 0;

        // ponto central
        ctx_joystick.beginPath();
        ctx_joystick.arc(x, y, radius * 0.8, 0, Math.PI * 2);
        ctx_joystick.fillStyle = '#c89a00';
        ctx_joystick.fill();
    }

    var coord = { x: 0, y: 0 };

    function getPosition_joystick(event) {
        var mouse_x = event.clientX || (event.touches && event.touches[0].clientX) || 0;
        var mouse_y = event.clientY || (event.touches && event.touches[0].clientY) || 0;
        coord.x = mouse_x - canvas_joystick.offsetLeft;
        coord.y = mouse_y - canvas_joystick.offsetTop;
    }

    function in_circle() {
        var r = Math.sqrt(Math.pow(coord.x - origin_joystick.x, 2) + Math.pow(coord.y - origin_joystick.y, 2));
        return r <= (radius * radius_factor);
    }

    function startDrawing(event) {
        paint = true;
        getPosition_joystick(event);
        if (in_circle()) {
            joystick(coord.x, coord.y);
            Draw(event);
        }
    }

    function stopDrawing() {
        paint = false;
        joystick(origin_joystick.x, origin_joystick.y);
        document.getElementById('dbg-vel').textContent = 0;
        document.getElementById('dbg-ang').textContent = 0;
        if (movimento === 1) {
            send_joystick(0, 0);
            movimento = 0;
        }
    }

    function Draw(event) {
        if (!paint) return;
        getPosition_joystick(event);
        var angle = Math.atan2((coord.y - origin_joystick.y), (coord.x - origin_joystick.x));
        var x, y;
        if (in_circle()) {
            x = coord.x - radius / 2;
            y = coord.y - radius / 2;
        } else {
            x = radius * radius_factor * Math.cos(angle) + origin_joystick.x;
            y = radius * radius_factor * Math.sin(angle) + origin_joystick.y;
        }
        var speed = Math.round(100 * Math.sqrt(Math.pow(x - origin_joystick.x, 2) + Math.pow(y - origin_joystick.y, 2)) / (radius * radius_factor));
        if (speed > 100) speed = 100;

        var angle_in_degrees;
        if (Math.sign(angle) === -1) {
            angle_in_degrees = Math.round(-angle * 180 / Math.PI);
        } else {
            angle_in_degrees = Math.round(360 - angle * 180 / Math.PI);
        }

        joystick(x, y);
        document.getElementById('dbg-vel').textContent = speed;
        document.getElementById('dbg-ang').textContent = angle_in_degrees;

        if ((Date.now() - last_update) > 100) {
            last_update = Date.now();
            send_joystick(speed, angle_in_degrees);
        }
        movimento = 1;
    }
</script>
</body>
</html>
)rawliteral";
   
// --------------------------------------------------
// Pagina web (ocupado)

const char html_busy[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>

<head>
    <title>
        RoboCore Joystick
    </title>

    <meta charset="UTF-8">

    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />

    <style>

        html, body {width: 100%; height: 100%; padding: 0; margin: 0; }

        body {
            overflow: hidden;
            -moz-user-select: none; 
            -webkit-user-select: none;
            -ms-user-select:none; 
            user-select:none;
            -o-user-select:none;
        }

        .container {
            height: 26px;
            width: 50px;
            position: relative;
        }

        .container * {
            position: absolute;
        }
    </style>
</head>

<body style="height: 100%;  font-family: 'Gill Sans', 'Gill Sans MT', Calibri, 'Trebuchet MS', sans-serif ;">
    <div style="line-height: 26px; background-color: black; padding: 10px; padding-bottom: 0px;">
        <div style="width: 100%; border: 0px solid red; text-align: center;">
            <svg version="1.0" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" x="0px" y="0px"

            height="30px" viewBox="0 0 1420 225" xml:space="preserve">
            <g id="Layer_1" fill="#f0be00">
                <g>
                    <path id="Robo" fill-rule="evenodd" clip-rule="evenodd" d="M175.7,94.9c0,11.7-6.2,20.2-18.5,25.6c-9.8,4-21,5.9-33.8,5.9H84.5
                    l78.8,33.5c4.2,1.8,6.3,4.6,6.3,8.2c0,5.3-4.6,8-13.9,8c-3.7,0-6.6-0.3-8.7-0.8l-107.6-46v38.4c0,5.6-2.8,8.4-8.3,8.4
                    c-5.9,0-9.3-0.3-10.2-0.7c-3.4-1.1-5.1-3.6-5.1-7.7V54.8c0-5.6,2.8-8.3,8.3-8.3h95c14.7,0,26.8,1.9,36.3,5.7
                    c13.5,5.5,20.3,14.5,20.3,27V94.9L175.7,94.9z M152.1,95.1V78.7c0-8.7-11.1-13-33.1-13H39.4v41.5h83.8
                    C142.4,107.2,152.1,103.2,152.1,95.1L152.1,95.1z M343.9,141.2c0,23.2-20,34.7-60,34.7h-36.8c-16.2,0-29-2.1-38.3-6.2
                    c-14.3-5.8-21.5-15.3-21.5-28.5V81.6c0-23.4,19.9-35.1,59.8-35.1h36.8c18.6,0,31.8,1.6,39.4,4.9c13.7,5.3,20.6,15.4,20.6,30.2
                    V141.2L343.9,141.2z M320.3,140.6V82.1c0-10.9-11.9-16.4-35.8-16.4H247c-24.1,0-36.1,5.4-36.1,16.4v58.5c0,10.6,12,15.8,36.1,15.8
                    h37.5C308.4,156.5,320.3,151.2,320.3,140.6L320.3,140.6z M512.2,141.5c0,12.4-5.9,21.6-17.6,27.5c-9.1,4.6-20.5,6.9-34.2,6.9
                    H360.2c-5.5,0-8.3-2.8-8.3-8.4V54.8c0-5.5,2.8-8.3,8.3-8.3H459c8.8,0,16.9,0.7,24.2,2.1c7.9,1.9,14.5,5.4,20,10.5
                    c6,5.7,9,12.4,9,19.9c0,7.4-0.3,12.7-0.8,16c-0.1,0.6-0.6,1.7-1.4,3.2c-0.8,1.5-1.9,3.3-3.2,5.6c-2.6,4.5-3.9,7-3.9,7.5
                    c0,0.4,1.3,2.6,3.9,6.7c1.3,2.1,2.4,3.8,3.2,5.2c0.8,1.4,1.2,2.5,1.4,3.3c0.3,1.1,0.5,2.9,0.6,5.4
                    C512.1,134.4,512.2,137.6,512.2,141.5L512.2,141.5z M489.2,90.1c0-2.4-0.2-5.9-0.7-10.8c-0.7-9-10.9-13.5-30.6-13.5H376v36.3h82.8
                    C479.1,102.1,489.2,98.1,489.2,90.1L489.2,90.1z M488.5,134.9c0-8.9-9.8-13.4-29.2-13.4h-83.8v35h82.2c19,0,29.3-4.1,30.6-12.2
                    c0.1-0.4,0.1-1.4,0.2-3C488.5,139.7,488.5,137.6,488.5,134.9L488.5,134.9z M676.8,141.2c0,23.2-20,34.7-60,34.7H580
                    c-16.2,0-29-2.1-38.3-6.2c-14.3-5.8-21.5-15.3-21.5-28.5V81.6c0-23.4,19.9-35.1,59.8-35.1h36.8c18.6,0,31.8,1.6,39.4,4.9
                    c13.7,5.3,20.6,15.4,20.6,30.2V141.2L676.8,141.2z M653.1,140.6V82.1c0-10.9-11.9-16.4-35.8-16.4h-37.4
                    c-24.1,0-36.1,5.4-36.1,16.4v58.5c0,10.6,12,15.8,36.1,15.8h37.5C641.2,156.5,653.1,151.2,653.1,140.6z"/>

                    <polygon id="Bolt" fill-rule="evenodd" clip-rule="evenodd" points="724,11.6 765,11.6 726,104.8 779.6,76.9 750.7,189.8 
                    765,182.8 733.6,230.4 722,172.8 732.9,184 744.9,111.7 689.2,140.7      "/>

                    <path id="Core" fill-rule="evenodd" clip-rule="evenodd" d="M935.5,57.4c0,5.6-2.8,8.4-8.3,8.4h-84c-20.4,0-30.6,4.1-30.6,12.2
                    v64.1c0,9.6,11.3,14.4,33.9,14.4h80.2c5.8,0,8.6,3.2,8.6,9.7c0,6.5-2.9,9.8-8.6,9.8h-80.2c-10.8,0-18.6-0.4-23.4-1.3
                    c-5.2-1-10.6-2.9-16.1-5.7c-6.4-3.3-10.8-6.8-13.4-10.4c-3-4.1-4.6-9.5-4.6-16.1V78.1c0-8.2,3.7-15.1,11.1-20.8
                    c6.2-4.8,13.8-7.9,22.7-9.6c2.3-0.4,5-0.6,8.3-0.8c3.3-0.2,7.1-0.3,11.6-0.3h84.5C932.7,46.5,935.5,50.2,935.5,57.4L935.5,57.4z
                    M1101.8,141.2c0,23.2-20,34.7-60,34.7H1005c-16.2,0-29-2.1-38.3-6.2c-14.3-5.8-21.5-15.3-21.5-28.5V81.6
                    c0-23.4,19.9-35.1,59.8-35.1h36.8c18.6,0,31.8,1.6,39.4,4.9c13.7,5.3,20.6,15.4,20.6,30.2V141.2L1101.8,141.2z M1078.1,140.6V82.1
                    c0-10.9-11.9-16.4-35.8-16.4h-37.4c-24.1,0-36.1,5.4-36.1,16.4v58.5c0,10.6,12,15.8,36.1,15.8h37.5
                    C1066.2,156.5,1078.1,151.2,1078.1,140.6L1078.1,140.6z M1267.8,94.9c0,11.7-6.2,20.2-18.5,25.6c-9.8,4-21,5.9-33.8,5.9h-38.9
                    l78.8,33.5c4.2,1.8,6.3,4.6,6.3,8.2c0,5.3-4.6,8-13.9,8c-3.7,0-6.6-0.3-8.7-0.8l-107.6-46v38.4c0,5.6-2.8,8.4-8.3,8.4
                    c-5.9,0-9.3-0.3-10.2-0.7c-3.4-1.1-5.1-3.6-5.1-7.7V54.8c0-5.6,2.8-8.3,8.3-8.3h95c14.7,0,26.8,1.9,36.3,5.7
                    c13.5,5.5,20.3,14.5,20.3,27V94.9L1267.8,94.9z M1244.2,95.1V78.7c0-8.7-11.1-13-33.1-13h-79.6v41.5h83.8
                    C1234.6,107.2,1244.2,103.2,1244.2,95.1L1244.2,95.1z M1416.5,111.2c0,6.9-2.9,10.3-8.8,10.3h-113.6v21.3
                    c0,9.1,10.1,13.7,30.3,13.7h83.3c5.5,0,8.3,2.8,8.3,8.4c0,7.4-2.8,11.1-8.3,11.1h-84.4c-12.3,0-23.6-2.2-33.9-6.6
                    c-12.6-5.7-18.9-14.3-18.9-25.7V79.5c0-6.2,2.1-11.8,6.3-16.8c9.1-10.8,25.3-16.1,48.6-16.1h23.1c5.4,0,8.2,3.6,8.2,10.8
                    c0,5.6-2.8,8.4-8.2,8.4h-3.4h-2.1h-1.2c-0.8,0-1.2,0-1.3-0.1c-1.8-0.1-3.3-0.1-4.6-0.1c-1.3,0-2.3-0.1-3.2-0.1
                    c-25.9,0-38.8,4.7-38.8,14v22.6h113.9C1413.6,102.1,1416.5,105.1,1416.5,111.2z"/>

                </g>
            </g>
        </svg>
    </div>
</div>

<div style="display: table; width:100%; height: calc(100% - 80px); border: 0px solid green;">
    <div style="padding: 10px; background-color: yellow; text-align: center;">Outro usuário já está conectado neste robô</div>
</div>
</body>

</html>
)rawliteral";

// --------------------------------------------------
// Prototipos

void configurar_servidor_web(void);
void handleWebSocketMessage(uint32_t, void *, uint8_t *, size_t);
void onEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType,
             void *, uint8_t *, size_t);

// --------------------------------------------------
// --------------------------------------------------

void setup(){
  // configura a comunicacao serial
  Serial.begin(115200);
  Serial.println("RoboCore - Vespa Rocket + RoboARM");
  Serial.println("\t(v1.2 - 20/01/2025)\n");

  // configura o LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // configura os servos
  servos[0].attach(VESPA_SERVO_S1, SERVO_MIN, SERVO_MAX);
  servos[1].attach(VESPA_SERVO_S2, SERVO_MIN, SERVO_MAX);
  servos[2].attach(VESPA_SERVO_S3, SERVO_MIN, SERVO_MAX);
  servos[3].attach(VESPA_SERVO_S4, SERVO_MIN, SERVO_MAX);
  // atualiza os motores para as posicoes iniciais
  for(uint8_t i=0 ; i < 4 ; i++){
    servos[i].write(sangulos[i].atual);
  }

  WiFi.mode(WIFI_AP);
  // Verifica a versao do pacote de placas ESP32
#if ESP_ARDUINO_VERSION_MAJOR > 2  // Arduino ESP v3.0.x
  WiFi.softAPdisconnect();
  delay(100);
  WiFi.softAP("Vespa", "12345");
  const char *mac = WiFi.softAPmacAddress().c_str(); // obtem o MAC
#else // Arduino ESP v2.0.x
  const char *mac = WiFi.macAddress().c_str(); // obtem o MAC
#endif
  Serial.println(mac);
  // configura o ponto de acesso (Access Point)
  Serial.print("Configurando a rede Wi-Fi... ");
  char ssid[] = "Vespa-xxxxx"; // mascara do SSID (ate 63 caracteres)
  char *senha = "robocore"; // senha padrao da rede (no minimo 8 caracteres)
  // atualiza o SSID em funcao do MAC
  for(uint8_t i=6 ; i < 11 ; i++){
    ssid[i] = mac[i+6];
  }
  // WiFi.mode(WIFI_AP); // NEW
  if(!WiFi.softAP(ssid, senha)){
    Serial.println("ERRO");
    // trava a execucao
    while(1){
      digitalWrite(PIN_LED, HIGH);
      delay(100);
      digitalWrite(PIN_LED, LOW);
      delay(100);
    }
  }
  Serial.println("OK");
  Serial.printf("A rede \"%s\" foi gerada\n", ssid);
  Serial.print("IP de acesso: ");
  Serial.println(WiFi.softAPIP());

  // configura e iniciar o servidor web
  configurar_servidor_web();
  server.begin();
  Serial.println("Servidor iniciado\n");
}

// --------------------------------------------------

void loop() {
  // le a tensao da bateria e envia para o cliente
  if(millis() > timeout_vbat){
    // le a tensao da bateria
    uint32_t tensao = vbat.readVoltage();

    // verificar se a tensao esta critica
    if((tensao < 7000) && (vbat_critico == 0xFF)){
      Serial.printf("Tensao critica (%u mV)\n", tensao);
      vbat_critico = LOW;
      digitalWrite(PIN_LED, vbat_critico);
      timeout_led_vbat = millis() + INTERVALO_LED_VBAT_LOW;
    } else if((tensao >= 7000) && (vbat_critico < 0xFF)){
      vbat_critico = 0xFF; // reset
      // atualiza o estado do LED em funcao da conexao ativa
      if(ws.count() > 0){
        digitalWrite(PIN_LED, HIGH);
      } else {
        digitalWrite(PIN_LED, LOW);
      }
    }
    
    // atualiza se houver clientes conectados
    if(ws.count() > 0){
      // cria a mensagem
      const int json_tamanho = JSON_OBJECT_SIZE(1); // objeto JSON com um membro
      StaticJsonDocument<json_tamanho> json;
      json[ALIAS_VBAT] = tensao;
      size_t mensagem_comprimento = measureJson(json);
      char mensagem[mensagem_comprimento + 1];
      serializeJson(json, mensagem, (mensagem_comprimento+1));
      mensagem[mensagem_comprimento] = 0; // EOS (mostly for debugging)
  
      // send the message
      ws.textAll(mensagem, mensagem_comprimento);
      Serial.printf("Tensao atualizada: %u mV\n", tensao);
    }
    
    timeout_vbat = millis() + INTERVALO_ATUALIZACAO_VBAT; // atualiza
  }

  // pisca o LED se a tensao estiver critica
  if(millis() > timeout_led_vbat){
    if(vbat_critico < 0xFF){
      if(vbat_critico == LOW){
        vbat_critico = HIGH;
        timeout_led_vbat = millis() + INTERVALO_LED_VBAT_HIGH;
      } else {
        vbat_critico = LOW;
        timeout_led_vbat = millis() + INTERVALO_LED_VBAT_LOW;
      }
      digitalWrite(PIN_LED, vbat_critico);
    }
  }

  // verifica se e para parar os motores porque nao ha clientes conectados
  if(millis() > timeout_desconexao){
    if((ws.count() == 0) && habilitar_reset_motores){
      Serial.println("Reset dos motores");
      motores.stop();
      // atualiza os motores para as posicoes iniciais
      for(uint8_t i=0 ; i < 4 ; i++){
        servos[i].write(sangulos[i].atual);
      }
      habilitar_reset_motores = false; // reset
    }
    
    timeout_desconexao = millis() + INTERVALO_ATUALIZACAO_DESCONEXAO; // atualiza
  }
}

// --------------------------------------------------
// --------------------------------------------------

// Configurar o servidor web
void configurar_servidor_web(void) {
  ws.onEvent(onEvent); // define o manipulador do evento do WebSocket
  server.addHandler(&ws); // define o manipulador do WebSocket no servidor
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ // define a resposta da pagina padrao
    if(ws.count() == 0){
      request->send_P(200, "text/html", html_index);
    } else {
      request->send_P(200, "text/html", html_busy);
    }
  });
}

// --------------------------------------------------

// Manipulador para mensagens WebSocket
//  @param (client) : ID do cliente [uint32_t]
//         (arg) : xxx [void *]
//         (data) : xxx [uint8_t *]
//         (length) : xxx [size_t]
void handleWebSocketMessage(uint32_t client, void *arg, uint8_t *data, size_t length) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == length && info->opcode == WS_TEXT) {
    data[length] = 0;

    // verifica se e para controlar os motores
    if(strstr(reinterpret_cast<char*>(data), ALIAS_VELOCIDADE) != nullptr){
      // cria um documento JSON
      const int json_tamanho = JSON_OBJECT_SIZE(2); // objeto JSON com dois membros
      StaticJsonDocument<json_tamanho> json;
      DeserializationError erro = deserializeJson(json, data, length);
      
      // extrai os valores do JSON
      int16_t angulo = json[ALIAS_ANGULO]; // [0;360]
      int16_t velocidade = json[ALIAS_VELOCIDADE]; // [0;100]

      // debug
      Serial.print("Velocidade: ");
      Serial.print(velocidade);
      Serial.print(" | Angulo: ");
      Serial.println(angulo);

      // atualiza os motores
   
      //curva frente para a esquerda
      if((angulo >= 90) && (angulo <= 180)){
        motores.turn(velocidade * (135 - angulo) / 45 , velocidade);

      //curva frente para a direita  
      } else if((angulo >= 0) && (angulo < 90)){
        motores.turn(velocidade, velocidade * (angulo - 45) / 45);

      //curva tras esquerda   
      } else if((angulo > 180) && (angulo <= 270)){
        motores.turn(-1 * velocidade, -1 * velocidade * (angulo - 225) / 45);

      //curva tras direita   
      } else if(angulo > 270){
        motores.turn(-1 * velocidade * (315 - angulo) / 45, -1 * velocidade);

      } else {
        motores.stop();
      }
   
    }
    // verifica se e para controlar um servo motor
    else if(strstr(reinterpret_cast<char*>(data), ALIAS_SERVO) != nullptr){
      // cria um documento JSON
      const int json_tamanho = JSON_OBJECT_SIZE(2); // objeto JSON com dois membros
      StaticJsonDocument<json_tamanho> json;
      DeserializationError erro = deserializeJson(json, data, length);

      // extrai os valores do JSON
      int16_t angulo = json[ALIAS_POSICAO]; // [0;180]
      int16_t servo = json[ALIAS_SERVO]; // [1-4]

      // debug
      Serial.print("Servo: ");
      Serial.print(servo);
      Serial.print(" | Angulo: ");
      Serial.println(angulo);

      // inversao de angulo dos servos "base" e "altura"
      if((servo == 4) || (servo == 3)){
        angulo = abs(angulo - 180);
      }

      // verifica os valores
      if((servo < 1) && (servo > 4)){
        Serial.printf("Servo invalido (%u)\n", servo);
        return;
      }
      if((angulo < sangulos[servo-1].min) && (angulo > sangulos[servo-1].max)){
        Serial.printf("Angulo invalido (%u)\n", servo);
        return;
      }

      // atualiza o servo
      servos[servo-1].write(angulo);
    }
    // dados invalidos
    else {
      Serial.printf("[%i] Recebidos dados invalidos (%s)\n", client, data);
    }
  } else {
    Serial.printf("[%i] Frame invalido: [%i]\n", client, info->opcode);
  }
}

// --------------------------------------------------

// Manipulador dos eventos do WebSocket
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t length) {
  switch (type) {
    case WS_EVT_CONNECT: {
      digitalWrite(PIN_LED, HIGH); // acende o LED
      // permitir apenas um cliente conectado
      if(ws.count() == 1){ // o primeiro cliente ja e considerado como conectado
        Serial.printf("Cliente WebSocket #%u conectado de %s\n", client->id(), client->remoteIP().toString().c_str());
      } else {
        Serial.printf("Cliente WebSocket #%u de %s foi rejeitado\n", client->id(), client->remoteIP().toString().c_str());
        ws.close(client->id());
      }
      break;
    }
    case WS_EVT_DISCONNECT: {
      if(ws.count() == 0){
        digitalWrite(PIN_LED, LOW); // apaga o LED
      }
      Serial.printf("Cliente WebSocket #%u desconectado\n", client->id());
      break;
    }
    case WS_EVT_DATA: {
      handleWebSocketMessage(client->id(), arg, data, length);
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// --------------------------------------------------