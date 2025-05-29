// Autobalanceador_v6.ino
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BluetoothSerial.h>
#include "PIDControl.h"

int pid_mode = 3;

MPU6050 mpu;
WebServer server(80);
BluetoothSerial SerialBT;

const int AIN1 = 0, AIN2 = 2, PWMA = 5;
const int BIN1 = 4, BIN2 = 16, PWMB = 17;
const int STBY = 15;

float setpoint = 0.0, measured_value = 0.0, dt = 0.005, output = 0;
float kp = 20.0, ki = 0.1, kd = 10.0;
PID* pid = nullptr;

const char* ssid = "ESP32_Robot";
const char* password = "12345678";

String statusInfo = "";
float desplazamientoX = 0.0, desplazamientoY = 0.0;
bool seguimientoColorActivo = false;
bool controlJoystickActivo = true;

String pageContent() {
  return R"rawliteral(
<!DOCTYPE html><html lang="es"><head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>Robot Autobalanceador</title>
  <script src='https://cdn.jsdelivr.net/npm/chart.js'></script>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #111; color: #fff; }
    h1 { text-align: center; padding: 10px 0; font-size: 22px; }
    table { width: 100%; table-layout: fixed; border-spacing: 0; }
    td:first-child { width: 33.33%; }
    td:last-child { width: 66.66%; }

    .box {
      background: #222; padding: 12px; border-radius: 12px;
      box-shadow: 0 0 10px #000; height: 100%; min-height: 240px;
    }
    .status-table {
      width: 100%; border-collapse: collapse;
    }
    .status-table th, .status-table td {
      border: 1px solid #444; padding: 8px; text-align: center;
      font-size: 16px;
    }
    input[type=number], select {
      width: 90%; margin: 4px 0;
      background: #000; color: #fff;
      border: 1px solid #555; border-radius: 6px; padding: 6px;
      font-size: 16px;
    }
    button {
      width: 100%; padding: 10px; font-size: 16px;
      border: none; background: #007aff; color: white;
      border-radius: 6px; font-weight: bold; margin-top: 8px;
    }
    canvas {
      display: block; width: 100% !important;
      height: 280px !important;
      background: #000; border-radius: 10px;
    }

    #joystick {
      width: 160px; height: 160px; margin: auto;
      background: #333; border-radius: 50%;
      position: relative; touch-action: none;
    }
    #dot {
      width: 20px; height: 20px;
      background: red; border-radius: 50%;
      position: absolute; top: 70px; left: 70px;
    }
  </style>
</head><body>
<h1>Robot Autobalanceador</h1>
<table><tr>
  <td><div class="box"><h3>Informacion</h3>
    <table id="status-table" class="status-table">
      <tr><th>Angulo</th><td>-</td></tr>
      <tr><th>Salida PID</th><td>-</td></tr>
      <tr><th>Setpoint</th><td>-</td></tr>
      <tr><th>Modo PID</th><td>-</td></tr>
      <tr><th>Estado</th><td>-</td></tr>
    </table>
  </div></td>
  <td><div class="box"><h3>Imagen</h3>
    <img id="cam" src="http://192.168.4.2/capture" width="100%" style="border-radius: 10px;"/>
  </div></td>
</tr><tr>
  <td><div class="box"><h3>Parametros PID</h3>
    Kp: <input type="number" id="kp" value="20"><br>
    Ki: <input type="number" id="ki" value="0.1"><br>
    Kd: <input type="number" id="kd" value="10"><br>
    Modo: <select id="pidmode">
      <option value="1">Discreto</option>
      <option value="2">IIR</option>
      <option value="3" selected>Filtrado</option>
    </select>
    <button onclick="setPID()">Actualizar PID</button>
  </div></td>
  <td><div class="box"><h3>Grafica</h3>
    <canvas id="chart"></canvas>
  </div></td>
</tr><tr>
  <td><div class="box"><h3>Modo</h3>
    <input type="radio" name="modo" onclick="setModo('reposo')"> Reposo<br>
    <input type="radio" name="modo" onclick="setModo('color')"> Seguimiento de Color<br>
    <input type="radio" name="modo" onclick="setModo('joystick')" checked> Joystick
  </div></td>
  <td><div class="box"><h3>Joystick</h3>
    <div id="joystick"><div id="dot"></div></div>
  </div></td>
</tr></table>

<script>
let angleData=[], pidData=[], setpointData=[], labels=[], maxPoints=20;
const ctx = document.getElementById('chart').getContext('2d');
const chart = new Chart(ctx, {
  type: 'line',
  data: {
    labels: labels,
    datasets: [
      { label: 'angulo', data: angleData, borderColor: 'lime', fill: false },
      { label: 'PID', data: pidData, borderColor: 'magenta', fill: false },
      { label: 'Setpoint', data: setpointData, borderColor: 'cyan', fill: false }
    ]
  },
  options: {
    animation: false,
    scales: { y: { suggestedMin: -90, suggestedMax: 90 } },
    plugins: { legend: { labels: { color: '#fff' } } }
  }
});

function updateChart() {
  fetch('/data').then(r => r.json()).then(d => {
    if (angleData.length >= maxPoints) {
      angleData.shift(); pidData.shift(); setpointData.shift(); labels.shift();
    }
    angleData.push(d.angle);
    pidData.push(d.pid);
    setpointData.push(d.setpoint || 0);
    labels.push('');
    chart.update();
  });
}

function updateStatus() {
  fetch('/status').then(r => r.text()).then(h => {
    document.getElementById('status-table').innerHTML = h;
  });
}

function updateImage() {
  document.getElementById('cam').src = "http://192.168.4.2/capture?ts=" + Date.now();
}

function setPID() {
  const kp = document.getElementById('kp').value;
  const ki = document.getElementById('ki').value;
  const kd = document.getElementById('kd').value;
  const mode = document.getElementById('pidmode').value;
  fetch(`/setPID?kp=${kp}&ki=${ki}&kd=${kd}&mode=${mode}`);
}

function setModo(modo) {
  if (modo === 'color') {
    fetch('/joystick/off'); fetch('/colorTracking/on');
  } else if (modo === 'joystick') {
    fetch('/colorTracking/off'); fetch('/joystick/on');
  } else {
    fetch('/colorTracking/off'); fetch('/joystick/off');
  }
}

const dot = document.getElementById('dot');
const joystick = document.getElementById('joystick');
joystick.addEventListener('pointermove', e => {
  const r = joystick.getBoundingClientRect();
  let x = e.clientX - r.left - r.width / 2;
  let y = e.clientY - r.top - r.height / 2;
  const radius = r.width / 2;
  x = Math.max(-radius, Math.min(radius, x));
  y = Math.max(-radius, Math.min(radius, y));
  dot.style.left = (radius + x - 10) + 'px';
  dot.style.top = (radius + y - 10) + 'px';
  fetch(`/desplazamiento?valueX=${-x}&valueY=${-y}`);
});
['pointerup','pointerleave'].forEach(ev =>
  joystick.addEventListener(ev, () => {
    const radius = joystick.getBoundingClientRect().width / 2;
    dot.style.left = (radius - 10) + 'px';
    dot.style.top = (radius - 10) + 'px';
    fetch('/desplazamiento?valueX=0&valueY=0');
  })
);

setInterval(updateStatus, 500);
setInterval(updateChart, 500);
setInterval(updateImage, 1000);
</script>
</body></html>
)rawliteral";
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); mpu.initialize();
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  WiFi.softAP(ssid, password);
  delay(300);
  server.begin();
  SerialBT.begin("ESP32_Robot_BT");

  server.on("/", []() { server.send(200, "text/html", pageContent()); });
  server.on("/status", []() { server.send(200, "text/html", statusInfo); });
  server.on("/desplazamiento", []() {
    if (!controlJoystickActivo) return server.send(200, "text/plain", "Joystick desactivado");
    desplazamientoY = server.arg("valueY").toFloat();
    desplazamientoX = -server.arg("valueX").toFloat();
    server.send(200, "text/plain", "Actualizado");
  });
  server.on("/data", []() {
    String json = "{";
    json += "\"angle\":" + String(measured_value, 2) + ",\"pid\":" + String(output, 2);
    json += ",\"setpoint\":" + String(setpoint, 2);
    json += ",\"kp\":" + String(kp, 2) + ",\"ki\":" + String(ki, 2) + ",\"kd\":" + String(kd, 2);
    json += ",\"mode\":" + String(pid_mode);
    json += "}";
    server.send(200, "application/json", json);
  });
  server.on("/setPID", []() {
    kp = server.arg("kp").toFloat(); ki = server.arg("ki").toFloat(); kd = server.arg("kd").toFloat();
    int mode = server.hasArg("mode") ? server.arg("mode").toInt() : 3;
    delete pid;
    pid_mode = mode;
    switch (mode) {
      case 1: pid = new PID_Discreto(kp, ki, kd, dt); break;
      case 2: pid = new PID_IIR(kp, ki, kd, dt); break;
      default: pid = new PID_Filtrado(kp, ki, kd, dt, 5.0); break;
    }
    server.send(200, "text/plain", "PID actualizado");
  });
  server.on("/colorTracking/on", []() {
    seguimientoColorActivo = true;
    controlJoystickActivo = false;
    SerialBT.println("START_COLOR");
    server.send(200, "text/plain", "Seguimiento activado");
  });
  server.on("/colorTracking/off", []() {
    seguimientoColorActivo = false;
    SerialBT.println("STOP_COLOR");
    desplazamientoX = desplazamientoY = 0;
    server.send(200, "text/plain", "Seguimiento desactivado");
  });
  server.on("/joystick/on", []() {
    controlJoystickActivo = true;
    seguimientoColorActivo = false;
    SerialBT.println("STOP_COLOR");
    server.send(200, "text/plain", "Joystick activado");
  });
  server.on("/joystick/off", []() {
    controlJoystickActivo = false;
    server.send(200, "text/plain", "Joystick desactivado");
  });

  pid = new PID_Filtrado(kp, ki, kd, dt, 5.0);
}

void applyMotorA(float pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(AIN1, pwm > 0 ? LOW : HIGH);
  digitalWrite(AIN2, pwm > 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(pwm));
}

void applyMotorB(float pwm) {
  pwm = constrain(-pwm, -255, 255);
  digitalWrite(BIN1, pwm > 0 ? LOW : HIGH);
  digitalWrite(BIN2, pwm > 0 ? HIGH : LOW);
  analogWrite(PWMB, abs(pwm));
}

void applyMotors(float pwm1, float pwm2) {
  applyMotorA(pwm1);
  applyMotorB(pwm2);
}

float getAngle() {
  int16_t ax, ay, az; mpu.getAcceleration(&ax, &ay, &az);
  return atan2(ay, az) * 180 / PI;
}

void loop() {
  static unsigned long lastControl = 0, lastServer = 0;
  unsigned long now = micros();
  if (now - lastControl >= dt * 1e6) {
    lastControl = now;
    measured_value = getAngle();
    
    // Solo ajustamos el setpoint con el joystick
    if (controlJoystickActivo) {    } else if (!seguimientoColorActivo) {
      setpoint = 0.0;  // Reposo
    }

    output = pid->compute(setpoint, measured_value);
    output = constrain(output, -255, 255);

    if (seguimientoColorActivo && SerialBT.available()) {
      String datos = SerialBT.readStringUntil('\n');
      int sep = datos.indexOf(',');
      if (sep > 0) {
        float x = datos.substring(0, sep).toFloat();
        float y = datos.substring(sep + 1).toFloat();
        desplazamientoX = (x - 160) / 1.6;
        desplazamientoY = -(y - 120) / 10;
      }
    }

    float pwmAvance = desplazamientoY * 1.75;
    float pwmGiro = - desplazamientoX * 1.75;
    applyMotors(output + pwmAvance - pwmGiro, output + pwmAvance + pwmGiro);
  }

  server.handleClient();

  if (millis() - lastServer > 10) {
    lastServer = millis();
    statusInfo = "<tr><th>angulo</th><td>" + String(measured_value, 2) + "</td></tr>";
    statusInfo += "<tr><th>Salida PID</th><td>" + String(output, 2) + "</td></tr>";
    statusInfo += "<tr><th>Setpoint</th><td>" + String(setpoint, 2) + "</td></tr>";
    statusInfo += "<tr><th>Modo PID</th><td>" + String(pid_mode) + "</td></tr>";
    statusInfo += "<tr><th>Estado</th><td>";
    statusInfo += seguimientoColorActivo ? "Color" : (controlJoystickActivo ? "Joystick" : "Reposo");
    statusInfo += "</td></tr>";
  }
}
