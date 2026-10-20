// ============================================================
//  ESP32 16-Channel IR Line Follower — LQR-Style v4
//  Features:
//   - Lost-line recovery (spin toward last known side)
//   - Black-on-white AND white-on-black surface support
//   - Dotted/dashed line handling (coast through gaps)
//   - Auto-spin 3-phase calibration (CW → CCW → CW sweep)
//   - Moving-average sensor filter + median+hysteresis detection
//   - Browser-tunable LQR-style state-feedback control
//   - Wide-line finish / lap detection
//   - v2.x Core API (ledcSetup/ledcAttachPin/ADC_11db)
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// ── Multiplexer (CD4067) pins ────────────────────────────────
#define S0 12
#define S1 13
#define S2 14
#define S3 15
#define MUX_OUT 34

// ── Sensor constants ─────────────────────────────────────────
#define NUM_CHANNELS         16
#define IR_SENSOR_COUNT      16
#define MUX_SETTLE_DELAY_US  100          // conservative settle for reliable reads
#define CONTROL_PERIOD_US    5000UL       // 200 Hz control loop
#define MIN_CALIBRATION_SPAN 200          // min ADC span to treat a channel as valid
#define ERROR_HYST           250          // hysteresis band (ADC counts) around median

// ── Motor driver (TB6612FNG) pins ────────────────────────────
#define ENA  33
#define IN1  32
#define IN2   4
#define ENB  27
#define IN3  26
#define IN4  25
#define STBY 21

// ── LEDC PWM — Core v2.x API ─────────────────────────────────
#define LEFT_PWM_CHANNEL  0
#define RIGHT_PWM_CHANNEL 1
#define PWM_FREQ      20000
#define PWM_RES_BITS      8

// ── Sensor weights — symmetric, no duplicate zero ─────────────
const int sensorWeights[IR_SENSOR_COUNT] = {
  -75, -65, -55, -45, -35, -25, -15, -5,
    5,  15,  25,  35,  45,  55,  65,  75
};

// ── Calibration arrays ───────────────────────────────────────
int minValues[NUM_CHANNELS];
int maxValues[NUM_CHANNELS];
int medianValues[NUM_CHANNELS];   // decision threshold per channel

// ── LQR-style state-feedback gains ───────────────────────────
volatile float Kp = 10.0f;   // K_position
volatile float Ki =  0.0f;   // intentionally unused; kept for dashboard compat
volatile float Kd =  3.0f;   // K_rate
volatile int   baseSpeed = 100;
volatile int   minSpeed  =  40;

float error          = 0;
float previous_error = 0;
float integral       = 0;
float derivative     = 0;
float lastCorrection = 0;

// ── Lost-line / dotted-line state ────────────────────────────
float lastValidError    = 0;
bool  lineWasPresent    = false;
unsigned long lastLineSeenMs = 0;

// ── Timing constants ─────────────────────────────────────────
#define DOTTED_COAST_MS      120   // coast this long before declaring line lost
#define RECOVERY_TIMEOUT_MS 2000   // spin-search timeout before stopping
#define LAUNCH_GRACE_MS      800   // motors stay stopped after /start

// ── Race / lap tracking ──────────────────────────────────────
unsigned long runStartMs      = 0;
bool          raceFinished    = false;
unsigned long finishTimeMs    = 0;
int           lapCount        = 0;
bool          wideLineLatched = false;
unsigned long wideLineStartMs = 0;
#define WIDE_LINE_ACTIVE_SENSORS 13   // sensors active to detect finish line
#define WIDE_LINE_HOLD_MS        700  // must hold this long to count

// ── Surface & inversion ──────────────────────────────────────
volatile bool blackLineOnWhite = true;

// ── Per-sensor hysteresis state ───────────────────────────────
bool sensorOnLinePrev[IR_SENSOR_COUNT] = {false};

// ── Moving average filter (4-sample) ─────────────────────────
#define FILTER_N 4
int sensorFilterBuf[IR_SENSOR_COUNT][FILTER_N] = {{0}};
int sensorFilterIdx[IR_SENSOR_COUNT]           = {0};

// ── Sensor data shared with web handler ──────────────────────
int sensorStates[IR_SENSOR_COUNT]    = {0};
int sensorRawValues[IR_SENSOR_COUNT] = {0};
int currentError  = 0;
int activeSensors = 0;

// ── Runtime flags ─────────────────────────────────────────────
volatile bool calibrationDone      = false;
volatile bool shouldRun            = false;
volatile bool calibrationRequested = false;

// ── 200 Hz control scheduler ──────────────────────────────────
uint32_t lastControlUs     = 0;
uint32_t nextControlTickUs = 0;
bool wasTracking = false;

AsyncWebServer server(80);
Preferences    prefs;

// ── Forward declarations ─────────────────────────────────────
void selectMuxChannel(byte channel);
void calibrateSensors(bool spin);
float getLineError();
void setMotorSpeed(int left, int right);
void resetPIDState();

// ============================================================
//  selectMuxChannel
// ============================================================
void selectMuxChannel(byte channel)
{
  digitalWrite(S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(S3, (channel & 0x08) ? HIGH : LOW);
  delayMicroseconds(MUX_SETTLE_DELAY_US);
}

// ============================================================
//  calibrateSensors
//  3-phase in-place rotation: CW 1.2 s → CCW 2.4 s → CW 1.4 s
//  Guarantees every sensor crosses both line and background.
// ============================================================
void calibrateSensors(bool spin)
{
  shouldRun = false;
  setMotorSpeed(0, 0);

  Serial.println("[CAL] Starting in 2 s — place bot on track first.");
  delay(2000);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }

  const uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < 5000UL) {
    const uint32_t elapsed = (uint32_t)(millis() - startMs);

    if (spin) {
      if      (elapsed < 1200) setMotorSpeed( 100, -100);   // CW
      else if (elapsed < 3600) setMotorSpeed(-100,  100);   // CCW
      else                     setMotorSpeed( 100, -100);   // CW
    }

    for (int i = 0; i < NUM_CHANNELS; i++) {
      selectMuxChannel((byte)i);
      (void)analogRead(MUX_OUT);           // discard first read after mux switch
      const int raw = analogRead(MUX_OUT);
      if (raw < minValues[i]) minValues[i] = raw;
      if (raw > maxValues[i]) maxValues[i] = raw;
    }
    delay(1);
  }
  setMotorSpeed(0, 0);

  int validCount = 0;
  Serial.println("CH\tMIN\tMAX\tMEDIAN\tVALID");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    const int span = maxValues[i] - minValues[i];
    if (span < MIN_CALIBRATION_SPAN) {
      // Poor contrast — fall back to full-range so the channel is neutral
      minValues[i] = 0;
      maxValues[i] = 4095;
    } else {
      validCount++;
    }
    medianValues[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.printf("%d\t%d\t%d\t%d\t%s\n", i, minValues[i], maxValues[i],
                  medianValues[i], (span >= MIN_CALIBRATION_SPAN) ? "YES" : "NO");
  }

  // Require at least 8 channels with usable contrast
  calibrationDone = (validCount >= 8);
  resetPIDState();

  Serial.printf("[CAL] Valid: %d/16 — %s\n", validCount,
                calibrationDone ? "PASSED" : "FAILED — re-run calibration");
}

// ============================================================
//  resetPIDState
// ============================================================
void resetPIDState()
{
  error = previous_error = integral = derivative = 0.0f;
  lastCorrection  = 0.0f;
  lastValidError  = 0.0f;
  lineWasPresent  = false;
  lastLineSeenMs  = millis();
  lastControlUs   = 0;
  nextControlTickUs = 0;
  wasTracking     = false;

  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    for (int k = 0; k < FILTER_N; k++) sensorFilterBuf[i][k] = 0;
    sensorFilterIdx[i]  = 0;
    sensorOnLinePrev[i] = false;
    sensorStates[i]     = 0;
  }
}

// ============================================================
//  getLineError
//  4-sample moving-average filter, median+hysteresis detection.
//  Returns weighted centroid error ≈ –75 … +75.
// ============================================================
float getLineError()
{
  long weightedSum = 0;
  activeSensors = 0;

  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    selectMuxChannel((byte)i);
    (void)analogRead(MUX_OUT);             // discard first read after mux switch
    const int rawValue = analogRead(MUX_OUT);

    // 4-sample moving average
    sensorFilterBuf[i][sensorFilterIdx[i]] = rawValue;
    sensorFilterIdx[i] = (sensorFilterIdx[i] + 1) % FILTER_N;
    int sum = 0;
    for (int k = 0; k < FILTER_N; k++) sum += sensorFilterBuf[i][k];
    const int filtered = sum / FILTER_N;
    sensorRawValues[i] = filtered;

    // Map filtered reading to 0–4095 using calibrated range
    int mapped;
    if (maxValues[i] - minValues[i] < MIN_CALIBRATION_SPAN) {
      mapped = filtered;   // uncalibrated channel — use raw
    } else {
      mapped = (int)constrain(
        map(filtered, minValues[i], maxValues[i], 0, 4095), 0, 4095);
    }

    // Hysteresis detection around per-channel median threshold
    const bool prevOn = sensorOnLinePrev[i];
    bool onLine;
    if (blackLineOnWhite) {
      // Black line → low mapped value
      onLine = prevOn ? (mapped < medianValues[i] + ERROR_HYST)
                      : (mapped < medianValues[i] - ERROR_HYST);
    } else {
      // White line → high mapped value
      onLine = prevOn ? (mapped > medianValues[i] - ERROR_HYST)
                      : (mapped > medianValues[i] + ERROR_HYST);
    }
    sensorOnLinePrev[i] = onLine;
    sensorStates[i]     = onLine ? 1 : 0;

    if (onLine) {
      weightedSum += sensorWeights[i];
      activeSensors++;
    }
  }

  if (activeSensors > 0) {
    const float err = (float)weightedSum / activeSensors;
    currentError   = (int)err;
    lastValidError = err;
    lastLineSeenMs = millis();
    lineWasPresent = true;
    return err;
  }

  lineWasPresent = false;
  currentError   = (int)lastValidError;
  return lastValidError;
}

// ============================================================
//  setMotorSpeed  — leftSpeed/rightSpeed: –255…+255
// ============================================================
void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  digitalWrite(STBY, HIGH);

  if (leftSpeed >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); leftSpeed = -leftSpeed; }
  ledcWrite(LEFT_PWM_CHANNEL, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); rightSpeed = -rightSpeed; }
  ledcWrite(RIGHT_PWM_CHANNEL, constrain(rightSpeed, 0, 255));
}

// ============================================================
//  setup
// ============================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("[BOOT] Line Follower LQR v4");

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Core v2.x LEDC API
  ledcSetup(LEFT_PWM_CHANNEL,  PWM_FREQ, PWM_RES_BITS);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(ENA, LEFT_PWM_CHANNEL);
  ledcAttachPin(ENB, RIGHT_PWM_CHANNEL);

  analogReadResolution(12);
  analogSetPinAttenuation(MUX_OUT, ADC_11db);

  // Safe full-range defaults until real calibration is done
  for (int i = 0; i < NUM_CHANNELS; i++) {
    minValues[i]    = 0;
    maxValues[i]    = 4095;
    medianValues[i] = 2048;
  }
  calibrationDone = false;

  WiFi.softAP("LQR-Bot", "447643899");
  Serial.print("[WIFI] AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Load saved settings from NVS
  prefs.begin("lqrbot1", false);
  Kp               = prefs.getFloat("kp",      10.0f);
  Ki               = 0.0f;   // always zero; LQR does not use integral
  Kd               = prefs.getFloat("kd",       3.0f);
  baseSpeed        = prefs.getInt  ("speed",     100);
  minSpeed         = prefs.getInt  ("minspeed",   40);
  blackLineOnWhite = prefs.getBool ("blw",       true);
  Serial.printf("[NVS] Kp=%.2f Kd=%.2f speed=%d minSpeed=%d blw=%d\n",
                (float)Kp, (float)Kd, (int)baseSpeed, (int)minSpeed, (int)blackLineOnWhite);

  // ── Web dashboard ──────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = R"raw(<!DOCTYPE html>
<html>
<head>
  <title>Line Follower — LQR v4</title>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <style>
    *{box-sizing:border-box}
    body{background:linear-gradient(135deg,#0f2027,#203a43,#2c5364);
      color:#fff;font-family:'Segoe UI',sans-serif;
      margin:0;padding:20px;min-height:100vh}
    h2{text-align:center;color:#00ffcc;margin:0 0 16px}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:18px;max-width:1100px;margin:0 auto}
    @media(max-width:800px){.grid{grid-template-columns:1fr}}
    .card{background:rgba(255,255,255,0.07);border:1px solid rgba(255,255,255,0.1);
      border-radius:18px;padding:20px;backdrop-filter:blur(10px)}
    h3{color:#00ffcc;margin:0 0 14px;font-size:1.05rem}
    .lbl{display:flex;justify-content:space-between;font-size:.88rem;
      color:#a0aec0;margin-bottom:3px}
    .lbl .v{color:#00ffcc;font-weight:700}
    .row{display:flex;gap:10px;align-items:center;margin-bottom:10px}
    input[type=range]{flex:1;accent-color:#00ffcc;height:5px}
    .num{width:78px;padding:7px;border:1px solid rgba(255,255,255,0.2);
      border-radius:8px;background:rgba(0,0,0,0.3);color:#fff;
      text-align:center;font-size:.9rem;outline:none}
    .num:focus{border-color:#00ffcc}
    .bgrp{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
    button{font-family:inherit;font-weight:600;font-size:.88rem;
      padding:10px 14px;border-radius:10px;border:none;cursor:pointer;transition:all .2s}
    .bp{background:#00ffcc;color:#0f2027}
    .bp:hover{background:#00d8b0;transform:translateY(-1px)}
    .bd{background:#ef4444;color:#fff}
    .bd:hover{background:#dc2626;transform:translateY(-1px)}
    .bo{background:rgba(255,255,255,0.08);color:#fff;
      border:1px solid rgba(255,255,255,0.15)}
    .bo:hover{border-color:#00ffcc;background:rgba(0,255,204,0.08)}
    .bg{background:#10b981;color:#fff}
    .bg:hover{background:#059669;transform:translateY(-1px)}
    button:disabled{opacity:.35;cursor:not-allowed;transform:none!important}
    .sa{display:flex;gap:5px;flex-wrap:wrap;justify-content:center;margin:12px 0}
    .dot{width:28px;height:28px;border-radius:50%;border:2px solid rgba(255,255,255,0.2);
      display:flex;align-items:center;justify-content:center;
      font-size:9px;font-weight:700;transition:all .1s}
    .don{background:#00ffcc;color:#0f2027;border-color:#00ffcc;
      box-shadow:0 0 8px rgba(0,255,204,0.6)}
    .doff{background:rgba(255,255,255,0.06);color:#666}
    .ebar-wrap{width:100%;height:18px;background:rgba(0,0,0,0.3);
      border-radius:9px;overflow:hidden;margin:8px 0}
    .ebar{height:100%;width:50%;background:#00ffcc;border-radius:9px;
      transition:width .1s,background .1s}
    .sbar{display:flex;align-items:center;gap:10px;
      background:rgba(0,0,0,0.2);padding:10px 16px;border-radius:50px;
      margin-bottom:18px;max-width:500px;margin-left:auto;margin-right:auto}
    .sdot{width:9px;height:9px;border-radius:50%;
      background:#ef4444;box-shadow:0 0 8px #ef4444}
    .sdot.on{background:#10b981;box-shadow:0 0 8px #10b981}
    .sdot.warn{background:#f59e0b;box-shadow:0 0 8px #f59e0b}
    .tog-wrap{display:flex;align-items:center;gap:10px;margin:8px 0;font-size:.9rem}
    .tog{position:relative;width:44px;height:24px;cursor:pointer}
    .tog input{opacity:0;width:0;height:0}
    .slider-tog{position:absolute;inset:0;background:#444;border-radius:12px;transition:.3s}
    .slider-tog:before{content:'';position:absolute;width:18px;height:18px;
      left:3px;top:3px;background:#fff;border-radius:50%;transition:.3s}
    input:checked+.slider-tog{background:#00ffcc}
    input:checked+.slider-tog:before{transform:translateX(20px)}
    .ki-row{opacity:.4;pointer-events:none}
    #toast{position:fixed;bottom:28px;left:50%;transform:translateX(-50%) translateY(80px);
      background:#10b981;color:#fff;font-weight:600;
      padding:9px 22px;border-radius:50px;
      box-shadow:0 4px 18px rgba(16,185,129,.4);
      transition:transform .3s;pointer-events:none;z-index:999;font-size:.9rem}
    #toast.show{transform:translateX(-50%) translateY(0)}
  </style>
</head>
<body>
<h2>&#129302; Line Follower LQR v4</h2>

<div class='sbar'>
  <div id='sdot' class='sdot'></div>
  <span id='stxt' style='font-size:.9rem;font-weight:500'>Connecting...</span>
</div>

<div class='grid'>

  <!-- LEFT CARD: Controls -->
  <div class='card'>
    <h3>&#9881; LQR-Style Gain Tuning</h3>

    <div class='lbl'>K position <span class='v' id='kpV'>--</span></div>
    <div class='row'>
      <input type='range' min='0' max='20' step='0.01' id='kpS' oninput='sync("kp",this.value);freezePID()'>
      <input type='number' min='0' max='20' step='0.01' class='num' id='kpN' oninput='syncS("kp",this.value,20);freezePID()'>
    </div>

    <div class='lbl ki-row'>Ki <span class='v' id='kiV'>0.000</span><span style='font-size:.75rem;color:#718096'>(unused)</span></div>
    <div class='row ki-row'>
      <input type='range' min='0' max='2' step='0.001' id='kiS' tabindex='-1'>
      <input type='number' min='0' max='2' step='0.001' class='num' id='kiN' tabindex='-1'>
    </div>

    <div class='lbl'>K rate <span class='v' id='kdV'>--</span></div>
    <div class='row'>
      <input type='range' min='0' max='10' step='0.01' id='kdS' oninput='sync("kd",this.value);freezePID()'>
      <input type='number' min='0' max='10' step='0.01' class='num' id='kdN' oninput='syncS("kd",this.value,10);freezePID()'>
    </div>

    <div class='bgrp'>
      <button class='bp' id='applyBtn' onclick='applyPID()'>Apply Gains</button>
      <button class='bo' onclick='resetGains()'>Defaults</button>
    </div>

    <hr style='border-color:rgba(255,255,255,0.08);margin:14px 0'>

    <h3>&#127947; Drive</h3>
    <div class='lbl'>Base Speed <span class='v' id='spV'>--</span></div>
    <div class='row'>
      <input type='range' min='0' max='255' step='1' id='spS' oninput='setSpeed(this.value)'>
    </div>

    <div class='lbl'>Min Curve Speed <span class='v' id='minSpV'>--</span></div>
    <div class='row'>
      <input type='range' min='0' max='150' step='1' id='minSpS' oninput='setMinSpeed(this.value)'>
    </div>

    <div class='bgrp'>
      <button id='startBtn' class='bg' onclick='startRun()' disabled>&#9654; Start</button>
      <button class='bd' onclick='stopRun()'>&#9632; Stop</button>
    </div>

    <hr style='border-color:rgba(255,255,255,0.08);margin:14px 0'>

    <h3>&#128295; Surface Mode</h3>
    <div class='tog-wrap'>
      <label class='tog'>
        <input type='checkbox' id='surfTog' checked onchange='setSurface(this.checked);freezeSurface()'>
        <span class='slider-tog'></span>
      </label>
      <span id='surfLbl'>Black line on white surface</span>
    </div>

    <hr style='border-color:rgba(255,255,255,0.08);margin:14px 0'>

    <h3>&#128295; Calibration</h3>
    <p style='font-size:.82rem;color:#a0aec0;margin:0 0 10px'>
      Bot spins automatically during the 5 s sweep. Place it on the track first.
    </p>
    <button class='bo' style='width:100%' onclick='triggerCalib()'>&#128260; Start Calibration</button>
  </div>

  <!-- RIGHT CARD: Diagnostics -->
  <div class='card'>
    <h3>&#128225; Live Diagnostics</h3>

    <div class='sa' id='sa'></div>

    <div class='lbl' style='margin-top:8px'>
      Error position <span class='v' id='errV'>0</span>
    </div>
    <div class='ebar-wrap'>
      <div class='ebar' id='ebar'></div>
    </div>
    <div style='display:flex;justify-content:space-between;font-size:.75rem;color:#718096;margin-bottom:14px'>
      <span>&#9664; LEFT</span><span>CENTER</span><span>RIGHT &#9654;</span>
    </div>

    <div style='display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-bottom:14px'>
      <div style='background:rgba(0,0,0,0.2);padding:10px;border-radius:10px;text-align:center'>
        <div style='font-size:.7rem;color:#a0aec0;text-transform:uppercase'>Active</div>
        <div style='font-size:1.1rem;font-weight:700;color:#00ffcc' id='actV'>0/16</div>
      </div>
      <div style='background:rgba(0,0,0,0.2);padding:10px;border-radius:10px;text-align:center'>
        <div style='font-size:.7rem;color:#a0aec0;text-transform:uppercase'>State</div>
        <div style='font-size:1.1rem;font-weight:700' id='stateV'>--</div>
      </div>
      <div style='background:rgba(0,0,0,0.2);padding:10px;border-radius:10px;text-align:center'>
        <div style='font-size:.7rem;color:#a0aec0;text-transform:uppercase'>Calib</div>
        <div style='font-size:1.1rem;font-weight:700' id='calibV'>--</div>
      </div>
    </div>

    <div style='display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:14px'>
      <div style='background:rgba(0,0,0,0.2);padding:10px;border-radius:10px;text-align:center'>
        <div style='font-size:.7rem;color:#a0aec0;text-transform:uppercase'>Time</div>
        <div style='font-size:1.1rem;font-weight:700;color:#00ffcc' id='timeV'>--</div>
      </div>
      <div style='background:rgba(0,0,0,0.2);padding:10px;border-radius:10px;text-align:center'>
        <div style='font-size:.7rem;color:#a0aec0;text-transform:uppercase'>Laps</div>
        <div style='font-size:1.1rem;font-weight:700;color:#00ffcc' id='lapV'>0</div>
      </div>
    </div>

    <h3 style='margin-bottom:8px'>&#128200; Error / Correction</h3>
    <canvas id='errGraph' width='460' height='80'
      style='width:100%;height:80px;background:rgba(0,0,0,0.2);border-radius:10px;margin-bottom:14px'></canvas>

    <h3 style='margin-bottom:8px'>&#128202; Raw ADC Values</h3>
    <svg id='adcSvg' viewBox='0 0 480 80' style='width:100%;height:80px'></svg>
  </div>

</div>

<div id='toast'>Gains Applied &#10003;</div>

<script>
  (function initDots() {
    var c = document.getElementById('sa');
    for (var i = 0; i < 16; i++) {
      var d = document.createElement('div');
      d.className = 'dot doff'; d.id = 'd' + i; d.textContent = i;
      c.appendChild(d);
    }
  })();

  var SVG_NS = "http://www.w3.org/2000/svg";
  (function initSvg() {
    var svg = document.getElementById('adcSvg');
    var W = 480, H = 80, N = 16, gap = 4;
    var bw = (W - gap * (N - 1)) / N;
    for (var i = 0; i < N; i++) {
      var x = i * (bw + gap);
      var r = document.createElementNS(SVG_NS, 'rect');
      r.setAttribute('x', x); r.setAttribute('y', H);
      r.setAttribute('width', bw); r.setAttribute('height', 0);
      r.setAttribute('rx', 3); r.setAttribute('fill', '#1e293b');
      r.id = 'abar' + i; svg.appendChild(r);
      var l1 = document.createElementNS(SVG_NS, 'line');
      l1.id = 'lmn' + i; l1.setAttribute('stroke','#f59e0b'); l1.setAttribute('stroke-width','2');
      var l2 = document.createElementNS(SVG_NS, 'line');
      l2.id = 'lmx' + i; l2.setAttribute('stroke','#ef4444'); l2.setAttribute('stroke-width','2');
      svg.appendChild(l1); svg.appendChild(l2);
    }
  })();

  window.addEventListener('keydown', function(e) {
    if (e.code === 'Space') { e.preventDefault(); stopRun(); }
  });

  var errHist = [], corrHist = [];

  // Freeze flags — prevent the 150 ms poll from overwriting controls while editing
  var pidFrozen = false, speedFrozen = false, minSpeedFrozen = false, surfaceFrozen = false;
  var _pidT = null, _spT = null, _minT = null, _surT = null;

  function freezePID()      { pidFrozen=true;      clearTimeout(_pidT); _pidT=setTimeout(function(){pidFrozen=false;},      1200); }
  function freezeSpeed()    { speedFrozen=true;    clearTimeout(_spT);  _spT =setTimeout(function(){speedFrozen=false;},    1000); }
  function freezeMinSpeed() { minSpeedFrozen=true; clearTimeout(_minT); _minT=setTimeout(function(){minSpeedFrozen=false;}, 1000); }
  function freezeSurface()  { surfaceFrozen=true;  clearTimeout(_surT); _surT=setTimeout(function(){surfaceFrozen=false;},  1000); }

  function sync(p, v) {
    var dec = p === 'ki' ? 3 : 2;
    document.getElementById(p+'V').textContent = parseFloat(v).toFixed(dec);
    document.getElementById(p+'N').value       = parseFloat(v).toFixed(dec);
  }
  function syncS(p, v, max) {
    var n = parseFloat(v);
    if (!isNaN(n) && n >= 0 && n <= max) { document.getElementById(p+'S').value = n; sync(p, v); }
  }

  function applyPID() {
    var kp = document.getElementById('kpN').value;
    var kd = document.getElementById('kdN').value;
    fetch('/set?kp=' + kp + '&ki=0&kd=' + kd).then(function(r) {
      if (r.ok) {
        freezePID();
        var t = document.getElementById('toast');
        t.classList.add('show');
        setTimeout(function(){ t.classList.remove('show'); }, 1800);
        var btn = document.getElementById('applyBtn');
        btn.textContent = 'Applied \u2713';
        setTimeout(function(){ btn.textContent = 'Apply Gains'; }, 1500);
      }
    });
  }

  function resetGains() {
    var d = { kp: 10.0, ki: 0.0, kd: 3.0 };
    ['kp','ki','kd'].forEach(function(p) {
      document.getElementById(p+'S').value = d[p]; sync(p, d[p]);
    });
  }

  function setSpeed(v)    { document.getElementById('spV').textContent=v;    freezeSpeed();    fetch('/setSpeed?speed='+v); }
  function setMinSpeed(v) { document.getElementById('minSpV').textContent=v; freezeMinSpeed(); fetch('/setMinSpeed?minspeed='+v); }
  function startRun()     { fetch('/start').then(function(r){ if(!r.ok) r.text().then(function(t){ alert('Cannot start: '+t); }); }); }
  function stopRun()      { fetch('/stop'); }

  function setSurface(isBlack) {
    document.getElementById('surfLbl').textContent =
      isBlack ? 'Black line on white surface' : 'White line on black surface';
    fetch('/setSurface?mode=' + (isBlack ? 1 : 0));
  }

  function triggerCalib() {
    document.getElementById('sdot').className = 'sdot warn';
    document.getElementById('stxt').textContent = 'Calibrating \u2014 bot is spinning\u2026';
    fetch('/calibrate');
  }

  function updateUI(d) {
    document.getElementById('sdot').className = 'sdot on';
    var txt = document.getElementById('stxt');

    if (!d.calibrationDone) {
      txt.textContent = 'Awaiting Calibration';
      document.getElementById('startBtn').disabled = true;
    } else if (d.running) {
      txt.textContent = 'Bot Running';
      document.getElementById('startBtn').style.display = 'none';
    } else {
      txt.textContent = 'Ready';
      document.getElementById('startBtn').disabled = false;
      document.getElementById('startBtn').style.display = 'inline-block';
    }

    document.getElementById('calibV').textContent = d.calibrationDone ? '\u2713 Done' : 'Pending';
    document.getElementById('calibV').style.color = d.calibrationDone ? '#00ffcc' : '#f59e0b';

    if (!pidFrozen) {
      ['kp','ki','kd'].forEach(function(p) {
        var dec = p === 'ki' ? 3 : 2;
        document.getElementById(p+'V').textContent = d[p].toFixed(dec);
        if (document.activeElement !== document.getElementById(p+'S'))
          document.getElementById(p+'S').value = d[p];
        if (document.activeElement !== document.getElementById(p+'N'))
          document.getElementById(p+'N').value = d[p].toFixed(dec);
      });
    }

    if (!speedFrozen && document.activeElement !== document.getElementById('spS')) {
      document.getElementById('spS').value = d.baseSpeed;
      document.getElementById('spV').textContent = d.baseSpeed;
    }
    if (!minSpeedFrozen && document.activeElement !== document.getElementById('minSpS')) {
      document.getElementById('minSpS').value = d.minSpeed;
      document.getElementById('minSpV').textContent = d.minSpeed;
    }
    if (!surfaceFrozen) {
      document.getElementById('surfTog').checked = !!d.blackLineOnWhite;
      document.getElementById('surfLbl').textContent =
        d.blackLineOnWhite ? 'Black line on white surface' : 'White line on black surface';
    }

    document.getElementById('errV').textContent = d.error;
    document.getElementById('actV').textContent = d.activeSensors + '/16';

    var stateEl = document.getElementById('stateV');
    if (d.botState === 0)      { stateEl.textContent = 'On Line';   stateEl.style.color = '#00ffcc'; }
    else if (d.botState === 1) { stateEl.textContent = 'Coasting';  stateEl.style.color = '#f59e0b'; }
    else                       { stateEl.textContent = 'Searching'; stateEl.style.color = '#ef4444'; }

    var pct = 50 + (d.error / 75) * 50;
    pct = Math.max(2, Math.min(98, pct));
    var ebar = document.getElementById('ebar');
    ebar.style.width = pct + '%';
    ebar.style.background = (Math.abs(d.error) > 40) ? '#ef4444' : '#00ffcc';

    for (var i = 0; i < 16; i++)
      document.getElementById('d'+i).className = 'dot ' + (d.sensorStates[i] ? 'don' : 'doff');

    var H=80, W=480, N=16, gap=4, bw=(W-gap*(N-1))/N;
    for (var i = 0; i < 16; i++) {
      var bar = document.getElementById('abar'+i);
      if (!bar || !d.sensorValues) continue;
      var h = Math.max(3, Math.min(70, ((4095-d.sensorValues[i])/4095)*65));
      bar.setAttribute('height', h);
      bar.setAttribute('y', H-h-5);
      bar.setAttribute('fill', d.sensorStates[i] ? '#00ffcc' : '#1e293b');
      if (d.sensorMin && d.sensorMax) {
        var x=i*(bw+gap);
        var hMn=((4095-d.sensorMin[i])/4095)*65, hMx=((4095-d.sensorMax[i])/4095)*65;
        var l1=document.getElementById('lmn'+i), l2=document.getElementById('lmx'+i);
        if(l1){l1.setAttribute('x1',x);l1.setAttribute('x2',x+bw);l1.setAttribute('y1',H-hMn-5);l1.setAttribute('y2',H-hMn-5);}
        if(l2){l2.setAttribute('x1',x);l2.setAttribute('x2',x+bw);l2.setAttribute('y1',H-hMx-5);l2.setAttribute('y2',H-hMx-5);}
      }
    }

    document.getElementById('lapV').textContent = d.lapCount;
    var tEl = document.getElementById('timeV');
    if (d.finished)     { tEl.textContent = (d.finishTimeMs/1000).toFixed(2)+'s'; }
    else if (d.running) { tEl.textContent = (d.elapsedMs/1000).toFixed(1)+'s'; }
    else                { tEl.textContent = '--'; }
    if (d.finished) { stateEl.textContent = 'Finished'; stateEl.style.color = '#00ffcc'; }

    errHist.push(d.error); corrHist.push(d.correction || 0);
    if (errHist.length > 120) { errHist.shift(); corrHist.shift(); }
    var cv = document.getElementById('errGraph');
    if (cv) {
      var c = cv.getContext('2d');
      c.clearRect(0, 0, cv.width, cv.height);
      c.strokeStyle = 'rgba(255,255,255,0.15)';
      c.beginPath(); c.moveTo(0,cv.height/2); c.lineTo(cv.width,cv.height/2); c.stroke();
      function plot(hist,color,range){
        c.strokeStyle=color; c.lineWidth=2; c.beginPath();
        for(var i=0;i<hist.length;i++){
          var x=i/119*cv.width, y=cv.height/2-(hist[i]/range)*(cv.height/2-4);
          i===0?c.moveTo(x,y):c.lineTo(x,y);
        }
        c.stroke();
      }
      plot(corrHist,'#a78bfa',150);
      plot(errHist, '#00ffcc', 75);
    }
  }

  function poll() {
    fetch('/sensorData')
      .then(function(r){ return r.json(); })
      .then(updateUI)
      .catch(function(){
        document.getElementById('sdot').className = 'sdot';
        document.getElementById('stxt').textContent = 'Disconnected';
      });
  }

  window.onload = function() { setInterval(poll, 150); };
</script>
</body>
</html>)raw";
    request->send(200, "text/html", html);
  });

  // LQR gains — AJAX (Ki always forced to 0)
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();
    Ki = 0.0f;
    Serial.printf("[LQR] K_pos=%.3f K_rate=%.3f\n", (float)Kp, (float)Kd);
    prefs.putFloat("kp", Kp); prefs.putFloat("kd", Kd);
    request->send(200, "text/plain", "ok");
  });

  // Surface mode
  server.on("/setSurface", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("mode"))
      blackLineOnWhite = (request->getParam("mode")->value().toInt() == 1);
    prefs.putBool("blw", blackLineOnWhite);
    Serial.printf("[SURFACE] blackLineOnWhite=%d\n", (int)blackLineOnWhite);
    request->send(200, "text/plain", "ok");
  });

  // Sensor data JSON — uses local webBotState to avoid name collision with enum BotState
  server.on("/sensorData", HTTP_GET, [](AsyncWebServerRequest *request) {
    int webBotState = 0;
    unsigned long gapMs = millis() - lastLineSeenMs;
    if (activeSensors == 0)
      webBotState = (gapMs < DOTTED_COAST_MS) ? 1 : 2;

    String json = "{\"sensorStates\":[";
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      json += String(sensorStates[i]);
      if (i < IR_SENSOR_COUNT - 1) json += ",";
    }
    json += "],\"sensorValues\":[";
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      json += String(sensorRawValues[i]);
      if (i < IR_SENSOR_COUNT - 1) json += ",";
    }
    json += "],\"error\":"           + String(currentError);
    json += ",\"activeSensors\":"    + String(activeSensors);
    json += ",\"kp\":"               + String(Kp, 4);
    json += ",\"ki\":"               + String(Ki, 5);
    json += ",\"kd\":"               + String(Kd, 4);
    json += ",\"calibrationDone\":"  + String(calibrationDone ? 1 : 0);
    json += ",\"running\":"          + String(shouldRun ? 1 : 0);
    json += ",\"baseSpeed\":"        + String(baseSpeed);
    json += ",\"minSpeed\":"         + String(minSpeed);
    json += ",\"blackLineOnWhite\":" + String(blackLineOnWhite ? 1 : 0);
    json += ",\"botState\":"         + String(webBotState);
    json += ",\"correction\":"       + String(lastCorrection, 1);
    json += ",\"lapCount\":"         + String(lapCount);
    json += ",\"finished\":"         + String(raceFinished ? 1 : 0);
    json += ",\"finishTimeMs\":"     + String(finishTimeMs);
    json += ",\"sensorMin\":[";
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      json += String(minValues[i]);
      if (i < IR_SENSOR_COUNT - 1) json += ",";
    }
    json += "],\"sensorMax\":[";
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      json += String(maxValues[i]);
      if (i < IR_SENSOR_COUNT - 1) json += ",";
    }
    json += "],\"elapsedMs\":" + String(shouldRun ? (millis() - runStartMs) : 0UL);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!calibrationDone) {
      request->send(400, "text/plain", "calibration_not_done");
      return;
    }
    resetPIDState();
    lapCount        = 0;
    raceFinished    = false;
    finishTimeMs    = 0;
    wideLineLatched = false;
    runStartMs      = millis();
    shouldRun       = true;
    request->send(200, "text/plain", "started");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
    shouldRun = false;
    setMotorSpeed(0, 0);
    request->send(200, "text/plain", "stopped");
  });

  server.on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("speed"))
      baseSpeed = constrain(request->getParam("speed")->value().toInt(), 0, 255);
    prefs.putInt("speed", baseSpeed);
    request->send(200, "text/plain", String(baseSpeed));
  });

  // hasParam guard prevents crash when parameter is missing
  server.on("/setMinSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("minspeed"))
      minSpeed = constrain(request->getParam("minspeed")->value().toInt(), 0, 150);
    prefs.putInt("minspeed", minSpeed);
    request->send(200, "text/plain", String(minSpeed));
  });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    shouldRun            = false;
    calibrationDone      = false;
    calibrationRequested = true;
    setMotorSpeed(0, 0);
    request->send(200, "text/plain", "calibration_requested");
  });

  server.begin();
  Serial.println("[WEB] Server started — connect to LQR-Bot / 192.168.4.1");
  Serial.println("[CAL] Press Calibrate on the dashboard before starting.");
}

// ── Bot operating state enum ──────────────────────────────────
enum BotState { STATE_ON_LINE, STATE_COASTING, STATE_SEARCHING };
BotState botState = STATE_ON_LINE;

// ============================================================
//  loop
// ============================================================
void loop()
{
  // Web-triggered calibration (flag set by /calibrate handler)
  if (calibrationRequested && !shouldRun) {
    calibrationRequested = false;
    calibrateSensors(true);   // spin=true: 3-phase auto-sweep
    return;
  }

  if (!shouldRun) {
    setMotorSpeed(0, 0);
    delay(2);
    return;
  }

  // ── 200 Hz control scheduler ─────────────────────────────
  uint32_t nowUs = micros();
  if (nextControlTickUs == 0) nextControlTickUs = nowUs;
  int32_t waitUs = (int32_t)(nextControlTickUs - nowUs);
  if (waitUs > 0) delayMicroseconds((uint32_t)waitUs);

  const uint32_t cycleUs = micros();
  nextControlTickUs += CONTROL_PERIOD_US;
  if ((int32_t)(cycleUs - nextControlTickUs) > 0)
    nextControlTickUs = cycleUs + CONTROL_PERIOD_US;   // resync if overrun

  const float currentErr  = getLineError();
  const uint32_t sampleUs = micros();
  const uint32_t nowMs    = millis();

  float dt = CONTROL_PERIOD_US * 1.0e-6f;
  if (lastControlUs != 0)
    dt = (float)(uint32_t)(sampleUs - lastControlUs) * 1.0e-6f;
  lastControlUs = sampleUs;
  if (!(dt > 0.0f) || dt > 0.050f) {
    dt             = CONTROL_PERIOD_US * 1.0e-6f;
    derivative     = 0.0f;
    previous_error = currentErr;
  }

  // ── State machine ─────────────────────────────────────────
  const uint32_t gapMs = nowMs - lastLineSeenMs;
  if (activeSensors > 0) {
    botState = STATE_ON_LINE;
  } else if (gapMs < DOTTED_COAST_MS) {
    botState = STATE_COASTING;
  } else if (gapMs < RECOVERY_TIMEOUT_MS) {
    botState = STATE_SEARCHING;
  } else {
    setMotorSpeed(0, 0);
    shouldRun   = false;
    wasTracking = false;
    Serial.println("[SAFETY] Line recovery timed out; stopped.");
    return;
  }

  // ── Launch grace ──────────────────────────────────────────
  if (nowMs - runStartMs < LAUNCH_GRACE_MS) {
    setMotorSpeed(0, 0);
    previous_error = currentErr;
    integral       = 0.0f;
    derivative     = 0.0f;
    wasTracking    = false;
    return;
  }

  // ── Finish / lap detection (sustained wide dark area) ─────
  if (activeSensors >= WIDE_LINE_ACTIVE_SENSORS && (nowMs - runStartMs) > 3000UL) {
    if (!wideLineLatched) { wideLineStartMs = nowMs; wideLineLatched = true; }
    if ((nowMs - wideLineStartMs) >= WIDE_LINE_HOLD_MS && !raceFinished) {
      lapCount++;
      raceFinished = true;
      finishTimeMs = nowMs - runStartMs;
      shouldRun    = false;
      setMotorSpeed(0, 0);
      Serial.printf("[RACE] Finished! Time=%lu ms  Laps=%d\n", finishTimeMs, lapCount);
      return;
    }
  } else {
    wideLineLatched = false;
  }

  // ── COASTING: hold last correction to stay on trajectory ──
  if (botState == STATE_COASTING) {
    wasTracking = false;
    integral    = 0.0f;
    const int coastBase = constrain((int)(baseSpeed * 0.65f), 0, 255);
    setMotorSpeed(constrain(coastBase + (int)lastCorrection, -255, 255),
                  constrain(coastBase - (int)lastCorrection, -255, 255));
    return;
  }

  // ── SEARCHING: spin toward last known error direction ──────
  if (botState == STATE_SEARCHING) {
    wasTracking = false;
    integral    = 0.0f;
    derivative  = 0.0f;
    const int searchSpeed = constrain((int)(baseSpeed * 0.45f), 45, 120);
    if (lastValidError > 0.0f) setMotorSpeed( searchSpeed, -searchSpeed);
    else                        setMotorSpeed(-searchSpeed,  searchSpeed);
    previous_error = currentErr;
    return;
  }

  // ── ON LINE: LQR-style two-state feedback ─────────────────
  // Observable states:
  //   x[0] = lateral position error, normalised to [-1, 1]
  //   x[1] = rate of position-error change, normalised to [-1, 1]
  // Control law: u = K_pos * x[0] + K_rate * x[1]  (scaled × 10)
  if (!wasTracking) {
    previous_error = currentErr;
    derivative     = 0.0f;
    wasTracking    = true;
  }

  const float dtScale       = constrain(dt / (CONTROL_PERIOD_US * 1.0e-6f), 0.25f, 4.0f);
  const float derivativeRaw = (currentErr - previous_error) / dtScale;
  const float derivativeTau = 0.015f;
  const float alpha         = dt / (derivativeTau + dt);
  derivative += alpha * (derivativeRaw - derivative);   // EMA derivative filter

  const float xPosition = constrain(currentErr / 75.0f, -1.0f, 1.0f);
  const float xRate     = constrain(derivative  / 30.0f, -1.0f, 1.0f);  // wider band → smoother Kd

  const float rawControl = 10.0f * ((float)Kp * xPosition + (float)Kd * xRate);
  const float correction = constrain(rawControl, -150.0f, 150.0f);

  // Adaptive base speed — slow proportionally to combined curve/rate severity
  const float curveSeverity = constrain(fabsf(currentErr) / 75.0f, 0.0f, 1.0f);
  const float rateSeverity  = constrain(fabsf(derivative)  / 30.0f, 0.0f, 1.0f);
  const float severity      = constrain(0.70f * curveSeverity + 0.30f * rateSeverity, 0.0f, 1.0f);
  const float speedFactor   = 1.0f - 0.55f * severity;

  const int requestedBase = constrain((int)(baseSpeed * speedFactor), 0, 255);
  const int safeFloor     = min((int)minSpeed, (int)(baseSpeed * 0.35f));
  const int effBase       = constrain(requestedBase, safeFloor, (int)baseSpeed);

  setMotorSpeed(constrain(effBase + (int)correction, -255, 255),
                constrain(effBase - (int)correction, -255, 255));

  previous_error = currentErr;
  lastCorrection = correction;
}
