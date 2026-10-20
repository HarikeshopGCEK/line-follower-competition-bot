// ============================================================
//  ESP32 16-Channel IR Line Follower — LQR-Style Experimental v1
//  New features:
//   - Lost-line recovery (spin toward last known side)
//   - Black-on-white AND white-on-black surface support
//   - Dotted/dashed line handling (coast through gaps)
//   - Browser-tunable two-state LQR-style feedback experiment
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
#define NUM_CHANNELS        16
#define IR_SENSOR_COUNT     16
#define MUX_SETTLE_DELAY_US 10
#define CONTROL_PERIOD_US   5000UL
#define MIN_CALIBRATION_SPAN 250
#define LINE_ON_THRESHOLD    180
#define LINE_OFF_THRESHOLD   120

// ── Motor driver (TB6612FNG) pins ────────────────────────────
#define ENA  33
#define IN1  32
#define IN2  4
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
int medianValues[NUM_CHANNELS];

// ── LQR-style state-feedback gains (manual experimental tuning) ─
volatile float Kp = 10.0f; // UI field kp is K_position
volatile float Ki = 0.0f;  // unused in LQR-style mode; kept for dashboard compatibility
volatile float Kd = 3.0f;  // UI field kd is K_rate
volatile int   baseSpeed = 100;
volatile int   minSpeed = 40;

float error          = 0;
float previous_error = 0;
float integral       = 0;
float derivative     = 0;
float lastCorrection = 0;   // last state-feedback control effort

// ── Lost-line / dotted-line state ────────────────────────────
float lastValidError    = 0;    // direction to spin when lost
bool  lineWasPresent    = false; // was line seen last cycle?
unsigned long lastLineSeenMs = 0; // timestamp of last line detection

// ── Dotted line: how long (ms) to coast before declaring lost ─
// Increase if your dotted gaps are wider / bot is slower
#define DOTTED_COAST_MS   120

// ── Race / lap tracking ──────────────────────────────────────
unsigned long runStartMs      = 0;   // set when /start
bool          raceFinished    = false;
unsigned long finishTimeMs    = 0;
int           lapCount        = 0;
bool          wideLineLatched = false;  // debounce for finish/lap edge
#define WIDE_LINE_ACTIVE_SENSORS 13
#define WIDE_LINE_HOLD_MS        700
unsigned long wideLineStartMs = 0;
#define LAUNCH_GRACE_MS 800   // motors stay stopped this long after /start
#define ERROR_HYST       250  // hysteresis band around threshold (ADC counts)

// ── Lost-line recovery: how long to spin before giving up ─────
#define RECOVERY_TIMEOUT_MS 2000

// ── Surface & inversion ──────────────────────────────────────
// true  = black line on white surface (sensor reads LOW on line)
// false = white line on black surface (sensor reads HIGH on line)
volatile bool blackLineOnWhite = true;

// Continuous differential steering is used; no hard tank-steer threshold.

// ── Per-sensor on-line state (for threshold hysteresis) ─────
bool sensorOnLinePrev[IR_SENSOR_COUNT] = {false};

// ── Moving average filter ────────────────────────────────────
#define FILTER_N 4
int sensorFilterBuf[IR_SENSOR_COUNT][FILTER_N] = {{0}};
int sensorFilterIdx[IR_SENSOR_COUNT]           = {0};

// Per-channel calibration validity and low-latency sensor filtering.
bool sensorCalValid[NUM_CHANNELS] = {false};
float sensorFiltered[NUM_CHANNELS] = {0.0f};
bool sensorFilterInitialized[NUM_CHANNELS] = {false};

// Web handlers only request calibration; the main loop performs it.
volatile bool calibrationRequested = false;

// Control scheduler and tracking transition state.
uint32_t lastControlUs = 0;
uint32_t nextControlTickUs = 0;
bool wasTracking = false;

// ── Sensor data shared with web handler ──────────────────────
int sensorStates[IR_SENSOR_COUNT]    = {0};
int sensorRawValues[IR_SENSOR_COUNT] = {0};
int currentError  = 0;
int activeSensors = 0;

// ── Runtime flags ─────────────────────────────────────────────
volatile bool calibrationDone = false;
volatile bool shouldRun       = false;

AsyncWebServer server(80);
Preferences prefs;

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
//  Robot must be moved over line + background during this
// ============================================================
void calibrateSensors(bool spin)
{
  // Manual calibration is safest on an unknown track and unknown motors.
  // The caller has already stopped the motors. Keep the argument for API compatibility.
  (void)spin;
  shouldRun = false;
  setMotorSpeed(0, 0);

  Serial.println("[CAL] For 5 seconds, move the sensor array across BOTH line and background.");
  Serial.println("[CAL] Starting in 2 seconds...");
  delay(2000);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
    sensorCalValid[i] = false;
  }

  const uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < 5000UL) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      selectMuxChannel((byte)i);
      (void)analogRead(MUX_OUT); // discard first conversion after mux switching
      const int raw = analogRead(MUX_OUT);
      if (raw < minValues[i]) minValues[i] = raw;
      if (raw > maxValues[i]) maxValues[i] = raw;
    }
    delay(1);
  }

  int validCount = 0;
  Serial.println("CH\tMIN\tMAX\tSPAN\tVALID");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    const int span = maxValues[i] - minValues[i];
    sensorCalValid[i] = (span >= MIN_CALIBRATION_SPAN);
    medianValues[i] = (minValues[i] + maxValues[i]) / 2;
    if (sensorCalValid[i]) validCount++;
    Serial.printf("%d\t%d\t%d\t%d\t%s\n", i, minValues[i], maxValues[i], span,
                  sensorCalValid[i] ? "YES" : "NO");
  }

  calibrationDone = (validCount >= 12);
  for (int i = 0; i < NUM_CHANNELS; i++) {
    sensorFilterInitialized[i] = false;
    sensorOnLinePrev[i] = false;
  }
  resetPIDState();

  Serial.printf("[CAL] Valid channels: %d/16. Calibration %s.\n",
                validCount, calibrationDone ? "PASSED" : "FAILED");
  if (!calibrationDone) {
    Serial.println("[CAL] Check sensor height, wiring, mux output, and line/background contrast.");
  }
}

// ============================================================
//  resetPIDState
// ============================================================
void resetPIDState()
{
  error = 0.0f;
  previous_error = 0.0f;
  integral = 0.0f;
  derivative = 0.0f;
  lastCorrection = 0.0f;
  lastValidError = 0.0f;
  lineWasPresent = false;
  lastLineSeenMs = millis();
  lastControlUs = 0;
  nextControlTickUs = 0;
  wasTracking = false;

  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    sensorFilterInitialized[i] = false;
    sensorOnLinePrev[i] = false;
    sensorStates[i] = 0;
  }
}

// ============================================================
//  getLineError
//  - Supports black-on-white and white-on-black via blackLineOnWhite
//  - Returns weighted centroid error
//  - activeSensors = 0 means no line detected this cycle
// ============================================================
float getLineError()
{
  float weightedSum = 0.0f;
  float totalStrength = 0.0f;
  activeSensors = 0;

  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    selectMuxChannel((byte)i);
    (void)analogRead(MUX_OUT); // discard first conversion after switching mux channel
    const int raw = analogRead(MUX_OUT);

    if (!sensorFilterInitialized[i]) {
      sensorFiltered[i] = (float)raw;
      sensorFilterInitialized[i] = true;
    } else {
      // Low-latency EMA; tune only after observing sensor noise.
      sensorFiltered[i] = 0.65f * raw + 0.35f * sensorFiltered[i];
    }

    const int filtered = (int)(sensorFiltered[i] + 0.5f);
    sensorRawValues[i] = filtered;

    if (!sensorCalValid[i]) {
      sensorStates[i] = 0;
      sensorOnLinePrev[i] = false;
      continue;
    }

    const int span = maxValues[i] - minValues[i];
    if (span < MIN_CALIBRATION_SPAN) {
      sensorStates[i] = 0;
      sensorOnLinePrev[i] = false;
      continue;
    }

    // Normalize each calibrated channel to 0...1000.
    const float normalized = constrain(
      1000.0f * (filtered - minValues[i]) / (float)span,
      0.0f, 1000.0f
    );

    // With black-on-white, black is normally the low raw extreme;
    // with white-on-black, white is normally the high raw extreme.
    const float lineStrength = blackLineOnWhite
      ? (1000.0f - normalized)
      : normalized;

    const bool wasOnLine = sensorOnLinePrev[i];
    const bool onLine = wasOnLine
      ? (lineStrength > LINE_OFF_THRESHOLD)
      : (lineStrength > LINE_ON_THRESHOLD);
    sensorOnLinePrev[i] = onLine;
    sensorStates[i] = onLine ? 1 : 0;
    if (onLine) activeSensors++;

    const float strength = (lineStrength > LINE_OFF_THRESHOLD)
      ? lineStrength - LINE_OFF_THRESHOLD
      : 0.0f;

    weightedSum += (float)sensorWeights[i] * strength;
    totalStrength += strength;
  }

  if (activeSensors > 0 && totalStrength > 0.0f) {
    const float err = weightedSum / totalStrength;
    currentError = (int)(err >= 0.0f ? err + 0.5f : err - 0.5f);
    lastValidError = err;
    lastLineSeenMs = millis();
    lineWasPresent = true;
    return err;
  }

  lineWasPresent = false;
  currentError = (int)lastValidError;
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
  Serial.println("[BOOT] Line Follower v3");

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

  // Do not treat uncalibrated sensors as valid.
  for (int i = 0; i < NUM_CHANNELS; i++) {
    minValues[i] = 0;
    maxValues[i] = 4095;
    medianValues[i] = 2048;
    sensorCalValid[i] = false;
  }
  calibrationDone = false;

  WiFi.softAP("LQR-Bot", "447643899");
  Serial.print("[WIFI] AP IP: ");
  Serial.println(WiFi.softAPIP());

  // ── Load saved settings from NVS ───────────────────────────
  prefs.begin("lqrbot1", false);
  Kp               = prefs.getFloat("kp",    10.0f);
  Ki               = 0.0f;
  Kd               = prefs.getFloat("kd",    3.0f);
  baseSpeed        = prefs.getInt ("speed", 100);
  minSpeed         = prefs.getInt ("minspeed", 40);
  blackLineOnWhite = prefs.getBool("blw",   true);
  Serial.printf("[LQR] K_position=%.2f K_rate=%.2f speed=%d blw=%d\n",
                (float)Kp, (float)Kd, (int)baseSpeed, (int)blackLineOnWhite);

  // ── Web dashboard ──────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = R"raw(<!DOCTYPE html>
<html>
<head>
  <title>Line Follower — LQR Experiment</title>
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
    .bgrp3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:10px}
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

    /* sensor dots */
    .sa{display:flex;gap:5px;flex-wrap:wrap;justify-content:center;margin:12px 0}
    .dot{width:28px;height:28px;border-radius:50%;border:2px solid rgba(255,255,255,0.2);
      display:flex;align-items:center;justify-content:center;
      font-size:9px;font-weight:700;transition:all .1s}
    .don{background:#00ffcc;color:#0f2027;border-color:#00ffcc;
      box-shadow:0 0 8px rgba(0,255,204,0.6)}
    .doff{background:rgba(255,255,255,0.06);color:#666}

    /* error bar */
    .ebar-wrap{width:100%;height:18px;background:rgba(0,0,0,0.3);
      border-radius:9px;overflow:hidden;margin:8px 0}
    .ebar{height:100%;width:50%;background:#00ffcc;border-radius:9px;
      transition:width .1s,background .1s}

    /* status */
    .sbar{display:flex;align-items:center;gap:10px;
      background:rgba(0,0,0,0.2);padding:10px 16px;border-radius:50px;
      margin-bottom:18px;max-width:500px;margin-left:auto;margin-right:auto}
    .sdot{width:9px;height:9px;border-radius:50%;
      background:#ef4444;box-shadow:0 0 8px #ef4444}
    .sdot.on{background:#10b981;box-shadow:0 0 8px #10b981}
    .sdot.warn{background:#f59e0b;box-shadow:0 0 8px #f59e0b}

    /* toggle */
    .tog-wrap{display:flex;align-items:center;gap:10px;margin:8px 0;font-size:.9rem}
    .tog{position:relative;width:44px;height:24px;cursor:pointer}
    .tog input{opacity:0;width:0;height:0}
    .slider-tog{position:absolute;inset:0;background:#444;border-radius:12px;transition:.3s}
    .slider-tog:before{content:'';position:absolute;width:18px;height:18px;
      left:3px;top:3px;background:#fff;border-radius:50%;transition:.3s}
    input:checked+.slider-tog{background:#00ffcc}
    input:checked+.slider-tog:before{transform:translateX(20px)}

    /* toast */
    #toast{position:fixed;bottom:28px;left:50%;transform:translateX(-50%) translateY(80px);
      background:#10b981;color:#fff;font-weight:600;
      padding:9px 22px;border-radius:50px;
      box-shadow:0 4px 18px rgba(16,185,129,.4);
      transition:transform .3s;pointer-events:none;z-index:999;font-size:.9rem}
    #toast.show{transform:translateX(-50%) translateY(0)}
  </style>
</head>
<body>
<h2>&#129302; Line Follower Pro v3</h2>

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
      <input type='range' min='0' max='20' step='0.01' id='kpS' oninput='sync("kp",this.value)'>
      <input type='number' min='0' max='20' step='0.01' class='num' id='kpN' oninput='syncS("kp",this.value,20)'>
    </div>

    <div class='lbl'>Ki (unused) <span class='v' id='kiV'>0</span></div>
    <div class='row'>
      <input type='range' min='0' max='2' step='0.001' id='kiS' oninput='sync("ki",this.value)'>
      <input type='number' min='0' max='2' step='0.001' class='num' id='kiN' oninput='syncS("ki",this.value,2)'>
    </div>

    <div class='lbl'>K rate <span class='v' id='kdV'>--</span></div>
    <div class='row'>
      <input type='range' min='0' max='10' step='0.01' id='kdS' oninput='sync("kd",this.value)'>
      <input type='number' min='0' max='10' step='0.01' class='num' id='kdN' oninput='syncS("kd",this.value,10)'>
    </div>

    <div class='bgrp'>
      <button class='bp' id='applyBtn' onclick='applyPID()'>Apply Gains</button>
      <button class='bo' onclick='resetPID()'>Defaults</button>
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
        <input type='checkbox' id='surfTog' checked onchange='setSurface(this.checked)'>
        <span class='slider-tog'></span>
      </label>
      <span id='surfLbl'>Black line on white surface</span>
    </div>

    <hr style='border-color:rgba(255,255,255,0.08);margin:14px 0'>

    <h3>&#128295; Calibration</h3>
    <p style='font-size:.82rem;color:#a0aec0;margin:0 0 10px'>
      Move bot over line + background during calibration (5 s window).
    </p>
    <button class='bo' style='width:100%' onclick='triggerCalib()'>&#128260; Start Calibration</button>
  </div>

  <!-- RIGHT CARD: Diagnostics -->
  <div class='card'>
    <h3>&#128225; Live Diagnostics</h3>

    <div class='sa' id='sa'></div>

    <div class='lbl' style='margin-top:8px'>
      Error position
      <span class='v' id='errV'>0</span>
    </div>
    <!-- Error bar: center = 50%, left = <50%, right = >50% -->
    <div class='ebar-wrap'>
      <div class='ebar' id='ebar'></div>
    </div>

    <div style='display:flex;justify-content:space-between;font-size:.75rem;color:#718096;margin-bottom:14px'>
      <span>◀ LEFT</span><span>CENTER</span><span>RIGHT ▶</span>
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

    <!-- Raw ADC bar graph -->
    <h3 style='margin-bottom:8px'>&#128202; Raw ADC Values</h3>
    <svg id='adcSvg' viewBox='0 0 480 80' style='width:100%;height:80px'></svg>
  </div>

</div>

<div id='toast'>Gains Applied</div>

<script>
  // ── Init sensor dots ──────────────────────────────────────
  (function initDots() {
    var c = document.getElementById('sa');
    for (var i = 0; i < 16; i++) {
      var d = document.createElement('div');
      d.className = 'dot doff'; d.id = 'd' + i; d.textContent = i;
      c.appendChild(d);
    }
  })();

  // ── Init ADC SVG bars ─────────────────────────────────────
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
      r.id = 'abar' + i;
      svg.appendChild(r);
      var l1 = document.createElementNS(SVG_NS, 'line');
      l1.id = 'lmn' + i; l1.setAttribute('stroke', '#f59e0b'); l1.setAttribute('stroke-width','2');
      var l2 = document.createElementNS(SVG_NS, 'line');
      l2.id = 'lmx' + i; l2.setAttribute('stroke', '#ef4444'); l2.setAttribute('stroke-width','2');
      svg.appendChild(l1); svg.appendChild(l2);
    }
  })();

  // ── Hotkey: Space = E-Stop ──────────────────────────────────
  window.addEventListener('keydown', function(e) {
    if (e.code === 'Space') { e.preventDefault(); stopRun(); }
  });

  // ── Rolling history for error graph ─────────────────────────
  var errHist = [], corrHist = [];

  // ── Freeze flag: stops poll overwriting sliders after Apply ──
  var pidFrozen = false;

  function sync(p, v) {
    var dec = p === 'ki' ? 3 : 2;
    document.getElementById(p + 'V').textContent = parseFloat(v).toFixed(dec);
    document.getElementById(p + 'N').value = parseFloat(v).toFixed(dec);
  }
  function syncS(p, v, max) {
    var n = parseFloat(v);
    if (!isNaN(n) && n >= 0 && n <= max) {
      document.getElementById(p + 'S').value = n; sync(p, v);
    }
  }

  // ── AJAX state-feedback gain apply ───────────────────────
  function applyPID() {
    var kp = document.getElementById('kpN').value;
    var ki = document.getElementById('kiN').value;
    var kd = document.getElementById('kdN').value;
    fetch('/set?kp=' + kp + '&ki=' + ki + '&kd=' + kd).then(function(r) {
      if (r.ok) {
        pidFrozen = true;
        setTimeout(function() { pidFrozen = false; }, 1200);
        var t = document.getElementById('toast');
        t.classList.add('show');
        setTimeout(function() { t.classList.remove('show'); }, 1800);
        var btn = document.getElementById('applyBtn');
        btn.textContent = 'Applied ✓';
        setTimeout(function() { btn.textContent = 'Apply Gains'; }, 1500);
      }
    });
  }

  function resetPID() {
    var d = { kp: 10.0, ki: 0.0, kd: 3.0 };
    ['kp','ki','kd'].forEach(function(p) {
      document.getElementById(p + 'S').value = d[p];
      sync(p, d[p]);
    });
  }

  function setSpeed(v) {
    document.getElementById('spV').textContent = v;
    fetch('/setSpeed?speed=' + v);
  }

  function setMinSpeed(v) {
    document.getElementById('minSpV').textContent = v;
    fetch('/setMinSpeed?minspeed=' + v);
  }

  function startRun() {
    fetch('/start').then(function(r) {
      if (!r.ok) r.text().then(function(t) { alert('Cannot start: ' + t); });
    });
  }
  function stopRun() { fetch('/stop'); }

  function setSurface(isBlackOnWhite) {
    document.getElementById('surfLbl').textContent =
      isBlackOnWhite ? 'Black line on white surface' : 'White line on black surface';
    fetch('/setSurface?mode=' + (isBlackOnWhite ? 1 : 0));
  }

  function triggerCalib() {
    document.getElementById('sdot').className = 'sdot warn';
    document.getElementById('stxt').textContent = 'Calibrating — move bot over line & background...';
    fetch('/calibrate');
  }

  // ── Poll & update UI ──────────────────────────────────────
  function updateUI(d) {
    // status
    var dot = document.getElementById('sdot');
    var txt = document.getElementById('stxt');
    dot.className = 'sdot on';

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

    document.getElementById('calibV').textContent = d.calibrationDone ? '✓ Done' : 'Pending';
    document.getElementById('calibV').style.color = d.calibrationDone ? '#00ffcc' : '#f59e0b';

    // State-feedback gain sliders (only when not frozen)
    if (!pidFrozen) {
      ['kp','ki','kd'].forEach(function(p) {
        var dec = p === 'ki' ? 3 : 2;
        document.getElementById(p + 'V').textContent = d[p].toFixed(dec);
        if (document.activeElement !== document.getElementById(p + 'S'))
          document.getElementById(p + 'S').value = d[p];
        if (document.activeElement !== document.getElementById(p + 'N'))
          document.getElementById(p + 'N').value = d[p].toFixed(dec);
      });
    }

    // Speed
    if (document.activeElement !== document.getElementById('spS')) {
      document.getElementById('spS').value = d.baseSpeed;
      document.getElementById('spV').textContent = d.baseSpeed;
    }

    if (document.activeElement !== document.getElementById('minSpS')) {
      document.getElementById('minSpS').value = d.minSpeed;
      document.getElementById('minSpV').textContent = d.minSpeed;
    }

    // Surface toggle
    document.getElementById('surfTog').checked = !!d.blackLineOnWhite;
    document.getElementById('surfLbl').textContent =
      d.blackLineOnWhite ? 'Black line on white surface' : 'White line on black surface';

    // Error & state
    document.getElementById('errV').textContent = d.error;
    document.getElementById('actV').textContent = d.activeSensors + '/16';

    var stateEl = document.getElementById('stateV');
    if (d.botState === 0)      { stateEl.textContent = 'On Line';   stateEl.style.color = '#00ffcc'; }
    else if (d.botState === 1) { stateEl.textContent = 'Coasting';  stateEl.style.color = '#f59e0b'; }
    else                       { stateEl.textContent = 'Searching'; stateEl.style.color = '#ef4444'; }

    // Error bar: map error (–75…+75) to 0–100%
    var pct = 50 + (d.error / 75) * 50;
    pct = Math.max(2, Math.min(98, pct));
    var ebar = document.getElementById('ebar');
    ebar.style.width = pct + '%';
    ebar.style.background = (Math.abs(d.error) > 40) ? '#ef4444' : '#00ffcc';

    // Sensor dots
    for (var i = 0; i < 16; i++) {
      document.getElementById('d' + i).className =
        'dot ' + (d.sensorStates[i] ? 'don' : 'doff');
    }

    // ADC bars with min/max markers
    var H = 80, W = 480, N = 16, gap = 4;
    var bw = (W - gap * (N - 1)) / N;
    for (var i = 0; i < 16; i++) {
      var bar = document.getElementById('abar' + i);
      if (!bar || !d.sensorValues) continue;
      var h = Math.max(3, Math.min(70, ((4095 - d.sensorValues[i]) / 4095) * 65));
      bar.setAttribute('height', h);
      bar.setAttribute('y', H - h - 5);
      bar.setAttribute('fill', d.sensorStates[i] ? '#00ffcc' : '#1e293b');

      if (d.sensorMin && d.sensorMax) {
        var x = i * (bw + gap);
        var hMn = ((4095 - d.sensorMin[i]) / 4095) * 65;
        var hMx = ((4095 - d.sensorMax[i]) / 4095) * 65;
        var l1 = document.getElementById('lmn' + i), l2 = document.getElementById('lmx' + i);
        if (l1) { l1.setAttribute('x1',x); l1.setAttribute('x2',x+bw);
                  l1.setAttribute('y1',H-hMn-5); l1.setAttribute('y2',H-hMn-5); }
        if (l2) { l2.setAttribute('x1',x); l2.setAttribute('x2',x+bw);
                  l2.setAttribute('y1',H-hMx-5); l2.setAttribute('y2',H-hMx-5); }
      }
    }

    // Time + laps + finished state
    document.getElementById('lapV').textContent = d.lapCount;
    var tEl = document.getElementById('timeV');
    if (d.finished) { tEl.textContent = (d.finishTimeMs/1000).toFixed(2) + 's'; }
    else if (d.running) { tEl.textContent = (d.elapsedMs/1000).toFixed(1) + 's'; }
    else { tEl.textContent = '--'; }

    if (d.finished) {
      stateEl.textContent = 'Finished'; stateEl.style.color = '#00ffcc';
    }

    // Error/correction graph
    errHist.push(d.error); corrHist.push(d.correction || 0);
    if (errHist.length > 120) { errHist.shift(); corrHist.shift(); }
    var cv = document.getElementById('errGraph');
    if (cv) {
      var c = cv.getContext('2d');
      c.clearRect(0, 0, cv.width, cv.height);
      c.strokeStyle = 'rgba(255,255,255,0.15)';
      c.beginPath(); c.moveTo(0, cv.height/2); c.lineTo(cv.width, cv.height/2); c.stroke();
      function plot(hist, color, range) {
        c.strokeStyle = color; c.lineWidth = 2; c.beginPath();
        for (var i = 0; i < hist.length; i++) {
          var x = i / 119 * cv.width;
          var y = cv.height/2 - (hist[i] / range) * (cv.height/2 - 4);
          i === 0 ? c.moveTo(x, y) : c.lineTo(x, y);
        }
        c.stroke();
      }
      plot(corrHist, '#a78bfa', 150);
      plot(errHist,  '#00ffcc', 75);
    }
  }

  function poll() {
    fetch('/sensorData')
      .then(function(r) { return r.json(); })
      .then(updateUI)
      .catch(function() {
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

  // State-feedback gains — AJAX, no redirect
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp")) Kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki")) Ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd")) Kd = request->getParam("kd")->value().toFloat();
    Serial.printf("[LQR] K_position=%.3f K_rate=%.3f\n", (float)Kp, (float)Kd);
    prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
    request->send(200, "text/plain", "ok");
  });

  // Surface mode toggle
  server.on("/setSurface", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("mode"))
      blackLineOnWhite = (request->getParam("mode")->value().toInt() == 1);
    Serial.printf("[SURFACE] blackLineOnWhite=%d\n", (int)blackLineOnWhite);
    prefs.putBool("blw", blackLineOnWhite);
    request->send(200, "text/plain", "ok");
  });

  // Sensor data JSON
  server.on("/sensorData", HTTP_GET, [](AsyncWebServerRequest *request) {
    // botState: 0=on line, 1=coasting (dotted gap), 2=searching (lost)
    int botState = 0;
    unsigned long gapMs = millis() - lastLineSeenMs;
    if (activeSensors == 0) {
      botState = (gapMs < DOTTED_COAST_MS) ? 1 : 2;
    }

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
    json += ",\"botState\":"         + String(botState);
    json += ",\"correction\":"       + String(lastCorrection, 1);
    json += ",\"lapCount\":"        + String(lapCount);
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
    json += "],\"elapsedMs\":" + String(shouldRun ? (millis() - runStartMs) : 0);
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

  server.on("/setMinSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    minSpeed = constrain(request->getParam("minspeed")->value().toInt(), 0, 150);
    prefs.putInt("minspeed", minSpeed);
    request->send(200, "text/plain", String(minSpeed));
  });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    shouldRun = false;
    calibrationDone = false;
    calibrationRequested = true;
    setMotorSpeed(0, 0);
    request->send(200, "text/plain", "calibration_requested");
  });

  server.begin();
  Serial.println("[WEB] Server started — connect to LQR-Bot / 192.168.4.1");

  Serial.println("[MODE] LQR-style two-state feedback; tune K_position and K_rate on the dashboard.");
  Serial.println("[CAL] Connect to LQR-Bot and press Calibrate before starting.");
}

// ── Bot operating state ───────────────────────────────────────
enum BotState { STATE_ON_LINE, STATE_COASTING, STATE_SEARCHING };
BotState botState = STATE_ON_LINE;

// ============================================================
//  loop
// ============================================================
void loop()
{
  // Calibration is explicit; a failed calibration must not loop forever.
  if (calibrationRequested && !shouldRun) {
    calibrationRequested = false;
    calibrateSensors(false);
    return;
  }

  if (!shouldRun) {
    setMotorSpeed(0, 0);
    delay(2);
    return;
  }

  // Nominal 200 Hz scheduler. If work overruns, resynchronize instead
  // of trying to catch up with multiple stale control iterations.
  uint32_t nowUs = micros();
  if (nextControlTickUs == 0) nextControlTickUs = nowUs;
  int32_t waitUs = (int32_t)(nextControlTickUs - nowUs);
  if (waitUs > 0) delayMicroseconds((uint32_t)waitUs);

  const uint32_t cycleUs = micros();
  nextControlTickUs += CONTROL_PERIOD_US;
  if ((int32_t)(cycleUs - nextControlTickUs) > 0) {
    nextControlTickUs = cycleUs + CONTROL_PERIOD_US;
  }

  const float currentErr = getLineError();
  const uint32_t sampleUs = micros();
  const uint32_t nowMs = millis();

  float dt = CONTROL_PERIOD_US * 1.0e-6f;
  if (lastControlUs != 0) {
    dt = (uint32_t)(sampleUs - lastControlUs) * 1.0e-6f;
  }
  lastControlUs = sampleUs;
  if (!(dt > 0.0f) || dt > 0.050f) {
    dt = CONTROL_PERIOD_US * 1.0e-6f;
    derivative = 0.0f;
    previous_error = currentErr;
  }

  const uint32_t gapMs = nowMs - lastLineSeenMs;
  if (activeSensors > 0) {
    botState = STATE_ON_LINE;
  } else if (gapMs < DOTTED_COAST_MS) {
    botState = STATE_COASTING;
  } else if (gapMs < RECOVERY_TIMEOUT_MS) {
    botState = STATE_SEARCHING;
  } else {
    setMotorSpeed(0, 0);
    shouldRun = false;
    wasTracking = false;
    Serial.println("[SAFETY] Line recovery timed out; stopped.");
    return;
  }

  if (nowMs - runStartMs < LAUNCH_GRACE_MS) {
    setMotorSpeed(0, 0);
    previous_error = currentErr;
    integral = 0.0f;
    derivative = 0.0f;
    wasTracking = false;
    return;
  }

  // Do not infer finish from a wide line: intersections can be wide too.
  // Add a marker-specific detector if your course has a unique finish marker.

  if (botState == STATE_COASTING) {
    wasTracking = false;
    integral = 0.0f;
    const int coastBase = constrain((int)(baseSpeed * 0.65f), 0, 255);
    const int left = constrain(coastBase + (int)lastCorrection, -255, 255);
    const int right = constrain(coastBase - (int)lastCorrection, -255, 255);
    setMotorSpeed(left, right);
    return;
  }

  if (botState == STATE_SEARCHING) {
    wasTracking = false;
    integral = 0.0f;
    derivative = 0.0f;
    const int searchSpeed = constrain((int)(baseSpeed * 0.45f), 45, 120);
    if (lastValidError > 0.0f) setMotorSpeed(searchSpeed, -searchSpeed);
    else setMotorSpeed(-searchSpeed, searchSpeed);
    previous_error = currentErr;
    return;
  }

  // Estimate the two available feedback states from the IR array:
  // x[0] = normalized lateral line-position error in [-1, 1]
  // x[1] = normalized rate of position-error change (no wheel encoders)
  // This is an LQR-style state-feedback experiment, NOT a model-solved
  // Riccati LQR: without encoders/heading sensing we use line position and
  // its filtered derivative as the observable states. Tune gains on track.
  if (!wasTracking) {
    previous_error = currentErr;
    derivative = 0.0f;
    wasTracking = true;
  }

  const float dtScale = constrain(dt / (CONTROL_PERIOD_US * 1.0e-6f), 0.25f, 4.0f);
  const float derivativeRaw = (currentErr - previous_error) / dtScale;
  const float derivativeTau = 0.015f;
  const float alpha = dt / (derivativeTau + dt);
  derivative += alpha * (derivativeRaw - derivative);

  const float xPosition = constrain(currentErr / 75.0f, -1.0f, 1.0f);
  const float xRate = constrain(derivative / 15.0f, -1.0f, 1.0f);

  // Positive line error means the line is to the robot's right, so positive
  // control effort increases left-wheel speed and decreases right-wheel speed.
  // Kp is used as K_position; Kd is used as K_rate. Ki is intentionally ignored.
  const float rawControl = 10.0f * ((float)Kp * xPosition + (float)Kd * xRate);
  float correction = constrain(rawControl, -150.0f, 150.0f);

  const float curveSeverity = constrain(fabsf(currentErr) / 75.0f, 0.0f, 1.0f);
  const float rateSeverity = constrain(fabsf(derivative) / 12.0f, 0.0f, 1.0f);
  const float severity = constrain(0.70f * curveSeverity + 0.30f * rateSeverity, 0.0f, 1.0f);
  float speedFactor = 1.0f - 0.55f * severity;
  if (activeSensors >= 12) speedFactor = min(speedFactor, 0.65f);

  const int requestedBase = constrain((int)(baseSpeed * speedFactor), 0, 255);
  const int safeFloor = min((int)minSpeed, (int)(baseSpeed * 0.35f));
  const int effBase = constrain(requestedBase, safeFloor, (int)baseSpeed);

  // Continuous differential steering; wheel reversal occurs only if demanded
  // by the correction, not at an arbitrary error threshold.
  const int leftSpeed = constrain(effBase + (int)correction, -255, 255);
  const int rightSpeed = constrain(effBase - (int)correction, -255, 255);
  setMotorSpeed(leftSpeed, rightSpeed);

  previous_error = currentErr;
  lastCorrection = correction;
}
