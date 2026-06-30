#include "Config.h"

/*
 * Globals.cpp - Dinh nghia THAT cua tat ca bien global khai bao extern trong Config.h.
 * Gia tri khoi tao giu nguyen 100% nhu ban goc final_sender_clean.
 */

// ---------------- ESP-NOW ----------------
uint8_t receiverAddress[] = {0xE0, 0x72, 0xA1, 0xE9, 0x90, 0xE4};
TelemetryPacket tele;
esp_now_peer_info_t peerInfo;

// ---------------- PINS ----------------
const int motorPins[] = {1, 4, 2, 3}; // RR, FR, RL, FL
const int rcPins[]    = {16, 15, 14, 39, 40, 41};

// ---------------- SENSOR OBJECTS ----------------
TwoWire I2C_IMU = TwoWire(1);
TwoWire I2C_TOF = TwoWire(0);
HungVo_IMU myIMU(I2C_IMU);
VL53L1X tof;
Bitcraze_PMW3901 flow(PMW_CS_PIN);
Kalman kalmanR, kalmanP;
Adafruit_NeoPixel pixels(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

// ---------------- INTER-CORE SHARED ----------------
TaskHandle_t NavigationTaskHandle;
TaskHandle_t TelemetryTaskHandle;

volatile float shared_altitudeCorrection = 0;
volatile float shared_smoothCorrection   = 0;
volatile float shared_pwm_opt_x = 0;
volatile float shared_pwm_opt_y = 0;
volatile bool  shared_pos_hold_active = false;

// ---------------- FLIGHT STATE (CORE 1) ----------------
volatile unsigned long pulseStart[6];
volatile int rcValue[6];
volatile int16_t rc4_min = 2000;

bool isArmed = false;
bool isReadyToFly = false;
unsigned long armStartTime = 0;
const int idleSpeed = 70;
const float dt = 0.004f;

float filtered_voltage = 4.2f;
float pitchOffset = 0, rollOffset = 0, yawGyroOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0;

float dynamicYawBias = 0;
float fRoll = 0, fPitch = 0, fYaw = 0;
float gz_filtered = 0;

float PAngle = 7.3f;
float PRate = 0.6f, IRate = 0.04f, DRate = 0.012f, PRateYaw = 7.5f;
float iTermRoll = 0, iTermPitch = 0, lastErrorRoll = 0, lastErrorPitch = 0, iTermYaw = 0, lastYawError = 0;

// ---------------- NAVIGATION STATE ----------------
bool altitudeHold = false;
bool lastAltHoldState = false;
String altMode = "OFF";
float tofHeight = 0, filteredHeight = 0, lastHeight = 0;
float verticalSpeed = 0, targetHeight = 0, altitudeCorrection = 0;
float P_alt = 0.45f, I_alt = 0.35f, D_alt = 0.95f, iTermAlt = 0;

float V_real_x = 0.0f, V_real_y = 0.0f;
volatile float pos_x = 0.0f, pos_y = 0.0f;
volatile int16_t deltaX = 0, deltaY = 0;
bool pmwInitOk = false;
unsigned long lastFlowTime = 0;
