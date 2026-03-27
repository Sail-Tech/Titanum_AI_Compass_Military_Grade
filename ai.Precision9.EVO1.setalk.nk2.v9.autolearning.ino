/**
 * HIGH-PRECISION NAVIGATION SYSTEM (PRECISION-9 / EVO1 EMULATION)
 * Version: 10.0 MIL-SPEC (Military Spec) - Final, Audited & Bulletproof (English UI)
 * Authors: ICM-20948 + Bernd Cirotzki + Martin Weigel + Bareboat Math + BME280
 * Hardware: ESP32 + ICM-20948 + BME280 + CAN Transceiver
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include <ICM_20948.h>
#include <Adafruit_BME280.h> 
#include <deque>
#include <vector>
#include <math.h>

// --- BLE Libraries (Bluetooth Low Energy) ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- Hardware Configuration ---
#define SDA_PIN 16
#define SCL_PIN 17
#define CAN_TX_PIN GPIO_NUM_32
#define CAN_RX_PIN GPIO_NUM_34
#define I2C_FREQ 400000

// --- Physical and Nautical Constants ---
#define RAD_TO_DEG 57.29577951308232
#define G_STD 9.80665 
#define QUATERNION_EPS (1e-4)

// --- Heave Constants ---
#define ACCSMOOTHCOUNTER 10
#define GRAVITYCOUNTER 20
#define TENDENZCOUNTER 20

// --- Instances ---
ICM_20948_I2C icm;
Adafruit_BME280 bme; 
Preferences nvs;
tNMEA2000 *N2K;

// --- BLE UART Configuration ---
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String bleCommand = "";
bool bleCommandReady = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// --- MENU STATES ---
enum MenuState { MAIN, BOAT, NET, CAL, PARAMS, WAITING_INPUT };
MenuState currentMenu = MAIN;
String pendingParam = ""; 

// --- DATA STRUCTURES ---

typedef struct Quaternion {
    float w; float v[3];
} Quaternion;

typedef struct Sample {
    float value; uint32_t timeMicroSec;
    bool operator<(const Sample& o) const { return value < o.value; }
    bool operator>(const Sample& o) const { return value > o.value; }
} SampleType;

typedef struct min_max_lemire {
    Sample min; Sample max;
    std::deque<Sample> min_wedge; std::deque<Sample> max_wedge;
} MinMaxLemire;

struct SensorCal {
    float accBias[3], gyrBias[3], magScale[3], magBias[3];
    float tempAtCal; 
};

struct BoatConfig {
    uint8_t boatType; 
    uint8_t netMode;  
    float length, tonnage;
    float sensorX, sensorY, sensorZ; 
    float rollOffset, pitchOffset, headingOffset;
    uint8_t envLocation; 
    bool autoCalMag; 
    uint8_t mountOrientation; 
    float manualVariation;    
};

struct EKFParams {
    float mahonyKp, mahonyKi, Q_hdg_smooth, R_hdg_smooth;
};

typedef struct mahony_AHRS_vars {
    float twoKp, twoKi, q0=1.0, q1=0, q2=0, q3=0, integralFBx=0, integralFBy=0, integralFBz=0;
} Mahony_AHRS_Vars;

// --- GLOBAL VARIABLES ---
BoatConfig boat; EKFParams ekf; SensorCal scal;
Mahony_AHRS_Vars mahony = {2.0f * 4.0f, 2.0f * 0.0001f};
MinMaxLemire heaveStats;

float Gravity[GRAVITYCOUNTER], AccSmooth[ACCSMOOTHCOUNTER];
float Postitivdiff[TENDENZCOUNTER], Negativdiff[TENDENZCOUNTER];
int Gravity_index = 0, AccSmooth_index = 0, diffcounter = 0;
float GravityAverage = 0, HeaveValue = 0, PreviousVelocity = 0, Heave_vorher = 0;
float Heave_Max_positiv = 0, Heave_Max_negativ = 0;
unsigned long Last_Heave_Max_pos_T = 0, Last_Heave_Max_neg_T = 0;
bool HaveGravityAverage = false, Abwaerts = false, Aufwaerts = false;

// Critical Variables (Shared between Cores) - Protected by navMutex
portMUX_TYPE navMutex = portMUX_INITIALIZER_UNLOCKED;
volatile float roll = 0, pitch = 0, heading = 0, rot = 0;
volatile float smoothedRoT = 0; 
volatile float finalHeave = 0.0f; 

volatile float waveHeight = 0, waveHeightAvg = 0, waveLength = 0, wavePeriod = 0, waveFreq = 0;
volatile float ambientTemp = 0, ambientPress = 0, ambientHum = 0, sensorTemp = 0; 
volatile float heading_true = 0, heading_variation = 0;

unsigned long last_variation_rx = 0; 

float magDeviation[36];
unsigned char SID = 0;
bool streamMode = false, isCalibratingSensors = false;
bool send_heading_true = false;
bool bme_ok = false; // Environmental sensor sanity flag

// Health Monitoring
bool health_thermalAlert = false;
bool health_magAnomaly = false; 

// --- FILTERING CLASSES ---

template <typename T, size_t N>
class STL_RingBuffer {
public:
    void add(T sample) { _buffer[_head] = sample; _head = (_head + 1) % N; if (_size < N) _size++; }
    T average() { if (_size == 0) return 0; T sum = 0; for (size_t i = 0; i < _size; i++) sum += _buffer[i]; return sum / _size; }
private: T _buffer[N]; size_t _head = 0, _size = 0;
};
STL_RingBuffer<float, 50> waveHistory;

class Kalman1D {
public:
    Kalman1D(float q=0.01, float r=0.1, float p=1.0, float x=0) : _q(q), _r(r), _p(p), _x(x) {}
    float updateHeading(float m) {
        _p += _q; float k = _p/(_p+_r); float d = m-_x;
        if(d>180) d-=360; if(d<-180) d+=360;
        _x += k*d; if(_x>=360) _x-=360; if(_x<0) _x+=360;
        _p *= (1-k); return _x;
    }
    void setParams(float q, float r) { _q=q; _r=r; }
private: float _q, _r, _p, _x;
};
Kalman1D smoothHeading(0.01, 0.5, 1.0, 0);

class AranovskiyFilter {
public:
    AranovskiyFilter(float g, float dt) : _g(g), _dt(dt) { x1=0; x2=0; th=1.0; }
    float update(float s) {
        float e = s-x1; x1 += x2*_dt; x2 += (_g*e-th*x1)*_dt; th += (_g*x1*e)*_dt;
        if(th<0.01f) th=0.01f; return sqrt(th);
    }
    float getFreq() { return sqrt(th) / (2.0f * PI); }
private: float x1, x2, th, _g, _dt;
};
AranovskiyFilter waveEstimator(0.5f, 0.01f);

// --- MATHEMATICAL UTILITIES ---

float invSqrt(float x) {
    union { float f; int32_t i; } y;
    y.f = x; y.i = 0x5f375a86 - (y.i >> 1);
    y.f = y.f * (1.5f - (x * 0.5f * y.f * y.f));
    return y.f;
}

void Quaternion_set(float w, float v1, float v2, float v3, Quaternion* output) {
    output->w = w; output->v[0] = v1; output->v[1] = v2; output->v[2] = v3;
}

void Quaternion_rotate(Quaternion* q, float v[3], float out[3]) {
    float ww = q->w * q->w, xx = q->v[0] * q->v[0], yy = q->v[1] * q->v[1], zz = q->v[2] * q->v[2];
    float wx = q->w * q->v[0], wy = q->w * q->v[1], wz = q->w * q->v[2];
    float xy = q->v[0] * q->v[1], xz = q->v[0] * q->v[2], yz = q->v[1] * q->v[2];
    out[0] = ww*v[0] + 2*wy*v[2] - 2*wz*v[1] + xx*v[0] + 2*xy*v[1] + 2*xz*v[2] - zz*v[0] - yy*v[0];
    out[1] = 2*xy*v[0] + yy*v[1] + 2*yz*v[2] + 2*wz*v[0] - zz*v[1] + ww*v[1] - 2*wx*v[2] - xx*v[1];
    out[2] = 2*xz*v[0] + 2*yz*v[1] + zz*v[2] - 2*wy*v[0] - yy*v[2] + 2*wx*v[1] - xx*v[2] + ww*v[2];
}

void Quaternion_toEulerZYX(Quaternion* q, float out[3]) {
    out[0] = atan2(2.0*(q->w*q->v[0] + q->v[1]*q->v[2]), 1.0 - 2.0*(q->v[0]*q->v[0] + q->v[1]*q->v[1]));
    double sinp = 2.0*(q->w*q->v[1] - q->v[2]*q->v[0]);
    out[1] = (fabs(sinp) >= 1) ? copysign(M_PI/2, sinp) : asin(fmaxf(-1.0f, fminf(1.0f, sinp))); // Extra NaN protection
    out[2] = atan2(2.0*(q->w*q->v[2] + q->v[0]*q->v[1]), 1.0 - 2.0*(q->v[1]*q->v[1] + q->v[2]*q->v[2]));
}

void mahony_AHRS_update_ultimate(Mahony_AHRS_Vars* m, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    float recipNorm, hex, hey, hez, qa, qb, qc;
    float accMag = sqrt(ax*ax + ay*ay + az*az);
    float dynamicKp = m->twoKp;
    
    // Active Centrifugal Compensation
    if (fabs(accMag - 1.0f) > 0.15f) dynamicKp = m->twoKp * 0.1f; 

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        float q0q0 = m->q0 * m->q0, q0q1 = m->q0 * m->q1, q0q2 = m->q0 * m->q2, q0q3 = m->q0 * m->q3;
        float q1q1 = m->q1 * m->q1, q1q2 = m->q1 * m->q2, q1q3 = m->q1 * m->q3;
        float q2q2 = m->q2 * m->q2, q2q3 = m->q2 * m->q3;
        float q3q3 = m->q3 * m->q3;

        float hvx = 2.0f * (q1q3 - q0q2);
        float hvy = 2.0f * (q0q1 + q2q3);
        float hvz = q0q0 - q1q1 - q2q2 + q3q3;

        hex = (ay*hvz - az*hvy); 
        hey = (az*hvx - ax*hvz); 
        hez = (ax*hvy - ay*hvx);

        if (!(mx == 0.0f && my == 0.0f && mz == 0.0f)) {
            recipNorm = invSqrt(mx*mx + my*my + mz*mz);
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            float hx = 2.0f * (mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
            float hy = 2.0f * (mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
            float bx = sqrt(hx*hx + hy*hy);
            float bz = 2.0f * (mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

            float halfwx = bx*(0.5f - q2q2 - q3q3) + bz*(q1q3 - q0q2);
            float halfwy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
            float halfwz = bx*(q0q2 + q1q3) + bz*(0.5f - q1q1 - q2q2);

            hex += (my*halfwz - mz*halfwy);
            hey += (mz*halfwx - mx*halfwz);
            hez += (mx*halfwy - my*halfwx);
        }

        if (m->twoKi > 0.0f) {
            m->integralFBx += m->twoKi*hex*dt; 
            m->integralFBy += m->twoKi*hey*dt; 
            m->integralFBz += m->twoKi*hez*dt;
            gx += m->integralFBx; gy += m->integralFBy; gz += m->integralFBz;
        }
        gx += dynamicKp*hex; gy += dynamicKp*hey; gz += dynamicKp*hez;
    }
    gx *= (0.5f*dt); gy *= (0.5f*dt); gz *= (0.5f*dt);
    qa = m->q0; qb = m->q1; qc = m->q2;
    m->q0 += (-qb*gx - qc*gy - m->q3*gz); 
    m->q1 += (qa*gx + qc*gz - m->q3*gy);
    m->q2 += (qa*gy - qb*gz + m->q3*gx); 
    m->q3 += (qa*gz + qb*gy - qc*gx);
    recipNorm = invSqrt(m->q0*m->q0 + m->q1*m->q1 + m->q2*m->q2 + m->q3*m->q3);
    m->q0 *= recipNorm; m->q1 *= recipNorm; m->q2 *= recipNorm; m->q3 *= recipNorm;
}

// --- TROCHOIDAL MODEL ---
float troch_wave_length(float period) { return G_STD * period * period / (2.0f * PI); }

// --- HEAVE ALGORITHMS (BERND CIROTZKI STYLE) ---
bool MakeGravity(float acc) {
    if (acc == 0) return false;
    HaveGravityAverage = true; GravityAverage = 0;
    for (int i=0; i<GRAVITYCOUNTER; i++) { if (Gravity[i] == 0) { HaveGravityAverage = false; break; } GravityAverage += Gravity[i]; }
    Gravity[Gravity_index] = acc; if (++Gravity_index >= GRAVITYCOUNTER) Gravity_index = 0;
    return HaveGravityAverage;
}

float CheckHeave(float heavein) {
    if (Heave_vorher > heavein) { 
        if (!Abwaerts) { Abwaerts = true; Aufwaerts = false; Heave_Max_positiv = heavein; Last_Heave_Max_pos_T = millis(); }
        else if ((Last_Heave_Max_neg_T + 15000) < millis()) { Heave_Max_negativ = heavein; Last_Heave_Max_neg_T = millis(); Last_Heave_Max_pos_T = millis() + 7000; } 
    }
    if (Heave_vorher < heavein) { 
        if (!Aufwaerts) { Abwaerts = false; Aufwaerts = true; Heave_Max_negativ = heavein; Last_Heave_Max_neg_T = millis(); }
        else if ((Last_Heave_Max_pos_T + 15000) < millis()) { Heave_Max_positiv = heavein; Last_Heave_Max_pos_T = millis(); Last_Heave_Max_neg_T = millis() + 7000; } 
    }
    Heave_vorher = heavein;
    if(++diffcounter >= TENDENZCOUNTER) diffcounter = 0;
    float Heaveoffset = (Heave_Max_positiv + Heave_Max_negativ) / 2.0f;
    uint8_t pos = 0, neg = 0;
    if (Heaveoffset > 0) { Postitivdiff[diffcounter] = 1; Negativdiff[diffcounter] = 0; }
    else { Postitivdiff[diffcounter] = 0; Negativdiff[diffcounter] = 1; }
    for(int f=0; f<TENDENZCOUNTER; f++) { pos += Postitivdiff[f]; neg += Negativdiff[f]; }
    if (pos > neg) Heaveoffset += 0.005f; else Heaveoffset -= 0.005f;
    float aH = abs(Heaveoffset);
    if (aH > 2.0f) return (heavein * 0.93f - Heaveoffset * 0.07f);
    if (aH > 0.5f) return (heavein * 0.955f - Heaveoffset * 0.045f);
    return (heavein * 0.992f - Heaveoffset * 0.008f);
}

void min_max_lemire_update(MinMaxLemire* minMax, Sample sample, uint32_t window_size_micro_sec) {
    while (!minMax->max_wedge.empty() && minMax->max_wedge.back().value <= sample.value) minMax->max_wedge.pop_back();
    minMax->max_wedge.push_back(sample);
    while (!minMax->max_wedge.empty() && (sample.timeMicroSec - minMax->max_wedge.front().timeMicroSec > window_size_micro_sec)) minMax->max_wedge.pop_front();
    if (!minMax->max_wedge.empty()) minMax->max = minMax->max_wedge.front();
    while (!minMax->min_wedge.empty() && minMax->min_wedge.back().value >= sample.value) minMax->min_wedge.pop_back();
    minMax->min_wedge.push_back(sample);
    while (!minMax->min_wedge.empty() && (sample.timeMicroSec - minMax->min_wedge.front().timeMicroSec > window_size_micro_sec)) minMax->min_wedge.pop_front();
    if (!minMax->min_wedge.empty()) minMax->min = minMax->min_wedge.front();
}

void applyAdaptiveGains() {
    float inertia = (boat.tonnage / 10.0f) * (boat.length / 12.0f);
    if (inertia < 0.2f) inertia = 0.2f; if (inertia > 5.0f) inertia = 5.0f;
    mahony.twoKp = 3.5f / sqrt(inertia);
    float baseR = 0.5f * inertia; if (baseR > 2.5f) baseR = 2.5f;
    ekf.R_hdg_smooth = baseR;
}

// --- BLE CALLBACKS & OPTIMIZATION ---

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++) {
          if (bleCommand.length() > 200) bleCommand = ""; // Anti-Overflow Attack
          if (rxValue[i] == '\n' || rxValue[i] == '\r') {
            if (bleCommand.length() > 0) bleCommandReady = true;
          } else {
            if (!bleCommandReady) bleCommand += rxValue[i];
          }
        }
      }
    }
};

void cliPrintf(const char *format, ...) {
    char buf[1024]; 
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    Serial.print(buf); 

    if (deviceConnected && pTxCharacteristic) {
        uint8_t* p = (uint8_t*)buf;
        while (len > 0) {
            int chunk = (len > 128) ? 128 : len;
            pTxCharacteristic->setValue(p, chunk);
            pTxCharacteristic->notify();
            p += chunk;
            len -= chunk;
            delay(15); 
        }
    }
}

// --- PROCESSING TASKS (CORE 1) ---

void TaskSensorFusion(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long last_call = micros();
    unsigned long last_env = 0;
    
    for (;;) {
        if (!isCalibratingSensors && icm.dataReady()) {
            icm.getAGMT();
            unsigned long now = micros();
            
            float dt;
            if (now > last_call) dt = (float)(now - last_call) / 1000000.0f;
            else dt = (float)((0xFFFFFFFF - last_call) + now + 1) / 1000000.0f;
            last_call = now;
            
            // DT Clamp: Prevents integration jumps in case of temporary scheduler lockup
            if (dt > 0.5f || dt < 0.001f) dt = 0.01f; 

            // 1. Raw readings and Standard Bias Correction
            float c_ax = (icm.accX() - scal.accBias[0]) / 1000.0f; 
            float c_ay = (icm.accY() - scal.accBias[1]) / 1000.0f;
            float c_az = (icm.accZ() - scal.accBias[2]) / 1000.0f;
            float c_gx = ((icm.gyrX() / 65.5f) - scal.gyrBias[0]) * (M_PI/180.0f);
            float c_gy = ((icm.gyrY() / 65.5f) - scal.gyrBias[1]) * (M_PI/180.0f);
            float c_gz = ((icm.gyrZ() / 65.5f) - scal.gyrBias[2]) * (M_PI/180.0f);
            
            float rawMagX = icm.magX();
            float rawMagY = icm.magY();
            float rawMagZ = icm.magZ();

            // 2. Base Magnetic Correction (Raw -> Corrected)
            float c_mx = (rawMagX - scal.magBias[0]) * scal.magScale[0];
            float c_my = (rawMagY - scal.magBias[1]) * scal.magScale[1];
            float c_mz = (rawMagZ - scal.magBias[2]) * scal.magScale[2];

            // Auto-Cal Mag w/ Gradient Descent on natural sensor axis
            float currentMagRadius = sqrt(c_mx*c_mx + c_my*c_my + c_mz*c_mz);
            if (currentMagRadius > 150.0f || currentMagRadius < 5.0f) {
                health_magAnomaly = true;
            } else {
                health_magAnomaly = false;
                if (boat.autoCalMag && fabs(smoothedRoT) > 0.5f) { // Only learns if the boat is actively turning
                    float error = currentMagRadius - 100.0f; // Target ideal spherical radius
                    float mu = 0.00005f; // Super smooth atomic learning rate
                    if (currentMagRadius > 0.0f) {
                        scal.magBias[0] += mu * error * (c_mx / currentMagRadius);
                        scal.magBias[1] += mu * error * (c_my / currentMagRadius);
                        scal.magBias[2] += mu * error * (c_mz / currentMagRadius);
                    }
                }
            }

            // 3. Physical Mount Orientation Mapping (Flat, Bulkhead, or Upside Down)
            float ax, ay, az, gx, gy, gz, mx, my, mz;
            switch(boat.mountOrientation) {
                case 1: // Upside Down (Ceiling mounted)
                    ax = c_ax; ay = -c_ay; az = -c_az;
                    gx = c_gx; gy = -c_gy; gz = -c_gz;
                    mx = c_mx; my = -c_my; mz = -c_mz;
                    break;
                case 2: // Vertical Bulkhead (Wires pointing down)
                    ax = c_ax; ay = -c_az; az = c_ay;
                    gx = c_gx; gy = -c_gz; gz = c_gy;
                    mx = c_mx; my = -c_mz; mz = c_my;
                    break;
                default: // 0 = Standard Flat (Horizontal)
                    ax = c_ax; ay = c_ay; az = c_az;
                    gx = c_gx; gy = c_gy; gz = c_gz;
                    mx = c_mx; my = c_my; mz = c_mz;
                    break;
            }

            // Nuclear Mahony 9-DOF Fusion (AK09916 maps X-Mag with Y-Acc natively, handled via Y and Z signs)
            mahony_AHRS_update_ultimate(&mahony, gx, gy, gz, ax, ay, az, mx, -my, -mz, dt);
            
            Quaternion quat; Quaternion_set(mahony.q0, mahony.q1, mahony.q2, mahony.q3, &quat);
            float euler[3]; Quaternion_toEulerZYX(&quat, euler);
            
            float calc_roll = euler[0]*RAD_TO_DEG + boat.rollOffset; 
            float calc_pitch = euler[1]*RAD_TO_DEG + boat.pitchOffset;

            float rawH = euler[2] * RAD_TO_DEG;
            if(rawH < 0) rawH += 360.0f;
            
            float calc_heading = smoothHeading.updateHeading(rawH) + boat.headingOffset;
            if(calc_heading >= 360) calc_heading -= 360; if(calc_heading < 0) calc_heading += 360;

            // Bug Fix #2: RoT is already Bias-cleaned. Direct extraction of stabilized rotation.
            float calc_rot = gz * RAD_TO_DEG; 
            float calc_smoothedRoT = (smoothedRoT * 0.90f) + (calc_rot * 0.10f); 

            float v_acc[3] = {ax, ay, az}, rot_a[3]; Quaternion_rotate(&quat, v_acc, rot_a);
            float wAccZ = (rot_a[2] - 1.0f) * G_STD;
            
            if (boat.sensorZ != 0.0f) {
                wAccZ -= (gx * gx + gy * gy) * boat.sensorZ; 
            }

            float calc_finalHeave = finalHeave;
            if (MakeGravity(wAccZ)) {
                float acc_v = wAccZ - (GravityAverage / GRAVITYCOUNTER);
                AccSmooth[AccSmooth_index] = acc_v; if (++AccSmooth_index >= ACCSMOOTHCOUNTER) AccSmooth_index = 0;
                float accAvg = 0; for(int i=0; i<ACCSMOOTHCOUNTER; i++) accAvg += AccSmooth[i];
                accAvg /= ACCSMOOTHCOUNTER;
                float addV = accAvg * dt; if (abs(addV) < 0.01f) addV = 0;
                float vel = addV + PreviousVelocity * 0.98f; if (abs(vel) < 0.03f) vel = 0;
                PreviousVelocity = vel; HeaveValue += (PreviousVelocity * dt);
                calc_finalHeave = CheckHeave(HeaveValue); 
            }
            
            // ---> DATA TEARING PREVENTION: Atomic Update of Global Navigation Variables <---
            portENTER_CRITICAL(&navMutex);
            heading = calc_heading;
            pitch = calc_pitch;
            roll = calc_roll;
            smoothedRoT = calc_smoothedRoT;
            finalHeave = calc_finalHeave;
            portEXIT_CRITICAL(&navMutex);

            waveFreq = waveEstimator.update((float)wAccZ);
            wavePeriod = (waveFreq > 0.05f) ? (1.0f / waveFreq) : 0.0f;
            waveLength = troch_wave_length(wavePeriod);

            SampleType s = {(float)calc_finalHeave, (uint32_t)now};
            min_max_lemire_update(&heaveStats, s, 10000000); 
            waveHeight = heaveStats.max.value - heaveStats.min.value;
            waveHistory.add(waveHeight); waveHeightAvg = waveHistory.average();

            sensorTemp = icm.temp();
            if (fabs(sensorTemp - scal.tempAtCal) > 15.0f) health_thermalAlert = true; else health_thermalAlert = false;

            if (millis() - last_env > 1000) {
                if (bme_ok) {
                    ambientTemp = bme.readTemperature(); 
                    ambientPress = bme.readPressure(); 
                    ambientHum = bme.readHumidity();
                }
                last_env = millis();
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

// --- N2K TASK AND HANDLERS (CORE 0) ---

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

void TaskN2KBus(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        N2K->ParseMessages();
        
        static unsigned long lastHead = 0; // BUG Fix #1: Timer Rollover prevention
        if (millis() - lastHead >= 50) { 
            tN2kMsg msg;
            
            float safe_heading;
            portENTER_CRITICAL(&navMutex); safe_heading = heading; portEXIT_CRITICAL(&navMutex);

            SetN2kPGN127250(msg, SID, safe_heading*(M_PI/180.0f), N2kDoubleNA, N2kDoubleNA, N2khr_magnetic); 
            N2K->SendMsg(msg);
            
            // Mag Fallback Watchdog
            if (millis() - last_variation_rx > 5000) { heading_variation = boat.manualVariation * RAD_TO_DEG; send_heading_true = true; }
            if (send_heading_true) { 
                heading_true = safe_heading + heading_variation; if (heading_true >= 360) heading_true -= 360;
                SetN2kPGN127250(msg, SID, heading_true*(M_PI/180.0f), N2kDoubleNA, N2kDoubleNA, N2khr_true); 
                N2K->SendMsg(msg); 
            }
            lastHead = millis();
        }

        static unsigned long lastAtt = 0;
        if (millis() - lastAtt >= 100) { 
            tN2kMsg msg;
            float safe_pitch, safe_roll, safe_rot, safe_heave;
            
            portENTER_CRITICAL(&navMutex); 
            safe_pitch = pitch; safe_roll = roll; safe_rot = smoothedRoT; safe_heave = finalHeave;
            portEXIT_CRITICAL(&navMutex);

            SetN2kAttitude(msg, SID, N2kDoubleNA, safe_pitch*(M_PI/180.0f), safe_roll*(M_PI/180.0f)); N2K->SendMsg(msg);
            SetN2kHeave(msg, SID, (float)safe_heave); N2K->SendMsg(msg);
            SetN2kRateOfTurn(msg, SID, safe_rot*(M_PI/180.0f)); N2K->SendMsg(msg);
            
            SID++; if(SID>250) SID=0;
            lastAtt = millis();
        }

        static unsigned long lastEnv = 0;
        if (millis() - lastEnv >= 1000) { 
            tN2kMsg msg;
            tN2kTempSource tSrc;
            switch(boat.envLocation) {
                case 0: tSrc = N2kts_OutsideTemperature; break;
                case 1: tSrc = N2kts_InsideTemperature; break;
                case 2: tSrc = N2kts_EngineRoomTemperature; break;
                case 3: tSrc = N2kts_MainCabinTemperature; break;
                default: tSrc = N2kts_OutsideTemperature; break;
            }
            
            // BUG Fix #4: NaN N2K Prevention
            SetN2kTemperature(msg, SID, 1, tSrc, bme_ok ? CToKelvin(ambientTemp) : N2kDoubleNA); N2K->SendMsg(msg);
            
            tN2kHumiditySource hSrc = (boat.envLocation == 0) ? N2khs_OutsideHumidity : N2khs_InsideHumidity;
            SetN2kHumidity(msg, SID, 1, hSrc, bme_ok ? ambientHum : N2kDoubleNA); N2K->SendMsg(msg);
            SetN2kPressure(msg, SID, 1, N2kps_Atmospheric, bme_ok ? ambientPress : N2kDoubleNA); N2K->SendMsg(msg);
            lastEnv = millis();
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); 
    }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
    if (N2kMsg.PGN == 127258L) { 
        unsigned char varSID; tN2kMagneticVariation source; uint16_t days; double variation;
        if (ParseN2kMagneticVariation(N2kMsg, varSID, source, days, variation)) { 
            heading_variation = variation * RAD_TO_DEG; 
            send_heading_true = true; 
            last_variation_rx = millis(); 
        }
    } else if (N2kMsg.PGN == 130850L) { 
        int Index = 2; unsigned char devId = N2kMsg.GetByte(Index);
        if (devId == 65 || devId == 0xFF) { Index = 6; unsigned char cmd = N2kMsg.GetByte(Index); isCalibratingSensors = (cmd == 0); }
    }
}

// --- BLE INTERFACE V10.0 MIL-SPEC (ENGLISH) ---

void printStatusHeader() {
    float safe_h, safe_rot, safe_p;
    portENTER_CRITICAL(&navMutex); safe_h = heading; safe_rot = smoothedRoT; safe_p = pitch; portEXIT_CRITICAL(&navMutex);

    const char* locs[] = {"Outside", "Inside", "Engine Room", "Cabin"};
    const char* oris[] = {"Horizontal", "Upside Down", "Vertical Bulkhead"};
    
    cliPrintf("\n========================================\n"
              "   SYSTEM STATUS V10.0 MIL-SPEC\n"
              "========================================\n"
              " Network:  [%s]\n"
              " Boat:     [%.1f m | %.1f Tons]\n"
              " Mount:    [%s | %s]\n"
              " Environ:  [%.1f C | %.1f hPa | %.1f%% RH]\n"
              " Calib:    [Bias:%s | AutoMag:%s]\n"
              " Health:   [",
              boat.netMode == 0 ? "SIMRAD P-9" : "RAYMARINE EVO1",
              boat.length, boat.tonnage,
              locs[boat.envLocation], oris[boat.mountOrientation],
              ambientTemp, ambientPress/100.0f, ambientHum,
              (scal.accBias[2] != 0 ? "OK" : "Pending"),
              (boat.autoCalMag ? "ACTIVE" : "OFF"));
              
    if (health_thermalAlert) cliPrintf("TEMP WARN! ");
    if (health_magAnomaly) cliPrintf("MAG ANOMALY! ");
    if (!bme_ok) cliPrintf("BME280 FAIL! ");
    if (!health_thermalAlert && !health_magAnomaly && bme_ok) cliPrintf("OPTIMAL");
    cliPrintf("]\n----------------------------------------\n");
}

void showMenu() {
    printStatusHeader();
    switch(currentMenu) {
        case MAIN:
            cliPrintf("1. Boat Config (L/T/Pos/Loc/Ori)\n"
                      "2. Network Config (Navico/Raymarine)\n"
                      "3. Calibrations (Auto-Learn/IMU/Level)\n"
                      "4. Filter Params (Gains/MagVar)\n"
                      "5. Toggle Stream (Live Data)\n"
                      "6. SAVE (NVS)\n"
                      "7. REBOOT\n"); 
            break;
        case BOAT:
            cliPrintf("1. Dimensions (L T)\n"
                      "2. Sensor Position (X Y Z)\n"
                      "3. Heading Offset (H)\n"
                      "4. Environmental Location (0-3)\n"
                      "5. Mount Orientation (0-2)\n"
                      "0. BACK\n"); 
            break;
        case NET:
            cliPrintf("1. SIMRAD (P-9)\n"
                      "2. RAYMARINE (EVO1)\n"
                      "0. BACK\n"); 
            break;
        case CAL:
            cliPrintf("1. Calibrate Bias\n"
                      "2. Calibrate Compass (Mag)\n"
                      "3. Level Horizon\n"
                      "4. Continuous Auto-Calibration (ON/OFF)\n"
                      "0. BACK\n"); 
            break;
        case PARAMS:
            cliPrintf("1. Mahony Kp Ki\n"
                      "2. Kalman Smooth R\n"
                      "3. Manual Magnetic Variation (GPS Fallback)\n"
                      "0. BACK\n"); 
            break;
        default: break;
    }
    cliPrintf("\nSelect (0-9): ");
}

void handleCommand(String input) {
    input.trim(); if (input == "menu") { currentMenu = MAIN; showMenu(); return; }
    if (currentMenu == WAITING_INPUT) {
        if (pendingParam == "dim") { int s = input.indexOf(' '); boat.length = input.substring(0, s).toFloat(); boat.tonnage = input.substring(s+1).toFloat(); applyAdaptiveGains(); } 
        else if (pendingParam == "pos") { int s1 = input.indexOf(' '); int s2 = input.indexOf(' ', s1+1); boat.sensorX = input.substring(0, s1).toFloat(); boat.sensorY = input.substring(s1+1, s2).toFloat(); boat.sensorZ = input.substring(s2+1).toFloat(); }
        else if (pendingParam == "h_off") { boat.headingOffset = input.toFloat(); }
        else if (pendingParam == "loc") { boat.envLocation = (uint8_t)input.toInt(); }
        else if (pendingParam == "ori") { boat.mountOrientation = (uint8_t)input.toInt(); }
        else if (pendingParam == "mahony") { int s = input.indexOf(' '); mahony.twoKp = input.substring(0, s).toFloat(); mahony.twoKi = input.substring(s+1).toFloat(); }
        else if (pendingParam == "kalman") { ekf.R_hdg_smooth = input.toFloat(); smoothHeading.setParams(0.01, ekf.R_hdg_smooth); }
        else if (pendingParam == "var") { boat.manualVariation = input.toFloat(); }
        currentMenu = MAIN; showMenu(); return;
    }
    int choice = input.toInt(); if (choice == 0 && currentMenu != MAIN) { currentMenu = MAIN; showMenu(); return; }
    switch(currentMenu) {
        case MAIN:
            if (choice == 1) currentMenu = BOAT; else if (choice == 2) currentMenu = NET; else if (choice == 3) currentMenu = CAL; else if (choice == 4) currentMenu = PARAMS;
            else if (choice == 5) { streamMode = !streamMode; cliPrintf(streamMode ? "\nSTREAM ON\n" : "\nSTREAM OFF\n"); }
            else if (choice == 6) { nvs.begin("p9_master", false); nvs.putBytes("boat", &boat, sizeof(BoatConfig)); nvs.putBytes("ekf", &ekf, sizeof(EKFParams)); nvs.putBytes("scal", &scal, sizeof(SensorCal)); nvs.end(); cliPrintf("\n[NVS] SAVED.\n");}
            else if (choice == 7) ESP.restart(); 
            if (choice != 5) showMenu(); 
            break;
        case BOAT:
            if (choice == 1) { cliPrintf("\nEnter: <L> <T>\n"); currentMenu = WAITING_INPUT; pendingParam = "dim"; }
            else if (choice == 2) { cliPrintf("\nEnter: <X> <Y> <Z>\n"); currentMenu = WAITING_INPUT; pendingParam = "pos"; }
            else if (choice == 3) { cliPrintf("\nEnter Offset\n"); currentMenu = WAITING_INPUT; pendingParam = "h_off"; }
            else if (choice == 4) { cliPrintf("\n0:Outside, 1:Inside, 2:Engine, 3:Cabin\n"); currentMenu = WAITING_INPUT; pendingParam = "loc"; }
            else if (choice == 5) { cliPrintf("\n0:Flat, 1:UpsideDown, 2:Bulkhead\n"); currentMenu = WAITING_INPUT; pendingParam = "ori"; }
            else showMenu(); break;
        case CAL:
            if (choice == 1) { isCalibratingSensors = true; cliPrintf("\n[CAL] Keep still for 5s...\n"); float ab[3]={0}, gb[3]={0}; for (int i=0; i<400; i++) { if(icm.dataReady()){ icm.getAGMT(); ab[0]+=icm.accX(); ab[1]+=icm.accY(); ab[2]+=icm.accZ(); gb[0]+=icm.gyrX()/65.5f; gb[1]+=icm.gyrY()/65.5f; gb[2]+=icm.gyrZ()/65.5f; } delay(10); } for(int j=0; j<3; j++) { scal.accBias[j]=ab[j]/400.0f; scal.gyrBias[j]=gb[j]/400.0f; } scal.accBias[2]-=1000.0f; scal.tempAtCal = sensorTemp; isCalibratingSensors=false; cliPrintf("[OK] Bias Calibrated.\n"); currentMenu = MAIN; showMenu(); }
            else if (choice == 2) { cliPrintf("\n[!!!] START CIRCLES...\n"); float minM[3]={30000,30000,30000}, maxM[3]={-30000,-30000,-30000}; unsigned long st = millis(); while (millis()-st < 45000) { if(icm.dataReady()){ icm.getAGMT(); float rM[3]={icm.magX(), icm.magY(), icm.magZ()}; for(int i=0;i<3;i++){ if(rM[i]<minM[i]) minM[i]=rM[i]; if(rM[i]>maxM[i]) maxM[i]=rM[i]; } } delay(20); } for(int i=0;i<3;i++){ scal.magBias[i]=(maxM[i]+minM[i])/2.0f; float d=(maxM[i]-minM[i])/2.0f; scal.magScale[i]=(d!=0)?100.0f/d:1.0f; } cliPrintf("[OK] Mag Calibrated.\n"); currentMenu = MAIN; showMenu(); }
            else if (choice == 3) { float safe_r, safe_p; portENTER_CRITICAL(&navMutex); safe_r=roll; safe_p=pitch; portEXIT_CRITICAL(&navMutex); boat.rollOffset = -safe_r; boat.pitchOffset = -safe_p; cliPrintf("\n[OK] Horizon Leveled.\n"); currentMenu = MAIN; showMenu(); }
            else if (choice == 4) { boat.autoCalMag = !boat.autoCalMag; cliPrintf("\n[OK] Auto-Learning State Changed.\n"); currentMenu = MAIN; showMenu(); }
            else showMenu(); break;
        case PARAMS:
            if (choice == 1) { cliPrintf("\nEnter: <Kp> <Ki>\n"); currentMenu = WAITING_INPUT; pendingParam = "mahony"; }
            else if (choice == 2) { cliPrintf("\nEnter R value\n"); currentMenu = WAITING_INPUT; pendingParam = "kalman"; }
            else if (choice == 3) { cliPrintf("\nEnter Variation (e.g. -2.5 for W)\n"); currentMenu = WAITING_INPUT; pendingParam = "var"; }
            else showMenu(); break;
    }
}

void setup() {
    Serial.begin(115200); 
    
    // BUG Fix #3: Extreme I2C lockup prevention
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ); 
    Wire.setTimeOut(50); 
    
    BLEDevice::init("Precision9-BLE");
    BLEDevice::setMTU(512); 
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE Server Started.");

    if (!bme.begin(0x76)) { 
        if (!bme.begin(0x77)) {
            Serial.println("BME280 critical error!"); 
            bme_ok = false;
        } else bme_ok = true;
    } else bme_ok = true;
    
    nvs.begin("p9_master", false);
    if (!nvs.isKey("init10")) {
        boat = {0, 0, 12.0, 8.0, 0.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 1, true, 0, 0.0f}; 
        ekf = {3.5f, 0.001f, 0.01f, 1.0f}; scal = {{0,0,0}, {0,0,0}, {1,1,1}, {0,0,0}, 25.0f};
        nvs.putBytes("boat", &boat, sizeof(BoatConfig)); nvs.putBytes("ekf", &ekf, sizeof(EKFParams)); nvs.putBytes("scal", &scal, sizeof(SensorCal)); nvs.putBool("init10", true);
    } else { 
        nvs.getBytes("boat", &boat, sizeof(BoatConfig)); nvs.getBytes("ekf", &ekf, sizeof(EKFParams)); nvs.getBytes("scal", &scal, sizeof(SensorCal)); 
    }
    nvs.end(); 
    
    applyAdaptiveGains(); smoothHeading.setParams(0.01, ekf.R_hdg_smooth);
    icm.begin(Wire, 0); icm.swReset(); delay(100); icm.sleep(false);
    N2K = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
    if (boat.netMode == 1) { 
        N2K->SetProductInformation("107018103", 13233, "EVO1 Compass", "1.0.0", "1.0"); 
        N2K->SetDeviceInformation(1048678, 140, 60, 1851); 
    } else { 
        N2K->SetProductInformation("107018103", 13233, "Precision-9 Master", "10.0.0", "6.00"); 
        N2K->SetDeviceInformation(1048678, 140, 60, 275); 
    }
    N2K->SetMode(tNMEA2000::N2km_NodeOnly, 65); N2K->SetMsgHandler(HandleNMEA2000Msg); N2K->Open();
    xTaskCreatePinnedToCore(TaskSensorFusion, "Fusion", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(TaskN2KBus, "N2K", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(TaskCLI, "CLI", 4096, NULL, 1, NULL, 0);
}

void loop() {}

void TaskCLI(void *pvParameters) {
    for (;;) {
        if (deviceConnected && !oldDeviceConnected) {
            delay(500); 
            cliPrintf("\n*** PRECISION-9 / EVO1 MIL-SPEC V10.0 BLE ***\n"); 
            showMenu();
            oldDeviceConnected = deviceConnected;
        }
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); pServer->startAdvertising(); 
            oldDeviceConnected = deviceConnected;
        }

        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            handleCommand(input);
        }
        
        if (bleCommandReady) {
            handleCommand(bleCommand);
            bleCommand = "";
            bleCommandReady = false;
        }

        if (streamMode) {
            float safe_h, safe_rot, safe_p;
            portENTER_CRITICAL(&navMutex); safe_h = heading; safe_rot = smoothedRoT; safe_p = pitch; portEXIT_CRITICAL(&navMutex);
            cliPrintf("H:%.1f RoT:%.1f P:%.1f T:%.1fC\n", safe_h, safe_rot, safe_p, ambientTemp);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
