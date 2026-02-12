/**
 * ═══════════════════════════════════════════════════════════════════════════
 * ULTRA PRO AIR DRUM - ESP32 WROOM-32 + MPU6050 (FIXED VERSION)
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * TARGET: <15ms latency, professional velocity response, spatial drum mapping
 * HARDWARE: ESP32 WROOM-32, MPU6050 (I2C: SDA=21, SCL=22)
 * OUTPUT: BLE MIDI (NimBLE) with Serial debugging fallback
 * 
 * FEATURES:
 * - Complementary filter for drift-free orientation
 * - Dynamic threshold for ghost note elimination
 * - Power-law velocity mapping (natural drum feel)
 * - 4-zone spatial detection (Snare/HiHat/Crash/Splash)
 * - Non-blocking architecture for maximum responsiveness
 * 
 * FIX: Removed BLE Keyboard library to avoid NimBLE conflict
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <NimBLEDevice.h>

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

// I2C Configuration
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000  // 400kHz for low latency

// Hit Detection Parameters
#define ACCEL_THRESHOLD_BASE 2.0f      // Base threshold in G (dynamic adjustment)
#define ACCEL_THRESHOLD_MIN 1.5f       // Minimum threshold
#define ACCEL_THRESHOLD_MAX 3.5f       // Maximum threshold
#define COOLDOWN_MS 50                 // Minimum time between hits (anti-double-trigger)
#define PEAK_WINDOW_MS 20              // Window to capture peak acceleration

// Velocity Mapping (Acceleration → MIDI)
#define ACCEL_MIN 2.0f                 // Minimum detectable strike (G)
#define ACCEL_MAX 16.0f                // Maximum strike force (G)
#define VELOCITY_MIN 40                // MIDI velocity minimum
#define VELOCITY_MAX 127               // MIDI velocity maximum
#define VELOCITY_CURVE 0.6f            // Power-law exponent (0.5-0.7 feels natural)

// Spatial Zone Detection (Roll/Pitch angles in degrees)
#define ZONE_CENTER_ROLL_MIN -20.0f    // Snare zone
#define ZONE_CENTER_ROLL_MAX 20.0f
#define ZONE_LEFT_ROLL_MAX -20.0f      // Hi-Hat zone
#define ZONE_RIGHT_ROLL_MIN 20.0f      // Crash zone
#define ZONE_HIGH_PITCH_MIN 30.0f      // Splash/Ride zone (overhead)

// Complementary Filter
#define ALPHA 0.98f                    // Gyro weight (0.95-0.99 typical)
#define DT 0.005f                      // 5ms = 200Hz target loop rate

// MIDI Note Mapping
#define MIDI_SNARE 38                  // Acoustic Snare (GM)
#define MIDI_HIHAT 42                  // Closed Hi-Hat
#define MIDI_CRASH 49                  // Crash Cymbal 1
#define MIDI_SPLASH 55                 // Splash Cymbal

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS & STATE
// ═══════════════════════════════════════════════════════════════════════════

MPU6050 mpu;

// BLE MIDI
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
bool bleConnected = false;

// State Variables
float roll = 0.0f, pitch = 0.0f;           // Filtered orientation (degrees)
float accelMagnitude = 0.0f;               // Current acceleration magnitude (G)
float dynamicThreshold = ACCEL_THRESHOLD_BASE;

unsigned long lastHitTime = 0;             // For cooldown
bool inHitWindow = false;                  // Peak detection state
float peakAccel = 0.0f;                    // Peak during hit window
unsigned long hitWindowStart = 0;

// Performance Monitoring
unsigned long lastLoopTime = 0;
float loopFrequency = 0.0f;

// ═══════════════════════════════════════════════════════════════════════════
// BLE MIDI UUID DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

#define MIDI_SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

// ═══════════════════════════════════════════════════════════════════════════
// BLE CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        bleConnected = true;
        Serial.println("✓ BLE MIDI Connected!");
    }
    
    void onDisconnect(NimBLEServer* pServer) {
        bleConnected = false;
        Serial.println("✗ BLE MIDI Disconnected");
        NimBLEDevice::startAdvertising();
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// COMPLEMENTARY FILTER
// ═══════════════════════════════════════════════════════════════════════════
/**
 * Combines gyroscope (fast, no drift short-term) with accelerometer (accurate long-term)
 * to produce stable orientation estimates immune to motion artifacts.
 * 
 * MATH:
 * angle_filtered = ALPHA * (angle_prev + gyro_rate * dt) + (1-ALPHA) * accel_angle
 * 
 * - High ALPHA (0.98): Trust gyro more → responsive but may drift
 * - Low ALPHA (0.90): Trust accel more → stable but sluggish during motion
 */
void updateOrientation(float ax, float ay, float az, float gx, float gy) {
    // Accelerometer angles (from gravity vector)
    float accelRoll = atan2(ay, az) * RAD_TO_DEG;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    
    // Gyroscope integration (rate → angle)
    float gyroRollRate = gx * DT;   // deg/s * s = deg
    float gyroPitchRate = gy * DT;
    
    // Complementary fusion
    roll = ALPHA * (roll + gyroRollRate) + (1.0f - ALPHA) * accelRoll;
    pitch = ALPHA * (pitch + gyroPitchRate) + (1.0f - ALPHA) * accelPitch;
}

// ═══════════════════════════════════════════════════════════════════════════
// VELOCITY MAPPING: ACCELERATION → MIDI VELOCITY
// ═══════════════════════════════════════════════════════════════════════════
/**
 * Maps physical strike force to MIDI velocity using a power-law curve.
 * 
 * WHY POWER-LAW?
 * - Linear mapping feels "flat" - soft hits too loud, hard hits too soft
 * - Power-law (exponent < 1) compresses dynamic range naturally
 * - Mimics human perception of drum loudness
 * 
 * FORMULA:
 * normalized = (accel - MIN) / (MAX - MIN)          [0.0 to 1.0]
 * curved = normalized ^ CURVE                       [compression]
 * velocity = VELOCITY_MIN + curved * (VELOCITY_MAX - VELOCITY_MIN)
 */
uint8_t mapVelocity(float accel) {
    // Clamp to valid range
    accel = constrain(accel, ACCEL_MIN, ACCEL_MAX);
    
    // Normalize to [0, 1]
    float normalized = (accel - ACCEL_MIN) / (ACCEL_MAX - ACCEL_MIN);
    
    // Apply power-law curve
    float curved = pow(normalized, VELOCITY_CURVE);
    
    // Map to MIDI velocity range
    uint8_t velocity = VELOCITY_MIN + (uint8_t)(curved * (VELOCITY_MAX - VELOCITY_MIN));
    
    return constrain(velocity, VELOCITY_MIN, VELOCITY_MAX);
}

// ═══════════════════════════════════════════════════════════════════════════
// SPATIAL ZONE DETECTION
// ═══════════════════════════════════════════════════════════════════════════
/**
 * Determines which virtual drum was hit based on stick orientation.
 * 
 * ZONES (using Roll/Pitch):
 * - Center (-20° to +20° roll): SNARE (default)
 * - Left (< -20° roll): HI-HAT
 * - Right (> +20° roll): CRASH CYMBAL
 * - High (> +30° pitch): SPLASH/RIDE (overhead motion)
 */
enum DrumZone {
    ZONE_SNARE,
    ZONE_HIHAT,
    ZONE_CRASH,
    ZONE_SPLASH
};

const char* zoneNames[] = {"SNARE", "HIHAT", "CRASH", "SPLASH"};

DrumZone detectZone() {
    // Priority: High overhead motion (Splash)
    if (pitch > ZONE_HIGH_PITCH_MIN) {
        return ZONE_SPLASH;
    }
    
    // Left/Right based on roll
    if (roll < ZONE_LEFT_ROLL_MAX) {
        return ZONE_HIHAT;
    } else if (roll > ZONE_RIGHT_ROLL_MIN) {
        return ZONE_CRASH;
    }
    
    // Default: Center (Snare)
    return ZONE_SNARE;
}

uint8_t zoneToMidiNote(DrumZone zone) {
    switch(zone) {
        case ZONE_HIHAT:  return MIDI_HIHAT;
        case ZONE_CRASH:  return MIDI_CRASH;
        case ZONE_SPLASH: return MIDI_SPLASH;
        default:          return MIDI_SNARE;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// DYNAMIC THRESHOLD ADJUSTMENT
// ═══════════════════════════════════════════════════════════════════════════
/**
 * Adapts detection threshold based on background motion to prevent ghost notes.
 * - Slowly increases threshold if ambient vibration detected
 * - Decreases to baseline during stillness
 */
void adjustThreshold(float ambientAccel) {
    const float ADAPT_RATE = 0.02f;  // Slow adaptation
    
    if (ambientAccel > dynamicThreshold * 0.5f) {
        // Increase threshold if background motion detected
        dynamicThreshold += ADAPT_RATE;
    } else {
        // Decay back to baseline
        dynamicThreshold -= ADAPT_RATE * 0.5f;
    }
    
    dynamicThreshold = constrain(dynamicThreshold, 
                                  ACCEL_THRESHOLD_MIN, 
                                  ACCEL_THRESHOLD_MAX);
}

// ═══════════════════════════════════════════════════════════════════════════
// BLE MIDI OUTPUT
// ═══════════════════════════════════════════════════════════════════════════
/**
 * Sends MIDI Note On/Off via BLE MIDI characteristic.
 * Format: [header, timestamp, status, note, velocity]
 */
void sendMidiNote(uint8_t note, uint8_t velocity) {
    if (!bleConnected || pCharacteristic == nullptr) {
        return;
    }
    
    uint8_t midiMsg[5];
    midiMsg[0] = 0x80;  // Header (timestamp high bit)
    midiMsg[1] = 0x80;  // Timestamp low
    midiMsg[2] = 0x99;  // Note On, Channel 10 (drums)
    midiMsg[3] = note;
    midiMsg[4] = velocity;
    
    pCharacteristic->setValue(midiMsg, 5);
    pCharacteristic->notify();
    
    // Note Off after 10ms (simulate hit duration)
    delay(10);
    midiMsg[2] = 0x89;  // Note Off
    midiMsg[4] = 0;
    pCharacteristic->setValue(midiMsg, 5);
    pCharacteristic->notify();
}

// ═══════════════════════════════════════════════════════════════════════════
// HIT DETECTION CORE LOGIC
// ═══════════════════════════════════════════════════════════════════════════
void processHitDetection() {
    unsigned long now = millis();
    
    // Cooldown period (prevent double-triggering)
    if (now - lastHitTime < COOLDOWN_MS) {
        return;
    }
    
    // === STATE 1: Waiting for trigger ===
    if (!inHitWindow) {
        if (accelMagnitude > dynamicThreshold) {
            // Hit detected! Enter peak capture window
            inHitWindow = true;
            hitWindowStart = now;
            peakAccel = accelMagnitude;
        }
    }
    // === STATE 2: Capturing peak acceleration ===
    else {
        // Update peak if higher value found
        if (accelMagnitude > peakAccel) {
            peakAccel = accelMagnitude;
        }
        
        // Window expired → emit the hit
        if (now - hitWindowStart >= PEAK_WINDOW_MS) {
            // Determine zone and velocity
            DrumZone zone = detectZone();
            uint8_t velocity = mapVelocity(peakAccel);
            uint8_t note = zoneToMidiNote(zone);
            
            // Output via BLE MIDI
            sendMidiNote(note, velocity);
            
            // Debug output
            Serial.printf("🥁 HIT: %s | Peak=%.2fG | Vel=%d | Roll=%.1f° Pitch=%.1f°\n",
                         zoneNames[zone], peakAccel, velocity, roll, pitch);
            
            // Reset state
            inHitWindow = false;
            lastHitTime = now;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║   ULTRA PRO AIR DRUM - INITIALIZING   ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // === I2C & MPU6050 Setup ===
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);
    
    Serial.print("Initializing MPU6050... ");
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("❌ FAILED!");
        Serial.println("Check wiring: SDA=21, SCL=22");
        while(1) {
            delay(1000);
            Serial.println("MPU6050 not found. Retrying...");
        }
    }
    Serial.println("✓ OK");
    
    // Configure MPU6050 for low latency
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);  // ±16G range
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500°/s
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);              // 42Hz low-pass (balance noise vs latency)
    
    Serial.println("MPU6050 configured: ±16G, ±500°/s, 42Hz DLPF");
    
    // === BLE MIDI Setup ===
    Serial.print("Initializing BLE MIDI... ");
    NimBLEDevice::init("AirDrum Pro");
    
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    NimBLEService* pService = pServer->createService(MIDI_SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        MIDI_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    
    pService->start();
    
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(MIDI_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // iPhone connection optimization
    pAdvertising->start();
    
    Serial.println("✓ OK (advertising)");
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║          ✓ SYSTEM READY!              ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println("→ Connect via BLE MIDI from your device");
    Serial.println("→ Look for: 'AirDrum Pro'\n");
    
    lastLoopTime = micros();
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP (NON-BLOCKING, TARGET: >200Hz)
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    unsigned long loopStart = micros();
    
    // === Read MPU6050 (Minimize blocking time) ===
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    
    // Convert to physical units
    float ax = ax_raw / 2048.0f;  // ±16G range → 2048 LSB/G
    float ay = ay_raw / 2048.0f;
    float az = az_raw / 2048.0f;
    
    float gx = gx_raw / 65.5f;    // ±500°/s range → 65.5 LSB/(°/s)
    float gy = gy_raw / 65.5f;
    float gz = gz_raw / 65.5f;
    
    // === Complementary Filter Update ===
    updateOrientation(ax, ay, az, gx, gy);
    
    // === Calculate Acceleration Magnitude (Hit Detection) ===
    // Remove gravity bias (approximate for moving sensor)
    float ax_dynamic = ax - sin(pitch * DEG_TO_RAD);
    float ay_dynamic = ay + sin(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
    float az_dynamic = az - cos(roll * DEG_TO_RAD) * cos(pitch * DEG_TO_RAD);
    
    accelMagnitude = sqrt(sq(ax_dynamic) + sq(ay_dynamic) + sq(az_dynamic));
    
    // === Dynamic Threshold Adjustment ===
    adjustThreshold(accelMagnitude);
    
    // === Hit Detection & Output ===
    processHitDetection();
    
    // === Performance Monitoring ===
    unsigned long loopEnd = micros();
    unsigned long loopDuration = loopEnd - loopStart;
    loopFrequency = 1000000.0f / (loopEnd - lastLoopTime);
    lastLoopTime = loopEnd;
    
    // Debug every 1000 loops (~5 seconds at 200Hz)
    static int loopCounter = 0;
    if (++loopCounter >= 1000) {
        loopCounter = 0;
        Serial.printf("⚡ Performance: %.1f Hz | %.2f ms/loop | Thresh=%.2fG | BLE=%s\n",
                     loopFrequency, loopDuration / 1000.0f, dynamicThreshold,
                     bleConnected ? "Connected" : "Waiting");
    }
    
    // Maintain ~200Hz loop rate (5ms target, adjust if needed)
    unsigned long elapsed = micros() - loopStart;
    if (elapsed < 5000) {
        delayMicroseconds(5000 - elapsed);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// END OF CODE
// ═══════════════════════════════════════════════════════════════════════════