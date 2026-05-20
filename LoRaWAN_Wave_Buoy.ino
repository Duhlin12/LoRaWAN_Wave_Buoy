#include "Arduino.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <MPU6050_tockn.h>
#include "LoRaWan_APP.h"

/* --- LORAWAN ABP KEYS (PLACEHOLDERS) --- */
uint32_t devAddr = ( uint32_t )0x00000000; // Replace with your DevAddr (e.g., 0x01E02E0C)
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Replace with your NwkSKey
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Replace with your AppSKey

/* --- DUMMY KEYS (Required for ABP compilation) --- */
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* --- LORAWAN CONFIG --- */
uint16_t userChannelsMask[6] = { 0x0008, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }; 
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = false; 
bool loraWanAdr = false; 
bool keepNet = true;  
bool isTxConfirmed = false;          
uint8_t confirmedNbTrials = 4; 
uint8_t appPort = 2;

/* --- OPERATION SETTINGS --- */
#define SENSOR_POWER_PIN GPIO2   
#define MEASURE_DURATION_MS 1200000 // 20 minutes sampling
#define SLEEP_CYCLE_MS 2400000      // 40 minutes deep sleep (Total 60 min cycle)
#define GPS_MAX_WAIT_MS 300000    // 5 minutes timeout for GPS fix

/* --- OBJECTS --- */
OneWire oneWire(GPIO1);
DallasTemperature sensors(&oneWire);
TinyGPSPlus gps;
MPU6050 mpu6050(Wire);

/* --- GLOBAL MANUAL OFFSETS (Locks zero-point in RAM) --- */
float manual_offset_AX = 0.0f;
float manual_offset_AY = 0.0f;
float manual_offset_AZ = 0.0f;
float manual_pitch_offset = 0.0f;
float manual_roll_offset = 0.0f;

/* --- GLOBAL VARIABLES --- */
float DISP_GAIN = 0.4f;     
const float MIN_WAVE_M = 0.02f;
float currentHmax = 0, currentHs = 0, currentTemp = 0;
double currentLat = 0, currentLon = 0;
uint16_t currentBatt = 0;
uint32_t lastMeasureFinishTime = 0; 
static float waveHeights[600]; 

/* --- WAVE MEASUREMENT --- */
void measureWaves() {
  float velocity = 0, position = 0, prevVelocity = 0;
  float localMin = 0, localMax = 0;
  int waveCount = 0;
  currentHmax = 0; currentHs = 0;
  
  float estimatedOffset = 0.0f;

  unsigned long tStart = millis();
  unsigned long tPrev = micros();

  Serial.println("\n>>> WAVE MEASUREMENT STARTED <<<");
  Serial.print("[INFO] Applying locked land offsets -> X: "); Serial.print(manual_offset_AX, 3);
  Serial.print(" | Y: "); Serial.print(manual_offset_AY, 3);
  Serial.print(" | Z: "); Serial.println(manual_offset_AZ, 3);
  Serial.print("[INFO] Angle offsets -> Pitch: "); Serial.print(manual_pitch_offset, 2);
  Serial.print(" | Roll: "); Serial.println(manual_roll_offset, 2);

  while (millis() - tStart < MEASURE_DURATION_MS) {
    CyDelay(1); 
    mpu6050.update(); 

    unsigned long now = micros();
    float dt = (now - tPrev) * 1e-6f;
    if (dt <= 0 || dt > 0.1) dt = 0.02;
    tPrev = now;
    
    float ax = mpu6050.getAccX() - manual_offset_AX;
    float ay = mpu6050.getAccY() - manual_offset_AY;
    float az = mpu6050.getAccZ() - (manual_offset_AZ - 1.00f); 
    
    float roll = (mpu6050.getAngleX() - manual_roll_offset) * DEG_TO_RAD;
    float pitch = (mpu6050.getAngleY() - manual_pitch_offset) * DEG_TO_RAD;

    float accTrueVertical = ax * sin(pitch) - ay * sin(roll) * cos(pitch) + az * cos(roll) * cos(pitch);
    
    float accVertRaw = (accTrueVertical - 1.00f) * 9.81f; 

    estimatedOffset = (estimatedOffset * 0.999f) + (accVertRaw * 0.001f);
    
    float accVert = accVertRaw - estimatedOffset;

    if (abs(accVert) < 0.12f) accVert = 0;

    velocity = (velocity + accVert * dt) * 0.96f;   
    position = (position + velocity * dt) * 0.96f;  

    if (accVert == 0) { velocity *= 0.90f; position *= 0.90f; }

    if (position < localMin) localMin = position;
    if (position > localMax) localMax = position;

    if ((prevVelocity > 0 && velocity <= 0) || (prevVelocity < 0 && velocity >= 0)) {
      float h = (localMax - localMin) * DISP_GAIN;
      if (h > MIN_WAVE_M && waveCount < 600) {
        waveHeights[waveCount++] = h;
        if (h > currentHmax) currentHmax = h;
        
        Serial.print("   [WAVE DETECTED] Height: "); Serial.print(h, 2); 
        Serial.print("m | Total wave count: "); Serial.println(waveCount);
      }
      localMin = position; localMax = position;
    }
    prevVelocity = velocity;

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 5000) {
      Serial.print("   -> Time: "); Serial.print((millis() - tStart) / 1000); 
      Serial.print("/"); Serial.print(MEASURE_DURATION_MS / 1000);
      Serial.print(" sec | Pos: "); Serial.print(position, 3);
      Serial.print(" | AccVert: "); Serial.println(accVert, 3);
      lastLog = millis();
    }
    delay(25); 
  }

  if (waveCount > 0) {
    for (int i = 0; i < waveCount-1; i++) {
      for (int j = 0; j < waveCount-i-1; j++) {
        if (waveHeights[j] < waveHeights[j+1]) {
          float t = waveHeights[j]; waveHeights[j] = waveHeights[j+1]; waveHeights[j+1] = t;
        }
      }
    }
    int topThird = (waveCount + 2) / 3;
    float sumHs = 0;
    for (int i = 0; i < topThird; i++) sumHs += waveHeights[i];
    currentHs = sumHs / (float)topThird;
  }
  
  Serial.println(">>> WAVE MEASUREMENT COMPLETE <<<");
  Serial.print("[RESULT] Hmax: "); Serial.print(currentHmax, 2); 
  Serial.print("m | Hs: "); Serial.print(currentHs, 2); Serial.println("m");
}

/* --- MAIN SEQUENCE & PAYLOAD PRINTING --- */
static void prepareTxFrame(uint8_t port) {
  Serial.println("\n=============================================");
  Serial.println("--- NEW CYCLE: EXECUTING MEASUREMENTS ---");
  Serial.println("=============================================");

  currentBatt = getBatteryVoltage();
  
  digitalWrite(Vext, LOW); 
  digitalWrite(SENSOR_POWER_PIN, HIGH); 
  delay(500);

  Wire.end(); 
  delay(10);
  Wire.begin();
  mpu6050.begin(); 
  
  measureWaves();

  digitalWrite(SENSOR_POWER_PIN, LOW); 
  delay(1200); 
  
  sensors.begin();
  sensors.requestTemperatures(); 
  delay(850); 
  float t = sensors.getTempCByIndex(0);
  if (t == 85.00 || t == -127.00) {
      sensors.requestTemperatures();
      delay(850); 
      t = sensors.getTempCByIndex(0);
  }
  currentTemp = t;

  // GPS (Precision requires HDOP <= 2.0 AND at least 8 satellites, max wait time 5 minutes)
  unsigned long gpsStart = millis();
  bool gotGoodFix = false;

  // --- MEMORY FIX: Resets GPS object and coordinates before searching ---
  gps = TinyGPSPlus();
  currentLat = 0.0;
  currentLon = 0.0;

  while (millis() - gpsStart < GPS_MAX_WAIT_MS) {
    CyDelay(1); 
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        // hdop.value() <= 200 (HDOP <= 2.0) and satellites.value() >= 8 (min 8 satellites)
        if (gps.location.isValid() && 
            gps.hdop.isValid() && gps.hdop.value() <= 200 && 
            gps.satellites.isValid() && gps.satellites.value() >= 8) {
          currentLat = gps.location.lat();
          currentLon = gps.location.lng();
          gotGoodFix = true;
          break;
        }
      }
    }
    if (gotGoodFix) break; 
  }

  if (!gotGoodFix && gps.location.isValid()) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
  }

  Wire.end();
  digitalWrite(Vext, HIGH); 
  digitalWrite(SENSOR_POWER_PIN, HIGH); 

  uint16_t tempInt = (uint16_t)((currentTemp + 20) * 100); 
  uint16_t battInt = currentBatt;
  uint8_t hMaxByte = (uint8_t)(currentHmax * 100);
  uint8_t hSByte = (uint8_t)(currentHs * 100);
  int32_t latInt = currentLat * 100000;
  int32_t lonInt = currentLon * 100000;

  appDataSize = 14;
  appData[0] = (battInt >> 8) & 0xFF;
  appData[1] = battInt & 0xFF;
  appData[2] = hMaxByte;
  appData[3] = hSByte;
  appData[4] = (tempInt >> 8) & 0xFF;
  appData[5] = tempInt & 0xFF;
  appData[6] = (latInt >> 24) & 0xFF;
  appData[7] = (latInt >> 16) & 0xFF;
  appData[8] = (latInt >> 8) & 0xFF;
  appData[9] = latInt & 0xFF;
  appData[10] = (lonInt >> 24) & 0xFF;
  appData[11] = (lonInt >> 16) & 0xFF;
  appData[12] = (lonInt >> 8) & 0xFF;
  appData[13] = lonInt & 0xFF;

  Serial.println("\n--- OUTGOING PAYLOAD DATA ---");
  Serial.print("Battery:      "); Serial.print(currentBatt); Serial.println(" mV");
  Serial.print("Hmax:         "); Serial.print(currentHmax, 2); Serial.println(" m");
  Serial.print("Hs:           "); Serial.print(currentHs, 2); Serial.println(" m");
  Serial.print("Temperature:  "); Serial.print(currentTemp, 1); Serial.println(" C");
  Serial.print("Latitude:     "); Serial.print(currentLat, 5); Serial.println();
  Serial.print("Longitude:    "); Serial.print(currentLon, 5); Serial.println();

  // --- PRINT ACTUAL PAYLOAD IN HEX ---
  Serial.print("Raw Payload (HEX): ");
  for (int i = 0; i < appDataSize; i++) {
    if (appData[i] < 0x10) Serial.print("0"); 
    Serial.print(appData[i], HEX);
    Serial.print(" ");
  }
  Serial.println("\n");

  // Transmit Power set to 14 dBm
  Radio.SetTxConfig( MODEM_LORA, 14, 0, 0, 12, 1, 8, false, true, 0, 0, false, 3000 );
  lastMeasureFinishTime = millis();
}

void setup() {
  boardInitMcu();
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  
  Serial.println("\n=============================================");
  Serial.println("--- BUOY STARTING (COLD START/RESET) ---");
  Serial.println("=============================================");

  digitalWrite(Vext, LOW); 
  digitalWrite(SENSOR_POWER_PIN, HIGH); 
  delay(1000);
  Wire.begin();
  mpu6050.begin();
  
  Serial.println("[MPU] Beginning one-time calibration on land... KEEP DEVICE STILL!");
  
  float sumX = 0, sumY = 0, sumZ = 0;
  float sumPitch = 0, sumRoll = 0;
  
  for(int i=0; i<100; i++) {
    mpu6050.update();
    sumX += mpu6050.getAccX();
    sumY += mpu6050.getAccY();
    sumZ += mpu6050.getAccZ();
    sumRoll += mpu6050.getAngleX();
    sumPitch += mpu6050.getAngleY();
    delay(20);
  }
  
  manual_offset_AX = sumX / 100.0f;
  manual_offset_AY = sumY / 100.0f;
  manual_offset_AZ = sumZ / 100.0f; 
  manual_roll_offset = sumRoll / 100.0f;
  manual_pitch_offset = sumPitch / 100.0f;

  Serial.println("[MPU] Calibration complete! Hardware and angle offsets stored in RAM.");
  Serial.print("-> Stored X: "); Serial.println(manual_offset_AX, 4);
  Serial.print("-> Stored Y: "); Serial.println(manual_offset_AY, 4);
  Serial.print("-> Stored Z: "); Serial.println(manual_offset_AZ, 4);
  Serial.print("-> Stored Pitch error: "); Serial.println(manual_pitch_offset, 2);
  Serial.print("-> Stored Roll error: "); Serial.println(manual_roll_offset, 2);
  
  Wire.end();
  digitalWrite(Vext, HIGH);
  
  deviceState = DEVICE_STATE_INIT;
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDataRateForNoADR(0); 
      deviceState = DEVICE_STATE_JOIN;
      break;
    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      break;
    case DEVICE_STATE_SEND:
      if (lastMeasureFinishTime != 0 && (millis() - lastMeasureFinishTime < SLEEP_CYCLE_MS)) {
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    case DEVICE_STATE_CYCLE:
      txDutyCycleTime = SLEEP_CYCLE_MS + randr(0, 1000);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    case DEVICE_STATE_SLEEP:
      LoRaWAN.sleep();
      break;
    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}