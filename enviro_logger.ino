#include <PDM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino_LSM6DSOX.h>

// PDM microphone setup
// Default number of output channels
static const char channels = 1;
// Default output frequency
static const int frequency = 16000;
// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];
// Number of audio samples read
volatile int samplesRead;
// Number of samples in recording
// (seconds multiplied by frequency)
static const int samplesRec = 1 * frequency;
// Number of samples sent
volatile int samplesSent = 0;

// BME280 setup
Adafruit_BME280 bme;

// variables to store information which sensors are present
bool pdm_present = false;
bool bme_present = false;
bool imu_present = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configure the data receive callback for PDM
  PDM.onReceive(onPDMdata);

  // Initialize sensors
  pdm_present = PDM.begin(channels, frequency);
  bme_present = bme.begin(0x76, &Wire);
  imu_present = IMU.begin();

  // configure for weather monitoring
  if (bme_present)
  {
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
  }
}

void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void returnSound() {
  samplesRead = 0;
  samplesSent = 0;
  while (samplesSent < samplesRec) {
    if (samplesRead) {
      for (int i = 0; i < samplesRead; i++) {
        // Print samples to the serial monitor or plotter
        Serial.println(sampleBuffer[i]);

        samplesSent++;
        if (samplesSent >= samplesRec) {
          break;
        }
      }
    }
    // Clear the read count
    samplesRead = 0;
  }
}

void returnEnviro() {
  // create buffer for data
  String buf;
  
  bme.takeForcedMeasurement();
  delay(25);
  float val = bme.readTemperature();
  buf += String(val, 2);
  buf += F(",");
  val = bme.readHumidity();
  buf += String(val, 2);
  buf += F(",");
  val = bme.readPressure() / 100.0F;
  buf += String(val, 2);

  Serial.println(buf);
}

void returnIMU() {
  // create buffer for data
  String buf;

  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }
  buf += String(x, 2);
  buf += F(",");
  buf += String(y, 2);
  buf += F(",");
  buf += String(z, 2);
  buf += F(",");
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
  }
  buf += String(x, 2);
  buf += F(",");
  buf += String(y, 2);
  buf += F(",");
  buf += String(z, 2);
  
  Serial.println(buf);
}

void loop() {
  while (Serial.available() > 0) {
    char incChar = Serial.read();

    switch (incChar) {
      case 's':
        if (pdm_present) {
          returnSound();
        }
        else {
          Serial.println("No microphone");
        }
        break;
      
      case 'e':
        if (bme_present) {
          returnEnviro();
        }
        else {
          Serial.println("No environmental sensor");
        }
        break;
      
      case 'i':
        if (imu_present) {
          returnIMU();
        }
        else {
          Serial.println("No inertial measurement unit");
        }
        break;
    }
  }
}