#include <Arduino.h>
#include <PL_ADXL355.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

//==============================================================================

// Pin connections for using SPI or I2C interface
// ADXL355 pin    Arduino pin
//                SPI     I2C
// CS/SCL          2      SCL
// MOSI/SDA       MOSI    SDA
// MISO/ASEL      MISO    GND
// SCLK/Vssio     SCLK    GND

PL::ADXL355 adxl355;
Adafruit_MPU6050 mpu;
//uint8_t spiCsPin = 2;
uint8_t i2cAddress = 0x53;
uint8_t I2c_SDAPIN=21;
uint8_t I2c_SCLPIN=37;
const float L=0.5;

// ADXL355 range: +/- 2 g
auto range = PL::ADXL355_Range::range2g;

//==============================================================================

void setup() {
  
  // Initialize ADXL355 (uncomment one of the following 2 lines to use either SPI or I2C)
  //adxl355.beginSPI(spiCsPin);
  Wire.begin(I2c_SDAPIN, I2c_SCLPIN);
  adxl355.beginI2C(i2cAddress);

  // Set ADXL355 range
  adxl355.setRange(range);
  // Enable ADXL355 measurement
  adxl355.enableMeasurement();
  
  // Initialize Serial at 115200 kbps
  Serial.begin(115200);
  // Wait for Serial ready state
  while(!Serial);
  Serial.println("\nI2C Scanner");

  auto deviceInfo = adxl355.getDeviceInfo();
  if (deviceInfo.deviceId != 0xED) {
      Serial.println("Could not find ADXL355!");
      while (1);
  }
  Serial.println("ADXL355 initialized successfully!");
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  delay(1000);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}


//==============================================================================
void lermedidas(){
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(1000);   
  int nummedidas = 10;
  float xsoma = 0, ysoma = 0, zsoma = 0;
  float somaOx = 0, somaOy = 0, somaOz = 0;
  float deviacaocumulativax = 0, deviacaocumulativay = 0, deviacaocumulativaz = 0;

  for (int j = 0; j < nummedidas; j++) {
    auto accelerations = adxl355.getAccelerations();
    
    float Ox = atan2(accelerations.x, (sqrt(accelerations.y * accelerations.y)) + (sqrt(accelerations.z * accelerations.z))) * 180 / M_PI;
    float Oy = atan2(accelerations.y, (sqrt(accelerations.x * accelerations.x)) + (sqrt(accelerations.z * accelerations.z))) * 180 / M_PI;
    float Oz = atan2(accelerations.z, (sqrt(accelerations.x * accelerations.x)) + (sqrt(accelerations.y * accelerations.y))) * 180 / M_PI;

    xsoma += accelerations.x;
    ysoma += accelerations.y;
    zsoma += accelerations.z;
    somaOx += Ox;
    somaOy += Oy;
    somaOz += Oz;

    deviacaocumulativax += (Ox * L) * (M_PI / 180);
    deviacaocumulativay += (Oy * L) * (M_PI / 180);
    deviacaocumulativaz += (Oz * L) * (M_PI / 180);
  }

  float xraw = xsoma / nummedidas;
  float yraw = ysoma / nummedidas;
  float zraw = zsoma / nummedidas;
  float Oxraw = somaOx / nummedidas;
  float Oyraw = somaOy / nummedidas;
  float Ozraw = somaOz / nummedidas;

  Serial.print("Accelerations: X: "); Serial.print(xraw);
  Serial.print(" g, Y: "); Serial.print(yraw);
  Serial.print(" g, Z: "); Serial.print(zraw); Serial.println(" g");

  Serial.print("Angles : OX: "); Serial.print(Oxraw);
  Serial.print(" degrees, OY: "); Serial.print(Oyraw);
  Serial.print(" degrees, OZ: "); Serial.print(Ozraw); Serial.println(" degrees");

  Serial.print("Deviation : DX: "); Serial.print(deviacaocumulativax);
  Serial.print(" meters, DY: "); Serial.print(deviacaocumulativay);
  Serial.print(" meters, DZ: "); Serial.print(deviacaocumulativaz); Serial.println(" meters");

  Serial.println("Cálculo de nova média");

  return;
}


void loop() {
 lermedidas();
 sensors_event_t a, g, temp;
 mpu.getEvent(&a, &g, &temp);

 /* Print out the values */
 Serial.print("Acceleration X: ");
 Serial.print(a.acceleration.x);
 Serial.print(", Y: ");
 Serial.print(a.acceleration.y);
 Serial.print(", Z: ");
 Serial.print(a.acceleration.z);
 Serial.println(" m/s^2");

 Serial.print("Rotation X: ");
 Serial.print(g.gyro.x);
 Serial.print(", Y: ");
 Serial.print(g.gyro.y);
 Serial.print(", Z: ");
 Serial.print(g.gyro.z);
 Serial.println(" rad/s");

 Serial.print("Temperature: ");
 Serial.print(temp.temperature);
 Serial.println(" degC");

 Serial.println("");
 delay(1000);
  
 
}
