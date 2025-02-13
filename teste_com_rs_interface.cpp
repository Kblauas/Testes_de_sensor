#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PL_ADXL355.h>
#include <math.h>

// Initialize our sensors
Adafruit_MPU6050 mpu;
PL::ADXL355 adxl355;

// ADXL355 configuration
uint8_t i2cAddress = 0x53;
uint8_t I2c_SDAPIN = 21;
uint8_t I2c_SCLPIN = 37;
auto range = PL::ADXL355_Range::range2g;

// RS485 configuration
HardwareSerial mySerial(1);  // Use Serial1 for RS485
#define RS485_CONTROL_PIN 3   // Pin to control RS485 direction
bool sistemaAtivo = true;     // System state control

// Variables for sensor offsets (calibration)
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Variables for orientation calculation
float pitch = 0.0;
float roll = 0.0;
float a_pitch = 0.0;
float a_roll = 0.0;

// Constants for calculations
const float kalman_gain = 0.28;  // Kalman filter gain for sensor fusion
const float l = 0.5;             // Length constant for position calculation
const int INTERVALO_LEITURA = 1; // Reading interval in milliseconds

// Variables for averaging
const int AVERAGE_COUNT = 50;    // Number of readings to average
float mpu_ax_sum = 0, mpu_ay_sum = 0, mpu_az_sum = 0;
float mpu_gx_sum = 0, mpu_gy_sum = 0;
float adxl_ax_sum = 0, adxl_ay_sum = 0, adxl_az_sum = 0;
float pitch_sum = 0, roll_sum = 0;
float a_pitch_sum = 0, a_roll_sum = 0;
int reading_count = 0;

// Variables to store averages for RS485 transmission
float avg_pitch = 0, avg_roll = 0;
float avg_a_pitch = 0, avg_a_roll = 0;

void calcularOffset() {
    Serial.println("Calculating offset...");
    const int numLeituras = 100;
    sensors_event_t a, g, temp;

    // Take multiple readings to establish baseline
    for (int i = 0; i < numLeituras; i++) {
        mpu.getEvent(&a, &g, &temp);
        ax_offset += a.acceleration.x;
        ay_offset += a.acceleration.y;
        az_offset += a.acceleration.z;
        gx_offset += g.gyro.x;
        gy_offset += g.gyro.y;
        gz_offset += g.gyro.z;
        delay(5);
    }

    // Calculate average offsets
    ax_offset /= numLeituras;
    ay_offset /= numLeituras;
    az_offset /= numLeituras;
    gx_offset /= numLeituras;
    gy_offset /= numLeituras;
    gz_offset /= numLeituras;

    Serial.println("Offset calculated!");
}

void setupRS485() {
    // Initialize RS485 serial communication
    mySerial.begin(115200, SERIAL_8N1, 5, 4);  // RX=5, TX=4
    pinMode(RS485_CONTROL_PIN, OUTPUT);
    digitalWrite(RS485_CONTROL_PIN, LOW);  // Set to receive mode initially
}

void setup() {
    Serial.begin(115200);
    setupRS485();
    Wire.begin(I2c_SDAPIN, I2c_SCLPIN);

    Serial.println("Initializing...");

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Could not find MPU6050!");
        while (1);
    }
    Serial.println("MPU6050 initialized successfully!");

    // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

    // Initialize ADXL355
    adxl355.beginI2C(i2cAddress);
    adxl355.setRange(range);
    adxl355.enableMeasurement();

    // Verify ADXL355 connection
    auto deviceInfo = adxl355.getDeviceInfo();
    if (deviceInfo.deviceId != 0xED) {
        Serial.println("Could not find ADXL355!");
        while (1);
    }
    Serial.println("ADXL355 initialized successfully!");

    // Calculate initial offsets
    calcularOffset();
}

void calcularPosicao(float x, float y, float z, float gx, float gy, float dt, float ax, float ay, float az) {
    dt = dt / 1000;  // Convert milliseconds to seconds
    
    // Update angles using gyroscope data
    //pitch += gy * dt;
    //roll += gx * dt;

    // Calculate angles from accelerometer data
    float pitch_accel = degrees(atan2(x, (sqrt(y * y) +sqrt( z * z)))); 
    float roll_accel = degrees(atan2(y, (sqrt(x * x) + sqrt(z * z)))); 

    // Combine using Kalman filter
    pitch = (1 - kalman_gain) * pitch + kalman_gain * pitch_accel;
    roll = (1 - kalman_gain) * roll + kalman_gain * roll_accel;

    // Calculate angles from ADXL355 data
    a_pitch = atan2(ax, (sqrt(ay * ay)) + (sqrt(az * z))) * 180 / M_PI;
    a_roll = atan2(ay, (sqrt(ax * ax)) + (sqrt(az * az))) * 180 / M_PI;
}

void enviarDadosRS485() {
    // Calculate position from pitch angle
    int posicao = (sin(radians(avg_pitch)) * l) * 100;
    int a_pos = (sin(radians(avg_a_pitch)) * l) * 100;
    
    // Convert floating point values to fixed point (multiply by 100)
    int p = avg_pitch * 100;
    int r = avg_roll * 100;
    int a_p = avg_a_pitch * 100;
    int a_r = avg_a_roll * 100;

    // Prepare data packet
    byte buffer[13];
    buffer[0] = 0xAA;  // Header byte
    
    // Pack pitch and roll data
    buffer[1] = (p >> 8) & 0xFF;
    buffer[2] = p & 0xFF;
    buffer[3] = (r >> 8) & 0xFF;
    buffer[4] = r & 0xFF;
    
    // Pack position data
    buffer[5] = (posicao >> 8) & 0xFF;
    buffer[6] = posicao & 0xFF;
    
    // Pack ADXL pitch and roll data
    buffer[7] = (a_p >> 8) & 0xFF;
    buffer[8] = a_p & 0xFF;
    buffer[9] = (a_r >> 8) & 0xFF;
    buffer[10] = a_r & 0xFF;
    
    // Pack ADXL position data
    buffer[11] = (a_pos >> 8) & 0xFF;
    buffer[12] = a_pos & 0xFF;

    // Send data over RS485
    digitalWrite(RS485_CONTROL_PIN, HIGH);  // Enable transmission
    mySerial.write(buffer, 13);            // Send buffer
    mySerial.flush();                      // Wait for transmission to complete
    digitalWrite(RS485_CONTROL_PIN, LOW);   // Return to receive mode
}

void verificarComandoRS485() {
    if (mySerial.available()) {
        String comando = "";
        while (mySerial.available()) {
            char c = mySerial.read();
            comando += c;
        }
        
        // Process received commands
        if (comando == "P") {
            sistemaAtivo = false;
            Serial.println("System paused!");
        } else if (comando == "R") {
            sistemaAtivo = true;
            Serial.println("System resumed!");
        }
    }
}

void printAverages() {
    // Calculate averages
    avg_pitch = pitch_sum / AVERAGE_COUNT;
    avg_roll = roll_sum / AVERAGE_COUNT;
    avg_a_pitch = a_pitch_sum / AVERAGE_COUNT;


    avg_a_roll = a_roll_sum / AVERAGE_COUNT;

    // Print MPU6050 averages
    Serial.println("=== AVERAGED READINGS ===");
    Serial.println("MPU6050 Averages:");
    Serial.print("Accel: X="); Serial.print(mpu_ax_sum / AVERAGE_COUNT);
    Serial.print(" Y="); Serial.print(mpu_ay_sum / AVERAGE_COUNT);
    Serial.print(" Z="); Serial.println(mpu_az_sum / AVERAGE_COUNT);
    Serial.print("Gyro: X="); Serial.print(mpu_gx_sum / AVERAGE_COUNT);
    Serial.print(" Y="); Serial.println(mpu_gy_sum / AVERAGE_COUNT);

    // Print ADXL355 averages
    Serial.println("ADXL355 Averages:");
    Serial.print("Accel: X="); Serial.print(adxl_ax_sum / AVERAGE_COUNT);
    Serial.print(" Y="); Serial.print(adxl_ay_sum / AVERAGE_COUNT);
    Serial.print(" Z="); Serial.println(adxl_az_sum / AVERAGE_COUNT);

    // Print angle averages
    Serial.println("Average Angles:");
    Serial.print("Pitch="); Serial.print(avg_pitch);
    Serial.print(" Roll="); Serial.println(avg_roll);
    Serial.print("ADXL Pitch="); Serial.print(avg_a_pitch);
    Serial.print(" ADXL Roll="); Serial.println(avg_a_roll);
    Serial.println("--------------------");

    // Send data over RS485
    enviarDadosRS485();

    // Reset sums for next batch
    mpu_ax_sum = 0; mpu_ay_sum = 0; mpu_az_sum = 0;
    mpu_gx_sum = 0; mpu_gy_sum = 0;
    adxl_ax_sum = 0; adxl_ay_sum = 0; adxl_az_sum = 0;
    pitch_sum = 0; roll_sum = 0;
    a_pitch_sum = 0; a_roll_sum = 0;
    reading_count = 0;
}

void loop() {
    // Check for RS485 commands
    verificarComandoRS485();

    if (sistemaAtivo) {
        static unsigned long ultima_leitura = 0;
        unsigned long tempo_atual = millis();

        if (tempo_atual - ultima_leitura >= INTERVALO_LEITURA) {
            ultima_leitura = tempo_atual;

            // Read from ADXL355
            auto accelerations = adxl355.getAccelerations();
            
            // Read from MPU6050
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            // Apply offsets to MPU6050 readings
            float ax = a.acceleration.x - ax_offset;
            float ay = a.acceleration.y - ay_offset;
            float az = a.acceleration.z - az_offset + 9.81;  // Add gravity compensation
            float gx = g.gyro.x - gx_offset;
            float gy = g.gyro.y - gy_offset;
            ax=ax/9.81;
            ay=ay/9.81;
            az=az/9.81;

            // Calculate position and orientation
            calcularPosicao(ax, ay, az, gx, gy, INTERVALO_LEITURA, 
                          accelerations.x, accelerations.y, accelerations.z);

            // Accumulate values for averaging
            mpu_ax_sum += ax;
            mpu_ay_sum += ay;
            mpu_az_sum += az;
            mpu_gx_sum += gx;
            mpu_gy_sum += gy;
            adxl_ax_sum += accelerations.x;
            adxl_ay_sum += accelerations.y;
            adxl_az_sum += accelerations.z;
            pitch_sum += pitch;
            roll_sum += roll;
            a_pitch_sum += a_pitch;
            a_roll_sum += a_roll;
            
            reading_count++;

            // When we have enough readings, print and transmit the averages
            if (reading_count >= AVERAGE_COUNT) {
                printAverages();
                delay(100);  // Add small delay to make output readable
            }
        }
    }
}
