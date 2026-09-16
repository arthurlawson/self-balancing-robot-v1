/**
 * @file balancing_robot.ino
 * @brief Self-Balancing Robot Firmware (Version 1 - Prototype)

 * @author https://github.com/temperancee (Kalman and PID system)
 * @author Arthur Lawson (Refactoring code and control design)
 */

#include <Wire.h>
#include "pid.h"
#include "kalman.h"
#include "Adafruit_MPU6050.h"
#include "drv8833.h"

//================================================================================
// CONFIGURATION
//================================================================================

// Main control loop execution interval (in millesecoonds)
const uint8_t DELTA_T = 5;

// PID control loop gains
const float KP = 20.0f;                 // Proportional gain
const float KI = 150.0f;                // Integral gain
const float KD = 0.7f;                  // Derivative gain
const float MIN_PWM = 30.0f;            // Minimum PWM for torque to beat static friction of the motors
const float DEFAULT_SETPOINT = 1.60f;   // Target center of mass point

// Angle error where aggressive gain scaling activates
const float START_EXPONENTIAL = 1.5f; 

// Hardware Pinout
const uint16_t IN1 = 4, IN2 = 5, IN3 = 6, IN4 = 7;  // DRV8833 Motor driver logic control pins
const uint16_t SCL = 17, SDA = 18;                  // IMU logic control pins
const uint16_t BATTERY_PIN = 2, LED_PIN = 21;       // Analog voltage divider and status LED pins

// Safety Thresholds
const float ACTIVATION_ANGLE = 3.0f;        // Angle to activate the robot
const float LOW_VOLTAGE_THRESHOLD = 6.0f;   // Prevents lipo battery damage
const float STOP_ANGLE = 80.0f;             // Fall over past this angle = the motors will stop to prevent damage

//================================================================================
// GLOBAL
//================================================================================

// Clock coordinator between the hardware ISR and loop thread
volatile SemaphoreHandle_t timerSemaphore;

// Global Objects
Adafruit_MPU6050    mpu;
KalmanFilter        kf(DELTA_T);
DRV8833             motors(IN1, IN2, IN3, IN4);
PID                 pid(DEFAULT_SETPOINT, KP, KI, KD, DELTA_T/1000.0);

// Pre allocated Registers
sensors_event_t accel, gyro;
float gyro_readings[3];
float accel_readings[3];

// Operational variables
bool robot_active = false;
float linear_speed = 0.0f;
float last_pid_out = 0.0f;
float dynamic_setpoint = DEFAULT_SETPOINT;

//================================================================================
// MOVEMENT MANAGEMENT
//================================================================================

void handle_idle(float cur_angle) {
  // Low pass filter for estimatic drift velocity
  linear_speed = (linear_speed * 0.98) + (last_pid_out * -0.02);
  Serial.println(linear_speed);

  // Anti-drift adjustments to modify the target leaning setpoint if center of mass shifts
  float target_setpoint = DEFAULT_SETPOINT;
  if (abs(linear_speed) > 5.0f) {
    float excess_speed = abs(linear_speed) - 5.0f;
    float braking_offset = excess_speed * 0.06f;
    if (linear_speed < 0) braking_offset *= -1.0f;
    target_setpoint = DEFAULT_SETPOINT - braking_offset;
  }

  // Setpoint interpolation profiles (Snap back faster)
  float smoothing = 0.1f;
  if (abs(target_setpoint - DEFAULT_SETPOINT) < abs(dynamic_setpoint - DEFAULT_SETPOINT)) {
    smoothing = 0.3f; // "Snap" back to center if pushed
  }
  dynamic_setpoint += (target_setpoint - dynamic_setpoint) * smoothing;
  dynamic_setpoint = constrain(dynamic_setpoint, DEFAULT_SETPOINT - 2.5f, DEFAULT_SETPOINT + 2.5f);

  float angle_error = abs(dynamic_setpoint - cur_angle);

  // Temporary variable placeholders for processing control logic
  float kp_exp_scaler = 1.0f;
  float kd_exp_scaler = 1.0f;
  float dynamic_kp = KP;
  float dynamic_kd = KD;
  float dynamic_min_pwm = MIN_PWM;

  // Exponential gain scaling - scales the loop coefficients if the robot enters high deviation states
  if (angle_error > START_EXPONENTIAL) {
    float overshoot = angle_error - START_EXPONENTIAL;
    kp_exp_scaler = 1.0f + pow(overshoot, 4.5f);
    kd_exp_scaler = 1.0f + pow(overshoot, 4.0f);
    
    dynamic_kp = KP * kp_exp_scaler;
    dynamic_kp = constrain(dynamic_kp, KP, 700.0f);

    dynamic_kd = KD * kd_exp_scaler;
    dynamic_kd = constrain(dynamic_kd, KD, 50.0f);

    dynamic_min_pwm = MIN_PWM + (pow(overshoot, 3.0f) * 3.0f); 
    dynamic_min_pwm = constrain(dynamic_min_pwm, MIN_PWM, 200.0f);
  }

  // Assign modifications to the PID system
  pid.set_kp(dynamic_kp);
  pid.set_kd(dynamic_kd);
  pid.set_setpoint(dynamic_setpoint);

  // Emergency fall protection safety latch
  if (abs(cur_angle - DEFAULT_SETPOINT) > STOP_ANGLE || accel.acceleration.z < -8.0f) {
    motors.stop_both();
    digitalWrite(LED_PIN, HIGH);
    robot_active = false;
    kf.reset();
    pid.reset();
    return;
  }

  // Compute the final drive values from the PID algorithm
  float pid_out = pid.control(cur_angle, 255.0f);
  last_pid_out = pid_out;

  float left_out = pid_out;
  float right_out = pid_out;

  // Handle motor deadzone mapping constraints
  if (angle_error > 0.05f) { // was 0.2
    if (left_out > 0)       left_out = map(left_out, 0, 255, dynamic_min_pwm, 255);
    else if (left_out < 0)  left_out = map(left_out, -255, 0, -255, -dynamic_min_pwm);
    if (right_out > 0)      right_out = map(right_out, 0, 255, dynamic_min_pwm, 255);
    else if (right_out < 0) right_out = map(right_out, -255, 0, -255, -dynamic_min_pwm);
    motors.drive((int16_t)constrain(left_out, -255, 255), (int16_t)constrain(right_out, -255, 255));
  }
  else {
    // If insnide the balance deadzone limits, slowly draint he integral to prevent windup
    pid.leak_integral(0.95f);
    motors.stop_both();
  }
}

//================================================================================
// BATTERY MANAGEMENT & INTERUPT SERVICE ROUTINE (ISR)
//================================================================================

// Checks battery voltage and latches a shutdown if low for >3 seconds.
bool is_battery_low() {
  static bool latched_low = false;
  static unsigned long low_start_time = 0;
  static unsigned long last_check_time = 0;

  const unsigned long CHECK_INTERVAL = 100; // Sample voltage telemetry every 100ms
  const unsigned long KILL_DURATION = 3000; // If the battery is below the threshold voltage for 3 seconds, latch the robot off

  if (latched_low) return true; 

  if (millis() - last_check_time >= CHECK_INTERVAL) {
    last_check_time = millis();
    
    // Convert 12-bit ADC range (0-4095) relative to 3.3V, scale by the 3x hardware resistor divider ratio
    int sensor_value = analogRead(BATTERY_PIN);
    float voltage = (sensor_value / 4095.0f) * 3.3f * (3.0f);
    
    if (voltage < LOW_VOLTAGE_THRESHOLD) {
      if (low_start_time == 0) {
        low_start_time = millis(); // Initialize safety deadline check
      }
      else if (millis() - low_start_time >= KILL_DURATION) {
        latched_low = true; // Lock the system low battery proteection latch
      }
    } else {
      low_start_time = 0; // Clear counters if supply voltage stabalizes above the threshold voltage
    }
  }
  return latched_low;
}

// ISR triggered by a hardware timer clock.
void ARDUINO_ISR_ATTR onTimer() {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

//================================================================================
// MAIN SETUP AND LOOP
//================================================================================

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Status Indicator: System booting up

  Serial.begin(115200);

  Wire.begin(17, 18);
  Wire.setClock(400000);  // Fast clock speed
  Wire.setTimeOut(50);    // Prevent I2C hangs
  delay(1000);            // Initial warm up time for setup
  
  // Initialize the MPU6050
  if (!mpu.begin()) {
    while (1) { // Flashing LED if IMU tracking not working
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  // Hardware timer setup
  hw_timer_t *timer = NULL;
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(1000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, DELTA_T, true, 0);

  digitalWrite(LED_PIN, LOW); // LED off if working
}

void loop() {

  // Battery Protection safety latch
  if (is_battery_low()) {
    motors.stop_both();
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Cycle execution by the hardware timer sephamores
  if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {

    // Try read IMU data
    if (!mpu.getEvent(&accel, &gyro)) {
      Wire.begin(17, 18);
      mpu.begin();
      delay(10);
      return; 
    }

    // Read the IMU data
    gyro_readings[0] = gyro.gyro.x;
    gyro_readings[1] = gyro.gyro.y;
    gyro_readings[2] = gyro.gyro.z;
    accel_readings[0] = accel.acceleration.x;
    accel_readings[1] = accel.acceleration.y;
    accel_readings[2] = accel.acceleration.z;

    // Process Kalman Filter matrix math
    kf.predict(gyro_readings);
    kf.measurement_task(accel_readings);
    float cur_angle = -kf.get_roll();  // Invert output vector due to inverted physical orientation mapping

    // Activation Logic (only if held upright will the motors begin)
    if (!robot_active) {
      kf.reset();  // Keep filters fresh
      pid.reset(); // Prevent build up while holding
      linear_speed = 0;
      last_pid_out = 0;
      dynamic_setpoint = DEFAULT_SETPOINT;

      if (abs(cur_angle - DEFAULT_SETPOINT) < ACTIVATION_ANGLE) {
        robot_active = true; // Arm the system drivers
      } else {
        motors.stop_both();
        digitalWrite(LED_PIN, HIGH);
        return;
      }
    }

    // Active balancing LED status off
    digitalWrite(LED_PIN, LOW); 

    // Handle the only state for the robot (IDLE)
    handle_idle(cur_angle);
  }
}
