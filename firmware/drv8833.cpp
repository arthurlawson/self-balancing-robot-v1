// Define the control inputs
#include "drv8833.h"
#include "esp32-hal.h"
#include <cstdint>

DRV8833::DRV8833(uint8_t iN1, uint8_t iN2, uint8_t iN3, uint8_t iN4)
{
    // Assign the pins
    IN1 = iN1;
    IN2 = iN2;
    IN3 = iN3;
    IN4 = iN4;

    // Set all the motor control inputs to OUTPUT
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Set PWM frequencies for the motors (20kHz)
    analogWriteFrequency(IN1, 20000);
    analogWriteFrequency(IN2, 20000);
    analogWriteFrequency(IN3, 20000);
    analogWriteFrequency(IN4, 20000);
}

void DRV8833::drive(int16_t left, int16_t right) {
    left = constrain(left, -255, 255);
    if (left > 0) {
        analogWrite(IN1, 255 - left);
        analogWrite(IN2, 255);
    } else if (left < 0) {
        analogWrite(IN1, 255);
        analogWrite(IN2, 255 + left);
    } else {
        stop_left();
    }

    right = constrain(right, -255, 255);
    if (right > 0) {
        analogWrite(IN3, 255 - right);
        analogWrite(IN4, 255);
    } else if (right < 0) {
        analogWrite(IN3, 255);
        analogWrite(IN4, 255 + right);
    } else {
        stop_right();
    }
}

void DRV8833::stop_both()
{
    analogWrite(IN1, 255);
    analogWrite(IN2, 255);
    analogWrite(IN3, 255);
    analogWrite(IN4, 255);
}