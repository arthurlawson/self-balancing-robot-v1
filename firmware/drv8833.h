/**
 * @file drv8833.cpp
 * @brief DRV8833 Dual H-Bridge Motor Driver Source Configuration
 * @details Implements active slow-decay (braking) for the 2 BDC TT Motors
 * 
 * @author Arthur Lawson (Hardware driver architecture & design)
 */

#ifndef DRV8833_H
#define DRV8833_H
#include "Arduino.h"

class DRV8833 {
    public:
        DRV8833(uint8_t iN1, uint8_t iN2, uint8_t iN3, uint8_t iN4);
        void drive(int16_t left, int16_t right);
        void stop_left();
        void stop_right();
        void stop_both();
    private:
        uint8_t IN1, IN2, IN3, IN4;
};

#endif // !DRV8833_H
