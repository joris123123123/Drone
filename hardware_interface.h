#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

/*
 * DRONE HARDWARE INTERFACE
 * ========================
 *
 * MOTOR CONFIGURATION:
 * - Motor order: back-right, front-left, front-right, back-left
 * - Pulse range: 1000-2000 microseconds
 * - Idle pulse: 1000 microseconds
 *
 * RC INPUT:
 * - Channels: Roll, Pitch, Throttle, Yaw, Arm, Mode
 * - Pulse range: 1000-2000 microseconds
 * - Arm threshold: 1800 microseconds
 * - Disarm threshold: 1500 microseconds
 *
 * SENSOR HARDWARE:
 * - MPU6050 gyro/accelerometer
 * - Gyro scale factor: 131 LSB per deg/s
 * - Accelerometer calibration: 0.75g to 1.25g range
 *
 * CONTROL LOOP:
 * - Rate: 100 Hz
 * - PID parameters (pitch axis):
 *   - Kp: 10
 *   - Ki: 0  
 *   - Kd: 0
 *   - I limit: 30000
 *   - Output limit: 800
 *   - D filter alpha: 64
 *
 * HARDWARE CONNECTIONS:
 * - ESCs connected to: 
 *   - Back-right: PORTD bit 7 (ESC_BR)
 *   - Front-left: PORTD bit 6 (ESC_FL)  
 *   - Front-right: PORTD bit 5 (ESC_FR)
 *   - Back-left: PORTB bit 4 (ESC_BL)
 * - MPU6050 connected via I2C:
 *   - SDA: PORTC bit 4
 *   - SCL: PORTC bit 5
 * - SBUS receiver connected to USART0:
 *   - RX: PORTD bit 0
 *   - TX: PORTD bit 1
 *
 * POWER REQUIREMENTS:
 * - ESCs: 5V power supply
 * - MPU6050: 3.3V or 5V
 * - Microcontroller: 5V (ATmega328P)
 */

#endif // HARDWARE_INTERFACE_H
