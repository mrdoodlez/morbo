#ifndef __I2CMOTORDRIVER_H__
#define __I2CMOTORDRIVER_H__

/******I2C command definitions*************/
#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
/**************Motor ID**********************/
#define MOTOR1 			          1
#define MOTOR2 			          2
/**************Motor Direction***************/
#define BothClockWise             0x0a
#define BothAntiClockWise         0x05
#define M1CWM2ACW                 0x06
#define M1ACWM2CW                 0x09
/**************Motor Direction***************/
#define ClockWise                 0x0a
#define AntiClockWise             0x05
/**************Prescaler Frequence***********/
#define F_31372Hz                 0x01
#define F_3921Hz                  0x02
#define F_490Hz                   0x03
#define F_122Hz                   0x04
#define F_30Hz                    0x05


// Set the direction of both motors
// _direction: BothClockWise, BothAntiClockWise, M1CWM2ACW, M1ACWM2CW
void grove_direction(unsigned char _direction);

// Initialize I2C with an I2C address you set on Grove - I2C Motor Driver v1.3
// default i2c address: 0x0f
int grove_begin(unsigned char i2c_add);

// Set the speed of a motor, speed is equal to duty cycle here
// motor_id: MOTOR1, MOTOR2
// _speed: -100~100, when _speed>0, dc motor runs clockwise;
// when _speed<0, dc motor runs anticlockwise
void grove_speed(unsigned char motor_id, int _speed);

// Set the frequence of PWM(cycle length = 510, system clock = 16MHz)
// F_3921Hz is default
// _frequence: F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz
void grove_frequence(unsigned char _frequence);

// Stop one motor
// motor_id: MOTOR1, MOTOR2
void grove_stop(unsigned char motor_id);

#endif //__I2CMOTORDRIVER_H__