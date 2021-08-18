#include "grove_driver.h"

static struct {
    unsigned char _speed1;
    unsigned char _speed2;
    int _M1_direction;
    int _M2_direction;
    unsigned char _i2c_add;

    unsigned char _msg_buff[8];
    unsigned char _msg_pos;
} _state = {
    ._speed1 = 0,
    ._speed2 = 0,
    ._M1_direction = 1,
    ._M2_direction = 1,
    ._i2c_add = 0x0f,
    ._msg_pos = 0
};

// ****************************************************************************

static void log_println(const char* msg);
static unsigned char map(unsigned char val, unsigned char val_min,
        unsigned char val_max, unsigned char out_min, unsigned char out_max);
static void i2c_msg_write(unsigned char byte);
static int i2c_msg_send();

extern int i2c_write(const unsigned char *buff, unsigned int len);
extern void delay(unsigned int ms);

// *********************************Initialize*********************************
// Initialize I2C with an I2C address you set on Grove - I2C Motor Driver v1.3
// default i2c address: 0x0f
int grove_begin(unsigned char i2c_add) {
    if (i2c_add > 0x0f) {
        log_println("Error! I2C address must be between 0x00 to 0x0F");
        return (-1); // I2C address error
    }
    _state._i2c_add = i2c_add;
    
    // Set default frequence to F_3921Hz
    grove_frequence(F_3921Hz);

    return (0); // OK
}

// *****************************Private Function*******************************
// Set the direction of 2 motors
// _direction: M1CWM2ACW(M1 ClockWise M2 AntiClockWise), M1ACWM2CW, BothClockWise, BothAntiClockWise,
void grove_direction(unsigned char _direction) {
    i2c_msg_write(_state._i2c_add); // begin transmission
    i2c_msg_write(DirectionSet);    // Direction control header
    i2c_msg_write(_direction);      // send direction control information
    i2c_msg_write(Nothing);         // need to send this byte as the third byte(no meaning)
    i2c_msg_send();                 // stop transmitting
}

// *****************************DC Motor Function******************************
// Set the speed of a motor, speed is equal to duty cycle here
// motor_id: MOTOR1, MOTOR2
// _speed: -100~100, when _speed>0, dc motor runs clockwise; when _speed<0,
// dc motor runs anticlockwise
void grove_speed(unsigned char motor_id, int _speed) {
    if (motor_id < MOTOR1 || motor_id > MOTOR2) {
        log_println("Motor id error! Must be MOTOR1 or MOTOR2");
        return;
    }

    if (motor_id == MOTOR1) {
        if (_speed >= 0) {
            _state._M1_direction = 1;
            _speed = _speed > 100 ? 100 : _speed;
            _state._speed1 = map(_speed, 0, 100, 0, 255);
        } else if (_speed < 0) {
            _state._M1_direction = -1;
            _speed = _speed < -100 ? 100 : -(_speed);
            _state._speed1 = map(_speed, 0, 100, 0, 255);
        }
    } else if (motor_id == MOTOR2) {
        if (_speed >= 0) {
            _state._M2_direction = 1;
            _speed = _speed > 100 ? 100 : _speed;
            _state._speed2 = map(_speed, 0, 100, 0, 255);
        } else if (_speed < 0) {
            _state._M2_direction = -1;
            _speed = _speed < -100 ? 100 : -(_speed);
            _state._speed2 = map(_speed, 0, 100, 0, 255);
        }
    }
    // Set the direction
    if (_state._M1_direction == 1 && _state._M2_direction == 1) {
        grove_direction(BothClockWise);
    }
    if (_state._M1_direction == 1 && _state._M2_direction == -1) {
        grove_direction(M1CWM2ACW);
    }
    if (_state._M1_direction == -1 && _state._M2_direction == 1) {
        grove_direction(M1ACWM2CW);
    }
    if (_state._M1_direction == -1 && _state._M2_direction == -1) {
        grove_direction(BothAntiClockWise);
    }
    // send command
    i2c_msg_write(_state._i2c_add); // begin transmission
    i2c_msg_write(MotorSpeedSet);   // set pwm header
    i2c_msg_write(_state._speed1);  // send speed of motor1
    i2c_msg_write(_state._speed2);  // send speed of motor2
    i2c_msg_send();    		        // stop transmitting
}

// Set the frequence of PWM(cycle length = 510, system clock = 16MHz)
// F_3921Hz is default
// _frequence: F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz
void grove_frequence(unsigned char _frequence) {
    if (_frequence < F_31372Hz || _frequence > F_30Hz) {
        log_println("frequence error! Must be F_31372Hz, F_3921Hz, F_490Hz, F_122Hz, F_30Hz");
        return;
    }

    i2c_msg_write(_state._i2c_add); // begin transmission
    i2c_msg_write(PWMFrequenceSet); // set frequence header
    i2c_msg_write(_frequence);      // send frequence
    i2c_msg_write(Nothing);         // need to send this byte as the third byte(no meaning)
    i2c_msg_send();                 // stop transmitting
}

// Stop one motor
// motor_id: MOTOR1, MOTOR2
void grove_stop(unsigned char motor_id) {
    if (motor_id < MOTOR1 || motor_id > MOTOR2) {
        log_println("Motor id error! Must be MOTOR1 or MOTOR2");
        return;
    }
    grove_speed(motor_id, 0);
}

// ****************************************************************************

static void log_println(const char* msg) {
}

static unsigned char map(unsigned char val, unsigned char val_min,
        unsigned char val_max, unsigned char out_min, unsigned char out_max) {
    unsigned int val_dl = val - val_min;
    unsigned int val_dx = val_max - val_min;
    unsigned int val_dy = out_max - out_min;

    val_dl <<= 8;
    val_dl /= val_dx;
    val_dl *= val_dy;
    val_dl >>= 8;

    return out_min + val_dl;
}

static void i2c_msg_write(unsigned char byte) {
    _state._msg_buff[_state._msg_pos++] = byte;
}

static int i2c_msg_send() {
    int res = i2c_write(_state._msg_buff, _state._msg_pos);
    _state._msg_pos = 0;
    delay(100);
    return res;
}
