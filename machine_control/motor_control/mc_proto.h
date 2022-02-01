#ifndef _MC_PROTO_H_
#define _MC_PROTO_H_

#define BOARD_TRANSFER_CHUNK			16

#define MC_RC_CODE_SET_PWM				0
#define MC_RC_CODE_GET_ENCODERS			1

typedef struct {
	uint8_t m;
	uint8_t b;
	uint8_t code;
	uint8_t payload[BOARD_TRANSFER_CHUNK - 3];
} __attribute__((packed)) mc_control_cmd_t;

typedef struct {
	float pwm_l;
	float pwm_r;
} __attribute__((packed)) mc_control_speeds_t;

typedef struct {
	int32_t pulses_l;
	int32_t pulses_r;
} __attribute__((packed)) mc_control_encoders_t;

#endif //_MC_PROTO_H_
