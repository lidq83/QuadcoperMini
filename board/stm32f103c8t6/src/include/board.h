#ifndef __BOARD_H_
#define __BOARD_H_

#ifndef MM_SIZE_HEAP
#define MM_SIZE_HEAP (1024 * 10)
#endif

#ifndef SERIAL_BAUTRATE
#define SERIAL_BAUTRATE (115200)
#endif

#define PROI_CONTROLLER (12)
#define PROI_MPU6050 (18)
#define PROI_PARAM (20)
#define PROI_NRF (22)
#define PROI_LED (30)

#define MOTOR_CNT (4)

#define PWM_VAL_MAX (1980)
#define PWM_VAL_MIN (880)

#define PWM_VAL_MID (PWM_VAL_MIN + ((PWM_VAL_MAX - PWM_VAL_MIN) / 2))

#endif