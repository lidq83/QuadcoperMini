#ifndef __BOARD_H_
#define __BOARD_H_

#ifndef MM_SIZE_HEAP
#define MM_SIZE_HEAP (1024 * 10)
#endif

#ifndef SERIAL_BAUTRATE
#define SERIAL_BAUTRATE (115200)
#endif

#define PROI_LED (30)
#define PROI_BUZZER (28)
#define PROI_DT (26)
#define PROI_DT_TM (25)
#define PROI_MOTOR (24)
#define PROI_NRF (22)
#define PROI_KICK (20)

#define MOTOR_CNT (2)

#define PWM_VAL_MAX (2000)
#define PWM_VAL_MIN (0)

#define PWM_VAL_MID (PWM_VAL_MIN + ((PWM_VAL_MAX - PWM_VAL_MIN) / 2))

#endif