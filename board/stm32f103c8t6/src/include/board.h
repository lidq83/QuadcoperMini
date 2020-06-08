#ifndef __BOARD_H_
#define __BOARD_H_

#ifndef MM_SIZE_HEAP
#define MM_SIZE_HEAP (1024 * 10)
#endif

#ifndef SERIAL_BAUTRATE
#define SERIAL_BAUTRATE (115200)
#endif


#define MOTOR_CNT (2)

// #define MOTOR_TYPE_CAR
#define MOTOR_TYPE_BOAT



#ifdef MOTOR_TYPE_CAR
#define PWM_VAL_MAX (2000)
#define PWM_VAL_MIN (0)
#endif

#ifdef MOTOR_TYPE_BOAT
#define PWM_VAL_MAX (2000)
#define PWM_VAL_MIN (800)
#endif

#define PWM_VAL_MID	(PWM_VAL_MIN+((PWM_VAL_MAX-PWM_VAL_MIN)/2))

#endif