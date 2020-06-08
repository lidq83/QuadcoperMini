#include <pwm.h>

static int pwm_open(struct file *fs);
static int pwm_close(struct file *fs);
static int pwm_ioctl(struct file *fs, unsigned int cmd, unsigned long arg);

int pwm_open(struct file *fs)
{
	return 0;
}

int pwm_close(struct file *fs)
{
	return 0;
}

int pwm_ioctl(struct file *fs, unsigned int cmd, unsigned long arg)
{
	if (arg > PWM_VAL_MAX)
	{
		arg = PWM_VAL_MAX;
	}
	if (arg < PWM_VAL_MIN)
	{
		arg = PWM_VAL_MIN;
	}

	uint16_t pwm_val = (uint16_t)arg;

	switch (cmd)
	{
	case PWM_CMD_SET_CH0_VALUE:
		tim2_set_value(0, pwm_val);
		break;

	case PWM_CMD_SET_CH1_VALUE:
		tim2_set_value(1, pwm_val);
		break;

	case PWM_CMD_SET_CH2_VALUE:
		tim2_set_value(2, pwm_val);
		break;

	case PWM_CMD_SET_CH3_VALUE:
		tim2_set_value(3, pwm_val);
		break;

	case PWM_CMD_SET_CH4_VALUE:
		tim4_set_value(0, pwm_val);
		break;

	case PWM_CMD_SET_CH5_VALUE:
		tim4_set_value(1, pwm_val);
		break;

	case PWM_CMD_SET_CH6_VALUE:
		tim4_set_value(2, pwm_val);
		break;

	case PWM_CMD_SET_CH7_VALUE:
		tim4_set_value(3, pwm_val);
		break;

	default:
		break;
	}
	return 0;
}

void pwm_init(void)
{
	file_operations_s ops = {0};

	ops.open = &pwm_open;
	ops.close = &pwm_close;
	ops.write = NULL;
	ops.read = NULL;
	ops.ioctl = &pwm_ioctl;

	fs_register_dev("/dev/pwm", ops);
}