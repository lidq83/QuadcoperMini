#include <stdint.h>
#include <fcntl.h>
#include <fs.h>
#include <std.h>
#include <serial1.h>

int stdout_write(struct file *f, const void *buff, size_t size)
{
	uint8_t *p = (uint8_t *)buff;
	for (int i = 0; i < size; i++)
	{
		serial1_write(p[i]);
	}
	return size;
}

size_t stdin_read(struct file *fs, void *buff, size_t size)
{
	uint8_t *p = (uint8_t *)buff;
	size_t read_len = 0;
	for (int i = 0; i < size; i++)
	{
		if (!serial1_read(&p[read_len]))
		{
			return read_len;
		}
		read_len++;
	}
	return size;
}

int stdin_init(void)
{
	file_operations_s ops = {0};
	ops.read = stdin_read;
	return fs_register_dev("/dev/stdin", ops);
	return 0;
}

int stdout_init(void)
{
	file_operations_s ops = {0};
	ops.write = stdout_write;
	return fs_register_dev("/dev/stdout", ops);
}

int stderr_init(void)
{
	return 0;
}

int std_init(void)
{
	stdin_init();
	stdout_init();
	stderr_init();

	return 0;
}
