#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <string.h>

#define UART_DEVICE_NAME	"UART_4"
static const char *poll_data = "This is a POLL test.\r\n";

static int test_poll_out(void)
{
	int i;
	struct device *uart_dev = device_get_binding(UART_DEVICE_NAME);

	if (!uart_dev) {
		printk("Cannot get UART device\n");
		return 1;
	}

	for (i = 0; i < strlen(poll_data); i++) {
		uart_poll_out(uart_dev, poll_data[i]);
	}

	return 0;
}

void main(void)
{
	int ret;

	printk("Hello World! %s\n", CONFIG_BOARD);

	ret = test_poll_out();
	if (ret)
		printk("Unable to write\n");
}
