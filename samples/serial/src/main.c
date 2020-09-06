#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <string.h>
#include <sys/ring_buffer.h>

#define UART_DEVICE_NAME	"UART_4"
struct ring_buf ring;
uint8_t rx[128];

static void uart_fifo_callback(const struct device *dev, void *user_data)
{
	uint8_t buf;

	if (!uart_irq_update(dev)) {
		printk("retval should always be 1\n");
		return;
	}

	if (uart_irq_rx_ready(dev)) {
		uart_fifo_read(dev, &buf, 1);
		ring_buf_put(&ring, &buf, 1);
	}
}

void main(void)
{
	uint8_t buf;
	const struct device *uart_dev = device_get_binding(UART_DEVICE_NAME);
	if (!uart_dev) {
		printk("Cannot get UART device\n");
		return;
	}

	ring_buf_init(&ring, sizeof(rx), rx);

	uart_irq_callback_set(uart_dev, uart_fifo_callback);
	uart_irq_rx_enable(uart_dev);

	while(true) {
		if(!ring_buf_is_empty(&ring)) {
			if (ring_buf_get(&ring, &buf, 1)) {
				printk("%c", buf);
			}
		}

		k_msleep(100);
	}
}

#if 0
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
#endif
