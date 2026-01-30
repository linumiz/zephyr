#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define led0_THREAD_STACK_SIZE  1024
#define led1_THREAD_STACK_SIZE  1024
K_THREAD_STACK_DEFINE(led0_thread_stack, led0_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(led1_thread_stack, led1_THREAD_STACK_SIZE);
static struct k_thread led0_thread_data;
static struct k_thread led1_thread_data;

#define led0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(led0_NODE, gpios);

#define led1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(led1_NODE, gpios);

static void led0_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	bool led0_state = false;

	if (!gpio_is_ready_dt(&led0)) {
		return;
	}

	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	while (1) {
		led0_state = !led0_state;
		gpio_pin_set_dt(&led0, led0_state);
		printk("\nLED T0: CPU: %d\n", _current_cpu->id);
		k_busy_wait(50000);
	}
}

static void led1_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	bool led1_state = false;

	if (!gpio_is_ready_dt(&led1)) {
		return;
	}

	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

	while (1) {
		led1_state = !led1_state;
		gpio_pin_set_dt(&led1, led1_state);
		printk("\nLED T1: CPU: %d\n", _current_cpu->id);
		k_busy_wait(50000);
	}
}

int main(void)
{
	printk("Main thread from core: [M7_%d]\n", _current_cpu->id);

	k_thread_create(&led0_thread_data,
	                                led0_thread_stack,
	                                led0_THREAD_STACK_SIZE,
	                                led0_thread_entry,
	                                NULL, NULL, NULL,
	                                5,
	                                0,
	                                K_NO_WAIT);

	k_thread_create(&led1_thread_data,
	                                led1_thread_stack,
	                                led1_THREAD_STACK_SIZE,
	                                led1_thread_entry,
	                                NULL, NULL, NULL,
	                                5,
	                                0,
	                                K_NO_WAIT);
	return 0;
}
