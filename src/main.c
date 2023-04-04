/*
*To-Do List: 
*3. Declare setup channels, check_interfaces,and read_adc
*4. Write the code for setup channels, and check_interfaces
*5. Check the errors in the main 
*6. While Loop read_adc channels and output voltages
*8 Set Up 3 LEDS
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <nrfx_power.h>

LOG_MODULE_REGISTER(Final_Project, LOG_LEVEL_DBG);


#define ADC_DT_SPEC_GET_BY_ALIAS(node_id)                         \
    {                                                            \
        .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(node_id))),        \
        .channel_id = DT_REG_ADDR(DT_ALIAS(node_id)),            \
        ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(node_id))            \
    }                                                            \

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* ADC channels (specified in DT overlay) */

static const struct adc_dt_spec adc_sin100 = ADC_DT_SPEC_GET_BY_ALIAS(sin100);
static const struct adc_dt_spec adc_sin500 = ADC_DT_SPEC_GET_BY_ALIAS(sin500);

/* PWM Channels*/
static const struct pwm_dt_spec mtr_drv1 = PWM_DT_SPEC_GET(DT_ALIAS(drv1)); 
static const struct pwm_dt_spec mtr_drv2 = PWM_DT_SPEC_GET(DT_ALIAS(drv2));


/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return;
		}
		k_msleep(1000);
	}
}
