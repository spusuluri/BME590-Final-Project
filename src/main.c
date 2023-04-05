/*
*To-Do List: 
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
static const struct pwm_dt_spec board_led1_drv = PWM_DT_SPEC_GET(DT_ALIAS(drv1)); 
static const struct pwm_dt_spec board_led2_drv = PWM_DT_SPEC_GET(DT_ALIAS(drv2));

/* LEDs*/
static const struct gpio_dt_spec board_led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec board_led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec board_led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/* Static Variables*/
static int adc_sin100_mV;
static int adc_sin500_mV;

/*Declarations*/
int setup_channels_and_pins(void);
int check_interfaces_ready(void);
int read_adc(struct adc_dt_spec adc_channel);



void main(void)
{
	int err;
	err = check_interfaces_ready();
	if (err){
		LOG_ERR("Device interface not ready (err = %d)", err);
	}
	err = setup_channels_and_pins();
	if (err){
		LOG_ERR("Error configuring IO pins (err = %d", err);
	}

	while (1) {
		err = gpio_pin_toggle_dt(&board_led1);
		if (err < 0) {
			return;
		}
		k_msleep(1000);
	}
}
int read_adc(struct adc_dt_spec adc_channel)
{
	int16_t buf;
	int32_t val_mv;
	int ret;

	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf), // bytes
	};

	//LOG_INF("Measuring %s (channel %d)... ", adc_channel.dev->name, adc_channel.channel_id);

	(void)adc_sequence_init_dt(&adc_channel, &sequence);

	ret = adc_read(adc_channel.dev, &sequence);
	if (ret < 0) {
		//LOG_ERR("Could not read (%d)", ret);
	} else {
		//LOG_DBG("Raw ADC Buffer: %d", buf);
	}

	val_mv = buf;
	ret = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
	if (ret < 0) {
		//LOG_WRN("Buffer cannot be converted to mV; returning raw buffer value.");
		return buf;
	} else {
		//LOG_INF("ADC Value (mV): %d", val_mv);
		return val_mv;
	}
}
int check_interfaces_ready(void){
	/* This should check for the entire gpio0 interface*/
	if (!device_is_ready(board_led1.port)) {
		LOG_ERR("gpio0 interface not ready.");
		return -1;
	}
	/* This should check ADC channels*/
	if (!device_is_ready(adc_sin100.dev) || !device_is_ready(adc_sin500.dev)) {
		LOG_ERR("ADC controller device(s) not ready");
		return -1;
	}
	if (!device_is_ready(board_led1_drv.dev)) {
		LOG_ERR("PWM Device %s is not ready.", board_led1_drv.dev->name);
		return -1;
	}
	
	return 0;	
}
int setup_channels_and_pins(void){
	int ret;

	/* Setup ADC channels */
	ret = adc_channel_setup_dt(&adc_sin100);
	if (ret < 0) {
		LOG_ERR("Could not setup Sin 100 ADC channel (%d)", ret);
		return ret;
	}
	ret = adc_channel_setup_dt(&adc_sin500);
	if (ret < 0) {
		LOG_ERR("Could not setup Sin 500 ADC channel (%d)", ret);
		return ret;
	}
	/* Configure GPIO pins*/
	ret = gpio_pin_configure_dt(&board_led1, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
	if (ret < 0) {
		LOG_ERR("Cannot configure board led 1.");
		return ret;
	}
	ret = gpio_pin_configure_dt(&board_led2, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
	if (ret < 0){
		LOG_ERR("Cannot configure board led 2.");
		return ret;
	}
	ret = gpio_pin_configure_dt(&board_led3, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
	if (ret < 0){
		LOG_ERR("Cannot configure board led 3.");
		return ret;
	}
	/* Need to Configure Buttons*/
	return 0;
}