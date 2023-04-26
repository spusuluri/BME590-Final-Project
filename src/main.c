/*
*To-Do List: 
Note: BLE_DATA_POINTS IS ALREADY DEFINED
Test VBUS code
QUESTIONS: 
2. Check about brightness (~invert?)
3. Can LED1 & LED2 be on when VBUS is detected? Why are there two semicolons?
4. Where is VBUS on thing? (TEST VBUS)
5. I can't read voltages beyond 3 V for Battery Vol? 
Tried changing different parameters Suggestions?
7. What is the notification? Is that the actually sending of the bluetooth data?
8. How to demonstarte things work? Video? 
9. Experimentally, test that theoretical value?
MAIN PROBLEM: Other code will mess up the timing of the reading of the adc ; need to check
the at what sampling to check the waveforms. 
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <nrfx_power.h>
#include <math.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/services/bas.h>

LOG_MODULE_REGISTER(Final_Project, LOG_LEVEL_DBG);

#define ADC_SIN100_SAMPLE_RATE_MSEC 1
#define ADC_SIN500_SAMPLE_RATE_MSEC 1
#define LED3_ON_TIME_MS 1000
#define RMS_DATA_SAMPLE_RATE_MSEC 1000
#define ADC_SIN100_SAMPLE_SIZE 1000
#define ADC_SIN500_SAMPLE_SIZE 1000
#define VPP_CONV_RMS M_SQRT2
#define BLE_DATA_POINTS 5
#define LED_MAX_BRIGHTNESS 1
#define ADC_SIN100_MIN_VPP 5
#define ADC_SIN100_MAX_VPP 50
#define ADC_SIN500_MIN_VPP 10
#define ADC_SIN500_MAX_VPP 150 

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
static const struct adc_dt_spec adc_vbat = ADC_DT_SPEC_GET_BY_ALIAS(vbat);

/* PWM Channels*/
static const struct pwm_dt_spec board_led1_drv = PWM_DT_SPEC_GET(DT_ALIAS(drv1)); 
static const struct pwm_dt_spec board_led2_drv = PWM_DT_SPEC_GET(DT_ALIAS(drv2));

/* LEDs*/
static const struct gpio_dt_spec board_led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec board_led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec board_led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/*Buttons*/
static const struct gpio_dt_spec board_button1 = GPIO_DT_SPEC_GET(DT_ALIAS(button0), gpios);
static const struct gpio_dt_spec board_button2 = GPIO_DT_SPEC_GET(DT_ALIAS(button1), gpios);

/* Static Variables*/
static int adc_sin100_mV = 0;
static int adc_sin500_mV = 0;
static int adc_vbat_mV = 0;
static float adc_sin100_RMS = 0.0;
static float adc_sin500_RMS = 0.0;
static int adc_sin100_VPP = 0;
static int adc_sin500_VPP = 0;
static float adc_sin100_percent_voltage;
static float adc_sin500_percent_voltage;
static int rms_data_count = 0;
static bool usbregstatus;
static bool vbus_state = 0;

/*Callback Declarations*/
static struct gpio_callback board_button1_cb;
static struct gpio_callback board_button2_cb;


/*Array Variables*/
int sin100_values_mV[ADC_SIN100_SAMPLE_SIZE] = {0}; 
int sin500_values_mV[ADC_SIN500_SAMPLE_SIZE] = {0};
float sin100_RMS_values[BLE_DATA_POINTS]={0.0};
float sin500_RMS_values[BLE_DATA_POINTS]={0.0};

/*Declarations*/
int setup_channels_and_pins(void);
int check_interfaces_ready(void);
int check_vbus(void);
void board_button1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void board_button2_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
int read_adc(struct adc_dt_spec adc_channel);
void read_adc_sin100(struct k_timer *adc_sin100_timer);
void read_adc_sin500(struct k_timer *adc_sin500_timer);
void collect_rms_data(struct k_timer *rms_collection_timer);
void stop_rms_data(struct k_timer *rms_collection_timer);
void boardled3_toggle(struct k_timer *vbus_timer);
void boardled3_stop(struct k_timer *vbus_timer);
float adc_sin100_calculate_led_brightness(int val_VPP);
float adc_sin500_calculate_led_brightness(int val_VPP);
float calculate_led_brightness(int val_VPP, int min_VPP, int max_VPP);
/* Define Timers*/
K_TIMER_DEFINE(adc_sin100_timer, read_adc_sin100, NULL);
K_TIMER_DEFINE(adc_sin500_timer, read_adc_sin500, NULL);
K_TIMER_DEFINE(rms_collection_timer, collect_rms_data, stop_rms_data);
K_TIMER_DEFINE(vbus_timer, boardled3_toggle, boardled3_stop);
/* Timer Functions*/
void read_adc_sin100(struct k_timer *adc_sin100_timer){
	for (int i=0; i < ADC_SIN100_SAMPLE_SIZE - 1; i++){
		sin100_values_mV[i] = sin100_values_mV[i+1];
	}
	sin100_values_mV[ADC_SIN100_SAMPLE_SIZE -1] = adc_sin100_mV;
}
void read_adc_sin500(struct k_timer *adc_sin500_timer){
	//LOG_DBG("Reading Sinusoid 500 Hz");
	for (int i=0; i < ADC_SIN500_SAMPLE_SIZE - 1; i++){
		sin500_values_mV[i] = sin500_values_mV[i+1];
	}
	sin500_values_mV[ADC_SIN500_SAMPLE_SIZE -1] = adc_sin500_mV;
}
void collect_rms_data(struct k_timer *rms_collection_timer){
	LOG_DBG("RMS Data Collected");
	sin100_RMS_values[rms_data_count]=adc_sin100_RMS;
	sin500_RMS_values[rms_data_count]=adc_sin500_RMS;
	if(rms_data_count == 4){
		rms_data_count = 0;
		k_timer_stop(rms_collection_timer);
	}
	else{
		rms_data_count++;
	}
}
void stop_rms_data(struct k_timer *rms_collection_timer){
	LOG_DBG("RMS Data Stopped");
}
void boardled3_toggle(struct k_timer *vbus_timer){
	gpio_pin_toggle_dt(&board_led3);
	LOG_DBG("LED 3 Toggle");
}
void boardled3_stop(struct k_timer *vbus_timer){
	LOG_DBG("LED 3 turned off.");
	gpio_pin_set_dt(&board_led3, 0);
}
/*Callbacks*/
void board_button1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_DBG("Button 1 pressed.");
	k_timer_start(&rms_collection_timer, K_MSEC(RMS_DATA_SAMPLE_RATE_MSEC), K_MSEC(RMS_DATA_SAMPLE_RATE_MSEC));
}
void board_button2_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_DBG("Button 2 pressed.");
}

float calculate_rms(int sin_arr[], int sample_size){
	float array_sum = 0.0;
	for (int i = 0; i < sample_size; i++){
		array_sum += (float)sin_arr[i] * (float)sin_arr[i];
	}
	return sqrtf(array_sum / (float)sample_size);
}

int calculate_VPP(float rms_value){
	return (int)(rms_value * VPP_CONV_RMS);
}

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
	/* Setup Callbacks*/
	err = gpio_pin_interrupt_configure_dt(&board_button1, GPIO_INT_EDGE_TO_ACTIVE);
	if (err < 0){
		LOG_ERR("Error configuring button 1 callback");
		return;
	}	
	gpio_init_callback(&board_button1_cb, board_button1_callback, BIT(board_button1.pin));
	gpio_add_callback(board_button1.port, &board_button1_cb);

	err = gpio_pin_interrupt_configure_dt(&board_button2, GPIO_INT_EDGE_TO_ACTIVE);
	if (err < 0){
		LOG_ERR("Error configuring button 2 callback");
		return;
	}	
	gpio_init_callback(&board_button2_cb, board_button2_callback, BIT(board_button2.pin));
	gpio_add_callback(board_button2.port, &board_button2_cb);
	gpio_pin_set_dt(&board_led1, 1);
	gpio_pin_set_dt(&board_led2, 1);
	k_timer_start(&adc_sin100_timer, K_MSEC(ADC_SIN100_SAMPLE_RATE_MSEC), K_MSEC(ADC_SIN100_SAMPLE_RATE_MSEC));
	k_timer_start(&adc_sin500_timer, K_MSEC(ADC_SIN500_SAMPLE_RATE_MSEC), K_MSEC(ADC_SIN500_SAMPLE_RATE_MSEC));
	while (1) {
		err = check_vbus();
		if (err){
			LOG_DBG("VBUS is connected.");
			vbus_state=1;
			k_timer_start(&vbus_timer, K_MSEC(LED3_ON_TIME_MS), K_MSEC(LED3_ON_TIME_MS));
			break;
		}
		k_timer_stop(&vbus_timer);
		/*
		if (err && vbus_state){
			LOG_DBG("VBUS is still connected.");
			break;
		}
		if (!err && vbus_state){
			LOG_DBG("VBUS was disconnected.");
			vbus_state=0;
			k_timer_stop(&vbus_timer);
		}
		*/
		adc_sin100_mV = read_adc(adc_sin100);
		//LOG_DBG("100 Hz Sinusoid ADC Value (mV): %d", adc_sin100_mV);
		adc_sin500_mV = read_adc(adc_sin500);
		//LOG_DBG("500 Hz Sinusoid ADC Value (mV): %d", adc_sin500_mV);
		/* DELETE THIS LINE AFTER TESTING VBAT
		adc_vbat_mV = read_adc(adc_vbat);
		LOG_DBG("Battery Voltage ADC Value (mV): %d", adc_vbat_mV);
		*/
		adc_sin100_RMS = calculate_rms(sin100_values_mV, ADC_SIN100_SAMPLE_SIZE);
		//LOG_DBG("100 Hz Sinusoid RMS Value: %f", adc_sin100_RMS);
		adc_sin500_RMS = calculate_rms(sin500_values_mV, ADC_SIN500_SAMPLE_SIZE);
		//LOG_DBG("500 Hz Sinusoid RMS Value: %f", adc_sin500_RMS);
		adc_sin100_VPP = calculate_VPP(adc_sin100_RMS);
		adc_sin500_VPP = calculate_VPP(adc_sin500_RMS);
		LOG_DBG("100 Hz Sinusoid VPP Value: %d", adc_sin100_VPP);
		//LOG_DBG("500 Hz Sinusoid VPP Value: %d", adc_sin500_VPP);
		adc_sin100_percent_voltage = adc_sin100_calculate_led_brightness(adc_sin100_VPP);
		if (adc_sin100_percent_voltage < 0){
			adc_sin100_percent_voltage = 0.0;
		}
		if (adc_sin100_percent_voltage > 1){
			adc_sin100_percent_voltage = 1.0;
		}
		//LOG_DBG("ADC Sin 100 Percent Voltage: %f", adc_sin100_percent_voltage);
		adc_sin500_percent_voltage = adc_sin500_calculate_led_brightness(adc_sin500_VPP);
		if (adc_sin500_percent_voltage < 0){
			adc_sin500_percent_voltage = 0.0;
		}
		if (adc_sin500_percent_voltage > 1){
			adc_sin500_percent_voltage = 1.0;
		}
		uint32_t board_led1_pulse = board_led1_drv.period * adc_sin100_percent_voltage;
		uint32_t board_led2_pulse = board_led2_drv.period * adc_sin500_percent_voltage;
		err = pwm_set_pulse_dt(&board_led1_drv, board_led1_pulse);
		if (err) {
			LOG_ERR("HERE Could not set Board LED 1 (PWM0)");
		}
		err = pwm_set_pulse_dt(&board_led2_drv, board_led2_pulse);
		if (err) {
			LOG_ERR("Could not set Board LED 2 (PWM0)");
		}
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
	if (!device_is_ready(adc_sin100.dev) || !device_is_ready(adc_sin500.dev) || !device_is_ready(adc_vbat.dev)) {
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
	ret = adc_channel_setup_dt(&adc_vbat);
	if (ret < 0) {
		LOG_ERR("Could not setup Voltage Battery ADC channel (%d)", ret);
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
	ret = gpio_pin_configure_dt(&board_button1, GPIO_INPUT);
	if (ret < 0){
		LOG_ERR("Cannot configure board 1 button.");
		return ret;
	}
	ret = gpio_pin_configure_dt(&board_button2, GPIO_INPUT);
	if (ret < 0){
		LOG_ERR("Cannot configure board 2 button.");
		return ret;
	}
	return 0;
}
float adc_sin100_calculate_led_brightness(int val_VPP){
	/*
	This function will output the percentage of brightness for the
	LED for the 100 Hz Sinusoid.

	Inputs: 
		int val_VPP: Input VPP
	*/
	return (float)(1.0/45.0) * (float)val_VPP - 1.0/9.0; 
}
float adc_sin500_calculate_led_brightness(int val_VPP){
	/*
	This function will output the percentage of brightness for the
	LED for the 500 Hz Sinusoid.

	Inputs: 
		int val_VPP: Input VPP
	*/
	return (float)(1.0/140.0) * (float)val_VPP - 1.0/14.0; 
}

float calculate_led_brightness(int val_VPP, int min_VPP, int max_VPP){
	float slope = ((float)LED_MAX_BRIGHTNESS) / ((float)max_VPP - (float)min_VPP);
	float intercept = ((float)min_VPP) / ((float)max_VPP - (float)min_VPP);
	return (float) (slope) * (float)(val_VPP) - (float)(intercept);
}

int check_vbus() {
    /* check for voltage on VBUS (USB-C charging cable attached)
    
    Returns:
        0 - VBUS not detected
        -1 - VBUS detected (need to kill device function)
    */

    usbregstatus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER);;
    if (usbregstatus) {
        LOG_ERR("VBUS voltage detected.  Device cannot be operated while charging.");
        return -1;
    }
    else {
        //LOG_DBG("VBUS voltage checked and not detected.");
    }

    return 0;
}
