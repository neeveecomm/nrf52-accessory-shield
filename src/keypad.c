#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>


 const uint8_t mute = 0xef;     	//1		mute
 const uint8_t up = 0x52;			//2		up_arrow
  const uint8_t back = 0xf1;		//3		back
 const uint8_t left = 0x50;			//4		left_arrow
 const uint8_t enter = 0x28;		//5		select
 const uint8_t right = 0x4f;		//6		right_arrow
 const uint8_t volume_down = 0xee;	//7		volume_down
 const uint8_t down = 0x51;			//8		down_arrow
 const uint8_t volume_up = 0xed;	//9		volume_up
 const uint8_t zero = 0xe9;			//0		play pause
 const uint8_t alp_a = 0x4a;		//a		home
 const uint8_t alp_b = 0xf8;		//b		sleep
 const uint8_t alp_c = 0x06;		//c		
 const uint8_t alp_d = 0x07;		//d
 const uint8_t star = 0x55;			//*
 const uint8_t hash = 0x20;			//#
 int cond = 0;

 //nterrupt
/* STEP 9 - Increase the sleep time from 100ms to 10 minutes  */
#define SLEEP_TIME_MS   10*60*1000

/* SW4_NODE is the devicetree node identifier for the node with alias "sw4" */
#define SW4_NODE	DT_ALIAS(sw4) 
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW4_NODE, gpios);


/* LED0_NODE is the devicetree node identifier for the node with alias "led0". */
#define LED0_NODE	DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);




#define I2C_NODE DT_NODELABEL(keyboard)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

uint8_t kp_i2c_read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t data;
	i2c_write_read_dt(&dev_i2c, &regAddr, sizeof(regAddr), &data, sizeof(data));
	return data;
}

uint8_t kp_i2c_write(uint8_t slaveAddr, uint8_t regAddr, uint8_t regVal)
{
	uint8_t buf[2];

	buf[0] = regAddr;
	buf[1] = regVal;

	return i2c_write_dt(&dev_i2c, buf, sizeof(buf));
}


/* Define the callback function */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    gpio_pin_toggle_dt(&led);
	
	cond = 1;
 }

 //call back 
 static struct gpio_callback button_cb_data;




int keypress (void)
{
	kp_i2c_write(0x34, 0x1D, 0x0F);
	kp_i2c_write(0x34, 0x1E, 0x0F);
	kp_i2c_write(0x34, 0x20, 0x0F);
	kp_i2c_write(0x34, 0x21, 0x0F);


	int data1 = kp_i2c_read(0x34, 0x04);
	// printf("data1 : 0x %02x\n", data1 );


	int data2 = kp_i2c_read(0x34, 0x05);
	// printf("data2 : 0x %02x\n", data2 );

	int data3 = kp_i2c_read(0x34, 0x06);
	// printf("data3 : 0x %02x\n", data3 );


	int data4 = kp_i2c_read(0x34, 0x07);
	//  printf("data4 : 0x %02x\n", data4 );



if((data1 == 0xa2)|(data2 == 0xa2)|(data3 == 0xa2)|(data4 == 0xa2))
{
	// printk("1");
	return mute;

}

if((data1 == 0x98)|(data2 == 0x98)|(data3 == 0x98)|(data4 == 0x98))
{
		// printk("2");

	return up;
}

if((data1 == 0x8e)|(data2 == 0x8e)|(data3 == 0x8e)|(data4 == 0x8e))
{
	
	return back;
}

if((data1 == 0x84)|(data2 == 0x84)|(data3 == 0x84)|(data4 == 0x84))
{
	return alp_a;
}

if((data1 == 0xa1)|(data2 == 0xa1)|(data3 == 0xa1)|(data4 == 0xa1))
{
	return left;
}

if((data1 == 0x97)|(data2 == 0x97)|(data3 == 0x97)|(data4 == 0x97))
{
	return enter;
}

if((data1 == 0x8d)|(data2 == 0x8d)|(data3 == 0x8d)|(data4 == 0x8d))
{
	return right;
}

if((data1 == 0x83)|(data2 == 0x83)|(data3 == 0x83)|(data4 == 0x83))
{
	return alp_b;
}

if((data1 == 0xa0)|(data2 == 0xa0)|(data3 == 0xa0)|(data4 == 0xa0))
{
	return volume_down;
}

if((data1 == 0x96)|(data2 == 0x96)|(data3 == 0x96)|(data4 == 0x96))
{
	return down;
}

if((data1 == 0x8c)|(data2 == 0x8c)|(data3 == 0x8c)|(data4 == 0x8c))
{
	return volume_up;
}

if((data1 == 0x82)|(data2 == 0x82)|(data3 == 0x82)|(data4 == 0x82))
{
	return alp_c;
}

if((data1 == 0x9f)|(data2 == 0x9f)|(data3 == 0x9f)|(data4 == 0x9f))
{
	return star;
}

if((data1 == 0x95)|(data2 == 0x95)|(data3 == 0x95)|(data4 == 0x95))
{
	return zero;
}

if((data1 == 0x8b)|(data2 == 0x8b)|(data3 == 0x8b)|(data4 == 0x8b))
{
	return hash;
}

if((data1 == 0x81)|(data2 == 0x81)|(data3 == 0x81)|(data4 == 0x81))
{
	return alp_d;
}
return 0;
}

void keypad_init(void)
{
	
	if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return;
	}

	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	if (!device_is_ready(button.port)) {
		return;
	}


	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0) {
		return;
	}
	// printf("entered for loop");
	


	//interrupt congiguration

	/*  Configure the interrupt on the button's pin */
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE  );

	/*  Initialize the static struct gpio_callback variable   */
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin)); 	
	
	/* Add the callback function by calling gpio_add_callback()   */
	gpio_add_callback(button.port, &button_cb_data);
	// printk("value : %d\n",value);

	 kp_i2c_write(0x34, 0x1D, 0x0F);  //r0 - r3 to kp matrix

	kp_i2c_write(0x34, 0x1E, 0x0F);  // c0 -c3 to kp matrix


	kp_i2c_write(0x34, 0x01, 0x03);  //configaration registers


}