#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>

#define MMA8652FC_I2C_ADDR 0x1D
#define OUT_X_MSB 0x01
#define OUT_Y_MSB 0x03
#define FF_MT_CFG 0x15
#define FF_MT_THS 0x17
#define FF_MT_COUNT 0x18
#define CTRL_REG4 0x2D
#define CTRL_REG5 0x2E
#define INT_SOURCE 0x0C
#define CTRL_REG1 0x2a


// extern uint16_t x_axis;
// extern uint16_t y_axis;

extern bool right ;
extern bool left ;
extern bool up ;
extern bool down ;
uint8_t data;
static int flag;
extern int16_t x_val_def;
extern int16_t y_val_def;

#define SW4_NODE  DT_ALIAS(sw4)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW4_NODE,gpios);

#define I2C_NODE DT_NODELABEL(mysensor)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

uint8_t aa_i2c_read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t data;
	i2c_write_read_dt(&dev_i2c, &regAddr, sizeof(regAddr), &data, sizeof(data));
	return data;
}

int aa_i2c_write(uint8_t slaveAddr, uint8_t regAddr, uint8_t regVal)
{
	uint8_t buf[2];

	buf[0] = regAddr;
	buf[1] = regVal;

	return i2c_write_dt(&dev_i2c, buf, sizeof(buf));
}


void interrupt_ocu(const struct device *dev, struct gpio_callback *cb,uint32_t pins)
{

		flag =1;
	
}


static struct gpio_callback button_cb_data;

void accelero_main(void)
{
	int ret;
	
	if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return;
	}


	// if(!device_is_ready(button.port))
	// {
	// 	return;
	// }

	// ret = gpio_pin_configure_dt(&button,GPIO_INPUT);
	// if(ret <0)
	// {
	// 	return;
	// }

	

    

    
	// aa_i2c_write(0x1d,CTRL_REG4, 0x04); // Enable motion detection interrupt on INT1 pin
    // aa_i2c_write(0x1d,CTRL_REG5, 0x04); // Route motion detection interrupt to INT1 pin



	// Configure MMA8652FC for motion detection
    aa_i2c_write(0x1d,CTRL_REG1, 0x00); // Standby mode

	// Configure motion detection axis (X, Y, and Z axes)
    
	aa_i2c_write(0x1d,FF_MT_CFG, 0x58); // Enable detection on all axes
	
	
	// aa_i2c_read(0x1d,0x15);
	


    // Configure motion detection threshold and debounce counter
    aa_i2c_write(0x1d,FF_MT_THS, 0x81); // Set threshold (adjust as needed)
    aa_i2c_write(0x1d,FF_MT_COUNT, 0x00); // Set debounce counter (adjust as needed)


	// aa_i2c_read(0x1d,0x16);

	uint8_t int_source = aa_i2c_read(0x1d,0x16);
		printk("disable : %d \n",int_source);

    // aa_i2c_write(0x1d,0x0e, 0x10); // high_pass

    // Configure interrupt settings
    aa_i2c_write(0x1d,CTRL_REG4, 0x04); // Enable motion detection interrupt on INT1 pin
    aa_i2c_write(0x1d,CTRL_REG5, 0x04); // Route motion detection interrupt to INT1 pin

    aa_i2c_write(0x1d,CTRL_REG1, 0x01); // Active mode





	ret = gpio_pin_interrupt_configure_dt(&button,GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&button_cb_data,interrupt_ocu,BIT(button.pin));

	gpio_add_callback(button.port, &button_cb_data);

	// data = aa_i2c_read(0x1d, 0x0D);                //who_iam_register

	// aa_i2c_write(0x1d, 0x2A, 0x01);                // ACTIVE = 0x01

	// data = aa_i2c_read(0x1d, 0x2A);

	// uint8_t int_source = aa_i2c_read(0x1d,INT_SOURCE);

	// Configure interrupt settings
    // aa_i2c_write(0x1d,CTRL_REG4, 0x04); // Enable motion detection interrupt on INT1 pin
    // aa_i2c_write(0x1d,CTRL_REG5, 0x04); // Route motion detection interrupt to INT1 pin
}

// int mouse_value(uint16_t *x_axix,uint16_t *y_axix) 
// {
//     // int x_axix;
//     // int y_axix;
// 	uint16_t x_axix_def = *x_axix;
// 	uint16_t y_axix_def = *y_axix;
// 	// if (x_axix_def <= 48) {
//     //     x_axix_def = x_axix_def - 64;
//     // }

//     // if (y_axix_def <= 48) {
//     //     y_axix_def = y_axix_def - 64;
//     // }
//     static uint16_t x_axis_pre;
//     static uint16_t y_axis_pre;
//     int x_value ;
//     int y_value ;
//     right = false;
//     left = false;
//     up = false;
//     down = false;
    
//     if (x_axix_def >= 0 & x_axix_def <= 1080)
//     {
//         x_value =(int) ( x_axis_pre - x_axix_def);	
//         if(x_value < 0)
//         {
//             left = true;
//             x_value *= -1 ;
// 			// printk("x_value : %d  y_value : %d \n",x_value,y_axix_def);
//         }
//         else if (x_value>0)
//         {
//             right = true;
//         }
        
//     }


//     if (x_axix_def >= 3200 & x_axix_def <= 4100)
//     {
//         x_value = (int)( x_axis_pre - x_axix_def)  ;
// 		// printk("x_value : %d  y_value : %d \n",x_value,y_axix_def);
//         if(x_value < 0)
//         {
//             left = true;
//             x_value *= -1 ;
			
//         }
//         else if (x_value>0)
//         {
//             right = true;
//         }
        
//     }

    
//     if (y_axix_def >= 3020 & y_axix_def <= 4100)
//     {
//         y_value =  (int)(y_axis_pre - y_axix_def);
//         if(y_value < 0)
//         {
//             up = true;
//             y_value *= -1 ;
// 			// printk("x_value : %d  y_value : %d \n",x_value,y_value);
//         }
//         else if (y_value>0)
//         {
//             down = true;
//         }
        
//     }
//     if (y_axix_def >= 0 & y_axix_def <= 1080)
//     {
//         y_value = (int) (y_axis_pre - y_axix_def);
//         if(y_value < 0)
//         {
//             up = true;
//             y_value *= -1 ;
//         }
//         else if (y_value>0)
//         {
//             down = true;
//         }
        
//     }
//     x_axis_pre=*x_axix;
//     y_axis_pre=*y_axix;
// 	*x_axix = (uint16_t)x_value;
// 	*y_axix = (uint16_t)y_value;
// 	// printk("x_value : %d  y_value : %d \n",*x_axix,*y_axix);
// }


int mouse_value(uint16_t *x_axix,uint16_t *y_axix) 
{
    int x_axis_def = (int)*x_axix;
    int y_axis_def = (int)*y_axix;
    int x_value;
    int y_value;
    if (x_axis_def >=0 & x_axis_def <= 1080)
    {
        x_value= abs((x_axis_def/2)-540);
        left = true;
    }
    if(x_axis_def >=3020 & x_axis_def <=4100)
    {
        x_value = (((abs(x_axis_def-4100))/2)+540);
        left = true;
    }
    if (y_axis_def >=0 & y_axis_def <= 1080)
    {
        y_value= abs((y_axis_def/2)-540);
        up = true;
    }
    if(y_axis_def >=3020 & y_axis_def <=4100)
    {
        y_value = (((abs(y_axis_def-4100))/2)+540);
        up = true;
        
    }
    *x_axix= (uint16_t)x_value;
    *y_axix= (uint16_t)y_value;
	// printk("x_value : %d  y_value : %d \n",*x_axix,*y_axix);
}

int accele_fun(uint16_t *x_axis,uint16_t *y_axis) {

		// uint8_t int_source = aa_i2c_read(0x1d,INT_SOURCE);
		// printk("int_source : %d \n",int_source);
		

	
	if(flag)
	{
		 
		/* X Offset MSB */
		data = aa_i2c_read(0x1d, 0x01);                     // OUT_X_MSB(1)(2) = 0X01
		*x_axis = (data << 4);

		/* X Offset LSB */
		data = (aa_i2c_read(0x1d, 0x02)>>4);                     // OUT_X_LSB(1)(2) = 0X02
		*x_axis |= data;

		/* Y Offset MSB */
		data = aa_i2c_read(0x1d, 0x03);                     // OUT_Y_MSB(1)(2) = 0X03
		*y_axis = (data << 4);

		/* Y Offset LSB */
		data = (aa_i2c_read(0x1d, 0x04)>>4);                     // OUT_Y_LSB(1)(2) = 0X04
		*y_axis |= data ;

		// /* Z Offset MSB */
		// data = aa_i2c_read(0x1d, 0x05);                     // OUT_Z_MSB(1)(2) = 0X05
		// uint16_t zoff = (data << 8);

		// /* Z Offset LSB */
		// data = aa_i2c_read(0x1d, 0x06);                     // OUT_Z_LSB(1)(2) = 0X06
		// zoff |= data;

		printf("XOFF: %d YOFF: %d    \n", *x_axis, *y_axis);
		// k_sleep(K_MSEC(500));

// uint8_t int_source = aa_i2c_read(0x1d,0x16);
		
// 	printk("disable : %d \n",int_source);
	
	
		// uint16_t disable =aa_i2c_read(0x1d,0x16);
		// int_source = aa_i2c_read(0x1d,INT_SOURCE);
		// printk("disable : %d \n",int_source);
		// aa_i2c_read(0x1d,0x16);
		// uint8_t int_source = aa_i2c_read(0x1d,INT_SOURCE);
		// printk("disable : %d \n",int_source);
		flag =0;
		

		
	}
	}