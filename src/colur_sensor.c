#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>


#define Slave_Addr 0x38

#define BH1745_SA_LOW 0x38

#define MODE_CONTROL2 0x42

#define MODE_CONTROL3 0x44



extern uint8_t red;
extern uint8_t green;
extern uint8_t blue;
extern uint8_t cdata;




#define I2C_NODE DT_NODELABEL(colour_sen)
 struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

uint8_t cs_i2c_read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t data;
	i2c_write_read_dt(&dev_i2c, &regAddr, sizeof(regAddr), &data, sizeof(data));
	return data;
}


void cs_i2c_write(uint8_t slaveAddr, uint8_t regAddr,uint8_t value1 )
{
	uint8_t buf[2];

	buf[0] = regAddr;
	buf[1] = value1;
	 i2c_write_dt(&dev_i2c, buf, sizeof(buf));
	return ;
}

void intital(void)
{
	cs_i2c_write(BH1745_SA_LOW , 0x41, 0x00); //default time set 160 sec 
	// uint16_t value_0x41 = cs_i2c_read(0x38, 0x41);
	// 	printk("0x41 : %x \n",value_0x41);
	cs_i2c_write(BH1745_SA_LOW , MODE_CONTROL2, 0x10); // enable rgbc 
	// int16_t value_0x42 = cs_i2c_read(0x38, 0x42);
	// 	printk("0x42 : %x \n",value_0x42);
	cs_i2c_write(BH1745_SA_LOW , MODE_CONTROL3, 0x02); //default value for mc3
	// int16_t value_0x44 = cs_i2c_read(0x38, 0x44);
	// 	printk("0x44 : %x \n",value_0x44);
	k_sleep(K_MSEC(500));
}




void RGB_DATA_FOR_READ(void)
{


	/* IF WE WANT TO READ RGBC
	 *
	 *       MODE_CONTROL_3:
	 *                      we have to write 0x02 in mode control 3
	 *
	 *              WE HAVE TO CALL THIS FUNCTION "   MODE_CONTROL3(); " BEFORE CALL " RGB_DATA_FOR_READ(); "
	 *
	 *        RGBC_ENABLE:
	 *                    we have to enable RGBC
	 *
	 *              WE HAVE TO CALL THIS FUNCTION "  RGBC_ENABLE ();" BEFORE CALL " RGB_DATA_FOR_READ(); "
	 *
	 */


	int data1 = cs_i2c_read(0x38, 0x50);
//    printf("RED_DATA_LBS: 0x%02x \n", data1);

    int data2 = cs_i2c_read(0x38, 0x51);
//    printf("RED_DATA_MBS: 0x%02x \n", data2);

    int data3 = cs_i2c_read(0x38, 0x52);
//    printf("GREEN_DATA_LBS: 0x%02x \n", data3);

    int data4 = cs_i2c_read(0x38, 0x53);
    // printf("GREEN_DATA_MBS: 0x%02x \n", data4);

    int data5 = cs_i2c_read(0x38, 0x54);
//    printf("BLUE_DATA_LBS: 0x%02x \n", data5);

    int data6 = cs_i2c_read(0x38, 0x55);
//    printf("BLUE_DATA_MBS: 0x%02x \n", data6);

    int data7 = cs_i2c_read(0x38, 0x56);
//    printf("CLEAR_DATA_LBS: 0x%02x \n", data7);

    int data8 = cs_i2c_read(0x38, 0x57);
//    printf("CLEAR_DATA_MBS: 0x%02x \n", data8);


    red = ((data2 * 256 )+ data1);
	green = ((data4 * 256) + data3);
   	blue = ((data6 * 256 )+ data5);
   	cdata = ((data8 * 256 )+ data7);


  	printf("Red color luminance : %d lux \n", red);
   	printf("Green color luminance : %d lux \n", green);
    printf("Blue color luminance : %d lux \n", blue);
    printf("Clear Data  Luminance : %d lux \n ", cdata);

}

 


void colour(void)
{
 if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return;
	}
		cs_i2c_write(BH1745_SA_LOW , 0x40, 0x4b);
		// uint16_t value_0x40 = cs_i2c_read(0x38, 0x40);
		// printk("0x40 : %x \n",value_0x40);
		intital();
		// getManufacturerId(600);


}
