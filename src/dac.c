#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>


#define Slave_Addr 0x64

#define Reg_Addr 0x40
int dac_value;
extern char dac_output[4];


#define I2C_NODE DT_NODELABEL(digitaltoanalog)
 struct i2c_dt_spec dev_i2c2 = I2C_DT_SPEC_GET(I2C_NODE);

uint16_t dac_i2c_read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t data[5];
	i2c_write_read_dt(&dev_i2c2, &regAddr, sizeof(regAddr), &data, 5);
	uint16_t msb = data[1]<<4;
	// printf("msb : %x\n",msb);
	uint16_t lsb = data[2]>>4 ;
	// printf("lsb : %x\n",lsb);
	uint16_t value =lsb +msb;
	// uint16_t Value = ((msb & 0x00F) << 4) | ((msb & 0xFF0) >> 8	);
	// printf("value : %x\n",value);
	return value;
}


void dac_i2c_write(uint8_t slaveAddr,uint8_t value1 , uint8_t value2)
{
	uint8_t buf[2];

	// buf[0] = regAddr;
	buf[0] = value1;
    buf[1] = value2;
	 i2c_write_dt(&dev_i2c2, buf, sizeof(buf));
	return ;
}


void setVoltage_DAC_Mode(float volt)
{

	uint16_t value = (volt*4096.0)/3.3 ;

	// printf("  data : %x \n ", value);
			 
	uint16_t var1 =((value >>8) & 0x0ff );
	// printf(" data of 0xff0 : %x \n ", var1);

	uint16_t var2 =(value & 0xff ) ;
	// printf(" data of 0xf : %x \n ", var2);

	dac_i2c_write(Slave_Addr, var1, var2);

			

}



 


void dac_main(void)
{
if (!device_is_ready(dev_i2c2.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c2.bus->name);
		return;
	}
    int dac_default = 10;
	float volt = (float)dac_value/dac_default ;  //initial value
	setVoltage_DAC_Mode(volt);
	
	uint16_t data_val = dac_i2c_read(Slave_Addr,0x00 );
	float value = (data_val*3.3)/4096.0;
    // dac_output= (int)value*10;
    sprintf(dac_output,"%.2f",value);
	printk("voltage value is %.1fV\n ",value);
	
	return 0 ;
}
