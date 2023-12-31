#include "ble_char.h"
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <dk_buttons_and_leds.h>
extern int cTemp;
extern int humidity;
extern int ble_temp;
extern int ble_hum;
extern int timer_val;
extern struct k_timer tcp_timer;

int cdata;
int blue;
int red;
int green;

extern int dac_value;
char dac_output[4];


static bool temp_notify_enabled;
static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                    uint16_t value)
{
    temp_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static bool hum_notify_enabled;
static void hum_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                    uint16_t value)
{
    hum_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static bool red_notify_enabled;
static void red_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                    uint16_t value)
{
    red_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static bool blue_notify_enabled;
static void blue_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                    uint16_t value)
{
    blue_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static bool green_notify_enabled;
static void green_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                    uint16_t value)
{
    green_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static bool clear_notify_enabled;
static void clear_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                    uint16_t value)
{
    clear_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}



static ssize_t ble_rd(struct bt_conn *conn,
                      const struct bt_gatt_attr *attr,
                      void *buff,
                      uint16_t len,
                      uint16_t offset)
{
    const char *value = attr->user_data;



    return bt_gatt_attr_read(conn, attr, buff, len, offset, attr->user_data, sizeof(attr->user_data));
}

static ssize_t ble_wr(struct bt_conn *conn,
                      const struct bt_gatt_attr *attr,
                      void *buff,
                      uint16_t len,
                      uint16_t offset)
{
    const char *value = attr->user_data;
    printk("led write is done : %s\n", buff);

   
}

static ssize_t ble_wr_tr(struct bt_conn *conn,
                      const struct bt_gatt_attr *attr,
                      void *buff,
                      uint16_t len,
                      uint16_t offset)

{
    uint8_t *buffer;
          buffer = buff;

    timer_val = *buffer;
   
    const char *value = attr->user_data;
    printk("timer value : %d\n",timer_val );
    k_timer_start(&tcp_timer, K_SECONDS(0), K_SECONDS(timer_val));


   
}


static ssize_t dac_wr(struct bt_conn *conn,
                      const struct bt_gatt_attr *attr,
                      void *buff,
                      uint16_t len,
                      uint16_t offset)

{
    uint8_t *buffer;
          buffer = buff;

     dac_value = *buffer;
    // memcpy(&dac_value, buffer, sizeof(float));
    //  float *floatPtr = (float *)buffer;
    // dac_value = *floatPtr;
    // union {
    //     uint8_t bytes[sizeof(float)];
    //     float value;
    // } data;

    // memcpy(data.bytes, buffer, sizeof(float));
    // dac_value = data.value;
   
    const char *value = attr->user_data;
    printk("dac_value : %f\n",dac_value );
   

   
}


/*characteristic for temperature and huminity*/

BT_GATT_SERVICE_DEFINE(ess_svc,
                       BT_GATT_PRIMARY_SERVICE(ESS_UUID_BASE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, ble_wr, &ble_temp),
                       BT_GATT_CCC(temp_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, ble_wr, &ble_hum),
                       BT_GATT_CCC(hum_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
                        BT_GATT_CHARACTERISTIC(BT_UUID_TIMER,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE ,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, ble_wr_tr, &timer_val),);


/*characteristic for colour sensor*/
BT_GATT_SERVICE_DEFINE(colour_sensor,
                       BT_GATT_PRIMARY_SERVICE(BT_COLUR_UUID_BASE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_RED,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, ble_wr, &red),
                       BT_GATT_CCC(red_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                        BT_GATT_CHARACTERISTIC(BT_UUID_BLUE_V,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, ble_wr, &blue),
                        BT_GATT_CCC(blue_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_GREEN_V,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, ble_wr, &green),
                       BT_GATT_CCC(green_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_CLEAR_V,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, ble_wr, &cdata),
                       BT_GATT_CCC(clear_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),);


BT_GATT_SERVICE_DEFINE(dac,
                       BT_GATT_PRIMARY_SERVICE(BT_DAC_UUID_BASE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_DAC,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              ble_rd, dac_wr, &dac_output),);




int temp_notify(void)
{
    if (!temp_notify_enabled)
    {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &ess_svc.attrs[2],
                          &ble_temp,
                          sizeof(ble_temp));
}

int hum_notify(void)
{
    if (!hum_notify_enabled)
    {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &ess_svc.attrs[4],
                          &ble_hum,
                          sizeof(ble_hum));
}

int red_notify(void)
{
    if (!red_notify_enabled)
    {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &ess_svc.attrs[4],
                          &red,
                          sizeof(red));
}

int blue_notify(void)
{
    if (!blue_notify_enabled)
    {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &ess_svc.attrs[4],
                          &blue,
                          sizeof(blue));
}

int green_notify(void)
{
    if (!green_notify_enabled)
    {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &ess_svc.attrs[4],
                          &green,
                          sizeof(green));
}

int clear_notify(void)
{
    if (!clear_notify_enabled)
    {
        return -EACCES;
    }

    return bt_gatt_notify(NULL, &ess_svc.attrs[4],
                          &cdata,
                          sizeof(cdata));
}