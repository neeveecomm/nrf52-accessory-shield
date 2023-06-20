
#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>


#define BT_UUID_COLOUR_VAL 0x1235
/**
 *  @brief Environmental Sensing Service
 */
#define BT_COLUR_UUID_BASE \
	BT_UUID_DECLARE_16(BT_UUID_COLOUR_VAL)

#define BT_UUID_DAC_VAL 0x1240
/**
 *  @brief Environmental Sensing Service
 */
#define BT_DAC_UUID_BASE \
	BT_UUID_DECLARE_16(BT_UUID_DAC_VAL)



#define ESS_UUID_BASE         BT_UUID_DECLARE_16(0x181A)


#define BT_UUID_TIMER_VAL 0x1234

#define BT_UUID_RED_V 0x1236

#define BT_UUID_GREEN_V 0x1237

#define BT_UUID_BLUE_V 0x1238

#define BT_UUID_CLEAR_V 0x1239

#define BT_UUID_DAC_V 0x1241

#define BT_UUID_TIMER \
	BT_UUID_DECLARE_16(BT_UUID_TIMER_VAL)

#define BT_UUID_RED \
	BT_UUID_DECLARE_16(BT_UUID_RED_V)

#define BT_UUID_GREEN \
	BT_UUID_DECLARE_16(BT_UUID_GREEN_V)

#define BT_UUID_BLUE \
	BT_UUID_DECLARE_16(BT_UUID_BLUE_V)

#define BT_UUID_CLEAR \
	BT_UUID_DECLARE_16(BT_UUID_CLEAR_V)

#define BT_UUID_DAC \
	BT_UUID_DECLARE_16(BT_UUID_DAC_V)



#ifdef __cplusplus
}
#endif

void MEASURE_TEMPERATURE_NO_HOLD_MASTER_MODE (void);
void MEASURE_RELATIVE_HUMIDITY_NO_HOLD_MASTER_MODE (void);
void si7006(void);
