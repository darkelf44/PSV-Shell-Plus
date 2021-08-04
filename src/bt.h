#ifndef _BT_H_
#define _BT_H_

typedef enum {
    PSVS_BT_TOUCH_DISABLED = 0,
    PSVS_BT_TOUCH_TOGGLE_F_B,
    PSVS_BT_TOUCH_TOGGLE_F_B_X,
    PSVS_BT_TOUCH_MAX,
} psvs_bt_touch_t;

typedef enum {
    PSVS_BT_MOTION_DISABLED = 0,
    PSVS_BT_MOTION_NORMAL,
    PSVS_BT_MOTION_VERTICAL,
    PSVS_BT_MOTION_MAX,
} psvs_bt_motion_t;

typedef enum {
    PSVS_MOTION_FLAG_ENABLE_MOTION = 1,
    PSVS_MOTION_FLAG_ENABLE_DEAD_BAND = 2,
    PSVS_MOTION_FLAG_ENABLE_TILT_CORRECTION = 4,
    PSVS_MOTION_FLAG_ALL = 7,
} psvs_motion_flags_t;



void psvs_bt_init();
void psvs_bt_done();

bool psvs_bt_connected(unsigned int mac0, unsigned int mac1);
void psvs_bt_on_hid_transfer(SceBtHidRequest * request);

int psvs_bt_touch_filter_input(bool peek, uint32_t port, SceTouchData *pData, uint32_t nBufs);
int psvs_bt_motion_filter_state(SceMotionState * motionState);
int psvs_bt_motion_filter_sensorstate(SceMotionSensorState *sensorState, int numRecords);

void psvs_bt_motion_reset();
bool psvs_bt_motion_get_flag(psvs_motion_flags_t flag);
void psvs_bt_motion_set_flag(psvs_motion_flags_t flag, bool enabled);

#endif
