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

extern uint32_t psvs_bt_x;
extern uint32_t psvs_bt_y;

void psvs_bt_init();
void psvs_bt_done();

bool psvs_bt_connected(unsigned int mac0, unsigned int mac1);
void psvs_bt_on_hid_transfer(SceBtHidRequest * request);

int psvs_bt_touch_filter_input(bool peek, uint32_t port, SceTouchData *pData, uint32_t nBufs);
int psvs_bt_motion_filter_input();


#endif
