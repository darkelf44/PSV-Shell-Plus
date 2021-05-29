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

#endif
