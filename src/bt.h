#ifndef _BT_H_
#define _BT_H_

enum {
    SCE_MOTION_DEV_ENTRY_HAS_GYRO_DATA = 1 << 0,
    SCE_MOTION_DEV_ENTRY_HAS_ACCEL_DATA = 1 << 2,
    SCE_MOTION_DEV_ENTRY_HAS_MAGNETOMETER_DATA = 1 << 3,
};

typedef struct {
    /* 00 */ int32_t reserved;
    /* 04 */ SceFVector3 zero;
} SceMotionDevGyroBias; /* 10 */

typedef struct {
    /* 00 */    SceFVector3 zero;
    /* 0C */    float xPosScale;
    /* 10 */    SceFVector3 xPos;
    /* 1C */    float yPosScale;
    /* 20 */    SceFVector3 yPos;
    /* 2C */    float zPosScale;
    /* 30 */    SceFVector3 zPos;
    /* 3C */    float xNegScale;
    /* 40 */    SceFVector3 xNeg;
    /* 4C */    float yNegScale;
    /* 50 */    SceFVector3 yNeg;
    /* 5C */    float zNegScale;
    /* 60 */    SceFVector3 zNeg;
} SceMotionDevGyroCalibData;    /* 6C */

typedef struct {
    /* 00 */    float xPosScale;
    /* 04 */    SceFVector3 xPos;
    /* 10 */    float yPosScale;
    /* 14 */    SceFVector3 yPos;
    /* 20 */    float zPosScale;
    /* 24 */    SceFVector3 zPos;
    /* 30 */    float xNegScale;
    /* 34 */    SceFVector3 xNeg;
    /* 40 */    float yNegScale;
    /* 44 */    SceFVector3 yNeg;
    /* 50 */    float zNegScale;
    /* 54 */    SceFVector3 zNeg;
} SceMotionDevAccCalibData; /* 60 */

typedef struct {
    /* 00 */    uint16_t reserved0;
    /* 02 */    uint16_t flags;
    /* 04 */    uint16_t gyro_x;
    /* 06 */    uint16_t gyro_y;
    /* 08 */    uint16_t gyro_z;
    /* 0A */    uint16_t reserved1[3];
    /* 10 */    uint16_t accel_x;
    /* 12 */    uint16_t accel_y;
    /* 14 */    uint16_t accel_z;
    /* 16 */    uint16_t magn_x;
    /* 18 */    uint16_t magn_y;
    /* 1A */    uint16_t magn_z;
    /* 1C */    uint16_t reserved[4];
} SceMotionDevEntry;    /* 24 */

typedef struct {
    /* 00 */    uint64_t timestamp;
    /* 08 */    uint8_t b0;
    /* 09 */    uint8_t b1;
    /* 0A */    uint8_t b2;
    /* 0B */    uint8_t entryCount;
    /* 0C */    uint32_t magnCalibIndex;
    /* 10 */    uint32_t magnFieldStab;
    /* 14 */    uint32_t gyroCalibIndex;
    /* 18 */    uint32_t timeInMSec;
    /* 1C */    SceMotionDevEntry entryList[5];
    /* D0 */    uint8_t reserved[32];
} SceMotionDevResult;   /* F0 */

typedef enum {
    PSVS_BT_TOUCH_DISABLED = 0,
    PSVS_BT_TOUCH_FRONT,
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

void psvs_bt_init();
void psvs_bt_done();

bool psvs_bt_connected(unsigned int mac0, unsigned int mac1);
void psvs_bt_on_hid_transfer(SceBtHidRequest * request);

int psvs_bt_touch_filter_input(bool peek, uint32_t port, SceTouchData *pData, uint32_t nBufs);
int psvs_bt_motion_filter_read(SceMotionDevResult * resultList, uint32_t count, int * setFlag);

void psvs_bt_motion_set_device_info(const uint32_t * info);
void psvs_bt_motion_set_gyro_bias(const SceMotionDevGyroBias * bias);
void psvs_bt_motion_set_gyro_calib_data(const SceMotionDevGyroCalibData * data);
void psvs_bt_motion_set_accel_calib_data(const SceMotionDevAccCalibData * data);

int psvs_bt_motion_reset_device_info(uint32_t * info);
int psvs_bt_motion_reset_gyro_bias(SceMotionDevGyroBias * bias);
int psvs_bt_motion_reset_gyro_calib_data(SceMotionDevGyroCalibData * data);
int psvs_bt_motion_reset_accel_calib_data(SceMotionDevAccCalibData * data);

#endif
