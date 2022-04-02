#include "headers.h"

struct psvs_ds3_input_report_t {
    unsigned char report_id;
    unsigned char unk0;

    unsigned char buttons0;
    unsigned char buttons1;

    unsigned char ps       : 1;
    unsigned char not_used : 7;

    unsigned char unk1;

    unsigned char left_x;
    unsigned char left_y;
    unsigned char right_x;
    unsigned char right_y;

    unsigned int unk2;

    unsigned char up_sens;
    unsigned char right_sens;
    unsigned char down_sens;
    unsigned char left_sens;

    unsigned char L2_sens;
    unsigned char R2_sens;
    unsigned char L1_sens;
    unsigned char R1_sens;

    unsigned char triangle_sens;
    unsigned char circle_sens;
    unsigned char cross_sens;
    unsigned char square_sens;

    unsigned short unk3;
    unsigned char unk4;

    unsigned char status;
    unsigned char power_rating;
    unsigned char comm_status;
    unsigned int unk5;
    unsigned int unk6;
    unsigned char unk7;

    unsigned short accel_x;
    unsigned short accel_y;
    unsigned short accel_z;

    unsigned short gyro_z;
} __attribute__((packed, aligned(32)));

struct psvs_ds4_input_report_t {
    unsigned char report_id;
    unsigned char left_x;
    unsigned char left_y;
    unsigned char right_x;
    unsigned char right_y;

    unsigned char buttons0;
    unsigned char buttons1;

    unsigned char ps   : 1;
    unsigned char tpad : 1;
    unsigned char cnt1 : 6;

    unsigned char l_trigger;
    unsigned char r_trigger;

    unsigned char cnt2;
    unsigned char cnt3;

    unsigned char battery;

    // This is the angular velocity for each axis
    signed short gyro_x;     // +X is forward to up (around left-right axis)
    signed short gyro_y;     // +Y is right to forward (around up-down axis)
    signed short gyro_z;     // +Z is right to up (around forward-backward axis)

    // This is the direction of acceleration (gravity), compared to the controller
    signed short accel_x;    // +X is left
    signed short accel_y;    // +Y is down
    signed short accel_z;    // +Z is forward

    unsigned char unk1[5];

    unsigned char battery_level : 4;
    unsigned char usb_plugged   : 1;
    unsigned char headphones    : 1;
    unsigned char microphone    : 1;
    unsigned char padding       : 1;

    unsigned char unk2[2];
    unsigned char trackpadpackets;
    unsigned char packetcnt;

    unsigned int finger1_id        : 7;
    unsigned int finger1_activelow : 1;
    unsigned int finger1_x         : 12;
    unsigned int finger1_y         : 12;

    unsigned int finger2_id        : 7;
    unsigned int finger2_activelow : 1;
    unsigned int finger2_x         : 12;
    unsigned int finger2_y         : 12;
} __attribute__((packed, aligned(32)));

typedef struct psvs_ds3_input_report_t psvs_ds3_input_report_t;
typedef struct psvs_ds4_input_report_t psvs_ds4_input_report_t;

#define PSVS_BT_CONNECTION_TIMEOUT (5 * 1000 * 1000)
#define PSVS_BT_PACKET_TIMEOUT (1 * 1000 * 1000)

typedef enum psvs_gamepad_type_t {
    PSVS_GAMEPAD_NONE = 0,
    PSVS_GAMEPAD_DS3,
    PSVS_GAMEPAD_DS4,
} psvs_gamepad_type_t;

#define PSVS_TOUCH_MAX_FRAMES 4

typedef struct psvs_touch_point_t {
    uint8_t id;
    uint8_t r0;
    uint16_t x;
    uint16_t y;
} psvs_touch_point_t;

typedef struct psvs_touch_frame_t {
    uint64_t timestamp;
    int8_t port;
    int8_t count;
    psvs_touch_point_t points[2];
} psvs_touch_frame_t;

typedef struct psvs_touch_info_t {
    bool pad_down;
    int8_t pad_port;
    volatile int last; // Last frame index
    psvs_touch_frame_t frames[PSVS_TOUCH_MAX_FRAMES];
} psvs_touch_info_t;

#define PSVS_MOTION_MAX_FRAMES 64
#define PSVS_MOTION_FRAME_TIMEOUT (100 * 1000)

typedef struct psvs_float3_t {
    float x;
    float y;
    float z;
} psvs_float3_t;

typedef struct psvs_float4_t {
    float x;
    float y;
    float z;
    float w;
} psvs_float4_t;

typedef struct psvs_motion_frame_t {
    uint64_t timestamp;
    uint32_t counter;
    psvs_float3_t gyro;
    psvs_float3_t accel;
} psvs_motion_frame_t;

typedef struct psvs_motion_info_t {
	// Flags for gyro and accel
	int8_t gyro_flags;
	int8_t accel_flags;

    // Axis flis for gyro and accel (based on device info)
    int8_t gyro_axis_flip[3];
    int8_t accel_axis_flip[3];

	// Axis scales for gyro and accel
	SceFVector3 gyroZero;
	SceFVector3 gyroPosScale;
	SceFVector3 gyroNegScale;
	SceFVector3 accelZero;
	SceFVector3 accelPosScale;
	SceFVector3 accelNegScale;

	uint32_t counter;
    volatile int last; // Last frame index
    psvs_motion_frame_t frames[PSVS_MOTION_MAX_FRAMES]; // All motion frames
} psvs_motion_info_t;

typedef struct psvs_gamepad_t {
    psvs_gamepad_type_t type;
    uint64_t timestamp;
    int mac0;
    int mac1;
    psvs_touch_info_t touch;
    psvs_motion_info_t motion;
} psvs_gamepad_t;

static psvs_gamepad_t g_gamepad = {
    .type = PSVS_GAMEPAD_NONE,
    .touch = {
        .pad_down = 0,
        .pad_port = 0,
        .last = 0,
    },
    .motion = {
        .gyro_flags = 0,
        .accel_flags = 0,
        .counter = 0,
        .last = 0,
    },
};

// Force inline functions
#define INLINE __attribute__((always_inline)) __inline__

// Math constants
#define TAU 6.28318530717959f

#define PSVS_BT_VID_SONY  0x054C
#define PSVS_BT_PID_DS3   0x0268
#define PSVS_BT_PID_DS4_1 0x05C4
#define PSVS_BT_PID_DS4_2 0x09CC

// The actual resolution is 1920x942, but the PSTV adds a ton of deadzone (TODO: make the deadzone optional)
#define PSVS_DS4_TOUCH_X (0 + 60)
#define PSVS_DS4_TOUCH_Y (0 + 120)
#define PSVS_DS4_TOUCH_W (1920 - 2*60)
#define PSVS_DS4_TOUCH_H (942 - 2*120)

#define PSVS_VITA_TOUCH_X 0
#define PSVS_VITA_TOUCH_Y 0
#define PSVS_VITA_TOUCH_W (960*2)
#define PSVS_VITA_TOUCH_H (544*2)

#define PSVS_VITA_TOUCH_BACK_X 0
#define PSVS_VITA_TOUCH_BACK_Y (54*2)
#define PSVS_VITA_TOUCH_BACK_W (960*2)
#define PSVS_VITA_TOUCH_BACK_H (445*2 - 54*2)

#define PSVS_MOTION_DEADBAND_THRESHOLD (TAU/0x200)

INLINE static float _psvs_clamp(float x, float lo, float hi) {
    return (x > hi) ? hi : (x < lo) ? lo : x;
}

INLINE static float _psvs_vec_len3(float x, float y, float z) {
    float r = x * x + y * y + z * z;
    return (r > 0.0f) ? __builtin_sqrt(r) : 0.0f;
}

void psvs_bt_init() {
}

void psvs_bt_done() {
}

static psvs_gamepad_type_t _psvs_bt_get_gamepad_type(unsigned short vid, unsigned short pid) {
    // Check for DS3 and DS4 controllers
    if (vid == PSVS_BT_VID_SONY) {
        if (pid == PSVS_BT_PID_DS3)
            return PSVS_GAMEPAD_DS3;
        if (pid == PSVS_BT_PID_DS4_1 || pid == PSVS_BT_PID_DS4_2)
            return PSVS_GAMEPAD_DS4;
    }
    return PSVS_GAMEPAD_NONE;
}

bool psvs_bt_connected(unsigned int mac0, unsigned int mac1) {

    // No gamepad is currently connected
    if (!g_gamepad.type || ksceKernelGetSystemTimeWide() - g_gamepad.timestamp > PSVS_BT_CONNECTION_TIMEOUT) {
        // Get VID and PID
        unsigned short result[2];
        ksceBtGetVidPid(mac0, mac1, result);
        // Get gamepad type
        g_gamepad.type = _psvs_bt_get_gamepad_type(result[0], result[1]);
        // Initialize gamepad
        if (g_gamepad.type) {
            g_gamepad.mac0 = mac0;
            g_gamepad.mac1 = mac1;
            g_gamepad.timestamp = ksceKernelGetSystemTimeWide();
            g_gamepad.touch.pad_port = 0;
        }
        return !! g_gamepad.type;
    }

    // Connected gamepad
    if (g_gamepad.mac0 == mac0 && g_gamepad.mac1 == mac1) {
        g_gamepad.timestamp = ksceKernelGetSystemTimeWide();
        return true;
    }

    // Not the connected gamepad
    return false;
}

static uint32_t _psvs_rescale_touch_x(int port, int32_t x) {
    x -= PSVS_DS4_TOUCH_X;
    x = (x < 0) ? 0 : x;
    x = (x >= PSVS_DS4_TOUCH_W) ? PSVS_DS4_TOUCH_W - 1 : x;
    if (port == SCE_TOUCH_PORT_FRONT)
        return (x * (PSVS_VITA_TOUCH_W - 1)) / (PSVS_DS4_TOUCH_W - 1) + PSVS_VITA_TOUCH_X;
    if (port == SCE_TOUCH_PORT_BACK)
        return (x * (PSVS_VITA_TOUCH_BACK_W - 1)) / (PSVS_DS4_TOUCH_W - 1) + PSVS_VITA_TOUCH_BACK_X;
    return 0;
}

static uint32_t _psvs_rescale_touch_y(int port, int32_t y) {
    y -= PSVS_DS4_TOUCH_Y;
    y = (y < 0) ? 0 : y;
    y = (y >= PSVS_DS4_TOUCH_H) ? PSVS_DS4_TOUCH_H - 1 : y;
    if (port == SCE_TOUCH_PORT_FRONT)
        return (y * (PSVS_VITA_TOUCH_H - 1)) / (PSVS_DS4_TOUCH_H - 1) + PSVS_VITA_TOUCH_Y;
    if (port == SCE_TOUCH_PORT_BACK)
        return (y * (PSVS_VITA_TOUCH_BACK_H - 1)) / (PSVS_DS4_TOUCH_H - 1) + PSVS_VITA_TOUCH_BACK_Y;
    return 0;
}

void psvs_bt_on_hid_transfer(SceBtHidRequest * head) {

    // Handle touch events
    if (g_profile.bt_touch && g_gamepad.type == PSVS_GAMEPAD_DS4) {
        for (SceBtHidRequest * request = head; request; request = request->next) {
            if ((request->type == 0) && request->buffer && request->length >= sizeof(psvs_ds4_input_report_t)) {

                // Process DS4 input
                psvs_ds4_input_report_t * report = (psvs_ds4_input_report_t*) request->buffer;

                // Read touchpad press
                bool pressed = report->tpad && !g_gamepad.touch.pad_down;
                g_gamepad.touch.pad_down = report->tpad;

                // Toggle active port
                if (pressed && g_profile.bt_touch != PSVS_BT_TOUCH_FRONT) {
                    if (g_gamepad.touch.pad_port < SCE_TOUCH_PORT_MAX_NUM - 1)
                        ++ g_gamepad.touch.pad_port;
                    else
                        g_gamepad.touch.pad_port = (g_profile.bt_touch == PSVS_BT_TOUCH_TOGGLE_F_B) ? 0 : -1;
                }

                // Create touch data (only for the last request)
                if (!request->next)
                {
                    // Active port
                    int port = g_gamepad.touch.pad_port;

                    // Frame data
                    psvs_touch_frame_t frame = {
                        .timestamp = g_gamepad.timestamp,
                        .port = port,
                        .count = 0,
                    };

                    // Add 1st finger
                    if (!report->finger1_activelow) {
                        psvs_touch_point_t point = {.id = report->finger1_id, .x = report->finger1_x, .y = report->finger1_y};
                        frame.points[frame.count ++] = point;
                    }
                    // Add 2nd finger
                    if (!report->finger2_activelow) {
                        psvs_touch_point_t point = {.id = report->finger2_id, .x = report->finger2_x, .y = report->finger2_y};
                        frame.points[frame.count ++] = point;
                    }

                    // Append frame data
                    int next = (g_gamepad.touch.last + 1) % PSVS_TOUCH_MAX_FRAMES;
                    g_gamepad.touch.frames[next] = frame;
                    __atomic_store_n(&g_gamepad.touch.last, next, __ATOMIC_SEQ_CST);
                }
            }
        }
    }

    // Handle motion events
    if (g_profile.bt_motion) {
        for (SceBtHidRequest * request = head; request; request = request->next) {
            if (g_gamepad.type == PSVS_GAMEPAD_DS3) {
                if ((request->type == 0) && request->buffer && request->length >= sizeof(psvs_ds3_input_report_t)) {
                    // Process DS3 input
                    //psvs_ds3_input_report_t * report = (psvs_ds3_input_report_t*) request->buffer;

                    // FUTURE: Maybe if I get a DS3, I'll implement this
                }
            } else if (g_gamepad.type == PSVS_GAMEPAD_DS4) {
                if ((request->type == 0) && request->buffer && request->length >= sizeof(psvs_ds4_input_report_t)) {
                    // Process DS4 input
                    psvs_ds4_input_report_t * report = (psvs_ds4_input_report_t*) request->buffer;

                    // Last input
                    if (!request->next) {
                        // Frame data (from previous frame)
                        psvs_motion_frame_t frame;

                        // Add raw motion data (degree / sec)
                        frame.gyro.x = report->gyro_x * 0.064f;
                        frame.gyro.y = report->gyro_y * 0.064f;
                        frame.gyro.z = report->gyro_z * 0.064f;

                        // Convert acceleration data (one G ~ 8200 ~ 0x2000 on DS4)
                        frame.accel.x = report->accel_x / (float) 0x2000;
                        frame.accel.y = report->accel_y / (float) 0x2000;
                        frame.accel.z = report->accel_z / (float) 0x2000;

                        // Update timestamp and counter
                        frame.counter = g_gamepad.motion.counter ++;
                        frame.timestamp = g_gamepad.timestamp;

                        // Append frame data
                        int next = (g_gamepad.motion.last + 1) % PSVS_MOTION_MAX_FRAMES;
                        g_gamepad.motion.frames[next] = frame;
                        __atomic_store_n(&g_gamepad.motion.last, next, __ATOMIC_SEQ_CST);
                    }
                }
            }
        }
    }
}

int psvs_bt_touch_filter_input(bool peek, uint32_t port, SceTouchData *pData, uint32_t nBufs) {

    // Validate parameters
    if (port >= SCE_TOUCH_PORT_MAX_NUM || nBufs < 0 || nBufs > 64)
        return SCE_TOUCH_ERROR_INVALID_ARG;

    // On connection dropped
    if (!g_gamepad.type || ksceKernelGetSystemTimeWide() - g_gamepad.timestamp > PSVS_BT_CONNECTION_TIMEOUT) {
        g_gamepad.type = PSVS_GAMEPAD_NONE;
        return nBufs;
    }

    // On very old data
    if (ksceKernelGetSystemTimeWide() - g_gamepad.timestamp > PSVS_BT_PACKET_TIMEOUT) {
        port = SCE_TOUCH_PORT_MAX_NUM + 1; // Make both panels inactive
    }

    // Get latest frame
    int last = __atomic_load_n(&g_gamepad.touch.last, __ATOMIC_SEQ_CST); // Atomic ensures that we never read the buffer that is currently written
    psvs_touch_frame_t * frame = &g_gamepad.touch.frames[last];

    // Base data
    SceTouchData data = {
        .timeStamp = frame->timestamp,
        .status = 0,
        .reportNum = 0,
    };

    // On active panel
    if (port == frame->port) {
        data.reportNum = frame->count;
        for (int j = 0; j < frame->count; ++ j) {
            SceTouchReport point = {
                .id = frame->points[j].id,
                .force = 128,
                .x = _psvs_rescale_touch_x(port, frame->points[j].x),
                .y = _psvs_rescale_touch_y(port, frame->points[j].y),
            };
            data.report[j] = point;
        }
    }

    // TODO: use more then one frame
    for (int i = 0; i < nBufs; ++ i) {
        // Override data
        pData[i] = data;
    }

    // Return touch data
    return nBufs;
}

int psvs_bt_motion_filter_read(SceMotionDevResult * resultList, uint32_t count, int * setFlag) {

	// Kernel side data buffer
	SceMotionDevResult buffer;

	// Count is always 64, but it does not hurt to check
	if (count == 0)
		return count;

    // Get latest frame
    int last = __atomic_load_n(&g_gamepad.motion.last, __ATOMIC_SEQ_CST); // Atomic ensures that we never read the buffer that is currently written
    psvs_motion_frame_t frame = g_gamepad.motion.frames[last];

    // Fill out buffer
	buffer.timestamp = frame.timestamp;
	buffer.entryCount = 1;
	buffer.magnCalibIndex = 0;
	buffer.magnFieldStab = 0;
	buffer.gyroCalibIndex = 0;
	buffer.timeInMSec = ksceKernelGetSystemTimeWide() / 1000;

	// Fill out entry
	buffer.entryList[0].flags = SCE_MOTION_DEV_ENTRY_HAS_GYRO_DATA | SCE_MOTION_DEV_ENTRY_HAS_ACCEL_DATA;

	// Scale acceleration
	frame.accel.x *= (frame.accel.x > 0.0f ? g_gamepad.motion.accelPosScale.x : g_gamepad.motion.accelNegScale.x);
	frame.accel.y *= (frame.accel.y > 0.0f ? g_gamepad.motion.accelPosScale.y : g_gamepad.motion.accelNegScale.y);
	frame.accel.z *= (frame.accel.z > 0.0f ? g_gamepad.motion.accelPosScale.z : g_gamepad.motion.accelNegScale.z);

	// Scale gyro
	frame.gyro.x *= (frame.gyro.x > 0.0f ? g_gamepad.motion.gyroPosScale.x : g_gamepad.motion.gyroNegScale.x);
	frame.gyro.y *= (frame.gyro.y > 0.0f ? g_gamepad.motion.gyroPosScale.y : g_gamepad.motion.gyroNegScale.y);
	frame.gyro.z *= (frame.gyro.z > 0.0f ? g_gamepad.motion.gyroPosScale.z : g_gamepad.motion.gyroNegScale.z);

	// Apply orientation (accel_y and accel_x are swapped by SceMotion)
	if (g_profile.bt_motion == PSVS_BT_MOTION_NORMAL) {
		// TODO: Axis flips

		// Convert acceleration to uint16_t with offset
		buffer.entryList[0].accel_y = (uint16_t) (0.5f + frame.accel.x + g_gamepad.motion.accelZero.x);
		buffer.entryList[0].accel_x = (uint16_t) (0.5f + frame.accel.z + g_gamepad.motion.accelZero.z);
		buffer.entryList[0].accel_z = (uint16_t) (0.5f + frame.accel.y + g_gamepad.motion.accelZero.y);

		// Convert gyro to uint16_t with offset
		buffer.entryList[0].gyro_y = (uint16_t) (0.5f + frame.gyro.x + g_gamepad.motion.gyroZero.x);
		buffer.entryList[0].gyro_x = (uint16_t) (0.5f + frame.gyro.z + g_gamepad.motion.gyroZero.z);
		buffer.entryList[0].gyro_z = (uint16_t) (0.5f + frame.gyro.y + g_gamepad.motion.gyroZero.y);
	} else {
		// TODO: Axis flips

		// Convert acceleration to uint16_t with offset
		buffer.entryList[0].accel_y = (uint16_t) (0.5f + frame.accel.x + g_gamepad.motion.accelZero.x);
		buffer.entryList[0].accel_x = (uint16_t) (0.5f - frame.accel.y + g_gamepad.motion.accelZero.y);
		buffer.entryList[0].accel_z = (uint16_t) (0.5f + frame.accel.z + g_gamepad.motion.accelZero.z);

		// Convert gyro to uint16_t with offset
		buffer.entryList[0].gyro_y = (uint16_t) (0.5f + frame.gyro.x + g_gamepad.motion.gyroZero.x);
		buffer.entryList[0].gyro_x = (uint16_t) (0.5f - frame.gyro.y + g_gamepad.motion.gyroZero.y);
		buffer.entryList[0].gyro_z = (uint16_t) (0.5f + frame.gyro.z + g_gamepad.motion.gyroZero.z);
	}

	// Copy to user buffer
	ksceKernelMemcpyKernelToUser(resultList, &buffer, sizeof(buffer));

	// For now only a single event for each call
	return 1;
}

void psvs_bt_motion_set_device_info(const uint32_t * info) {
	// TODO: Implement
}

void psvs_bt_motion_set_gyro_bias(const SceMotionDevGyroBias * bias) {
	// TODO: Implement
}

void psvs_bt_motion_set_gyro_calib_data(const SceMotionDevGyroCalibData * data) {
	// TODO: Implement
}

void psvs_bt_motion_set_accel_calib_data(const SceMotionDevAccCalibData * data) {
	// TODO: Implement
}

int psvs_bt_motion_reset_device_info(uint32_t * info) {
	// Reset axis flips
	g_gamepad.motion.gyro_axis_flip[0] = false;
	g_gamepad.motion.gyro_axis_flip[1] = false;
	g_gamepad.motion.gyro_axis_flip[2] = false;
	g_gamepad.motion.accel_axis_flip[0] = false;
	g_gamepad.motion.accel_axis_flip[1] = false;
	g_gamepad.motion.accel_axis_flip[2] = false;
	// Set device info
	info = 0;
	// Return SCE_OK
	return 0;
}

int psvs_bt_motion_reset_gyro_bias(SceMotionDevGyroBias * bias) {
	// Reset gyro zero
	g_gamepad.motion.gyroZero.x = 0x8000;
	g_gamepad.motion.gyroZero.y = 0x8000;
	g_gamepad.motion.gyroZero.z = 0x8000;
	// Set gyro bias
	bias->zero = g_gamepad.motion.gyroZero;
	// Return SCE_OK
	return 0;
}

int psvs_bt_motion_reset_gyro_calib_data(SceMotionDevGyroCalibData * data) {
	// Reset gyro scale
	g_gamepad.motion.gyroZero.x = 0x8000;
	g_gamepad.motion.gyroZero.y = 0x8000;
	g_gamepad.motion.gyroZero.z = 0x8000;
	g_gamepad.motion.gyroPosScale.x = 10.0f; // resolution = 0.1 deg/sec ~ 0.0009 rad/sec
	g_gamepad.motion.gyroPosScale.y = 10.0f;
	g_gamepad.motion.gyroPosScale.z = 10.0f;
	g_gamepad.motion.gyroNegScale.x = 10.0f;
	g_gamepad.motion.gyroNegScale.y = 10.0f;
	g_gamepad.motion.gyroNegScale.z = 10.0f;
	// Set gyro calib data
	data->zero = g_gamepad.motion.gyroZero;
	data->xPosScale = 0x8000 * 0.1f;
	data->yPosScale = 0x8000 * 0.1f;
	data->zPosScale = 0x8000 * 0.1f;
	data->xPos.x = 0x10000;
	data->xPos.y = 0x8000;
	data->xPos.z = 0x8000;
	data->yPos.x = 0x8000;
	data->yPos.y = 0x10000;
	data->yPos.z = 0x8000;
	data->zPos.x = 0x8000;
	data->zPos.y = 0x8000;
	data->zPos.z = 0x10000;
	data->xNegScale = -0x8000 * 0.1f;
	data->yNegScale = -0x8000 * 0.1f;
	data->zNegScale = -0x8000 * 0.1f;
	data->xNeg.x = 0;
	data->xNeg.y = 0x8000;
	data->xNeg.z = 0x8000;
	data->yNeg.x = 0x8000;
	data->yNeg.y = 0;
	data->yNeg.z = 0x8000;
	data->zNeg.x = 0x8000;
	data->zNeg.y = 0x8000;
	data->zNeg.z = 0;
	// Return SCE_OK
	return 0;
}

int psvs_bt_motion_reset_accel_calib_data(SceMotionDevAccCalibData * data) {
	// Reset accel scale
	g_gamepad.motion.accelZero.x = 0x8000;
	g_gamepad.motion.accelZero.y = 0x8000;
	g_gamepad.motion.accelZero.z = 0x8000;
	g_gamepad.motion.accelPosScale.x = 1000.0f; // Resolution = 0.001 G
	g_gamepad.motion.accelPosScale.y = 1000.0f;
	g_gamepad.motion.accelPosScale.z = 1000.0f;
	g_gamepad.motion.accelNegScale.x = 1000.0f;
	g_gamepad.motion.accelNegScale.y = 1000.0f;
	g_gamepad.motion.accelNegScale.z = 1000.0f;
	// Set accel calib data
	data->xPosScale = 0x8000 * 0.001f;
	data->yPosScale = 0x8000 * 0.001f;
	data->zPosScale = 0x8000 * 0.001f;
	data->xPos.x = 0x10000;
	data->xPos.y = 0x8000;
	data->xPos.z = 0x8000;
	data->yPos.x = 0x8000;
	data->yPos.y = 0x10000;
	data->yPos.z = 0x8000;
	data->zPos.x = 0x8000;
	data->zPos.y = 0x8000;
	data->zPos.z = 0x10000;
	data->xNegScale = -0x8000 * 0.001f;
	data->yNegScale = -0x8000 * 0.001f;
	data->zNegScale = -0x8000 * 0.001f;
	data->xNeg.x = 0;
	data->xNeg.y = 0x8000;
	data->xNeg.z = 0x8000;
	data->yNeg.x = 0x8000;
	data->yNeg.y = 0;
	data->yNeg.z = 0x8000;
	data->zNeg.x = 0x8000;
	data->zNeg.y = 0x8000;
	data->zNeg.z = 0;
	// Return SCE_OK
	return 0;
}


