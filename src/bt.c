#include <vitasdkkern.h>
#include <psp2/touch.h>
#include <psp2/motion.h>
#include <taihen.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "oc.h"
#include "bt.h"
#include "profile.h"

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

    // This is the direction of accelearation (gravity), compared to the controller
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
    psvs_float3_t gyro;
    psvs_float3_t accel;
} psvs_motion_frame_t;

typedef struct psvs_motion_info_t {
    volatile psvs_motion_flags_t flags;
    volatile int last; // Last frame index
    psvs_float4_t rotation[2]; // Double buffered rotation data
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
        .flags = PSVS_MOTION_FLAG_ENABLE_DEAD_BAND | PSVS_MOTION_FLAG_ENABLE_TILT_CORRECTION,
        .rotation = {{0, 0, 0, 1}, {0, 0, 0, 1}},
        .last = 0,
    },
};

typedef struct psvs_motion_process_info_t {
    psvs_motion_flags_t flags;
    float angle_treshold;
} psvs_motion_process_info_t;

typedef struct psvs_bt_process_info_t {
    bool init;
    psvs_motion_process_info_t motion;
} psvs_bt_process_info_t;

static int g_bt_process_info_uid = -1;;

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

int32_t psvs_bt_debug_x = 0;
int32_t psvs_bt_debug_y = 0;
int32_t psvs_bt_debug_z = 0;
int32_t psvs_bt_debug_w = 0;

#define TAU 6.28318530717959f

#define INLINE __attribute__((always_inline))

INLINE static float _psvs_clamp(float x, float lo, float hi) {
    return (x > hi) ? hi : (x < lo) ? lo : x;
}

INLINE static float _psvs_vec_len3(float x, float y, float z) {
    float r = x * x + y * y + z * z;
    return (r > 0.0f) ? __builtin_sqrt(r) : 0.0f;
}

INLINE static float _psvs_vec_len4(float x, float y, float z, float w) {
    float r = x * x + y * y + z * z + w * w;
    return (r > 0.0f) ? __builtin_sqrt(r) : 0.0f;
}

// Sine function in the [-TAU/4, +TAU/4] range
INLINE static float _psvs_small_sin(float angle) {
    return ((0.00762664*angle*angle - 0.166017)*angle*angle + 0.999875)*angle;
}

// Cosine function in the [-TAU/2, +TAU/2] range
INLINE static float _psvs_small_cos(float angle) {
    if (angle < 0)
        angle = -angle;
    return _psvs_small_sin(TAU/4-angle);
}

// Full range sine function
static float _psvs_sin(float angle) {
    // Separate sign and magnitude
    bool sign = false;
    if (angle < 0.0f) {
        angle = -angle;
        sign = true;
    }
    
    // Extend the range of the function
    if (angle < TAU/4) {
        // value is in range
    } else if (angle < 3/4*TAU) {
        // Remove half a circle
        angle -= TAU/2;
    } else if (angle < 5/4*TAU) {
        // Remove a full circle
        angle -= TAU;
    } else {
        // Remove multiple half cirles
        do {
            int32_t d = (int32_t) ((angle - TAU/4) / (TAU/2));
            angle -= d * (TAU/2);
        } while (angle > TAU/4); // Prevent errors due to insufficient precision
    }
    
    // apply sign
    if (sign)
        angle = -angle;
    
    // Call small range sin function
    return _psvs_small_sin(angle);
}

// Full range cosine function
static float _psvs_cos(float angle) {
    // Discard sign
    if (angle < 0.0f) {
        angle = -angle;
    }

    // Extend the range of the function
    if (angle < TAU/2) {
        // value is in range
    } else if (angle < 3/2 *TAU) {
        // Remove a circle
        angle -= TAU;
    } else {
        // Remove multiple cirles
        do {
            int32_t d = (int32_t) ((angle - TAU/2) / TAU);
            angle -= d * TAU;
        } while (angle > TAU/2); // Prevent errors due to insufficient precision
    }

    // Call small range cos function
    return _psvs_small_cos(angle);
}

// Multiply quaternions
static psvs_float4_t _psvs_quat_mul(psvs_float4_t * left, psvs_float4_t * right) {
    psvs_float4_t result = {
        .x = left->x * right->w + left->w * right->x + left->y * right->z - left->z * right->y,
        .y = left->y * right->w + left->w * right->y + left->z * right->x - left->x * right->z,
        .z = left->z * right->w + left->w * right->z + left->x * right->y - left->y * right->x,
        .w = left->w * right->w - left->x * right->x - left->y * right->y - left->z * right->z,
    };
    return result;
}

void psvs_bt_init() {
    // Init Process local storage
    g_bt_process_info_uid = ksceKernelCreateProcessLocalStorage("psvs_bt_process_info", sizeof(psvs_bt_process_info_t));
}

void psvs_bt_done() {
}

static psvs_bt_process_info_t * _psvs_bt_process_info(void) {
    // Get process info
    psvs_bt_process_info_t * info = ksceKernelGetProcessLocalStorageAddr(g_bt_process_info_uid);
    // Initialize process info
    if (!info->init) {
        psvs_bt_process_info_t init = {
            .init = true,
            .motion = {
                .flags = PSVS_MOTION_FLAG_ENABLE_DEAD_BAND | PSVS_MOTION_FLAG_ENABLE_TILT_CORRECTION,
                .angle_treshold = 0,
            },
        };
        *info = init;
    }
    // Return process info
    return info;
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
                if (pressed) {
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
                    psvs_ds3_input_report_t * report = (psvs_ds3_input_report_t*) request->buffer;

                    // TODO: If I ever get a DS3, I'll implement this
                }
            } else if (g_gamepad.type == PSVS_GAMEPAD_DS4) {
                if ((request->type == 0) && request->buffer && request->length >= sizeof(psvs_ds4_input_report_t)) {
                    // Process DS4 input
                    psvs_ds4_input_report_t * report = (psvs_ds4_input_report_t*) request->buffer;

                    // Last input
                    if (!request->next) {
                        // Frame data (from previous frame)
                        psvs_motion_frame_t frame = g_gamepad.motion.frames[g_gamepad.motion.last];
                        psvs_float4_t rotation = g_gamepad.motion.rotation[g_gamepad.motion.last & 1];

                        // Add raw motion data (one radian ~ 1000 ~ 0x400 an DS4)
                        frame.gyro.x = report->gyro_x / (float) 0x400;
                        frame.gyro.y = report->gyro_y / (float) 0x400;
                        frame.gyro.z = report->gyro_z / (float) 0x400;

                        // Convert acceleration data (one G ~ 8200 ~ 0x2000 on DS4)
                        float accel_x = report->accel_x / (float) 0x2000;
                        float accel_y = report->accel_y / (float) 0x2000;
                        float accel_z = report->accel_z / (float) 0x2000;

                        // Apply smoothing to accelerometer data (dynamic EWMA filter to get rid of the jitter)
                        if (g_gamepad.timestamp - frame.timestamp < PSVS_MOTION_FRAME_TIMEOUT) {
                            // Distance from previous point
                            uint32_t d = (uint32_t) _psvs_vec_len3((accel_x - frame.accel.x),
                                (accel_y - frame.accel.y), (accel_z - frame.accel.z));

                            // Weight based on distance
                            int w = 1 + (d > 0x800 ? 15 : (15 * d) / 0x800);

                            // Filter based on weight
                            frame.accel.x = (accel_x * w + frame.accel.x * (32 - w)) / 32;
                            frame.accel.y = (accel_y * w + frame.accel.y * (32 - w)) / 32;
                            frame.accel.z = (accel_z * w + frame.accel.z * (32 - w)) / 32;
                        } else {
                            // Non-continous data
                            frame.accel.x = accel_x;
                            frame.accel.y = accel_y;
                            frame.accel.z = accel_z;
                        }

                        // Update timestamp
                        uint32_t dt = g_gamepad.timestamp - frame.timestamp;
                        frame.timestamp = g_gamepad.timestamp;

                        // Update global data
                        if (dt < PSVS_MOTION_FRAME_TIMEOUT) {
                            // Clamp angular speed, and halve the angles
                            psvs_float4_t delta = {
                                .x = _psvs_clamp(frame.gyro.x * (dt * 0.000001f), -TAU/8, +TAU/8) / 2,
                                .y = _psvs_clamp(frame.gyro.y * (dt * 0.000001f), -TAU/8, +TAU/8) / 2,
                                .z = _psvs_clamp(frame.gyro.z * (dt * 0.000001f), -TAU/8, +TAU/8) / 2,
                            };

                            // Combine gyro state into a single quaternion (unnormalized)
                            delta.x = _psvs_small_sin(delta.x) / _psvs_small_cos(delta.x);
                            delta.y = _psvs_small_sin(delta.y) / _psvs_small_cos(delta.y);
                            delta.z = _psvs_small_sin(delta.z) / _psvs_small_cos(delta.z);
                            delta.w = 1;

                            // Add to global rotation
                            rotation = _psvs_quat_mul(&rotation, &delta);
                        }

                        // Apply correction
                        if (g_gamepad.motion.flags & PSVS_MOTION_FLAG_ENABLE_TILT_CORRECTION) {
                        }

                        // Normalize global rotation
                        float q = 1 / _psvs_vec_len4(rotation.x, rotation.y, rotation.z, rotation.w);
                        rotation.x *= q;
                        rotation.y *= q;
                        rotation.z *= q;
                        rotation.w *= q;

                        // Append frame data
                        int next = (g_gamepad.motion.last + 1) % PSVS_MOTION_MAX_FRAMES;
                        g_gamepad.motion.frames[next] = frame;
                        g_gamepad.motion.rotation[next & 1] = rotation;
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


int psvs_bt_motion_filter_state(SceMotionState * motionState) {
    // Result data in Kernel
    SceMotionState result = {
    };

    // Get process info for motion
    psvs_motion_process_info_t * info = &_psvs_bt_process_info()->motion;

    // Last index and sample
    int last = __atomic_load_n(&g_gamepad.motion.last, __ATOMIC_SEQ_CST);
    psvs_motion_frame_t frame = g_gamepad.motion.frames[last];

    // Derive timestamps
    result.timestamp = frame.timestamp;
    result.hostTimestamp = frame.timestamp + 1000; // 1ms delay just for fun

    // Acceleration
    if (g_profile.bt_motion == PSVS_BT_MOTION_NORMAL) {
        // Vita held flat
        result.acceleration.x = - frame.accel.x; // +X is right on Vita
        result.acceleration.y = + frame.accel.z; // +Y is forward on vita
        result.acceleration.z = - frame.accel.y; // +Z is up on Vita
    } else {
        // Vita held vertically
        result.acceleration.x = - frame.accel.x; // +X is right when held vertically
        result.acceleration.y = - frame.accel.y; // +Y is up when held vertically
        result.acceleration.z = - frame.accel.z; // +Z is backward when held vertically
        
    }
    
    // Gyroscope
    if (g_profile.bt_motion == PSVS_BT_MOTION_NORMAL) {
        result.angularVelocity.x = + frame.gyro.x; // +X is forward to up on Vita
        result.angularVelocity.y = - frame.gyro.z; // +Y is up to right on Vita
        result.angularVelocity.z = + frame.gyro.y; // +Z is right to forwart on Vita
    } else {
        result.angularVelocity.x = + frame.gyro.x; // +X is forward to up when held vertically
    }

    // When filtering discard small rotations as error
    if (info->flags & PSVS_MOTION_FLAG_ENABLE_DEAD_BAND) {
    }
    
    // Copy result to User buffer
    //ksceKernelMemcpyKernelToUser((uintptr_t) motionState, &result, sizeof(result));

    // return SCE_OK
    return 0;
}

int psvs_bt_motion_filter_sensorstate(SceMotionSensorState *sensorState, int numRecords) {

    // Result data in Kernel
    SceMotionSensorState result = {
    };

    // Get last buffer index

    for (int i = 0; i < numRecords; ++ i) {


        // Copy result to User buffer
        //ksceKernelMemcpyKernelToUser((uintptr_t) (sensorState + i), &result, sizeof(result));
    }

    // return SCE_OK
    return 0;
}

bool psvs_bt_motion_get_flag(psvs_motion_flags_t flag) {
    // Return flag data
    return _psvs_bt_process_info()->motion.flags & flag;
}

void psvs_bt_motion_set_flag(psvs_motion_flags_t flag, bool value) {
    // Set flag data
    if (value)
        _psvs_bt_process_info()->motion.flags |= flag;
    else
        _psvs_bt_process_info()->motion.flags &= ~flag;
}
