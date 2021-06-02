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

    union {
        unsigned short gyro_z;
        unsigned short roll;
    };
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

    signed short accel_x;
    signed short accel_y;
    signed short accel_z;

    union {
        signed short roll;
        signed short gyro_z;
    };
    union {
        signed short yaw;
        signed short gyro_y;
    };
    union {
        signed short pitch;
        signed short gyro_x;
    };

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

#define PSVS_BT_CONNECTION_TIMEOUT 5 * (1000) * (1000)
#define PSVS_BT_PACKET_TIMEOUT 1 * (1000) * (1000)

typedef enum psvs_gamepad_type_t {
    PSVS_GAMEPAD_NONE = 0,
    PSVS_GAMEPAD_DS3,
    PSVS_GAMEPAD_DS4,
} psvs_gamepad_type_t;

typedef struct psvs_touch_panel_t {
    volatile uint64_t count;
    volatile int last; // last used buffer
    SceTouchData buffers[2]; // double buffered the input (cheaper then a mutex)
} psvs_touch_panel_t;

typedef struct psvs_touch_info_t {
    bool pad_down;
    int8_t pad_port;
    int8_t active_port;
    psvs_touch_panel_t active_panel; // Data for active panel
    psvs_touch_panel_t inactive_panel; // Data for inactive panels
} psvs_touch_info_t;

typedef struct psvs_motion_info_t {
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
};

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
            g_gamepad.touch.active_port = 0;
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
                // Interpret data
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

                    // Base data
                    SceTouchData data = {
                        .timeStamp = g_gamepad.timestamp,
                        .status = 0,
                        .reportNum = 0,
                    };

                    // Handle inactive panels
                    int next = (g_gamepad.touch.inactive_panel.last + 1) & 1;
                    g_gamepad.touch.inactive_panel.buffers[next] = data;
                    g_gamepad.touch.inactive_panel.count += 1;
                    __atomic_store_n(&g_gamepad.touch.inactive_panel.last, next, __ATOMIC_SEQ_CST);

                    // Handle active panel
                    if (port >= 0 && port < SCE_TOUCH_PORT_MAX_NUM) {
                        // Add 1st finger
                        if (!report->finger1_activelow) {
                            SceTouchReport finger = {
                                .id = report->finger1_id,
                                .force = 128,
                                .x = _psvs_rescale_touch_x(port, report->finger1_x),
                                .y = _psvs_rescale_touch_y(port, report->finger1_y),
                            };
                            data.report[data.reportNum ++] = finger;
                        }
                        // Add 2nd finger
                        if (!report->finger2_activelow) {
                            SceTouchReport finger = {
                                .id = report->finger2_id,
                                .force = 128,
                                .x = _psvs_rescale_touch_x(port, report->finger2_x),
                                .y = _psvs_rescale_touch_y(port, report->finger2_y),
                            };
                            data.report[data.reportNum ++] = finger;
                        }
                        // Update active port
                        g_gamepad.touch.active_port = port;
                        // Update panel data
                        next = (g_gamepad.touch.active_panel.last + 1) & 1;
                        g_gamepad.touch.active_panel.buffers[next] = data;
                        g_gamepad.touch.active_panel.count += 1;
                        __atomic_store_n(&g_gamepad.touch.active_panel.last, next, __ATOMIC_SEQ_CST);
                    }
                }
            }
        }
    }

    // Handle motion events
    /*
    if (g_profile.bt_motion) {
        for (SceBtHidRequest * request = head; request; request = request->next) {
            if (g_gamepad.type == PSVS_GAMEPAD_DS4) {
                if ((request->type == 0) && request->buffer && request->length >= sizeof(psvs_ds4_input_report_t)) {
                    // DS4 input received
                    psvs_ds4_input_report_t * report = (psvs_ds4_input_report_t*) request->buffer;
                }
            } else if (g_gamepad.type == PSVS_GAMEPAD_DS3) {
                if ((request->type == 0) && request->buffer && request->length >= sizeof(psvs_ds3_input_report_t)) {
                    // DS3 input received
                    psvs_ds3_input_report_t * report = (psvs_ds3_input_report_t*) request->buffer;
                }
            }
        }
    }
    */
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

    // Get active port
    int active_port = g_gamepad.touch.active_port;

    // On very old data
    if (ksceKernelGetSystemTimeWide() - g_gamepad.timestamp > PSVS_BT_PACKET_TIMEOUT) {
        active_port = -1; // Make both panels inactive
    }

    // Override touch data with the latest one
    for (int i = 0; i < nBufs; ++ i) {
        if (port == active_port) {
            int last = __atomic_load_n(&g_gamepad.touch.active_panel.last, __ATOMIC_SEQ_CST) & 1; // Atomic ensures that we never read the buffer that is currently written
            pData[i] = g_gamepad.touch.active_panel.buffers[last];
        } else {
            int last = __atomic_load_n(&g_gamepad.touch.inactive_panel.last, __ATOMIC_SEQ_CST) & 1; // Atomic ensures that we never read the buffer that is currently written
            pData[i] = g_gamepad.touch.inactive_panel.buffers[last];
        }
    }

    // Return touch data
    return nBufs;
}

// ksceKernelCreateProcessLocalStorage() + ksceKernelGetProcessLocalStorageAddr()
// Store motion and touch settings per process

