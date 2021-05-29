#include <vitasdkkern.h>
#include <taihen.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "oc.h"
#include "bt.h"
#include "profile.h"

// Declare helper getter/setter for GpuEs4
static int __kscePowerGetGpuEs4ClockFrequency() {
    int a1, a2;
    _kscePowerGetGpuEs4ClockFrequency(&a1, &a2);
    return a1;
}
static int __kscePowerSetGpuEs4ClockFrequency(int freq) {
    return _kscePowerSetGpuEs4ClockFrequency(freq, freq);
}

// Declare static getters/setters
PSVS_OC_DECL_GETTER(_kscePowerGetArmClockFrequency);
PSVS_OC_DECL_GETTER(_kscePowerGetBusClockFrequency);
PSVS_OC_DECL_GETTER(_kscePowerGetGpuXbarClockFrequency);
PSVS_OC_DECL_SETTER(_kscePowerSetArmClockFrequency);
PSVS_OC_DECL_SETTER(_kscePowerSetBusClockFrequency);
PSVS_OC_DECL_SETTER(_kscePowerSetGpuXbarClockFrequency);

static psvs_oc_devopt_t g_oc_devopt[PSVS_OC_DEVICE_MAX] = {
    [PSVS_OC_DEVICE_CPU] = {
        .freq_n = 8, .freq = {41, 83, 111, 166, 222, 333, 444, 500}, .default_freq = 333,
        .get_freq = __kscePowerGetArmClockFrequency,
        .set_freq = __kscePowerSetArmClockFrequency
    },
    [PSVS_OC_DEVICE_GPU_ES4] = {
        .freq_n = 6, .freq = {41, 55, 83, 111, 166, 222}, .default_freq = 111,
        .get_freq = __kscePowerGetGpuEs4ClockFrequency,
        .set_freq = __kscePowerSetGpuEs4ClockFrequency
    },
    [PSVS_OC_DEVICE_BUS] = {
        .freq_n = 5, .freq = {55, 83, 111, 166, 222}, .default_freq = 222,
        .get_freq = __kscePowerGetBusClockFrequency,
        .set_freq = __kscePowerSetBusClockFrequency
    },
    [PSVS_OC_DEVICE_GPU_XBAR] = {
        .freq_n = 3, .freq = {83, 111, 166}, .default_freq = 111,
        .get_freq = __kscePowerGetGpuXbarClockFrequency,
        .set_freq = __kscePowerSetGpuXbarClockFrequency
    },
};

int psvs_oc_get_freq(psvs_oc_device_t device) {
    return g_oc_devopt[device].get_freq();
}

int psvs_oc_set_freq(psvs_oc_device_t device, int freq) {
    return g_oc_devopt[device].set_freq(freq);
}

void psvs_oc_holy_shit() {
    // Apply mul:div (15:0)
    ScePervasiveForDriver_0xE9D95643(15, 16 - 0);

    // Store global freq & mul for kscePowerGetArmClockFrequency()
    *ScePower_41C8 = 500;
    *ScePower_41CC = 15;
}

int psvs_oc_get_target_freq(psvs_oc_device_t device, int default_freq) {
    if (g_profile.mode[device] == PSVS_OC_MODE_MANUAL)
        return g_profile.manual_freq[device];
    return default_freq;
}

void psvs_oc_set_target_freq(psvs_oc_device_t device) {
    // Refresh manual clocks
    if (g_profile.mode[device] == PSVS_OC_MODE_MANUAL)
        psvs_oc_set_freq(device, g_profile.manual_freq[device]);
    // Restore default clocks
    else if (g_profile.mode[device] == PSVS_OC_MODE_DEFAULT)
        psvs_oc_set_freq(device, psvs_oc_get_default_freq(device));
}

psvs_oc_mode_t psvs_oc_get_mode(psvs_oc_device_t device) {
    return g_profile.mode[device];
}

void psvs_oc_set_mode(psvs_oc_device_t device, psvs_oc_mode_t mode) {
    g_profile.mode[device] = mode;
    g_profile_has_changed = true;
    psvs_oc_set_target_freq(device);
}

int psvs_oc_get_default_freq(psvs_oc_device_t device) {
    int freq = g_oc_devopt[device].default_freq;

    if (g_pid == INVALID_PID)
        return freq;

    uintptr_t pstorage = 0;
    int ret = ksceKernelGetProcessLocalStorageAddrForPid(g_pid, *ScePower_0, (void **)&pstorage, 0);
    if (ret < 0 || pstorage == 0)
        return freq;

    switch (device) {
        case PSVS_OC_DEVICE_BUS: freq = *(uint32_t *)(pstorage + 40); break;
        default:
        case PSVS_OC_DEVICE_CPU: freq = *(uint32_t *)(pstorage + 44); break;
        case PSVS_OC_DEVICE_GPU_XBAR: freq = *(uint32_t *)(pstorage + 48); break;
        case PSVS_OC_DEVICE_GPU_ES4: freq = *(uint32_t *)(pstorage + 52); break;
    }

    // Validate
    bool valid = false;
    for (int i = 0; i < g_oc_devopt[device].freq_n; i++) {
        if (freq == g_oc_devopt[device].freq[i]) {
            valid = true;
            break;
        }
    }

    return valid ? freq : g_oc_devopt[device].default_freq;
}

void psvs_oc_reset_manual(psvs_oc_device_t device) {
    g_profile.manual_freq[device] = psvs_oc_get_freq(device);
    g_profile_has_changed = true;
}

void psvs_oc_change_manual(psvs_oc_device_t device, bool raise_freq) {
    int target_freq = g_profile.manual_freq[device]; // current manual freq

    for (int i = 0; i < g_oc_devopt[device].freq_n; i++) {
        int ii = raise_freq ? i : g_oc_devopt[device].freq_n - i - 1;
        if ((raise_freq && g_oc_devopt[device].freq[ii] > target_freq)
                || (!raise_freq && g_oc_devopt[device].freq[ii] < target_freq)) {
            target_freq = g_oc_devopt[device].freq[ii];
            break;
        }
    }

    g_profile.manual_freq[device] = target_freq;
    g_profile_has_changed = true;

    // Refresh manual clocks
    if (g_profile.mode[device] == PSVS_OC_MODE_MANUAL)
        psvs_oc_set_freq(device, g_profile.manual_freq[device]);
}

void psvs_oc_init() {
    g_profile_has_changed = true;
    for (int i = 0; i < PSVS_OC_DEVICE_MAX; i++) {
        g_profile.mode[i] = PSVS_OC_MODE_DEFAULT;
        psvs_oc_reset_manual(i);
    }
}
