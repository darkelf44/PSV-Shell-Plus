#include <vitasdkkern.h>
#include <taihen.h>
#include <stdbool.h>

#include "main.h"

SceUInt32 ksceKernelGetProcessTimeLowCore();
SceUInt32 ksceKernelSysrootGetCurrentAddressSpaceCB();

#define SECOND 1000000

#define PSVS_PERF_CPU_SAMPLERATE 500 * 1000
#define PSVS_PERF_PEAK_SAMPLES 10
static int g_perf_peak_usage_samples[PSVS_PERF_PEAK_SAMPLES] = {0};
static int g_perf_peak_usage_rotation = 0;
static int g_perf_usage[4] = {0, 0, 0, 0};

static SceUInt32 g_perf_tick_last = 0; // AVG CPU load
static SceUInt32 g_perf_tick_q_last = 0; // Peak CPU load
static SceUInt32 g_perf_tick_fps_last = 0; // Framerate
static SceUInt32 g_perf_tick_limit_last = 0; // FPS Limiter

static SceKernelSysClock g_perf_idle_clock_last[4] = {0, 0, 0, 0};
static SceKernelSysClock g_perf_idle_clock_q_last[4] = {0, 0, 0, 0};

static psvs_memory_t g_perf_memusage = {0};
static psvs_battery_t g_perf_batt = {0};

#define PSVS_PERF_FPS_SAMPLES 5
static uint32_t g_perf_frametime_sum = 0;
static uint8_t g_perf_frametime_n = 0;
static int g_perf_fps = 0;

int psvs_perf_get_fps() {
    return g_perf_fps;
}

int psvs_perf_get_load(int core) {
    return g_perf_usage[core];
}

int psvs_perf_get_peak() {
    int peak_total = 0;
    for (int i = 0; i < PSVS_PERF_PEAK_SAMPLES; i++)
        peak_total += g_perf_peak_usage_samples[i];
    return peak_total / PSVS_PERF_PEAK_SAMPLES;
}

psvs_battery_t *psvs_perf_get_batt() {
    return &g_perf_batt;
}

psvs_memory_t *psvs_perf_get_memusage() {
    return &g_perf_memusage;
}

void psvs_perf_calc_fps() {
    SceUInt32 tick_now = ksceKernelGetProcessTimeLowCore();
    uint32_t frametime = tick_now - g_perf_tick_fps_last;

    // Update FPS when enough samples are collected
    if (g_perf_frametime_n > PSVS_PERF_FPS_SAMPLES) {
        uint32_t frametime_avg = g_perf_frametime_sum / g_perf_frametime_n;
        g_perf_fps = (SECOND + (frametime_avg / 2) + 1) / frametime_avg;
        g_perf_frametime_n = 0;
        g_perf_frametime_sum = 0;
    }

    g_perf_frametime_n++;
    g_perf_frametime_sum += frametime;
    g_perf_tick_fps_last = tick_now;
}

void psvs_perf_poll_cpu() {
    SceUInt32 tick_now = ksceKernelGetProcessTimeLowCore();
    SceUInt32 tick_diff = tick_now - g_perf_tick_last;
    SceUInt32 tick_q_diff = tick_now - g_perf_tick_q_last;

    SceKernelSystemInfo info;
    info.size = sizeof(SceKernelSystemInfo);
    SceThreadmgrForDriver_0x7E280B69(&info);

    // Calculate AVG CPU usage
    if (tick_diff >= PSVS_PERF_CPU_SAMPLERATE) {
        for (int i = 0; i < 4; i++) {
            g_perf_usage[i] = (int)(100.0f - ((info.cpuInfo[i].idleClock - g_perf_idle_clock_last[i]) / (float)tick_diff) * 100);
            if (g_perf_usage[i] < 0)
                g_perf_usage[i] = 0;
            if (g_perf_usage[i] > 100)
                g_perf_usage[i] = 100;
            g_perf_idle_clock_last[i] = info.cpuInfo[i].idleClock;
        }

        g_perf_tick_last = tick_now;
    }

    // Calculate peak ST CPU usage
    int max_usage = 0;
    for (int i = 0; i < 4; i++) {
        int usage = (int)(100.0f - ((info.cpuInfo[i].idleClock - g_perf_idle_clock_q_last[i]) / (float)tick_q_diff) * 100);
        if (usage > max_usage)
            max_usage = usage;
        g_perf_idle_clock_q_last[i] = info.cpuInfo[i].idleClock;
    }
    if (max_usage < 0)
        max_usage = 0;
    if (max_usage > 100)
        max_usage = 100;
    g_perf_peak_usage_samples[g_perf_peak_usage_rotation] = max_usage;
    g_perf_peak_usage_rotation++;
    if (g_perf_peak_usage_rotation >= PSVS_PERF_PEAK_SAMPLES)
        g_perf_peak_usage_rotation = 0; // flip
    g_perf_tick_q_last = tick_now;
}

void psvs_perf_poll_memory() {
    // Takes caller's context into account
    SceSysmemAddressSpaceInfo info;
    uint32_t sysroot_cas = ksceKernelSysrootGetCurrentAddressSpaceCB();

    if (*(uint32_t *)(sysroot_cas + 328) > 0) {
        SceSysmemForKernel_0x3650963F(*(uint32_t *)(sysroot_cas + 328), &info);
        PSVS_CHECK_ASSIGN(g_perf_memusage, main_free, info.free);
        PSVS_CHECK_ASSIGN(g_perf_memusage, main_total, info.total);
    } else {
        PSVS_CHECK_ASSIGN(g_perf_memusage, main_free, 0);
        PSVS_CHECK_ASSIGN(g_perf_memusage, main_total, 0);
    }

    if (*(uint32_t *)(sysroot_cas + 332) > 0) {
        SceSysmemForKernel_0x3650963F(*(uint32_t *)(sysroot_cas + 332), &info);
        PSVS_CHECK_ASSIGN(g_perf_memusage, cdram_free, info.free);
        PSVS_CHECK_ASSIGN(g_perf_memusage, cdram_total, info.total);
    } else {
        PSVS_CHECK_ASSIGN(g_perf_memusage, cdram_free, 0);
        PSVS_CHECK_ASSIGN(g_perf_memusage, cdram_total, 0);
    }

    if (*(uint32_t *)(sysroot_cas + 316) > 0) {
        SceSysmemForKernel_0x3650963F(*(uint32_t *)(sysroot_cas + 316), &info);
        PSVS_CHECK_ASSIGN(g_perf_memusage, phycont_free, info.free);
        PSVS_CHECK_ASSIGN(g_perf_memusage, phycont_total, info.total);
    } else {
        PSVS_CHECK_ASSIGN(g_perf_memusage, phycont_free, 0);
        PSVS_CHECK_ASSIGN(g_perf_memusage, phycont_total, 0);
    }
}

void psvs_perf_poll_batt() {
    if (g_is_dolce)
        return;

    int val;

    // Grab batt percentage
    val = kscePowerGetBatteryLifePercent();
    if (val >= 0 && val <= 100) {
        PSVS_CHECK_ASSIGN(g_perf_batt, percent, val);
    }

    // Grab batt/case temp
    val = kscePowerGetBatteryTemp() / 100;
    if (val >= 0 && val <= 99) {
        PSVS_CHECK_ASSIGN(g_perf_batt, temp, val);
    }

    // Grab batt life time
    val = kscePowerGetBatteryLifeTime();
    if (val >= 0 && val < 100 * 60) {
        PSVS_CHECK_ASSIGN(g_perf_batt, lt_hours, val / 60);
        PSVS_CHECK_ASSIGN(g_perf_batt, lt_minutes, val - (g_perf_batt.lt_hours * 60));
    }

    // Grab charger
    bool bval = kscePowerIsBatteryCharging();
    PSVS_CHECK_ASSIGN(g_perf_batt, is_charging, bval);
}

void psvs_perf_limit_fps(int limit) {
	// Calculate the minimum frametime
	uint32_t frametime_limit = SECOND / (limit ? limit : 60);
	// Calculate the current frametime
	uint32_t frametime = (SceUInt32) ksceKernelGetProcessTimeLowCore() - g_perf_tick_limit_last;

	if (frametime < frametime_limit) {
		// Delay thread, if the limit is not reached
		ksceKernelDelayThread(frametime_limit - frametime);
	}
	// Start next frame after the delay
	g_perf_tick_limit_last = ksceKernelGetProcessTimeLowCore();
}
