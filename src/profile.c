#include <vitasdkkern.h>
#include <psp2/touch.h>
#include <psp2/motion.h>
#include <taihen.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "oc.h"
#include "bt.h"
#include "profile.h"

#define PSVS_PROFILES_DIR "ur0:data/PSVshell/profiles/"

static bool g_profile_exists = false;
static bool g_profile_exists_global = false;

psvs_app_session_t g_session = {
    .fps_limit = 0,
};

psvs_app_profile_t g_profile = {
    .ver = PSVS_VERSION_VER,
    .mode = {0},
    .manual_freq = {0},
    .swap_buttons = false,
    .disable_L3R3 = false,
    .bt_touch = PSVS_BT_TOUCH_DISABLED,
    .bt_motion = PSVS_BT_MOTION_DISABLED,
};
bool g_profile_has_changed = false;

psvs_app_profile_t * psvs_get_profile() {
    return &g_profile;
}

void psvs_set_profile(psvs_app_profile_t * profile) {
    g_profile = *profile;
    g_profile_has_changed = false;

    for (int i = 0; i < PSVS_OC_DEVICE_MAX; i++)
        psvs_oc_set_target_freq(i);
}

void psvs_profile_init() {
    ksceIoMkdir("ur0:data/", 0777);
    ksceIoMkdir("ur0:data/PSVshell/", 0777);
    ksceIoMkdir(PSVS_PROFILES_DIR, 0777);
}

bool psvs_profile_load() {
    g_profile_exists = false;
    g_profile_exists_global = false;

    SceUID fd = -1;
    char path[128];
    snprintf(path, 128, "%s%s", PSVS_PROFILES_DIR, g_titleid);

    // always check both so we can tell if both of them exist
    SceUID fd_title = ksceIoOpen(path, SCE_O_RDONLY, 0777);
    SceUID fd_global = ksceIoOpen(PSVS_PROFILES_DIR "global", SCE_O_RDONLY, 0777);
    if (fd_title < 0 && fd_global < 0)
        return false;

    // default to global profile
    if (fd_global >= 0) {
        fd = fd_global;
        g_profile_exists_global = true;
    }

    // if present, title profile has precedence
    if (fd_title >= 0) {
        fd = fd_title;
        g_profile_exists = true;
        if (fd_global >= 0) // global profile is not needed when both are present
            ksceIoClose(fd_global);
    }

    psvs_app_profile_t profile;
    int bytes = ksceIoRead(fd, &profile, sizeof(psvs_app_profile_t));
    ksceIoClose(fd);

    if (bytes != sizeof(psvs_app_profile_t))
        return false;

    if (strncmp(profile.ver, PSVS_VERSION_VER, 8))
        return false;

    psvs_set_profile(&profile);
    return true;
}

bool psvs_profile_save(bool global) {
    SceUID fd;

    if (!global) {
        char path[128];
        snprintf(path, 128, "%s%s", PSVS_PROFILES_DIR, g_titleid);
        fd = ksceIoOpen(path, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
    } else {
        fd = ksceIoOpen(PSVS_PROFILES_DIR "global", SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
    }

    if (fd < 0)
        return false;

    int bytes = ksceIoWrite(fd, &g_profile, sizeof(psvs_app_profile_t));
    ksceIoClose(fd);

    if (bytes != sizeof(psvs_app_profile_t))
        return false;

    // mark profile as present
    if (global) {
        g_profile_exists_global = true;
    } else {
        g_profile_exists = true;
        g_profile_has_changed = false;
    }

    return true;
}

bool psvs_profile_delete(bool global) {
    if (!global) {
        char path[128];
        snprintf(path, 128, "%s%s", PSVS_PROFILES_DIR, g_titleid);

        if (ksceIoRemove(path) < 0)
            return false;

        g_profile_exists = false;
        g_profile_has_changed = true;
    } else {
        if (ksceIoRemove(PSVS_PROFILES_DIR "global") < 0)
            return false;

        g_profile_exists_global = false;
    }

    return true;
}

bool psvs_profile_exists(bool global) {
    return global ? g_profile_exists_global : g_profile_exists;
}
