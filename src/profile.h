#ifndef _PROFILE_H_
#define _PROFILE_H_

typedef struct psvs_app_session_t
{
	// FPS Limit
	uint16_t fps_limit;
} psvs_app_session_t;

typedef struct psvs_app_profile_t
{
    // Version info
    char ver[8];
    // OC profile
    psvs_oc_mode_t mode[PSVS_OC_DEVICE_MAX];
    int manual_freq[PSVS_OC_DEVICE_MAX];
    // Control profile
    bool swap_buttons;
    bool disable_L3R3;
    // Bluetooth profile
    psvs_bt_touch_t bt_touch;
    psvs_bt_motion_t bt_motion;
} psvs_app_profile_t;

extern psvs_app_session_t g_session;
extern psvs_app_profile_t g_profile;
extern bool g_profile_has_changed;

// Profiles
psvs_app_profile_t * psvs_get_profile();
void psvs_set_profile(psvs_app_profile_t * profile);

// Saving and Loading profiles
void psvs_profile_init();
bool psvs_profile_load();
bool psvs_profile_save(bool global);
bool psvs_profile_delete(bool global);
bool psvs_profile_exists(bool global);

#endif
