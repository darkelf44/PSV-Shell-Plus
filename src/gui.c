#include <vitasdkkern.h>
#include <psp2/touch.h>
#include <psp2/motion.h>
#include <taihen.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "main.h"
#include "gui.h"
#include "gui_font_ter-u14b.h"
#include "gui_font_ter-u18b.h"
#include "gui_font_ter-u24b.h"
#include "perf.h"
#include "oc.h"
#include "bt.h"
#include "profile.h"

int vsnprintf(char *s, size_t n, const char *format, va_list arg);

static SceDisplayFrameBuf g_gui_fb = {
    .width  = 960,
    .height = 544,
    .pitch  = 960,
};
static int g_gui_fb_last_width = 960;
static float g_gui_fb_w_ratio = 1.0f;
static float g_gui_fb_h_ratio = 1.0f;

static rgba_t *g_gui_buffer;
static SceUID g_gui_buffer_uid = -1;

static const unsigned char *g_gui_font = FONT_TER_U24B;
static unsigned char g_gui_font_width  = 12;
static unsigned char g_gui_font_height = 24;

static float g_gui_font_scale = 1.0f;
static rgba_t g_gui_color_text = {.rgba = {255, 255, 255, 255}};
static rgba_t g_gui_color_bg   = {.rgba = {  0,   0,   0, 255}};

static uint32_t g_gui_input_buttons = 0;

static int g_gui_menu_control = PSVS_GUI_MENUCTRL_CPU;
static psvs_gui_mode_t g_gui_mode = PSVS_GUI_MODE_HIDDEN;
static bool g_gui_mode_changed = false;

#define PSVS_GUI_PAGE_COUNT 2
static const psvs_gui_page_t g_gui_page_list[PSVS_GUI_PAGE_COUNT] = {{
    .number = 0, // HOME page
    .draw_template = &psvs_gui_draw_home_template,
    .draw_content = &psvs_gui_draw_home_content,
    .init = &psvs_gui_init_home,
    .input = &psvs_gui_input_home,
},{
    .number = 1, // 1st page
    .draw_template = &psvs_gui_draw_page_1_template,
    .draw_content = &psvs_gui_draw_page_1_content,
    .init = &psvs_gui_init_page_1,
    .input = &psvs_gui_input_page_1,
}};
const psvs_gui_page_t * g_gui_page = &g_gui_page_list[0];

static bool g_gui_lazydraw_batt = false;
static bool g_gui_lazydraw_memusage = false;

#define GUI_CORNERS_XD_RADIUS 9
static const int8_t GUI_CORNERS_XD[GUI_CORNERS_XD_RADIUS] = {9, 7, 5, 4, 3, 2, 2, 1, 1};

#define GUI_SELECT_DOT_SIZE 4
#define GUI_SELECT_DOT_SPACE 12
static const int8_t GUI_SELECT_DOT[GUI_SELECT_DOT_SIZE][2] = {{-1, 1}, {-2, 2}, {-2, 2}, {-1, 1}};

static const rgba_t WHITE = {.rgba = {.r = 255, .g = 255, .b = 255, .a = 255}};
static const rgba_t BLACK = {.rgba = {.r = 0, .g = 0, .b = 0, .a = 255}};
static const rgba_t SELECTED = {.rgba = {.r = 0, .g = 200, .b = 255, .a = 255}};
static const rgba_t FPS_COLOR = {.rgba = {.r = 0, .g = 255, .b = 0, .a = 255}};
static const rgba_t FPS_LIMIT_COLOR = {.rgba = {.r = 255, .g = 0, .b = 0, .a = 255}};


psvs_gui_mode_t psvs_gui_get_mode() {
    return g_gui_mode;
}

static psvs_oc_device_t _psvs_gui_get_device_from_menuctrl(psvs_gui_menu_control_t ctrl) {
    switch (ctrl) {
        case PSVS_GUI_MENUCTRL_CPU:      return PSVS_OC_DEVICE_CPU;
        case PSVS_GUI_MENUCTRL_GPU_ES4:  return PSVS_OC_DEVICE_GPU_ES4;
        case PSVS_GUI_MENUCTRL_BUS:      return PSVS_OC_DEVICE_BUS;
        case PSVS_GUI_MENUCTRL_GPU_XBAR: return PSVS_OC_DEVICE_GPU_XBAR;
        default: return PSVS_OC_DEVICE_MAX;
    }
    return PSVS_OC_DEVICE_MAX;
}

static psvs_oc_device_t _psvs_gui_get_selected_device() {
    return _psvs_gui_get_device_from_menuctrl(g_gui_menu_control);
}

void psvs_gui_input_check(uint32_t buttons) {
    uint32_t buttons_new = buttons & ~g_gui_input_buttons;

    // Toggle menu
    if (buttons & SCE_CTRL_SELECT) {
        if (buttons_new & SCE_CTRL_UP && g_gui_mode < PSVS_GUI_MODE_MAX - 1) {
            g_gui_mode++; // Show
            g_gui_mode_changed = true;
        } else if (buttons_new & SCE_CTRL_DOWN && g_gui_mode > 0) {
            g_gui_mode--; // Hide
            g_gui_mode_changed = true;
        }
    }
    // In full menu
    else if (g_gui_mode == PSVS_GUI_MODE_FULL) {
        if ((buttons_new & SCE_CTRL_LTRIGGER) && (g_gui_page->number > 0)) {
            g_gui_page = &g_gui_page_list[g_gui_page->number - 1]; // Prev page
            g_gui_mode_changed = true;
            g_gui_page->init();
        } else if ((buttons_new & SCE_CTRL_RTRIGGER) && (g_gui_page->number < PSVS_GUI_PAGE_COUNT - 1)) {
            g_gui_page = &g_gui_page_list[g_gui_page->number + 1]; // Next page
            g_gui_mode_changed = true;
            g_gui_page->init();
        }
        g_gui_page->input(buttons, buttons_new);
    }

    g_gui_input_buttons = buttons;
}

void psvs_gui_init_home() {
    // Reset menu location
    g_gui_menu_control = PSVS_GUI_MENUCTRL_CPU;
}

void psvs_gui_input_home(uint32_t buttons_held, uint32_t buttons_down) {
    // Move U/D
    if (buttons_down & SCE_CTRL_DOWN && g_gui_menu_control < PSVS_GUI_MENUCTRL_MAX - 1) {
        g_gui_menu_control++;
    } else if (buttons_down & SCE_CTRL_UP && g_gui_menu_control > 0) {
        g_gui_menu_control--;
    }

    // Profile label
    if (g_gui_menu_control == PSVS_GUI_MENUCTRL_PROFILE) {
        if (buttons_down & SCE_CTRL_CROSS) {
            bool global = buttons_held & GUI_GLOBAL_PROFILE_BUTTON_MOD;
            if ((!global && g_profile_has_changed) || !psvs_profile_exists(global)) {
                psvs_profile_save(global);
            } else {
                psvs_profile_delete(global);
            }
        }
    }
    // Freq change
    else {
        psvs_oc_device_t device = _psvs_gui_get_selected_device();

        // In manual freq mode
        if (psvs_oc_get_mode(device) == PSVS_OC_MODE_MANUAL) {
            // Move L/R
            if (buttons_down & SCE_CTRL_RIGHT) {
                psvs_oc_change_manual(device, true);
            } else if (buttons_down & SCE_CTRL_LEFT) {
                psvs_oc_change_manual(device, false);
            }
            // Back to default
            else if (buttons_down & SCE_CTRL_CROSS) {
                psvs_oc_set_mode(device, PSVS_OC_MODE_DEFAULT);
            }
        }
        // In default freq mode
        else {
            if (buttons_down & SCE_CTRL_CROSS) {
                psvs_oc_reset_manual(device);
                psvs_oc_set_mode(device, PSVS_OC_MODE_MANUAL);
            }
        }
    }
}

void psvs_gui_init_page_1() {
    // Reset menu location
    g_gui_menu_control = PSVS_GUI_EXTRA_SWAP_BUTTONS;
}

void psvs_gui_input_page_1(uint32_t buttons_held, uint32_t buttons_down) {
    // Move U/D
    if (buttons_down & SCE_CTRL_DOWN && g_gui_menu_control < PSVS_GUI_EXTRA_MAX - 1) {
        g_gui_menu_control++;
    } else if (buttons_down & SCE_CTRL_UP && g_gui_menu_control > 0) {
        g_gui_menu_control--;
    }

    // X button
    if (buttons_down & SCE_CTRL_CROSS)
    {
        switch (g_gui_menu_control) {
            case PSVS_GUI_EXTRA_SWAP_BUTTONS:
                g_profile.swap_buttons = !g_profile.swap_buttons;
                g_profile_has_changed = true;
                break;

            case PSVS_GUI_EXTRA_DISABLE_L3R3:
                g_profile.disable_L3R3 = !g_profile.disable_L3R3;
                g_profile_has_changed = true;
                break;

            case PSVS_GUI_EXTRA_BT_TOUCH:
                g_profile.bt_touch = (g_profile.bt_touch + 1) % PSVS_BT_TOUCH_MAX;
                g_profile_has_changed = true;
                break;

            case PSVS_GUI_EXTRA_BT_MOTION:
                g_profile.bt_motion = (g_profile.bt_motion + 1) % PSVS_BT_MOTION_MAX;
                g_profile_has_changed = true;
                break;

            case PSVS_GUI_EXTRA_FPS_LIMIT:
                g_session.fps_limit = g_session.fps_limit ? (g_session.fps_limit == 30) ? 15 : 0 : 30;
                break;

            case PSVS_GUI_EXTRA_RESTART:
                kscePowerRequestColdReset();
                break;

            case PSVS_GUI_EXTRA_SHUTDOWN:
                kscePowerRequestStandby();
                break;
        }
    }

    // L/R buttons
    if (buttons_down & SCE_CTRL_LEFT)
    {
        switch (g_gui_menu_control) {
            case PSVS_GUI_EXTRA_SWAP_BUTTONS:
                if (g_profile.swap_buttons) {
                    g_profile.swap_buttons = false;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_DISABLE_L3R3:
                if (g_profile.disable_L3R3) {
                    g_profile.disable_L3R3 = false;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_BT_TOUCH:
                if (g_profile.bt_touch > 0) {
                    -- g_profile.bt_touch;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_BT_MOTION:
                if (g_profile.bt_motion > 0) {
                    -- g_profile.bt_motion;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_FPS_LIMIT:
                if (g_session.fps_limit > 0) {
                    -- g_session.fps_limit;
                }
                break;
        }
    }
    else if (buttons_down & SCE_CTRL_RIGHT) {
        switch (g_gui_menu_control) {
            case PSVS_GUI_EXTRA_SWAP_BUTTONS:
                if (!g_profile.swap_buttons) {
                    g_profile.swap_buttons = true;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_DISABLE_L3R3:
                if (!g_profile.disable_L3R3) {
                    g_profile.disable_L3R3 = true;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_BT_TOUCH:
                if (g_profile.bt_touch < PSVS_BT_TOUCH_MAX - 1) {
                    ++ g_profile.bt_touch;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_BT_MOTION:
                if (g_profile.bt_motion < PSVS_BT_MOTION_MAX - 1) {
                    ++ g_profile.bt_motion;
                    g_profile_has_changed = true;
                }
                break;

            case PSVS_GUI_EXTRA_FPS_LIMIT:
                if (g_session.fps_limit < 30) {
                    ++ g_session.fps_limit;
                }
                break;
        }
    }
}

void psvs_gui_set_framebuf(const SceDisplayFrameBuf *pParam) {
    memcpy(&g_gui_fb, pParam, sizeof(SceDisplayFrameBuf));

    g_gui_fb_w_ratio = pParam->width / 960.0f;
    g_gui_fb_h_ratio = pParam->height / 544.0f;

    // Set appropriate font
    if (pParam->width <= 640) {
        // 640x368 - Terminus 8x14 Bold
        g_gui_font = FONT_TER_U14B;
        g_gui_font_width = 8;
        g_gui_font_height = 14;
    } else if (pParam->width <= 720) {
        // 720x408 - Terminus 10x18 Bold
        g_gui_font = FONT_TER_U18B;
        g_gui_font_width = 9; // <- trim last col, better scaling
        g_gui_font_height = 18;
    } else {
        // 960x544 or more - Terminus 12x24 Bold
        g_gui_font = FONT_TER_U24B;
        g_gui_font_width = 12;
        g_gui_font_height = 24;
    }
}

bool psvs_gui_fb_res_changed() {
    bool changed = g_gui_fb_last_width != g_gui_fb.width;
    if (changed) {
        g_gui_lazydraw_batt = true;
        g_gui_lazydraw_memusage = true;
    }

    g_gui_fb_last_width = g_gui_fb.width;
    return changed;
}

bool psvs_gui_mode_changed() {
    bool changed = g_gui_mode_changed;
    if (changed) {
        g_gui_lazydraw_batt = true;
        g_gui_lazydraw_memusage = true;
    }

    g_gui_mode_changed = false;
    return changed;
}

void psvs_gui_set_back_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    g_gui_color_bg.rgba.r = r;
    g_gui_color_bg.rgba.g = g;
    g_gui_color_bg.rgba.b = b;
    g_gui_color_bg.rgba.a = a;
}

void psvs_gui_set_text_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    g_gui_color_text.rgba.r = r;
    g_gui_color_text.rgba.g = g;
    g_gui_color_text.rgba.b = b;
    g_gui_color_text.rgba.a = a;
}

void psvs_gui_set_text_color2(rgba_t color) {
    g_gui_color_text = color;
}

void psvs_gui_set_text_scale(float scale) {
    g_gui_font_scale = scale;
}

static void _psvs_gui_dd_prchar(const char character, int x, int y) {
    rgba_t color = (g_session.fps_limit) ? FPS_LIMIT_COLOR : FPS_COLOR;
    for (int yy = 0; yy < g_gui_font_height * g_gui_font_scale; yy++) {
        int yy_font = yy / g_gui_font_scale;

        uint32_t displacement = x + (y + yy) * g_gui_fb.pitch;
        if (displacement >= g_gui_fb.pitch * g_gui_fb.height)
            return; // out of bounds

        rgba_t *px = (rgba_t *)g_gui_fb.base + displacement;

        for (int xx = 0; xx < g_gui_font_width * g_gui_font_scale; xx++) {
            if (x + xx >= g_gui_fb.width)
                return; // out of bounds

            // Get px 0/1 from osd_font.h
            int xx_font = xx / g_gui_font_scale;
            uint32_t charPos = character * (g_gui_font_height * (((g_gui_font_width - 1) / 8) + 1));
            uint32_t charPosH = charPos + (yy_font * (((g_gui_font_width - 1) / 8) + 1));
            uint8_t charByte = g_gui_font[charPosH + (xx_font / 8)];

            if ((charByte >> (7 - (xx_font % 8))) & 1) {
                *(px + xx) = color;
            }
        }
    }
}

void psvs_gui_dd_fps() {
    char buf[4] = "";
    snprintf(buf, 4, "%d", psvs_perf_get_fps());
    size_t len = strlen(buf);

    uint32_t dacr;
    DACR_UNRESTRICT(dacr);

    for (int i = 0; i < len; i++) {
        _psvs_gui_dd_prchar(buf[i], 10 + i * g_gui_font_width * g_gui_font_scale, 10);
    }

    DACR_RESET(dacr);
}

void psvs_gui_dd_dot(int cx, int cy, int r, rgba_t color) {
    uint32_t dacr;
    DACR_UNRESTRICT(dacr);

    int rr = (r + 1) * (r + 1);
    for (int dy = -r; dy <= r; ++ dy) {
        int y = cy + dy;
        if (y >= 0 && y < g_gui_fb.height) {
            size_t offset = y * g_gui_fb.pitch;
            for (int dx = -r; dx <= r; ++ dx) {
                int x = cx + dx;
                if (x >= 0 && x < g_gui_fb.width && (dx * dx + dy * dy < rr)) {
                    *((rgba_t *) g_gui_fb.base + offset + x) = color;
                }
            }
        }
    }

    DACR_RESET(dacr);
}

void psvs_gui_clear() {
    for (int i = 0; i < GUI_WIDTH * GUI_HEIGHT; i++)
        g_gui_buffer[i] = g_gui_color_bg;
}

static void _psvs_gui_prchar(const char character, int x, int y) {
    // Draw spaces faster
    if (character == ' ') {
        for (int yy = 0; yy < g_gui_font_height * g_gui_font_scale; yy++) {
            rgba_t *buf = &g_gui_buffer[((y + yy) * GUI_WIDTH) + x];
            for (int xx = 0; xx < g_gui_font_width * g_gui_font_scale; xx++) {
                buf[xx] = g_gui_color_bg;
            }
        }

        return;
    }

    // TODO: rewrite
    for (int yy = 0; yy < g_gui_font_height * g_gui_font_scale; yy++) {
        int yy_font = yy / g_gui_font_scale;

        uint32_t displacement = x + (y + yy) * GUI_WIDTH;
        if (displacement >= GUI_WIDTH * GUI_HEIGHT)
            return; // out of bounds

        rgba_t *px = (rgba_t *)g_gui_buffer + displacement;

        for (int xx = 0; xx < g_gui_font_width * g_gui_font_scale; xx++) {
            if (x + xx >= GUI_WIDTH)
                return; // out of bounds

            // Get px 0/1 from osd_font.h
            int xx_font = xx / g_gui_font_scale;
            uint32_t charPos = character * (g_gui_font_height * (((g_gui_font_width - 1) / 8) + 1));
            uint32_t charPosH = charPos + (yy_font * (((g_gui_font_width - 1) / 8) + 1));
            uint8_t charByte = g_gui_font[charPosH + (xx_font / 8)];

            // Fix missing lines
            if (g_gui_font_scale < 0.75 && yy > 0)
                charByte |= g_gui_font[charPosH + (((g_gui_font_width - 1) / 8) + 1) + (xx_font / 8)];

            rgba_t color = ((charByte >> (7 - (xx_font % 8))) & 1) ? g_gui_color_text : g_gui_color_bg;
            *(px + xx) = color;
        }
    }
}

void psvs_gui_print(int x, int y, const char *str) {
    x = GUI_RESCALE_X(x);
    y = GUI_RESCALE_Y(y);

    size_t len = strlen(str);
    for (size_t i = 0; i < len; i++) {
        _psvs_gui_prchar(str[i], x + i * g_gui_font_width * g_gui_font_scale, y);
    }
}

void psvs_gui_printf(int x, int y, const char *format, ...) {
    char buffer[256] = "";
    va_list va;

    va_start(va, format);
    vsnprintf(buffer, 256, format, va);
    va_end(va);

    psvs_gui_print(x, y, buffer);
}

const char *psvs_gui_units_from_size(int bytes) {
    if (bytes >= 1024 * 1024)
        return "MB";
    else if (bytes >= 1024)
        return "kB";
    else
        return "B";
}

int psvs_gui_value_from_size(int bytes) {
    if (bytes >= 1024 * 1024)
        return bytes / 1024 / 1024;
    else if (bytes >= 1024)
        return bytes / 1024;
    else
        return bytes;
}

rgba_t psvs_gui_scale_color(int value, int min, int max) {
    if (value < min)
        value = min;
    if (value > max)
        value = max;

    int v = ((float)(value - min) / (max - min)) * 255; // 0-255

    rgba_t color;
    color.rgba.r = (v <= 127) ? v * 2 : 255;
    color.rgba.g = (v > 127) ? 255 - ((v - 128) * 2) : 255;
    color.rgba.b = 0;
    color.rgba.a = 255;

    return color;
}

static void _psvs_gui_draw_select_dots(int x, int y, int selected, int count)
{
    int d = GUI_RESCALE_X(GUI_SELECT_DOT_SPACE);
    x = GUI_RESCALE_X(x) - ((count - 1) * d) / 2;
    y = GUI_RESCALE_Y(y) - GUI_SELECT_DOT_SIZE / 2;

    rgba_t *px = (rgba_t *)g_gui_buffer + (y * GUI_WIDTH) + x;
    int i, xx, yy;

    for (yy = 0; yy < GUI_SELECT_DOT_SIZE; yy++) {
        for (i = 0; i < count; i++) {
            rgba_t color = (i == selected) ? SELECTED : WHITE;
            for (xx = GUI_SELECT_DOT[yy][0]; xx < GUI_SELECT_DOT[yy][1]; xx++)
                *(px + i * d + xx) = color;
        }
        px += GUI_WIDTH;
    }
}

static void _psvs_gui_draw_battery_template(int x, int y) {
    x = GUI_RESCALE_X(x);
    y = GUI_RESCALE_Y(y);
    int w = GUI_RESCALE_X(GUI_BATT_SIZE_W);
    int h = GUI_RESCALE_Y(GUI_BATT_SIZE_H);

    rgba_t *px = (rgba_t *)g_gui_buffer + (y * GUI_WIDTH) + x;
    int xx, yy;

    for (xx = 0; xx < w; xx++) {
        // top
        *(px + xx)
            = *(px + GUI_WIDTH + xx) = WHITE;
        // bottom
        *(px + (h * GUI_WIDTH) + xx)
            = *(px + ((h - 1) * GUI_WIDTH) + xx) = WHITE;
    }

    for (yy = 0; yy < h; yy++) {
        // left
        *(px + (yy * GUI_WIDTH))
            = *(px + (yy * GUI_WIDTH) + 1) = WHITE;
        // right
        if (yy < h / 3 || yy > h - (h / 3)) {
            *(px + (yy * GUI_WIDTH) + (w - 1))
                = *(px + (yy * GUI_WIDTH) + (w - 2)) = WHITE;
        } else {
            *(px + (yy * GUI_WIDTH) + (w - 1) + (h / 5))
                = *(px + (yy * GUI_WIDTH) + (w - 2) + (h / 5)) = WHITE;
        }
    }

    // dzindzik
    for (xx = 0; xx < (h / 5) + 2; xx++) {
        // top
        *(px + (GUI_WIDTH * (h / 3 - 1)) + (w - 2) + xx)
            = *(px + (GUI_WIDTH * (h / 3)) + (w - 2) + xx) = WHITE;
        // bottom
        *(px + (GUI_WIDTH * (h - (h / 3))) + (w - 2) + xx)
            = *(px + (GUI_WIDTH * (h - (h / 3) + 1)) + (w - 2) + xx) = WHITE;
    }
}

static void _psvs_gui_draw_battery(int x, int y, int state, bool is_charging, rgba_t color) {
    x = GUI_RESCALE_X(x);
    y = GUI_RESCALE_Y(y);
    int w = GUI_RESCALE_X(GUI_BATT_SIZE_W);
    int h = GUI_RESCALE_Y(GUI_BATT_SIZE_H);

    rgba_t *px = (rgba_t *)g_gui_buffer + (y * GUI_WIDTH) + x;
    int state_x = state > 95 ? w : (state * (w - 4) / 95) + 1;
    int xx, yy;

    for (xx = 2; xx < (w - 2); xx++) {
        for (yy = 2; yy < (h - 1); yy++) {
            if (xx <= state_x) {
                *(px + (GUI_WIDTH * yy) + xx) = color;
            } else {
                *(px + (GUI_WIDTH * yy) + xx) = BLACK;
            }
        }
    }

    // dzindzik
    for (xx = 0; xx < (h / 5); xx++) {
        for (yy = 2; yy < (h / 3) + 2; yy++) {
            if (state > 95) {
                *(px + (GUI_WIDTH * ((h / 3 - 1) + yy)) + w + xx - 2) = color;
            } else {
                *(px + (GUI_WIDTH * ((h / 3 - 1) + yy)) + w + xx - 2) = BLACK;
            }
        }
    }

    if (is_charging) {
        for (xx = 0; xx < (h / 3); xx++)
            *(px + (GUI_WIDTH * (h / 2)) + (w / 5) + xx) = WHITE;
        for (yy = 0; yy < (h / 3); yy++)
            *(px + (GUI_WIDTH * ((h / 3) + yy + 1)) + (w / 5) + (h / 6)) = WHITE;
    }
}

void psvs_gui_draw_osd_template() {
    psvs_gui_set_back_color(0, 0, 0, 255);
    psvs_gui_set_text_color(255, 255, 255, 255);
    psvs_gui_clear();

    // CPU
    psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(8, 0), "CPU:");
    psvs_gui_printf(GUI_ANCHOR_RX(10, 16), GUI_ANCHOR_TY(8, 0), "%%    %%    %%    %%");
    psvs_gui_printf(GUI_ANCHOR_LX(10, 10), GUI_ANCHOR_TY(10, 1), "%%");

    // FPS
    psvs_gui_printf(GUI_ANCHOR_LX(10, 3), GUI_ANCHOR_TY(10, 1), "FPS");

    // Battery
    psvs_gui_printf(GUI_ANCHOR_RX(20 + GUI_BATT_SIZE_W, 1), GUI_ANCHOR_TY(10, 1), "%%");
    _psvs_gui_draw_battery_template(GUI_ANCHOR_RX(14 + GUI_BATT_SIZE_W, 0), GUI_ANCHOR_TY(13, 1));
}

void psvs_gui_draw_osd_content() {
    psvs_gui_draw_osd_cpu();
    psvs_gui_draw_osd_fps();
    psvs_gui_draw_osd_batt();
}

void psvs_gui_draw_osd_cpu() {
    int val;

    // Draw AVG load
    for (int i = 0; i < 4; i++) {
        val = psvs_perf_get_load(i);
        psvs_gui_set_text_color2(psvs_gui_scale_color(val, 0, 100));
        psvs_gui_printf(GUI_ANCHOR_RX(10, 19 - i * 5), GUI_ANCHOR_TY(8, 0), "%3d", val);
    }

    // Draw peak load
    val = psvs_perf_get_peak();
    psvs_gui_set_text_color2(psvs_gui_scale_color(val, 0, 100));
    psvs_gui_printf(GUI_ANCHOR_LX(10, 7), GUI_ANCHOR_TY(10, 1), "%3d", val);

    psvs_gui_set_text_color(255, 255, 255, 255);
}

void psvs_gui_draw_osd_batt() {
    psvs_battery_t *batt = psvs_perf_get_batt();
    if (!batt->_has_changed && !g_gui_lazydraw_batt)
        return;

    batt->_has_changed = false;
    g_gui_lazydraw_batt = false;

    // Draw battery percentage
    rgba_t color = psvs_gui_scale_color(60 - batt->percent, 0, 100);
    psvs_gui_set_text_color2(color);
    psvs_gui_printf(GUI_ANCHOR_RX(20 + GUI_BATT_SIZE_W, 4), GUI_ANCHOR_TY(10, 1), "%3d", batt->percent);

    // Draw battery indicator
    color.rgba.r = (int)(color.rgba.r * 0.75f);
    color.rgba.g = (int)(color.rgba.g * 0.75f);
    color.rgba.b = (int)(color.rgba.b * 0.75f);
    _psvs_gui_draw_battery(GUI_ANCHOR_RX(14 + GUI_BATT_SIZE_W, 0), GUI_ANCHOR_TY(13, 1), batt->percent, batt->is_charging, color);

    psvs_gui_set_text_color(255, 255, 255, 255);
}

void psvs_gui_draw_osd_fps() {
    int fps = psvs_perf_get_fps();

    psvs_gui_set_text_color2(psvs_gui_scale_color(30 - fps, 0, 30));
    if (fps > 99)
        psvs_gui_printf(GUI_ANCHOR_LX(10, 0), GUI_ANCHOR_TY(10, 1), "%3d", fps);
    else
        psvs_gui_printf(GUI_ANCHOR_LX(10, 0), GUI_ANCHOR_TY(10, 1), "%2d ", fps);

    psvs_gui_set_text_color(255, 255, 255, 255);
}

void psvs_gui_draw_home_template() {
    psvs_gui_set_back_color(0, 0, 0, 255);
    psvs_gui_set_text_color(255, 255, 255, 255);
    psvs_gui_clear();

    // Header
    psvs_gui_set_text_scale(0.5f);
    psvs_gui_printf(GUI_ANCHOR_CX2(18, 0.5f),     GUI_ANCHOR_TY(8, 0), PSVS_VERSION_STRING);
    psvs_gui_printf(GUI_ANCHOR_RX2(10, 10, 0.5f), GUI_ANCHOR_TY(8, 0), "by Electry");
    psvs_gui_set_text_scale(1.0f);

    // Page
    _psvs_gui_draw_select_dots(GUI_ANCHOR_CX(0), GUI_ANCHOR_TY(28, 0), 0, PSVS_GUI_PAGE_COUNT);

    // Batt
    if (!g_is_dolce) {
        psvs_gui_printf(GUI_ANCHOR_RX(10, 9),  GUI_ANCHOR_TY(32, 0), "C");
        psvs_gui_printf(GUI_ANCHOR_RX(14 + 6 + GUI_BATT_SIZE_W, 1), GUI_ANCHOR_TY(32, 0), "%%");
        _psvs_gui_draw_battery_template(GUI_ANCHOR_RX(14 + GUI_BATT_SIZE_W, 0), GUI_ANCHOR_TY(35, 0));
    }

    // CPU
    psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(44, 1), "CPU:");
    psvs_gui_printf(GUI_ANCHOR_RX(10, 16), GUI_ANCHOR_TY(44, 1), "%%    %%    %%    %%");
    psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(44, 2), "Peak:");
    psvs_gui_printf(GUI_ANCHOR_RX(10, 1),  GUI_ANCHOR_TY(44, 2), "%%");

    // Memory
    psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(56, 3), "MEM:");
    psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(56, 4), "VMEM:");
    psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(56, 5), "PHY:");

    // Menu
    psvs_gui_printf(GUI_ANCHOR_CX(15),     GUI_ANCHOR_BY(10, 5), "CPU [         ]");
    psvs_gui_printf(GUI_ANCHOR_CX(15),     GUI_ANCHOR_BY(10, 4), "ES4 [         ]");
    psvs_gui_printf(GUI_ANCHOR_CX(15),     GUI_ANCHOR_BY(10, 3), "BUS [         ]");
    psvs_gui_printf(GUI_ANCHOR_CX(15),     GUI_ANCHOR_BY(10, 2), "XBR [         ]");
}

void psvs_gui_draw_home_content() {
    psvs_gui_draw_home_header();
    psvs_gui_draw_home_batt_section();
    psvs_gui_draw_home_cpu_section();
    psvs_gui_draw_home_memory_section();
    psvs_gui_draw_home_menu();
}

void psvs_gui_draw_home_header() {
    // Draw TITLEID
    psvs_gui_set_text_scale(0.5f);
    psvs_gui_printf(GUI_ANCHOR_LX(10, 0), GUI_ANCHOR_TY(8, 0), "%-9s", g_titleid);
    psvs_gui_set_text_scale(1.0f);
}

void psvs_gui_draw_home_batt_section() {
    // PSTV
    if (g_is_dolce) {
        psvs_gui_printf(GUI_ANCHOR_LX(10, 0), GUI_ANCHOR_TY(32, 0), "A/C");
        return;
    }

    psvs_battery_t *batt = psvs_perf_get_batt();
    if (!batt->_has_changed && !g_gui_lazydraw_batt)
        return;

    batt->_has_changed = false;
    g_gui_lazydraw_batt = false;

    // Draw temp
    psvs_gui_set_text_color2(psvs_gui_scale_color(batt->temp, 30, 60));
    psvs_gui_printf(GUI_ANCHOR_RX(10, 11), GUI_ANCHOR_TY(32, 0), "%2d", batt->temp);

    psvs_gui_set_text_color(255, 255, 255, 255);

    // Draw battery lifetime
    if (batt->is_charging) {
        psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(32, 0), "Charging   ");
    } else if (batt->lt_hours == 0 && batt->lt_minutes <= 1) {
        psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(32, 0), "Calibrating");
    } else {
        psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(32, 0), "  %s h   m   ", batt->lt_hours >= 10 ? " " : "");
        psvs_gui_printf(GUI_ANCHOR_LX(10, 0),  GUI_ANCHOR_TY(42, 0), "~");

        psvs_gui_set_text_color2(psvs_gui_scale_color((2 * 60) - ((batt->lt_hours * 60) + batt->lt_minutes), 0, 2 * 60));
        psvs_gui_printf(GUI_ANCHOR_LX(10, 2),  GUI_ANCHOR_TY(32, 0), "%d", batt->lt_hours);
        psvs_gui_printf(GUI_ANCHOR_LX(10, batt->lt_hours >= 10 ? 6 : 5),  GUI_ANCHOR_TY(32, 0), "%02d", batt->lt_minutes);
    }

    // Draw battery percentage
    rgba_t color = psvs_gui_scale_color(60 - batt->percent, 0, 100);
    psvs_gui_set_text_color2(color);
    psvs_gui_printf(GUI_ANCHOR_RX(20 + GUI_BATT_SIZE_W, 4), GUI_ANCHOR_TY(32, 0), "%3d", batt->percent);

    // Draw battery indicator
    color.rgba.r = (int)(color.rgba.r * 0.75f);
    color.rgba.g = (int)(color.rgba.g * 0.75f);
    color.rgba.b = (int)(color.rgba.b * 0.75f);
    _psvs_gui_draw_battery(GUI_ANCHOR_RX(14 + GUI_BATT_SIZE_W, 0), GUI_ANCHOR_TY(35, 0), batt->percent, batt->is_charging, color);

}

void psvs_gui_draw_home_cpu_section() {
    int load;

    // Draw AVG load
    for (int i = 0; i < 4; i++) {
        load = psvs_perf_get_load(i);
        psvs_gui_set_text_color2(psvs_gui_scale_color(load, 0, 100));
        psvs_gui_printf(GUI_ANCHOR_RX(10, 19 - (i * 5)), GUI_ANCHOR_TY(44, 1), "%3d", load);
    }

    // Draw peak load
    load = psvs_perf_get_peak();
    psvs_gui_set_text_color2(psvs_gui_scale_color(load, 0, 100));
    psvs_gui_printf(GUI_ANCHOR_RX(10, 4), GUI_ANCHOR_TY(44, 2), "%3d", load);

    psvs_gui_set_text_color(255, 255, 255, 255);
}

static void _psvs_gui_draw_memory_usage(int line, int total, int free, int limit) {
    if (total <= 0 && free <= 0) {
        psvs_gui_printf(GUI_ANCHOR_RX(10, 16), GUI_ANCHOR_TY(56, line), "          unused");
    }
    else if (total <= limit && total > free) {
        psvs_gui_printf(GUI_ANCHOR_RX(10, 11), GUI_ANCHOR_TY(56, line), "%-2s /     %-2s",
                        psvs_gui_units_from_size(total - free),
                        psvs_gui_units_from_size(total));

        psvs_gui_set_text_color2(psvs_gui_scale_color(total - free, total - (total / 10), total + (total / 10)));
        psvs_gui_printf(GUI_ANCHOR_RX(10, 16), GUI_ANCHOR_TY(56, line), " %3d",
                        psvs_gui_value_from_size(total - free));
        psvs_gui_printf(GUI_ANCHOR_RX(10, 7), GUI_ANCHOR_TY(56, line), " %3d",
                        psvs_gui_value_from_size(total));

        psvs_gui_set_text_color(255, 255, 255, 255);
    }
    else {
        psvs_gui_printf(GUI_ANCHOR_RX(10, 15 + (free >= 1024 ? 1 : 0)),
                        GUI_ANCHOR_TY(56, line),
                        "         %s free",
                        psvs_gui_units_from_size(free));
        psvs_gui_set_text_color(0, 255, 0, 255);
        psvs_gui_printf(GUI_ANCHOR_RX(10, 11 + (free >= 1024 ? 1 : 0)),
                        GUI_ANCHOR_TY(56, line),
                        " %3d",
                        psvs_gui_value_from_size(free));
        psvs_gui_set_text_color(255, 255, 255, 255);
    }
}

void psvs_gui_draw_home_memory_section() {
    psvs_memory_t *mem = psvs_perf_get_memusage();
    if (!mem->_has_changed && !g_gui_lazydraw_memusage)
        return;

    mem->_has_changed = false;
    g_gui_lazydraw_memusage = false;

    _psvs_gui_draw_memory_usage(3, mem->main_total, mem->main_free, 512 * 1024 * 1024);
    _psvs_gui_draw_memory_usage(4, mem->cdram_total, mem->cdram_free, 128 * 1024 * 1024);
    _psvs_gui_draw_memory_usage(5, mem->phycont_total, mem->phycont_free, 26 * 1024 * 1024);
}

static void _psvs_gui_draw_menu_item(int lines, int clock, psvs_gui_menu_control_t menuctrl) {
    if (g_gui_menu_control == menuctrl) {
        psvs_gui_set_text_color(0, 200, 255, 255);
        psvs_gui_printf(GUI_ANCHOR_CX(19),                        GUI_ANCHOR_BY(10, lines), ">");
        psvs_gui_printf(GUI_ANCHOR_CX(19) + GUI_ANCHOR_LX(0, 18), GUI_ANCHOR_BY(10, lines), "<");
        psvs_gui_set_text_color(255, 255, 255, 255);
    } else {
        psvs_gui_printf(GUI_ANCHOR_CX(19),                        GUI_ANCHOR_BY(10, lines), " ");
        psvs_gui_printf(GUI_ANCHOR_CX(19) + GUI_ANCHOR_LX(0, 18), GUI_ANCHOR_BY(10, lines), " ");
    }

    // Highlight freq if in manual mode
    if (psvs_oc_get_mode(_psvs_gui_get_device_from_menuctrl(menuctrl)) == PSVS_OC_MODE_MANUAL) {
        psvs_gui_set_text_color(0, 200, 255, 255);
    }
    psvs_gui_printf(GUI_ANCHOR_CX(15) + GUI_ANCHOR_LX(0, 6),  GUI_ANCHOR_BY(10, lines), "%3d MHz", clock);
    psvs_gui_set_text_color(255, 255, 255, 255);
}

void psvs_gui_draw_home_menu() {
    _psvs_gui_draw_menu_item(5, psvs_oc_get_freq(PSVS_OC_DEVICE_CPU), PSVS_GUI_MENUCTRL_CPU);
    _psvs_gui_draw_menu_item(4, psvs_oc_get_freq(PSVS_OC_DEVICE_GPU_ES4), PSVS_GUI_MENUCTRL_GPU_ES4);
    _psvs_gui_draw_menu_item(3, psvs_oc_get_freq(PSVS_OC_DEVICE_BUS), PSVS_GUI_MENUCTRL_BUS);
    _psvs_gui_draw_menu_item(2, psvs_oc_get_freq(PSVS_OC_DEVICE_GPU_XBAR), PSVS_GUI_MENUCTRL_GPU_XBAR);

    // Draw profile label separately
    bool show_global = g_gui_input_buttons & GUI_GLOBAL_PROFILE_BUTTON_MOD;
    bool save = (!show_global && g_profile_has_changed) || !psvs_profile_exists(show_global);

    if (save) {
        if (show_global)
            psvs_gui_printf(GUI_ANCHOR_CX(18), GUI_ANCHOR_BY(10, 1), "   save default   ");
        else
            psvs_gui_printf(GUI_ANCHOR_CX(18), GUI_ANCHOR_BY(10, 1), "   save profile   ");
    } else {
        if (show_global)
            psvs_gui_printf(GUI_ANCHOR_CX(18), GUI_ANCHOR_BY(10, 1), "  delete default  ");
        else
            psvs_gui_printf(GUI_ANCHOR_CX(18), GUI_ANCHOR_BY(10, 1), "  delete profile  ");
    }
    if (g_gui_menu_control == PSVS_GUI_MENUCTRL_PROFILE) {
        psvs_gui_set_text_color(0, 200, 255, 255);
        psvs_gui_printf(GUI_ANCHOR_CX(save ? 16 : 18), GUI_ANCHOR_BY(10, 1), ">");
        psvs_gui_printf(GUI_ANCHOR_CX(save ? 16 : 18) + GUI_ANCHOR_LX(0, save ? 15 : 17),
                        GUI_ANCHOR_BY(10, 1), "<");
        psvs_gui_set_text_color(255, 255, 255, 255);
    } else {
        psvs_gui_printf(GUI_ANCHOR_CX(save ? 16 : 18), GUI_ANCHOR_BY(10, 1), " ");
        psvs_gui_printf(GUI_ANCHOR_CX(save ? 16 : 18) + GUI_ANCHOR_LX(0, save ? 15 : 17),
                        GUI_ANCHOR_BY(10, 1), " ");
    }
}

void psvs_gui_draw_page_1_template() {
    psvs_gui_set_back_color(0, 0, 0, 255);
    psvs_gui_set_text_color(255, 255, 255, 255);
    psvs_gui_clear();

    // Header
    psvs_gui_set_text_scale(0.5f);
    psvs_gui_printf(GUI_ANCHOR_CX2(18, 0.5f),     GUI_ANCHOR_TY(8, 0), PSVS_VERSION_STRING);
    psvs_gui_printf(GUI_ANCHOR_RX2(10, 10, 0.5f), GUI_ANCHOR_TY(8, 0), "by Electry");
    psvs_gui_set_text_scale(1.0f);

    // Page
    _psvs_gui_draw_select_dots(GUI_ANCHOR_CX(0), GUI_ANCHOR_TY(28, 0), 1, PSVS_GUI_PAGE_COUNT);

    // Controls
    psvs_gui_printf(GUI_ANCHOR_LX(10, 2),  GUI_ANCHOR_TY(32, 0), "Swap X/O:");
    psvs_gui_printf(GUI_ANCHOR_LX(10, 2),  GUI_ANCHOR_TY(32, 1), "Disable L3/R3:");

    // Bluetooth
    psvs_gui_printf(GUI_ANCHOR_LX(10, 2),  GUI_ANCHOR_TY(44, 2), "Bt Touch:");
    psvs_gui_printf(GUI_ANCHOR_LX(10, 2),  GUI_ANCHOR_TY(44, 3), "Bt Motion:");

    // Utilities
    psvs_gui_printf(GUI_ANCHOR_LX(10, 2),  GUI_ANCHOR_TY(56, 4), "FPS Limit:");
}

static void _psvs_gui_draw_page_1_item(int offset, int line, int id, int size, const char * format, ...) {
    // Variables
    char buffer[256] = "";
    va_list va;

    // Format string
    va_start(va, format);
    vsnprintf(buffer, 256, format, va);
    va_end(va);

    if (g_gui_menu_control == id) {
        // Draw selected
        psvs_gui_set_text_color(0, 200, 255, 255);
        psvs_gui_print(GUI_ANCHOR_LX(10, 0), GUI_ANCHOR_TY(offset, line), ">");
        psvs_gui_print(GUI_ANCHOR_RX(10, size), GUI_ANCHOR_TY(offset, line), buffer);
        psvs_gui_set_text_color(255, 255, 255, 255);
    } else {
        // Draw other
        psvs_gui_print(GUI_ANCHOR_LX(10, 0), GUI_ANCHOR_TY(offset, line), " ");
        psvs_gui_print(GUI_ANCHOR_RX(10, size), GUI_ANCHOR_TY(offset, line), buffer);
    }
}

static const char * psvs_gui_button_title[2] = { "off", " on"};
static const char * psvs_gui_bt_touch_title[PSVS_BT_TOUCH_MAX] = { "  off", "front", "  F/B", "F/B/X"};
static const char * psvs_gui_bt_motion_title[PSVS_BT_MOTION_MAX] = { "     off", "  normal", "vertical"};

void psvs_gui_draw_page_1_content() {
    // Header
    psvs_gui_draw_home_header();

    // Controls
    _psvs_gui_draw_page_1_item(32, 0, PSVS_GUI_EXTRA_SWAP_BUTTONS, 3, psvs_gui_button_title[g_profile.swap_buttons]);
    _psvs_gui_draw_page_1_item(32, 1, PSVS_GUI_EXTRA_DISABLE_L3R3, 3, psvs_gui_button_title[g_profile.disable_L3R3]);

    // Bluetooth
    g_profile.bt_touch = (g_profile.bt_touch < PSVS_BT_TOUCH_MAX) ? g_profile.bt_touch : 0;
    g_profile.bt_motion = (g_profile.bt_motion < PSVS_BT_MOTION_MAX) ? g_profile.bt_motion : 0;
    _psvs_gui_draw_page_1_item(44, 2, PSVS_GUI_EXTRA_BT_TOUCH, 5, psvs_gui_bt_touch_title[g_profile.bt_touch]);
    _psvs_gui_draw_page_1_item(44, 3, PSVS_GUI_EXTRA_BT_MOTION, 8, psvs_gui_bt_motion_title[g_profile.bt_motion]);

    // Utilities
    g_session.fps_limit = (g_session.fps_limit <= 30) ? g_session.fps_limit : 0;
    _psvs_gui_draw_page_1_item(56, 4, PSVS_GUI_EXTRA_FPS_LIMIT, 3, (g_session.fps_limit == 0) ? "off" : "%3d", g_session.fps_limit);

    // System
    if (g_gui_menu_control == PSVS_GUI_EXTRA_RESTART) {
        psvs_gui_set_text_color(0, 200, 255, 255);
        psvs_gui_print(GUI_ANCHOR_CX(16), GUI_ANCHOR_BY(10, 2), "> Restart Vita <");
        psvs_gui_set_text_color(255, 255, 255, 255);
    } else {
        psvs_gui_printf(GUI_ANCHOR_CX(16), GUI_ANCHOR_BY(10, 2), "  Restart Vita  ");
    }
    if (g_gui_menu_control == PSVS_GUI_EXTRA_SHUTDOWN) {
        psvs_gui_set_text_color(0, 200, 255, 255);
        psvs_gui_print(GUI_ANCHOR_CX(17), GUI_ANCHOR_BY(10, 1), "> Shutdown Vita <");
        psvs_gui_set_text_color(255, 255, 255, 255);
    } else {
        psvs_gui_print(GUI_ANCHOR_CX(17), GUI_ANCHOR_BY(10, 1), "  Shutdown Vita  ");
    }
}

int psvs_gui_init() {
    int size = (GUI_WIDTH * GUI_HEIGHT * sizeof(rgba_t) + 0xfff) & ~0xfff;
    g_gui_buffer_uid = ksceKernelAllocMemBlock("psvs_gui", SCE_KERNEL_MEMBLOCK_TYPE_KERNEL_RW, size, NULL);
    if (g_gui_buffer_uid < 0) {
        return g_gui_buffer_uid;
    }

    ksceKernelGetMemBlockBase(g_gui_buffer_uid, (void **)&g_gui_buffer);

    return 0;
}

void psvs_gui_done() {
    if (g_gui_buffer_uid >= 0)
        ksceKernelFreeMemBlock(g_gui_buffer_uid);
}

void psvs_gui_cpy() {
    int height = (g_gui_mode == PSVS_GUI_MODE_OSD) ? GUI_OSD_HEIGHT : GUI_HEIGHT;

    int w = GUI_RESCALE_X(GUI_WIDTH);
    int h = GUI_RESCALE_Y(height);
    int x = (g_gui_mode == PSVS_GUI_MODE_OSD) ? 10 : (g_gui_fb.width / 2) - (w / 2);
    int y = (g_gui_mode == PSVS_GUI_MODE_OSD) ? 10 : (g_gui_fb.height / 2) - (h / 2);

    uint32_t dacr;
    DACR_UNRESTRICT(dacr);

    for (int line = 0; line < h; line++) {
        int xd = 0;
        int xd_line = line;
        if (g_gui_fb.height < 544.0f)
            xd_line = xd_line * (544.0f / g_gui_fb.height);

        // Top corners
        if (xd_line < GUI_CORNERS_XD_RADIUS) {
            xd = GUI_RESCALE_X(GUI_CORNERS_XD[xd_line]);
        }
        // Bottom corners
        else if (xd_line >= height - GUI_CORNERS_XD_RADIUS) {
            xd = GUI_RESCALE_X(GUI_CORNERS_XD[height - xd_line - 1]);
        }

        int off = ((line + y) * g_gui_fb.pitch + x + xd);

        void *src = &((rgba_t *)g_gui_buffer)[line * GUI_WIDTH + xd];
        void *dest = &((rgba_t *)g_gui_fb.base)[off];
        int size = sizeof(rgba_t) * (w - xd*2);

        memcpy(dest, src, size);
    }

    DACR_RESET(dacr);
}
