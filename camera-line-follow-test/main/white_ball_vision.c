#include "ball_vision.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#define MIN_WHITE_COMPONENT_PIXELS 10
#define MAX_WHITE_MISS_FRAMES 1
#define MAX_WHITE_JUMP_PIXELS 20
#define DETECTION_WINDOW_FRAMES 10
#define POSITION_DEADBAND_PIXELS 2
#define POSITION_SNAP_DISTANCE_PIXELS 12
#define POSITION_SNAP_CONFIDENCE 80
#define JUMP_CONFIRM_FRAMES 2
#define JUMP_MATCH_PIXELS 4

static uint8_t *s_seen;
static uint16_t *s_queue;
static size_t s_capacity;
static bool s_valid;
static int s_x;
static int s_y;
static int s_half;
static int s_missed;
static bool s_pending_jump_valid;
static int s_pending_jump_x;
static int s_pending_jump_y;
static int s_pending_jump_count;
static uint8_t s_detection_history[DETECTION_WINDOW_FRAMES];
static size_t s_detection_history_index;
static size_t s_detection_history_count;
static int s_recent_detection_count;

#define WHITE_TAG "white_vision"

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void clear_pending_jump(void)
{
    s_pending_jump_valid = false;
    s_pending_jump_count = 0;
}

static bool confirm_low_confidence_jump(int x, int y)
{
    if (s_pending_jump_valid &&
        abs(x - s_pending_jump_x) <= JUMP_MATCH_PIXELS &&
        abs(y - s_pending_jump_y) <= JUMP_MATCH_PIXELS) {
        s_pending_jump_count++;
    } else {
        s_pending_jump_valid = true;
        s_pending_jump_count = 1;
    }
    s_pending_jump_x = x;
    s_pending_jump_y = y;
    return s_pending_jump_count >= JUMP_CONFIRM_FRAMES;
}

static void log_white_ball_result(const ball_vision_result_t *result,
                                  bool detected_this_frame)
{
    if (s_detection_history_count == DETECTION_WINDOW_FRAMES) {
        s_recent_detection_count -=
            s_detection_history[s_detection_history_index];
    } else {
        s_detection_history_count++;
    }

    const uint8_t detected = detected_this_frame ? 1 : 0;
    s_detection_history[s_detection_history_index] = detected;
    s_recent_detection_count += detected;
    s_detection_history_index =
        (s_detection_history_index + 1) % DETECTION_WINDOW_FRAMES;

    const int center_x = detected_this_frame
                             ? (result->left + result->right) / 2 : -1;
    const int center_y = detected_this_frame
                             ? (result->top + result->bottom) / 2 : -1;
    ESP_LOGI(WHITE_TAG,
             "WHITE_BALL found=%d confidence=%d center=(%d,%d) recent10=%d",
             detected_this_frame, detected_this_frame ? result->confidence : 0,
             center_x, center_y,
             s_recent_detection_count);
}

static void pixel_rgb(const uint8_t *pixels, size_t index,
                      int *red, int *green, int *blue)
{
    const uint16_t color = ((uint16_t)pixels[index * 2] << 8) |
                           pixels[index * 2 + 1];
    *red = ((color >> 11) & 0x1f) * 255 / 31;
    *green = ((color >> 5) & 0x3f) * 255 / 63;
    *blue = (color & 0x1f) * 255 / 31;
}

static int pixel_luminance(const uint8_t *pixels, size_t index)
{
    int red;
    int green;
    int blue;
    pixel_rgb(pixels, index, &red, &green, &blue);
    return (77 * red + 150 * green + 29 * blue) >> 8;
}

static bool is_red_ball_shell(const uint8_t *pixels, size_t index)
{
    int red;
    int green;
    int blue;
    pixel_rgb(pixels, index, &red, &green, &blue);
    return red >= green + 45 && red >= blue + 35;
}

static bool is_white_golf_core(const uint8_t *pixels, size_t index)
{
    int red;
    int green;
    int blue;
    pixel_rgb(pixels, index, &red, &green, &blue);
    const int maximum = red > green ? (red > blue ? red : blue) :
                                      (green > blue ? green : blue);
    const int minimum = red < green ? (red < blue ? red : blue) :
                                      (green < blue ? green : blue);
    const int luminance = (77 * red + 150 * green + 29 * blue) >> 8;
    /* RGB565 quantisation makes both the ball and floor appear neutral, so a
     * blue-vs-red test is not useful on-device.  The ball's compact specular
     * core is still brighter than the floor; use that separation directly. */
    return luminance >= 205 && maximum - minimum <= 55;
}

esp_err_t white_ball_vision_init(size_t width, size_t height)
{
    const size_t pixels = width * height;
    if (pixels == 0 || pixels > UINT16_MAX) return ESP_ERR_INVALID_ARG;
    free(s_seen);
    free(s_queue);
    s_seen = heap_caps_calloc(pixels, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_queue = heap_caps_malloc(pixels * sizeof(*s_queue),
                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_seen || !s_queue) return ESP_ERR_NO_MEM;
    s_capacity = pixels;
    s_valid = false;
    s_missed = 0;
    clear_pending_jump();
    memset(s_detection_history, 0, sizeof(s_detection_history));
    s_detection_history_index = 0;
    s_detection_history_count = 0;
    s_recent_detection_count = 0;
    return ESP_OK;
}

void white_ball_vision_process(const uint8_t *rgb565, size_t width,
                               size_t height, ball_vision_result_t *result)
{
    memset(result, 0, sizeof(*result));
    const size_t pixels = width * height;
    if (!rgb565 || !s_seen || !s_queue || pixels != s_capacity) return;
    memset(s_seen, 0, pixels);
    int best_score = -1;
    int best_min_x = 0, best_max_x = 0, best_min_y = 0, best_max_y = 0;
    int best_area = 0;
    for (size_t seed = 0; seed < pixels; ++seed) {
        if (s_seen[seed] || !is_white_golf_core(rgb565, seed)) continue;
        size_t head = 0, tail = 0;
        s_seen[seed] = 1;
        s_queue[tail++] = (uint16_t)seed;
        int min_x = (int)(seed % width), max_x = min_x;
        int min_y = (int)(seed / width), max_y = min_y;
        int bright_luma_sum = 0;
        while (head < tail) {
            const size_t index = s_queue[head++];
            const int x = (int)(index % width), y = (int)(index / width);
            bright_luma_sum += pixel_luminance(rgb565, index);
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
            for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                const int nx = x + dx, ny = y + dy;
                if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 ||
                    nx >= (int)width || ny >= (int)height) continue;
                const size_t neighbour = (size_t)ny * width + nx;
                if (!s_seen[neighbour] && is_white_golf_core(rgb565, neighbour)) {
                    s_seen[neighbour] = 1;
                    s_queue[tail++] = (uint16_t)neighbour;
                }
            }
        }
        const int component_width = max_x - min_x + 1;
        const int component_height = max_y - min_y + 1;
        const int area = (int)tail;
        const int box_area = component_width * component_height;
        const int fill_percent = box_area > 0 ? area * 100 / box_area : 0;
        int ring_luma_sum = 0;
        int ring_pixels = 0;
        const int ring_left = clamp_int(min_x - 2, 0, (int)width - 1);
        const int ring_right = clamp_int(max_x + 2, 0, (int)width - 1);
        const int ring_top = clamp_int(min_y - 2, 0, (int)height - 1);
        const int ring_bottom = clamp_int(max_y + 2, 0, (int)height - 1);
        for (int y = ring_top; y <= ring_bottom; ++y) {
            for (int x = ring_left; x <= ring_right; ++x) {
                if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) continue;
                ring_luma_sum += pixel_luminance(rgb565, (size_t)y * width + x);
                ring_pixels++;
            }
        }
        const int bright_luma = bright_luma_sum / area;
        const int ring_luma = ring_pixels > 0 ? ring_luma_sum / ring_pixels : 255;
        const int contrast = bright_luma - ring_luma;
        /* A white ball on the white floor is confirmed by its nearby neutral
         * dark rim/shadow (and often its printed mark).  Search a wider local
         * window for this evidence.  A red pixel in that same shadow window
         * means the dark feature belongs to the red ball, not the white one. */
        int shadow_luma_sum = 0;
        int shadow_pixels = 0;
        int nearby_red_pixels = 0;
        const int shadow_left = clamp_int(min_x - 5, 0, (int)width - 1);
        const int shadow_right = clamp_int(max_x + 5, 0, (int)width - 1);
        const int shadow_top = clamp_int(min_y - 5, 0, (int)height - 1);
        const int shadow_bottom = clamp_int(max_y + 5, 0, (int)height - 1);
        for (int y = shadow_top; y <= shadow_bottom; ++y) {
            for (int x = shadow_left; x <= shadow_right; ++x) {
                const size_t nearby = (size_t)y * width + x;
                int red;
                int green;
                int blue;
                pixel_rgb(rgb565, nearby, &red, &green, &blue);
                const int maximum = red > green ? (red > blue ? red : blue) :
                                                  (green > blue ? green : blue);
                const int minimum = red < green ? (red < blue ? red : blue) :
                                                  (green < blue ? green : blue);
                const int luminance = (77 * red + 150 * green + 29 * blue) >> 8;
                if (is_red_ball_shell(rgb565, nearby)) nearby_red_pixels++;
                if (luminance <= 150 && maximum - minimum <= 60) {
                    shadow_luma_sum += luminance;
                    shadow_pixels++;
                }
            }
        }
        const int shadow_luma = shadow_pixels > 0
                                    ? shadow_luma_sum / shadow_pixels : 255;
        const int shadow_contrast = bright_luma - shadow_luma;
        int reject_mask = 0;
        if (area < MIN_WHITE_COMPONENT_PIXELS) reject_mask |= 1;
        if (component_width < 3 || component_height < 3 ||
            component_width > 16 || component_height > 16 ||
            component_width * 4 < component_height ||
            component_height * 4 < component_width || fill_percent < 40) {
            reject_mask |= 2;
        }
        if (contrast < 10) reject_mask |= 4;
        if (shadow_pixels < 6) reject_mask |= 8;
        if (shadow_contrast < 35) reject_mask |= 16;
        /* A few red pixels are normal JPEG/RGB565 bleed when the two balls
         * are close.  Treat it as an actual red-ball overlap only when there
         * is a meaningful red region in the white candidate's shadow window. */
        if (nearby_red_pixels >= 8) reject_mask |= 32;
        if (min_y < 4) reject_mask |= 64;
        if (reject_mask != 0) continue;
        const int candidate_x = (min_x + max_x) / 2;
        const int candidate_y = (min_y + max_y) / 2;
        const int round_score = 100 - abs(component_width - component_height) * 20;
        int score = area * 8 + contrast * 8 + shadow_contrast * 14 +
                    shadow_pixels * 10 + round_score * 12;
        if (s_valid) {
            const int dx = candidate_x - s_x, dy = candidate_y - s_y;
            const int jump_squared = dx * dx + dy * dy;
            /* Temporal proximity is a preference, not a hard rejection.  A
             * false old lock must never prevent the much stronger
             * bright-core + neutral-shadow candidate from taking over. */
            if (jump_squared <= MAX_WHITE_JUMP_PIXELS * MAX_WHITE_JUMP_PIXELS) {
                score += 500 - jump_squared;
            }
        }
        if (score > best_score) {
            best_score = score;
            best_min_x = min_x; best_max_x = max_x;
            best_min_y = min_y; best_max_y = max_y; best_area = area;
        }
    }
    /* Detailed candidate diagnostics are temporarily disabled.
    ESP_LOGI(WHITE_TAG,
             "diag area=%d box=%dx%d con=%d shadow=%d/%d red=%d reject=0x%x best=%d tracked=%d",
             diagnostic_area, diagnostic_width, diagnostic_height,
             diagnostic_contrast, diagnostic_shadow_pixels,
             diagnostic_shadow_contrast, diagnostic_red_pixels,
             diagnostic_reject_mask, best_score, s_valid);
    */
    bool detected_this_frame = false;
    if (best_score >= 0) {
        const int component_width = best_max_x - best_min_x + 1;
        const int component_height = best_max_y - best_min_y + 1;
        const int raw_x = (best_min_x + best_max_x) / 2;
        const int raw_y = (best_min_y + best_max_y) / 2;
        const int raw_half = ((component_width > component_height ?
                               component_width : component_height) + 1) / 2 + 3;
        const int confidence = best_area >= 20 ? 100 : best_area * 5;
        bool accept_candidate = true;
        bool snap_to_candidate = !s_valid;

        if (s_valid) {
            const int dx = raw_x - s_x;
            const int dy = raw_y - s_y;
            const bool large_jump =
                dx * dx + dy * dy > POSITION_SNAP_DISTANCE_PIXELS *
                                      POSITION_SNAP_DISTANCE_PIXELS;
            if (large_jump && confidence < POSITION_SNAP_CONFIDENCE) {
                accept_candidate = confirm_low_confidence_jump(raw_x, raw_y);
                snap_to_candidate = accept_candidate;
            } else {
                clear_pending_jump();
                snap_to_candidate = large_jump;
            }

            if (accept_candidate && !snap_to_candidate) {
                if (abs(dx) > POSITION_DEADBAND_PIXELS) {
                    s_x = (3 * s_x + 7 * raw_x + 5) / 10;
                }
                if (abs(dy) > POSITION_DEADBAND_PIXELS) {
                    s_y = (3 * s_y + 7 * raw_y + 5) / 10;
                }
                s_half = (3 * s_half + 7 * raw_half + 5) / 10;
            }
        }

        if (accept_candidate) {
            if (snap_to_candidate) {
                s_x = raw_x;
                s_y = raw_y;
                s_half = raw_half;
            }
            clear_pending_jump();
            s_valid = true;
            s_missed = 0;
            detected_this_frame = true;
            result->found = true;
            result->purple_pixels = best_area;
            result->confidence = confidence;
        } else if (s_valid && ++s_missed <= MAX_WHITE_MISS_FRAMES) {
            result->found = true;
            result->predicted = true;
            result->confidence = 50;
        } else {
            s_valid = false;
            clear_pending_jump();
            log_white_ball_result(result, false);
            return;
        }
    } else if (s_valid && ++s_missed <= MAX_WHITE_MISS_FRAMES) {
        result->found = true; result->predicted = true; result->confidence = 50;
    } else {
        s_valid = false;
        clear_pending_jump();
        log_white_ball_result(result, false);
        return;
    }
    result->center_x = s_x; result->center_y = s_y;
    result->left = clamp_int(s_x - s_half, 0, (int)width - 1);
    result->top = clamp_int(s_y - s_half, 0, (int)height - 1);
    result->right = clamp_int(s_x + s_half, 0, (int)width - 1);
    result->bottom = clamp_int(s_y + s_half, 0, (int)height - 1);
    log_white_ball_result(result, detected_this_frame);
}
