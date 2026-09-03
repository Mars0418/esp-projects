#include "black_marker_vision.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#define GOAL_MIN_PIXELS 20
#define GOAL_MIN_SIDE_PIXELS 5
#define GOAL_TRUST_ROI_WIDTH 100
#define GOAL_TRUST_ROI_HEIGHT 90
#define GOAL_MAX_MISS_FRAMES 1
#define GOAL_POSITION_DEADBAND_PIXELS 2
#define GOAL_POSITION_SNAP_DISTANCE_PIXELS 12
#define GOAL_POSITION_SNAP_CONFIDENCE 65
#define GOAL_JUMP_CONFIRM_FRAMES 2
#define GOAL_JUMP_MATCH_PIXELS 4

static const char *TAG = "goal_basic";
static uint8_t *s_seen;
static uint16_t *s_queue;
static size_t s_capacity;
static bool s_logging_enabled = true;
static bool s_track_valid;
static int s_track_x;
static int s_track_y;
static int s_track_half_width;
static int s_track_half_height;
static int s_missed_frames;
static bool s_pending_jump_valid;
static int s_pending_jump_x;
static int s_pending_jump_y;
static int s_pending_jump_count;
static goal_dark_thresholds_t s_thresholds = {
    .luminance_max = 85,
    .channel_max = 110,
    .rgb_spread_max = 24,
};

typedef struct {
    bool valid;
    int score;
    int confidence;
    int area;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int64_t sum_x;
    int64_t sum_y;
} goal_candidate_t;

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
        abs(x - s_pending_jump_x) <= GOAL_JUMP_MATCH_PIXELS &&
        abs(y - s_pending_jump_y) <= GOAL_JUMP_MATCH_PIXELS) {
        s_pending_jump_count++;
    } else {
        s_pending_jump_valid = true;
        s_pending_jump_count = 1;
    }
    s_pending_jump_x = x;
    s_pending_jump_y = y;
    return s_pending_jump_count >= GOAL_JUMP_CONFIRM_FRAMES;
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
    int red, green, blue;
    pixel_rgb(pixels, index, &red, &green, &blue);
    return (77 * red + 150 * green + 29 * blue) >> 8;
}

bool black_marker_vision_pixel_is_goal(const uint8_t *rgb565, size_t index)
{
    if (!rgb565) return false;
    int red, green, blue;
    pixel_rgb(rgb565, index, &red, &green, &blue);
    const int maximum = red > green ? (red > blue ? red : blue) :
                                      (green > blue ? green : blue);
    const int minimum = red < green ? (red < blue ? red : blue) :
                                      (green < blue ? green : blue);
    const int luminance = (77 * red + 150 * green + 29 * blue) >> 8;
    return luminance <= s_thresholds.luminance_max &&
           maximum <= s_thresholds.channel_max &&
           maximum - minimum <= s_thresholds.rgb_spread_max;
}

goal_dark_thresholds_t black_marker_vision_get_thresholds(void)
{
    return s_thresholds;
}

void black_marker_vision_set_thresholds(goal_dark_thresholds_t thresholds)
{
    s_thresholds = thresholds;
}

void black_marker_vision_set_logging(bool enabled)
{
    s_logging_enabled = enabled;
}

static void set_pixel(uint8_t *pixels, size_t width, size_t height,
                      int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= (int)width || y >= (int)height) return;
    const size_t offset = 2 * ((size_t)y * width + (size_t)x);
    pixels[offset] = color >> 8;
    pixels[offset + 1] = color & 0xff;
}

esp_err_t black_marker_vision_init(size_t width, size_t height)
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
    s_track_valid = false;
    s_missed_frames = 0;
    clear_pending_jump();
    return ESP_OK;
}

void black_marker_vision_process(const uint8_t *rgb565, size_t width,
                                 size_t height,
                                 black_marker_result_t *result)
{
    if (!result) return;
    memset(result, 0, sizeof(*result));
    const size_t pixels = width * height;
    if (!rgb565 || !s_seen || !s_queue || pixels != s_capacity) return;

    memset(s_seen, 0, pixels);
    goal_candidate_t best = {0};
    for (size_t seed = 0; seed < pixels; ++seed) {
        if (s_seen[seed] ||
            !black_marker_vision_pixel_is_goal(rgb565, seed)) {
            continue;
        }

        size_t head = 0;
        size_t tail = 0;
        s_seen[seed] = 1;
        s_queue[tail++] = (uint16_t)seed;
        int min_x = (int)(seed % width);
        int max_x = min_x;
        int min_y = (int)(seed / width);
        int max_y = min_y;
        int64_t sum_x = 0;
        int64_t sum_y = 0;
        int dark_luminance_sum = 0;

        while (head < tail) {
            const size_t index = s_queue[head++];
            const int x = (int)(index % width);
            const int y = (int)(index / width);
            dark_luminance_sum += pixel_luminance(rgb565, index);
            sum_x += x;
            sum_y += y;
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 ||
                        nx >= (int)width || ny >= (int)height) {
                        continue;
                    }
                    const size_t neighbour = (size_t)ny * width + nx;
                    if (!s_seen[neighbour] &&
                        black_marker_vision_pixel_is_goal(rgb565,
                                                          neighbour)) {
                        s_seen[neighbour] = 1;
                        s_queue[tail++] = (uint16_t)neighbour;
                    }
                }
            }
        }

        const int component_width = max_x - min_x + 1;
        const int component_height = max_y - min_y + 1;
        const int short_side = component_width < component_height
                                   ? component_width : component_height;
        const int long_side = component_width > component_height
                                  ? component_width : component_height;
        const int box_area = component_width * component_height;
        const int area = (int)tail;
        const int fill_percent = box_area > 0 ? area * 100 / box_area : 0;

        int ring_luminance_sum = 0;
        int ring_pixels = 0;
        const int ring_left = clamp_int(min_x - 2, 0, (int)width - 1);
        const int ring_right = clamp_int(max_x + 2, 0, (int)width - 1);
        const int ring_top = clamp_int(min_y - 2, 0, (int)height - 1);
        const int ring_bottom = clamp_int(max_y + 2, 0, (int)height - 1);
        for (int y = ring_top; y <= ring_bottom; ++y) {
            for (int x = ring_left; x <= ring_right; ++x) {
                if (x >= min_x && x <= max_x &&
                    y >= min_y && y <= max_y) {
                    continue;
                }
                ring_luminance_sum +=
                    pixel_luminance(rgb565, (size_t)y * width + x);
                ring_pixels++;
            }
        }
        const int dark_luminance = dark_luminance_sum / area;
        const int ring_luminance = ring_pixels > 0
                                       ? ring_luminance_sum / ring_pixels : 0;
        const int contrast = ring_luminance - dark_luminance;
        const int center_x = (int)((sum_x + area / 2) / area);
        const int center_y = (int)((sum_y + area / 2) / area);
        const int trust_left = ((int)width - GOAL_TRUST_ROI_WIDTH) / 2;
        const int trust_top = ((int)height - GOAL_TRUST_ROI_HEIGHT) / 2;
        const int trust_right = trust_left + GOAL_TRUST_ROI_WIDTH - 1;
        const int trust_bottom = trust_top + GOAL_TRUST_ROI_HEIGHT - 1;

        /* A quarter disk remains a compact, substantially filled component
         * under perspective. Thin court lines fail aspect/fill; neutral-dark
         * and local contrast reject coloured objects and soft shadows. */
        if (area < GOAL_MIN_PIXELS || short_side < GOAL_MIN_SIDE_PIXELS ||
            long_side > short_side * 3 || fill_percent < 35 ||
            fill_percent > 94 || contrast < 35 ||
            center_x < trust_left || center_x > trust_right ||
            center_y < trust_top || center_y > trust_bottom) {
            continue;
        }

        const int area_points = clamp_int(area / 3, 0, 25);
        const int fill_points = clamp_int(25 - abs(fill_percent - 78), 0, 25);
        const int shape_points = short_side * 20 / long_side;
        const int contrast_points = clamp_int((contrast - 20) / 2, 0, 30);
        const int confidence = clamp_int(area_points + fill_points +
                                         shape_points + contrast_points,
                                         0, 100);
        const int score = confidence * 1000 + clamp_int(area * 5, 0, 999);
        if (!best.valid || score > best.score) {
            best = (goal_candidate_t) {
                .valid = true,
                .score = score,
                .confidence = confidence,
                .area = area,
                .min_x = min_x,
                .max_x = max_x,
                .min_y = min_y,
                .max_y = max_y,
                .sum_x = sum_x,
                .sum_y = sum_y,
            };
        }
    }

    bool detected_this_frame = false;
    if (best.valid) {
        const int raw_x = (int)((best.sum_x + best.area / 2) / best.area);
        const int raw_y = (int)((best.sum_y + best.area / 2) / best.area);
        const int raw_half_width = (best.max_x - best.min_x + 2) / 2;
        const int raw_half_height = (best.max_y - best.min_y + 2) / 2;
        bool accept_candidate = true;
        bool snap_to_candidate = !s_track_valid;

        if (s_track_valid) {
            const int dx = raw_x - s_track_x;
            const int dy = raw_y - s_track_y;
            const bool large_jump =
                dx * dx + dy * dy >
                GOAL_POSITION_SNAP_DISTANCE_PIXELS *
                GOAL_POSITION_SNAP_DISTANCE_PIXELS;
            if (large_jump &&
                best.confidence < GOAL_POSITION_SNAP_CONFIDENCE) {
                accept_candidate = confirm_low_confidence_jump(raw_x, raw_y);
                snap_to_candidate = accept_candidate;
            } else {
                clear_pending_jump();
                snap_to_candidate = large_jump;
            }

            if (accept_candidate && !snap_to_candidate) {
                if (abs(dx) > GOAL_POSITION_DEADBAND_PIXELS) {
                    s_track_x = (3 * s_track_x + 7 * raw_x + 5) / 10;
                }
                if (abs(dy) > GOAL_POSITION_DEADBAND_PIXELS) {
                    s_track_y = (3 * s_track_y + 7 * raw_y + 5) / 10;
                }
                s_track_half_width =
                    (3 * s_track_half_width + 7 * raw_half_width + 5) / 10;
                s_track_half_height =
                    (3 * s_track_half_height + 7 * raw_half_height + 5) / 10;
            }
        }

        if (accept_candidate) {
            if (snap_to_candidate) {
                s_track_x = raw_x;
                s_track_y = raw_y;
                s_track_half_width = raw_half_width;
                s_track_half_height = raw_half_height;
            }
            clear_pending_jump();
            s_track_valid = true;
            s_missed_frames = 0;
            detected_this_frame = true;
            result->found = true;
            result->color_pixels = best.area;
            result->dark_pixels = best.area;
            result->confidence = best.confidence;
        } else if (++s_missed_frames <= GOAL_MAX_MISS_FRAMES) {
            result->found = true;
            result->predicted = true;
            result->confidence = 50;
        } else {
            s_track_valid = false;
            clear_pending_jump();
        }
    } else if (s_track_valid &&
               ++s_missed_frames <= GOAL_MAX_MISS_FRAMES) {
        result->found = true;
        result->predicted = true;
        result->confidence = 50;
        clear_pending_jump();
    } else {
        s_track_valid = false;
        clear_pending_jump();
    }

    if (result->found) {
        result->center_x = s_track_x;
        result->center_y = s_track_y;
        result->left = clamp_int(s_track_x - s_track_half_width,
                                 0, (int)width - 1);
        result->right = clamp_int(s_track_x + s_track_half_width,
                                  0, (int)width - 1);
        result->top = clamp_int(s_track_y - s_track_half_height,
                                0, (int)height - 1);
        result->bottom = clamp_int(s_track_y + s_track_half_height,
                                   0, (int)height - 1);
    } else {
        result->center_x = -1;
        result->center_y = -1;
    }

    if (s_logging_enabled) {
        ESP_LOGI(TAG,
                 "GOAL_BASIC found=%d detected=%d predicted=%d confidence=%d center_raw=(%d,%d) black_pixels=%d",
                 result->found, detected_this_frame, result->predicted,
                 result->confidence, result->center_x, result->center_y,
                 result->dark_pixels);
    }
}

void black_marker_vision_draw_overlay(uint8_t *rgb565, size_t width,
                                      size_t height,
                                      const black_marker_result_t *result)
{
    if (!rgb565 || !result || !result->found) return;
    const uint16_t box_color = result->predicted ? 0xffe0 : 0xf81f;
    const uint16_t center_color = 0xffff;
    for (int x = result->left; x <= result->right; ++x) {
        set_pixel(rgb565, width, height, x, result->top, box_color);
        set_pixel(rgb565, width, height, x, result->bottom, box_color);
    }
    for (int y = result->top; y <= result->bottom; ++y) {
        set_pixel(rgb565, width, height, result->left, y, box_color);
        set_pixel(rgb565, width, height, result->right, y, box_color);
    }
    for (int delta = -3; delta <= 3; ++delta) {
        set_pixel(rgb565, width, height, result->center_x + delta,
                  result->center_y, center_color);
        set_pixel(rgb565, width, height, result->center_x,
                  result->center_y + delta, center_color);
    }
}
