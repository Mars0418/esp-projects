#include "black_marker_vision.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"

#define MARKER_DARK_LUMA_MAX 95
#define MARKER_MIN_PIXELS 12

static uint8_t *s_seen;
static uint16_t *s_queue;
static size_t s_capacity;

typedef struct {
    bool valid;
    int base_score;
    int final_score;
    int area;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
} marker_candidate_t;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int pixel_luminance(const uint8_t *pixels, size_t index)
{
    const uint16_t color = ((uint16_t)pixels[index * 2] << 8) |
                           pixels[index * 2 + 1];
    const int red = ((color >> 11) & 0x1f) * 255 / 31;
    const int green = ((color >> 5) & 0x3f) * 255 / 63;
    const int blue = (color & 0x1f) * 255 / 31;
    return (77 * red + 150 * green + 29 * blue) >> 8;
}

static bool is_dark(const uint8_t *pixels, size_t index)
{
    const uint16_t color = ((uint16_t)pixels[index * 2] << 8) |
                           pixels[index * 2 + 1];
    const int red = ((color >> 11) & 0x1f) * 255 / 31;
    const int green = ((color >> 5) & 0x3f) * 255 / 63;
    const int blue = (color & 0x1f) * 255 / 31;
    const int maximum = red > green ? (red > blue ? red : blue) :
                                      (green > blue ? green : blue);
    const int minimum = red < green ? (red < blue ? red : blue) :
                                      (green < blue ? green : blue);
    /* Black marker is near-neutral.  This rejects the dark blue-violet part
     * of the purple billiard ball even when it has similar luminance. */
    return pixel_luminance(pixels, index) <= MARKER_DARK_LUMA_MAX &&
           maximum - minimum <= 18;
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
    s_seen = heap_caps_calloc(pixels, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_queue = heap_caps_malloc(pixels * sizeof(*s_queue),
                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_seen || !s_queue) return ESP_ERR_NO_MEM;
    s_capacity = pixels;
    return ESP_OK;
}

void black_marker_vision_process(const uint8_t *rgb565, size_t width,
                                 size_t height,
                                 black_marker_result_t *result)
{
    memset(result, 0, sizeof(*result));
    const size_t pixels = width * height;
    if (!rgb565 || !s_seen || !s_queue || pixels != s_capacity) return;
    memset(s_seen, 0, pixels);
    marker_candidate_t top_candidates[2] = {0};
    for (size_t seed = 0; seed < pixels; ++seed) {
        const int seed_x = (int)(seed % width), seed_y = (int)(seed / width);
        if (s_seen[seed] || !is_dark(rgb565, seed)) continue;
        size_t head = 0, tail = 0;
        s_seen[seed] = 1;
        s_queue[tail++] = (uint16_t)seed;
        int min_x = seed_x, max_x = seed_x, min_y = seed_y, max_y = seed_y;
        int dark_luma_sum = 0;
        while (head < tail) {
            const size_t index = s_queue[head++];
            const int x = (int)(index % width), y = (int)(index / width);
            dark_luma_sum += pixel_luminance(rgb565, index);
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
            for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                const int nx = x + dx, ny = y + dy;
                if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 ||
                    nx >= (int)width || ny >= (int)height) continue;
                const size_t neighbour = (size_t)ny * width + nx;
                if (!s_seen[neighbour] && is_dark(rgb565, neighbour)) {
                    s_seen[neighbour] = 1;
                    s_queue[tail++] = (uint16_t)neighbour;
                }
            }
        }
        const int component_width = max_x - min_x + 1;
        const int component_height = max_y - min_y + 1;
        const int box_area = component_width * component_height;
        const int fill_percent = box_area > 0 ? (int)tail * 100 / box_area : 0;
        int ring_luma_sum = 0;
        int ring_pixels = 0;
        const int ring_left = clamp_int(min_x - 2, 0, (int)width - 1);
        const int ring_right = clamp_int(max_x + 2, 0, (int)width - 1);
        const int ring_top = clamp_int(min_y - 2, 0, (int)height - 1);
        const int ring_bottom = clamp_int(max_y + 2, 0, (int)height - 1);
        for (int y = ring_top; y <= ring_bottom; ++y) {
            for (int x = ring_left; x <= ring_right; ++x) {
                if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
                    continue;
                }
                ring_luma_sum += pixel_luminance(rgb565, (size_t)y * width + x);
                ring_pixels++;
            }
        }
        const int dark_luma = dark_luma_sum / (int)tail;
        const int ring_luma = ring_pixels > 0 ? ring_luma_sum / ring_pixels : 0;
        const int contrast = ring_luma - dark_luma;
        int axis_counts[160] = {0};
        const bool horizontal_major_axis = component_width >= component_height;
        const int axis_length = horizontal_major_axis ? component_width :
                                                       component_height;
        int axis_max = 0;
        for (size_t index = 0; index < tail; ++index) {
            const int x = (int)(s_queue[index] % width);
            const int y = (int)(s_queue[index] / width);
            const int axis = horizontal_major_axis ? x - min_x : y - min_y;
            if (axis >= 0 && axis < axis_length) axis_counts[axis]++;
        }
        for (int axis = 0; axis < axis_length; ++axis) {
            if (axis_counts[axis] > axis_max) axis_max = axis_counts[axis];
        }
        const int endpoint_asymmetry = axis_max > 0
            ? abs(axis_counts[0] - axis_counts[axis_length - 1]) * 100 / axis_max
            : 0;
        const int fill_triangle_score = 100 - abs(fill_percent - 50) * 2;
        const int wedge_score = (endpoint_asymmetry * 2 +
                                 fill_triangle_score) / 3;
        /* The target is a filled wedge/trapezoid.  At distance it is only
         * about 29x7 pixels, so it must be allowed to be wider than 4:1.
         * Require a 2--8:1 major/minor ratio instead: that keeps the target,
         * rejects round ball patches, and the minimum thickness rejects the
         * thin black track line.  No absolute image position is used. */
        const int major_axis = horizontal_major_axis ? component_width :
                                                      component_height;
        const int minor_axis = horizontal_major_axis ? component_height :
                                                      component_width;
        if ((int)tail >= MARKER_MIN_PIXELS && component_width >= 3 &&
            component_height >= 4 && major_axis >= minor_axis * 2 &&
            major_axis <= minor_axis * 8 && fill_percent >= 25 &&
            fill_percent <= 80 && contrast >= 55) {
            const int base_score = (int)tail * 3 + contrast * 12;
            const marker_candidate_t candidate = {
                .valid = true,
                .base_score = base_score,
                /* Give the wedge/triangle decision more influence than raw
                 * area. This makes a wedge beat a similarly dark round blob. */
                .final_score = base_score + wedge_score * 24,
                .area = (int)tail,
                .min_x = min_x, .max_x = max_x,
                .min_y = min_y, .max_y = max_y,
            };
            if (!top_candidates[0].valid ||
                candidate.base_score > top_candidates[0].base_score) {
                top_candidates[1] = top_candidates[0];
                top_candidates[0] = candidate;
            } else if (!top_candidates[1].valid ||
                       candidate.base_score > top_candidates[1].base_score) {
                top_candidates[1] = candidate;
            }
        }
    }
    const marker_candidate_t *selected = NULL;
    for (size_t index = 0; index < 2; ++index) {
        if (!top_candidates[index].valid ||
            (selected && top_candidates[index].final_score <=
                         selected->final_score)) continue;
        selected = &top_candidates[index];
    }
    if (selected == NULL) return;
    result->found = true;
    result->dark_pixels = selected->area;
    result->left = clamp_int(selected->min_x - 1, 0, (int)width - 1);
    result->right = clamp_int(selected->max_x + 3, 0, (int)width - 1);
    result->top = clamp_int(selected->min_y - 2, 0, (int)height - 1);
    result->bottom = clamp_int(selected->max_y + 2, 0, (int)height - 1);
    result->center_x = (result->left + result->right) / 2;
    result->center_y = (result->top + result->bottom) / 2;
}

void black_marker_vision_draw_overlay(uint8_t *rgb565, size_t width,
                                      size_t height,
                                      const black_marker_result_t *result)
{
    if (!rgb565 || !result || !result->found) return;
    const uint16_t color = 0xf81f;
    for (int x = result->left; x <= result->right; ++x) {
        set_pixel(rgb565, width, height, x, result->top, color);
        set_pixel(rgb565, width, height, x, result->bottom, color);
    }
    for (int y = result->top; y <= result->bottom; ++y) {
        set_pixel(rgb565, width, height, result->left, y, color);
        set_pixel(rgb565, width, height, result->right, y, color);
    }
}
