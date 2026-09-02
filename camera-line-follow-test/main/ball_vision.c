#include "ball_vision.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"

#define MIN_RED_COMPONENT_PIXELS 8
#define MAX_TRACK_MISS_FRAMES 3
#define MAX_TRACK_JUMP_PIXELS 28

static uint8_t *s_seen;
static uint16_t *s_queue;
static size_t s_capacity;
static bool s_track_valid;
static int s_track_x;
static int s_track_y;
static int s_track_half_size;
static int s_missed_frames;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
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

static bool is_red_shell(const uint8_t *pixels, size_t index)
{
    int red;
    int green;
    int blue;
    pixel_rgb(pixels, index, &red, &green, &blue);
    const int maximum = red > green ? (red > blue ? red : blue) :
                                      (green > blue ? green : blue);
    const int minimum = red < green ? (red < blue ? red : blue) :
                                      (green < blue ? green : blue);
    /* Measured red shell is, for example, RGB=(176,74,77) and (199,73,57).
     * Detect the saturated red shell rather than its neutral white highlight;
     * this excludes both the white ball and the white playing surface. */
    return red >= green + 45 && red >= blue + 35 &&
           maximum - minimum >= 45;
}

static void set_pixel(uint8_t *pixels, size_t width, size_t height,
                      int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= (int)width || y >= (int)height) return;
    const size_t offset = 2 * ((size_t)y * width + (size_t)x);
    pixels[offset] = color >> 8;
    pixels[offset + 1] = color & 0xff;
}

static void draw_box(uint8_t *pixels, size_t width, size_t height,
                     int left, int top, int right, int bottom,
                     uint16_t color)
{
    for (int x = left; x <= right; ++x) {
        set_pixel(pixels, width, height, x, top, color);
        set_pixel(pixels, width, height, x, bottom, color);
    }
    for (int y = top; y <= bottom; ++y) {
        set_pixel(pixels, width, height, left, y, color);
        set_pixel(pixels, width, height, right, y, color);
    }
}

esp_err_t ball_vision_init(size_t width, size_t height)
{
    const size_t pixel_count = width * height;
    if (pixel_count == 0 || pixel_count > UINT16_MAX) return ESP_ERR_INVALID_ARG;
    free(s_seen);
    free(s_queue);
    s_seen = heap_caps_calloc(pixel_count, 1,
                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_queue = heap_caps_malloc(pixel_count * sizeof(*s_queue),
                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_seen == NULL || s_queue == NULL) {
        free(s_seen);
        free(s_queue);
        s_seen = NULL;
        s_queue = NULL;
        s_capacity = 0;
        return ESP_ERR_NO_MEM;
    }
    s_capacity = pixel_count;
    s_track_valid = false;
    s_missed_frames = 0;
    return ESP_OK;
}

void ball_vision_process(const uint8_t *rgb565, size_t width, size_t height,
                         ball_vision_result_t *result)
{
    memset(result, 0, sizeof(*result));
    const size_t pixel_count = width * height;
    if (rgb565 == NULL || s_seen == NULL || s_queue == NULL ||
        pixel_count != s_capacity) return;
    memset(s_seen, 0, pixel_count);

    int best_score = -1;
    int best_min_x = 0;
    int best_max_x = 0;
    int best_min_y = 0;
    int best_max_y = 0;
    int best_area = 0;

    for (size_t seed = 0; seed < pixel_count; ++seed) {
        if (s_seen[seed] || !is_red_shell(rgb565, seed)) continue;
        size_t head = 0;
        size_t tail = 0;
        s_seen[seed] = 1;
        s_queue[tail++] = (uint16_t)seed;
        int min_x = (int)(seed % width);
        int max_x = min_x;
        int min_y = (int)(seed / width);
        int max_y = min_y;

        while (head < tail) {
            const size_t index = s_queue[head++];
            const int x = (int)(index % width);
            const int y = (int)(index / width);
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= (int)width ||
                        ny >= (int)height) continue;
                    const size_t neighbour = (size_t)ny * width + nx;
                    if (s_seen[neighbour] ||
                        !is_red_shell(rgb565, neighbour)) continue;
                    s_seen[neighbour] = 1;
                    s_queue[tail++] = (uint16_t)neighbour;
                }
            }
        }

        const int component_width = max_x - min_x + 1;
        const int component_height = max_y - min_y + 1;
        const int area = (int)tail;
        if (area < MIN_RED_COMPONENT_PIXELS || component_width < 2 ||
            component_height < 2 || component_width * 4 < component_height ||
            component_height * 4 < component_width) {
            continue;
        }

        int score = area * 10 + (component_width + component_height) * 3;
        const int candidate_x = (min_x + max_x) / 2 + component_width / 8;
        const int candidate_y = (min_y + max_y) / 2;
        if (s_track_valid) {
            const int dx = candidate_x - s_track_x;
            const int dy = candidate_y - s_track_y;
            const int jump_squared = dx * dx + dy * dy;
            if (jump_squared <= MAX_TRACK_JUMP_PIXELS * MAX_TRACK_JUMP_PIXELS) {
                score += 2000 - jump_squared;
            } else {
                score -= 300;
            }
        }
        if (score > best_score) {
            best_score = score;
            best_min_x = min_x;
            best_max_x = max_x;
            best_min_y = min_y;
            best_max_y = max_y;
            best_area = area;
        }
    }

    if (best_score >= 0) {
        const int component_width = best_max_x - best_min_x + 1;
        const int component_height = best_max_y - best_min_y + 1;
        const int raw_x = (best_min_x + best_max_x) / 2 + component_width / 8;
        const int raw_y = (best_min_y + best_max_y) / 2;
        const int raw_half_size = ((component_width > component_height ?
                                      component_width : component_height) + 1) / 2 +
                                  2;
        if (s_track_valid) {
            s_track_x = (3 * s_track_x + raw_x + 2) / 4;
            s_track_y = (3 * s_track_y + raw_y + 2) / 4;
            s_track_half_size = (3 * s_track_half_size + raw_half_size + 2) / 4;
        } else {
            s_track_x = raw_x;
            s_track_y = raw_y;
            s_track_half_size = raw_half_size;
        }
        s_track_valid = true;
        s_missed_frames = 0;
        result->found = true;
        result->purple_pixels = best_area;
        result->confidence = best_area >= 20 ? 100 : best_area * 5;
    } else if (s_track_valid && ++s_missed_frames <= MAX_TRACK_MISS_FRAMES) {
        result->found = true;
        result->predicted = true;
        result->confidence = 50;
    } else {
        s_track_valid = false;
        return;
    }

    result->center_x = s_track_x;
    result->center_y = s_track_y;
    result->left = clamp_int(s_track_x - s_track_half_size, 0, (int)width - 1);
    result->top = clamp_int(s_track_y - s_track_half_size, 0, (int)height - 1);
    result->right = clamp_int(s_track_x + s_track_half_size, 0, (int)width - 1);
    result->bottom = clamp_int(s_track_y + s_track_half_size, 0, (int)height - 1);
}

void ball_vision_draw_overlay(uint8_t *rgb565, size_t width, size_t height,
                              const ball_vision_result_t *result)
{
    ball_vision_draw_overlay_color(rgb565, width, height, result, 0x07e0);
}

void ball_vision_draw_overlay_color(uint8_t *rgb565, size_t width,
                                    size_t height,
                                    const ball_vision_result_t *result,
                                    uint16_t color)
{
    if (rgb565 == NULL || result == NULL || !result->found) return;
    const uint16_t box_color = result->predicted ? 0xffe0 : color;
    draw_box(rgb565, width, height, result->left, result->top,
             result->right, result->bottom, box_color);
    set_pixel(rgb565, width, height, result->center_x, result->center_y,
              0xf800);
}
