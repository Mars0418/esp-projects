#include "line_vision.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define ROI_X_MIN 3
#define ROI_X_MAX 77
/* Camera/TFT mounting makes increasing raw y point farther ahead. */
#define ROI_Y_MIN 12
#define ROI_Y_MAX 58
#define NEAR_Y_MAX 28
#define CENTER_X_MIN 16
#define CENTER_X_MAX 64

#define PATH_SAMPLE_COUNT 14
#define MIN_PATH_SAMPLES 7
#define MIN_SEGMENT_SAMPLES 3
#define BIG_TURN_MIN_ANGLE_DEG 45
#define BIG_TURN_MIN_IMPROVEMENT_PERCENT 25
#define COMPONENT_AREA_SCORE_CAP 75

#define DEFAULT_RED_THRESHOLD 105
#define DEFAULT_GREEN_THRESHOLD 105
#define DEFAULT_BLUE_THRESHOLD 105

typedef struct {
    int x;
    int y;
} path_point_t;

static const char *TAG = "LINE_VISION";
static uint8_t *s_mask;
static uint8_t *s_morph;
static uint16_t *s_queue;
static uint16_t *s_best_component;
static uint16_t *s_parent;
static uint16_t *s_distance;
static size_t s_pixel_capacity;
static volatile uint32_t s_rgb_thresholds =
    (DEFAULT_RED_THRESHOLD << 16) |
    (DEFAULT_GREEN_THRESHOLD << 8) |
    DEFAULT_BLUE_THRESHOLD;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void pixel_rgb(const uint8_t *pixels, size_t index,
                      uint8_t *red, uint8_t *green, uint8_t *blue)
{
    const uint16_t color = ((uint16_t)pixels[index * 2] << 8) |
                           pixels[index * 2 + 1];
    *red = (uint8_t)(((color >> 11) & 0x1f) * 255 / 31);
    *green = (uint8_t)(((color >> 5) & 0x3f) * 255 / 63);
    *blue = (uint8_t)((color & 0x1f) * 255 / 31);
}

static uint8_t pixel_luminance(const uint8_t *pixels, size_t index)
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    pixel_rgb(pixels, index, &red, &green, &blue);
    return (uint8_t)((77 * red + 150 * green + 29 * blue) >> 8);
}

static void set_pixel(uint8_t *pixels, size_t width, size_t height,
                      int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= (int)width || y >= (int)height) return;
    const size_t offset = 2 * ((size_t)y * width + (size_t)x);
    pixels[offset] = color >> 8;
    pixels[offset + 1] = color & 0xff;
}

static void horizontal_line(uint8_t *pixels, size_t width, size_t height,
                            int y, int x0, int x1, uint16_t color)
{
    for (int x = x0; x <= x1; ++x) {
        set_pixel(pixels, width, height, x, y, color);
    }
}

static void vertical_line(uint8_t *pixels, size_t width, size_t height,
                          int x, int y0, int y1, uint16_t color)
{
    for (int y = y0; y <= y1; ++y) {
        set_pixel(pixels, width, height, x, y, color);
    }
}

static void draw_cross(uint8_t *pixels, size_t width, size_t height,
                       int x, int y, uint16_t color)
{
    for (int offset = -2; offset <= 2; ++offset) {
        set_pixel(pixels, width, height, x + offset, y, color);
        set_pixel(pixels, width, height, x, y + offset, color);
    }
}

static void draw_corner_marker(uint8_t *pixels, size_t width, size_t height,
                               int x, int y, uint16_t color)
{
    for (int offset = -4; offset <= 4; ++offset) {
        set_pixel(pixels, width, height, x + offset, y, color);
        set_pixel(pixels, width, height, x, y + offset, color);
    }
}

static void draw_line(uint8_t *pixels, size_t width, size_t height,
                      int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
        set_pixel(pixels, width, height, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        const int twice_error = 2 * error;
        if (twice_error >= dy) {
            error += dy;
            x0 += sx;
        }
        if (twice_error <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

static int histogram_percentile(const uint32_t histogram[256],
                                uint32_t count, int percentile)
{
    const uint32_t target = count * (uint32_t)percentile / 100;
    uint32_t accumulated = 0;
    for (int value = 0; value < 256; ++value) {
        accumulated += histogram[value];
        if (accumulated >= target) return value;
    }
    return 255;
}

static void morph_dilate_3x3(const uint8_t *source, uint8_t *destination,
                             size_t width, size_t height)
{
    memset(destination, 0, width * height);
    for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
        for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
            bool selected = false;
            for (int dy = -1; dy <= 1 && !selected; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx >= ROI_X_MIN && nx < ROI_X_MAX &&
                        ny >= ROI_Y_MIN && ny < ROI_Y_MAX &&
                        source[(size_t)ny * width + nx] != 0) {
                        selected = true;
                        break;
                    }
                }
            }
            destination[(size_t)y * width + x] = selected;
        }
    }
}

static void morph_erode_3x3(const uint8_t *source, uint8_t *destination,
                            size_t width, size_t height)
{
    memset(destination, 0, width * height);
    for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
        for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
            bool keep = true;
            for (int dy = -1; dy <= 1 && keep; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < ROI_X_MIN || nx >= ROI_X_MAX ||
                        ny < ROI_Y_MIN || ny >= ROI_Y_MAX ||
                        source[(size_t)ny * width + nx] == 0) {
                        keep = false;
                        break;
                    }
                }
            }
            destination[(size_t)y * width + x] = keep;
        }
    }
}

static void morph_close(size_t width, size_t height)
{
    /* Closing fills tiny gaps without erasing a one-pixel far-field line. */
    morph_dilate_3x3(s_mask, s_morph, width, height);
    morph_erode_3x3(s_morph, s_mask, width, height);
}

static void segment_vector(const path_point_t *points, int first, int last,
                           int *dx, int *dy)
{
    const int endpoint_count = last - first >= 3 ? 2 : 1;
    int start_x = 0;
    int start_y = 0;
    int end_x = 0;
    int end_y = 0;
    for (int index = 0; index < endpoint_count; ++index) {
        start_x += points[first + index].x;
        start_y += points[first + index].y;
        end_x += points[last - index].x;
        end_y += points[last - index].y;
    }
    *dx = end_x - start_x;
    *dy = end_y - start_y;
}

static int64_t segment_residual(const path_point_t *points,
                                int first, int last)
{
    int dx;
    int dy;
    segment_vector(points, first, last, &dx, &dy);
    const int64_t length_squared = (int64_t)dx * dx + (int64_t)dy * dy;
    if (length_squared == 0) return INT64_MAX / 4;
    int64_t residual = 0;
    const int x0 = points[first].x;
    const int y0 = points[first].y;
    for (int index = first; index <= last; ++index) {
        const int64_t cross =
            (int64_t)dx * (points[index].y - y0) -
            (int64_t)dy * (points[index].x - x0);
        residual += cross * cross / length_squared;
    }
    return residual;
}

static int find_near_anchor(size_t width, size_t best_pixel_count,
                            int best_min_y)
{
    int64_t x_sum = 0;
    int count = 0;
    for (size_t index = 0; index < best_pixel_count; ++index) {
        const int pixel = s_best_component[index];
        const int y = pixel / (int)width;
        if (y <= best_min_y + 2) {
            x_sum += pixel % (int)width;
            count++;
        }
    }
    if (count == 0) return s_best_component[0];
    const int target_x = (int)(x_sum / count);
    int best_pixel = s_best_component[0];
    int best_distance = INT_MAX;
    for (size_t index = 0; index < best_pixel_count; ++index) {
        const int pixel = s_best_component[index];
        const int y = pixel / (int)width;
        if (y > best_min_y + 2) continue;
        const int distance = abs(pixel % (int)width - target_x) +
                             2 * (y - best_min_y);
        if (distance < best_distance) {
            best_distance = distance;
            best_pixel = pixel;
        }
    }
    return best_pixel;
}

static int trace_component_path(size_t width, size_t height,
                                size_t best_pixel_count, int best_min_y,
                                path_point_t samples[PATH_SAMPLE_COUNT])
{
    memset(s_mask, 0, width * height);
    for (size_t index = 0; index < best_pixel_count; ++index) {
        s_mask[s_best_component[index]] = 1;
    }
    memset(s_parent, 0xff, width * height * sizeof(s_parent[0]));
    memset(s_distance, 0, width * height * sizeof(s_distance[0]));

    const int anchor = find_near_anchor(width, best_pixel_count, best_min_y);
    size_t head = 0;
    size_t tail = 0;
    s_queue[tail++] = (uint16_t)anchor;
    s_parent[anchor] = (uint16_t)anchor;
    int farthest = anchor;
    static const int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (head < tail) {
        const int pixel = s_queue[head++];
        const int x = pixel % (int)width;
        const int y = pixel / (int)width;
        if (s_distance[pixel] > s_distance[farthest] ||
            (s_distance[pixel] == s_distance[farthest] &&
             y > farthest / (int)width)) {
            farthest = pixel;
        }
        for (size_t direction = 0;
             direction < sizeof(dx) / sizeof(dx[0]); ++direction) {
            const int nx = x + dx[direction];
            const int ny = y + dy[direction];
            if (nx < ROI_X_MIN || nx >= ROI_X_MAX ||
                ny < ROI_Y_MIN || ny >= ROI_Y_MAX) continue;
            const int neighbour = ny * (int)width + nx;
            if (s_mask[neighbour] == 1 &&
                s_parent[neighbour] == UINT16_MAX) {
                s_parent[neighbour] = (uint16_t)pixel;
                s_distance[neighbour] = s_distance[pixel] + 1;
                s_queue[tail++] = (uint16_t)neighbour;
            }
        }
    }

    size_t path_length = 0;
    int pixel = farthest;
    while (path_length < s_pixel_capacity) {
        s_queue[path_length++] = (uint16_t)pixel;
        if (pixel == anchor) break;
        pixel = s_parent[pixel];
        if (pixel == UINT16_MAX) return 0;
    }
    for (size_t left = 0, right = path_length - 1; left < right;
         ++left, --right) {
        const uint16_t temporary = s_queue[left];
        s_queue[left] = s_queue[right];
        s_queue[right] = temporary;
    }

    const int sample_count = path_length < PATH_SAMPLE_COUNT
                                 ? (int)path_length
                                 : PATH_SAMPLE_COUNT;
    for (int sample = 0; sample < sample_count; ++sample) {
        const size_t first = (size_t)sample * path_length / sample_count;
        size_t last = (size_t)(sample + 1) * path_length / sample_count;
        if (last <= first) last = first + 1;
        int64_t x_sum = 0;
        int64_t y_sum = 0;
        for (size_t index = first; index < last; ++index) {
            x_sum += s_queue[index] % width;
            y_sum += s_queue[index] / width;
        }
        samples[sample].x = (int)(x_sum / (last - first));
        samples[sample].y = (int)(y_sum / (last - first));
    }
    return sample_count;
}

static void detect_corner(const path_point_t *points, int count,
                          line_vision_result_t *result)
{
    if (count < MIN_PATH_SAMPLES) return;
    const int64_t single_residual = segment_residual(points, 0, count - 1);
    int best_split = -1;
    int64_t best_residual = INT64_MAX;
    for (int split = MIN_SEGMENT_SAMPLES - 1;
         split <= count - MIN_SEGMENT_SAMPLES; ++split) {
        const int64_t residual = segment_residual(points, 0, split) +
                                 segment_residual(points, split, count - 1);
        if (residual < best_residual) {
            best_residual = residual;
            best_split = split;
        }
    }
    if (best_split < 0 || single_residual <= 0) return;

    int near_dx;
    int near_dy;
    int far_dx;
    int far_dy;
    segment_vector(points, 0, best_split, &near_dx, &near_dy);
    segment_vector(points, best_split, count - 1, &far_dx, &far_dy);
    const int64_t near_length_squared =
        (int64_t)near_dx * near_dx + (int64_t)near_dy * near_dy;
    const int64_t far_length_squared =
        (int64_t)far_dx * far_dx + (int64_t)far_dy * far_dy;
    if (near_length_squared < 16 || far_length_squared < 16) return;

    const int64_t cross =
        (int64_t)near_dx * far_dy - (int64_t)near_dy * far_dx;
    const int64_t dot =
        (int64_t)near_dx * far_dx + (int64_t)near_dy * far_dy;
    const int angle_deg = (int)lroundf(
        atan2f((float)llabs(cross), (float)dot) * 180.0f / 3.14159265f);
    const int improvement = clamp_int(
        (int)((single_residual - best_residual) * 100 /
              (single_residual > 0 ? single_residual : 1)),
        0, 100);

    result->turn_angle_deg = angle_deg;
    result->turn_direction = cross < 0 ? 1 : (cross > 0 ? -1 : 0);
    result->corner_x = points[best_split].x;
    result->corner_y = points[best_split].y;
    result->turn_confidence = clamp_int(
        (angle_deg - 20) * 2 + improvement / 2, 0, 100);
    result->big_turn = angle_deg >= BIG_TURN_MIN_ANGLE_DEG &&
                       improvement >= BIG_TURN_MIN_IMPROVEMENT_PERCENT &&
                       result->turn_direction != 0;
}

esp_err_t line_vision_init(size_t width, size_t height)
{
    ESP_RETURN_ON_FALSE(width * height <= UINT16_MAX, ESP_ERR_INVALID_SIZE,
                        TAG, "frame too large");
    s_pixel_capacity = width * height;
    s_mask = heap_caps_malloc(s_pixel_capacity, MALLOC_CAP_SPIRAM |
                                                MALLOC_CAP_8BIT);
    s_morph = heap_caps_malloc(s_pixel_capacity, MALLOC_CAP_SPIRAM |
                                                 MALLOC_CAP_8BIT);
    s_queue = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_best_component = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_parent = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_distance = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(s_mask && s_morph && s_queue && s_best_component &&
                            s_parent && s_distance,
                        ESP_ERR_NO_MEM, TAG, "vision buffers unavailable");
    const line_vision_rgb_thresholds_t thresholds =
        line_vision_get_rgb_thresholds();
    ESP_LOGI(TAG,
             "ROI x=%d..%d y=%d..%d RGB<=%u,%u,%u path=%d corner>=%ddeg",
             ROI_X_MIN, ROI_X_MAX - 1, ROI_Y_MIN, ROI_Y_MAX - 1,
             thresholds.red, thresholds.green, thresholds.blue,
             PATH_SAMPLE_COUNT, BIG_TURN_MIN_ANGLE_DEG);
    return ESP_OK;
}

void line_vision_set_rgb_thresholds(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint32_t packed = ((uint32_t)red << 16) |
                            ((uint32_t)green << 8) | blue;
    __atomic_store_n(&s_rgb_thresholds, packed, __ATOMIC_RELAXED);
}

line_vision_rgb_thresholds_t line_vision_get_rgb_thresholds(void)
{
    const uint32_t packed =
        __atomic_load_n(&s_rgb_thresholds, __ATOMIC_RELAXED);
    const line_vision_rgb_thresholds_t thresholds = {
        .red = (uint8_t)(packed >> 16),
        .green = (uint8_t)(packed >> 8),
        .blue = (uint8_t)packed,
    };
    return thresholds;
}

bool line_vision_pixel_selected(size_t pixel_index)
{
    return s_mask && pixel_index < s_pixel_capacity &&
           s_mask[pixel_index] == 1;
}

void line_vision_process(uint8_t *pixels, size_t width, size_t height,
                         line_vision_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->near_x = -1;
    result->far_x = -1;
    result->corner_x = -1;
    result->corner_y = -1;
    if (!pixels || width * height > s_pixel_capacity ||
        width < ROI_X_MAX || height < ROI_Y_MAX) return;

    uint32_t histogram[256] = {0};
    uint32_t sample_count = 0;
    for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
        for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
            histogram[pixel_luminance(pixels, (size_t)y * width + x)]++;
            sample_count++;
        }
    }
    const int dark_level = histogram_percentile(histogram, sample_count, 10);
    const int light_level = histogram_percentile(histogram, sample_count, 90);
    result->contrast = light_level - dark_level;
    const line_vision_rgb_thresholds_t thresholds =
        line_vision_get_rgb_thresholds();
    result->threshold = ((int)thresholds.red + thresholds.green +
                         thresholds.blue) / 3;

    memset(s_mask, 0, width * height);
    for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
        for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
            const size_t index = (size_t)y * width + x;
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            pixel_rgb(pixels, index, &red, &green, &blue);
            if (red <= thresholds.red && green <= thresholds.green &&
                blue <= thresholds.blue) {
                s_mask[index] = 1;
            }
        }
    }
    morph_close(width, height);

    int best_score = 0;
    int best_area = 0;
    int best_min_y = 0;
    int best_max_y = 0;
    int best_near_count = 0;
    int best_center_count = 0;
    size_t best_pixel_count = 0;
    static const int flood_dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int flood_dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    for (int start_y = ROI_Y_MIN; start_y < ROI_Y_MAX; ++start_y) {
        for (int start_x = ROI_X_MIN; start_x < ROI_X_MAX; ++start_x) {
            const size_t start = (size_t)start_y * width + start_x;
            if (s_mask[start] != 1) continue;
            size_t head = 0;
            size_t tail = 0;
            s_queue[tail++] = (uint16_t)start;
            s_mask[start] = 2;
            int area = 0;
            int min_y = start_y;
            int max_y = start_y;
            int near_count = 0;
            int center_count = 0;
            while (head < tail) {
                const size_t index = s_queue[head++];
                const int x = index % width;
                const int y = index / width;
                area++;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                if (y < NEAR_Y_MAX) near_count++;
                if (x >= CENTER_X_MIN && x < CENTER_X_MAX) center_count++;
                for (size_t direction = 0;
                     direction < sizeof(flood_dx) / sizeof(flood_dx[0]);
                     ++direction) {
                    const int nx = x + flood_dx[direction];
                    const int ny = y + flood_dy[direction];
                    if (nx < ROI_X_MIN || nx >= ROI_X_MAX ||
                        ny < ROI_Y_MIN || ny >= ROI_Y_MAX) continue;
                    const size_t neighbour = (size_t)ny * width + nx;
                    if (s_mask[neighbour] == 1) {
                        s_mask[neighbour] = 2;
                        s_queue[tail++] = (uint16_t)neighbour;
                    }
                }
            }
            const int vertical_span = max_y - min_y + 1;
            const int score = area + near_count * 8 +
                              vertical_span * 10 + center_count * 2;
            if (area >= 8 && area < (int)sample_count * 2 / 5 &&
                vertical_span >= 4 && near_count >= 3 &&
                center_count >= 3 && score > best_score) {
                best_score = score;
                best_area = area;
                best_min_y = min_y;
                best_max_y = max_y;
                best_near_count = near_count;
                best_center_count = center_count;
                best_pixel_count = tail;
                memcpy(s_best_component, s_queue,
                       tail * sizeof(s_best_component[0]));
            }
        }
    }

    path_point_t path[PATH_SAMPLE_COUNT] = {0};
    if (best_score > 0) {
        result->path_point_count = trace_component_path(
            width, height, best_pixel_count, best_min_y, path);
        result->vector_point_count = result->path_point_count;
        if (result->path_point_count >= 2) {
            result->near_x = path[0].x;
            result->far_x = path[result->path_point_count - 1].x;
            int near_last = result->path_point_count / 3;
            if (near_last < 2) near_last = 2;
            if (near_last >= result->path_point_count) {
                near_last = result->path_point_count - 1;
            }
            int near_dx;
            int near_dy;
            segment_vector(path, 0, near_last, &near_dx, &near_dy);
            const int image_center = (int)width / 2;
            result->lateral_error = clamp_int(
                (result->near_x - image_center) * 1000 / image_center,
                -1000, 1000);
            result->heading_error = near_dy == 0
                                        ? (near_dx >= 0 ? 1000 : -1000)
                                        : clamp_int(near_dx * 1000 /
                                                        abs(near_dy),
                                                    -1000, 1000);
            result->steering_error = clamp_int(
                (65 * result->lateral_error + 35 * result->heading_error) /
                    100,
                -1000, 1000);
            detect_corner(path, result->path_point_count, result);
        }

        const int span = best_max_y - best_min_y + 1;
        const int span_confidence = clamp_int(
            span * 35 / (ROI_Y_MAX - ROI_Y_MIN), 0, 35);
        const int area_confidence = clamp_int(
            best_area * 20 / COMPONENT_AREA_SCORE_CAP, 0, 20);
        const int near_confidence = best_near_count >= 5 ? 20 : 10;
        const int center_confidence = clamp_int(
            best_center_count * 10 / 10, 0, 10);
        const int path_confidence = clamp_int(
            result->path_point_count * 15 / PATH_SAMPLE_COUNT, 0, 15);
        result->confidence = clamp_int(
            span_confidence + area_confidence + near_confidence +
                center_confidence + path_confidence,
            0, 100);
        result->component_area = best_area;
        result->found = result->path_point_count >= 3 &&
                        result->confidence >= 25;
    }

    const uint16_t track_color = 0x07e0;
    const uint16_t roi_color = 0xffff;
    const uint16_t target_color = 0xf800;
    const uint16_t path_color = 0xf81f;
    const uint16_t near_fit_color = 0xffe0;
    const uint16_t far_fit_color = 0xfd20;
    const uint16_t corner_color = 0x001f;
    const uint16_t trigger_color = 0x07ff;
    memset(s_mask, 0, width * height);
    if (result->found) {
        for (size_t index = 0; index < best_pixel_count; ++index) {
            const size_t pixel_index = s_best_component[index];
            s_mask[pixel_index] = 1;
            set_pixel(pixels, width, height,
                      pixel_index % width, pixel_index / width, track_color);
        }
        for (int index = 0; index < result->path_point_count; ++index) {
            draw_cross(pixels, width, height,
                       path[index].x, path[index].y, path_color);
            if (index > 0) {
                draw_line(pixels, width, height,
                          path[index - 1].x, path[index - 1].y,
                          path[index].x, path[index].y, path_color);
            }
        }
        if (result->big_turn) {
            draw_line(pixels, width, height,
                      path[0].x, path[0].y,
                      result->corner_x, result->corner_y, near_fit_color);
            draw_line(pixels, width, height,
                      result->corner_x, result->corner_y,
                      path[result->path_point_count - 1].x,
                      path[result->path_point_count - 1].y,
                      far_fit_color);
        }
    }
    horizontal_line(pixels, width, height, ROI_Y_MIN,
                    ROI_X_MIN, ROI_X_MAX - 1, roi_color);
    horizontal_line(pixels, width, height, ROI_Y_MAX - 1,
                    ROI_X_MIN, ROI_X_MAX - 1, roi_color);
    vertical_line(pixels, width, height, (int)width / 2,
                  ROI_Y_MIN, ROI_Y_MAX - 1, target_color);
    horizontal_line(pixels, width, height, LINE_VISION_TURN_TRIGGER_Y,
                    ROI_X_MIN, ROI_X_MAX - 1, trigger_color);
    if (result->found && result->big_turn) {
        draw_corner_marker(pixels, width, height,
                           result->corner_x, result->corner_y, corner_color);
    }
}
