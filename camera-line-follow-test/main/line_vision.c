#include "line_vision.h"

#include <limits.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define ROI_X_MIN  6
#define ROI_X_MAX  154
#define ROI_Y_MIN  5
#define ROI_Y_MAX  80
#define FAR_Y_MIN  10
#define FAR_Y_MAX  38
#define NEAR_Y_MIN 48
#define NEAR_Y_MAX 78

static const char *TAG = "LINE_VISION";
static uint8_t *s_mask;
static uint16_t *s_queue;
static size_t s_pixel_capacity;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t pixel_luminance(const uint8_t *pixels, size_t index)
{
    const uint16_t color = ((uint16_t)pixels[index * 2] << 8) |
                           pixels[index * 2 + 1];
    const int red = ((color >> 11) & 0x1f) * 255 / 31;
    const int green = ((color >> 5) & 0x3f) * 255 / 63;
    const int blue = (color & 0x1f) * 255 / 31;
    return (uint8_t)((77 * red + 150 * green + 29 * blue) >> 8);
}

static void set_pixel(uint8_t *pixels, size_t width, size_t height,
                      int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= (int)width || y >= (int)height) {
        return;
    }
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

static int histogram_percentile(const uint32_t histogram[256],
                                uint32_t count, int percentile)
{
    const uint32_t target = count * (uint32_t)percentile / 100;
    uint32_t accumulated = 0;
    for (int value = 0; value < 256; ++value) {
        accumulated += histogram[value];
        if (accumulated >= target) {
            return value;
        }
    }
    return 255;
}

static int otsu_threshold(const uint32_t histogram[256], uint32_t count)
{
    uint64_t total_sum = 0;
    for (int value = 0; value < 256; ++value) {
        total_sum += (uint64_t)value * histogram[value];
    }

    uint32_t background_count = 0;
    uint64_t background_sum = 0;
    double best_score = 0.0;
    int best_threshold = 0;
    for (int value = 0; value < 255; ++value) {
        background_count += histogram[value];
        background_sum += (uint64_t)value * histogram[value];
        if (background_count == 0 || background_count == count) {
            continue;
        }
        const uint32_t foreground_count = count - background_count;
        const int64_t difference =
            (int64_t)background_sum * foreground_count -
            (int64_t)(total_sum - background_sum) * background_count;
        const double score =
            (double)difference * (double)difference /
            ((double)background_count * foreground_count);
        if (score > best_score) {
            best_score = score;
            best_threshold = value;
        }
    }
    return best_threshold;
}

esp_err_t line_vision_init(size_t width, size_t height)
{
    ESP_RETURN_ON_FALSE(width * height <= UINT16_MAX, ESP_ERR_INVALID_SIZE,
                        TAG, "frame too large");
    s_pixel_capacity = width * height;
    s_mask = heap_caps_malloc(s_pixel_capacity, MALLOC_CAP_SPIRAM |
                                                MALLOC_CAP_8BIT);
    s_queue = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(s_mask != NULL && s_queue != NULL, ESP_ERR_NO_MEM,
                        TAG, "vision buffers unavailable");
    ESP_LOGI(TAG, "ROI x=%d..%d y=%d..%d; lower image ignored",
             ROI_X_MIN, ROI_X_MAX - 1, ROI_Y_MIN, ROI_Y_MAX - 1);
    return ESP_OK;
}

void line_vision_process(uint8_t *pixels, size_t width, size_t height,
                         line_vision_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->near_x = -1;
    result->far_x = -1;
    if (pixels == NULL || width * height > s_pixel_capacity ||
        width < ROI_X_MAX || height < ROI_Y_MAX) {
        return;
    }

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
    int threshold = dark_level;
    if (result->contrast >= 8) {
        threshold = otsu_threshold(histogram, sample_count);
        threshold = clamp_int(threshold, dark_level + 4,
                              dark_level + result->contrast * 3 / 5);
    }
    result->threshold = threshold;

    memset(s_mask, 0, width * height);
    if (result->contrast >= 24) {
        for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
            for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
                const size_t index = (size_t)y * width + x;
                if (pixel_luminance(pixels, index) <= threshold) {
                    s_mask[index] = 1;
                }
            }
        }
    }

    int best_score = 0;
    int best_area = 0;
    int best_min_y = 0;
    int best_max_y = 0;
    int best_near_count = 0;
    int best_far_count = 0;
    int64_t best_near_sum = 0;
    int64_t best_far_sum = 0;

    static const int dx[] = {1, -1, 0, 0};
    static const int dy[] = {0, 0, 1, -1};
    for (int start_y = ROI_Y_MIN; start_y < ROI_Y_MAX; ++start_y) {
        for (int start_x = ROI_X_MIN; start_x < ROI_X_MAX; ++start_x) {
            const size_t start = (size_t)start_y * width + start_x;
            if (s_mask[start] != 1) {
                continue;
            }
            size_t head = 0;
            size_t tail = 0;
            s_queue[tail++] = (uint16_t)start;
            s_mask[start] = 2;
            int area = 0;
            int min_y = start_y;
            int max_y = start_y;
            int near_count = 0;
            int far_count = 0;
            int64_t near_sum = 0;
            int64_t far_sum = 0;

            while (head < tail) {
                const size_t index = s_queue[head++];
                const int x = index % width;
                const int y = index / width;
                area++;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                if (y >= NEAR_Y_MIN && y < NEAR_Y_MAX) {
                    near_sum += x;
                    near_count++;
                }
                if (y >= FAR_Y_MIN && y < FAR_Y_MAX) {
                    far_sum += x;
                    far_count++;
                }
                for (int direction = 0; direction < 4; ++direction) {
                    const int nx = x + dx[direction];
                    const int ny = y + dy[direction];
                    if (nx < ROI_X_MIN || nx >= ROI_X_MAX ||
                        ny < ROI_Y_MIN || ny >= ROI_Y_MAX) {
                        continue;
                    }
                    const size_t neighbour = (size_t)ny * width + nx;
                    if (s_mask[neighbour] == 1) {
                        s_mask[neighbour] = 2;
                        s_queue[tail++] = (uint16_t)neighbour;
                    }
                }
            }

            const int vertical_span = max_y - min_y + 1;
            const int score = area + vertical_span * 8;
            if (area >= 35 && area < (int)sample_count * 2 / 5 &&
                vertical_span >= 8 &&
                (near_count >= 5 || far_count >= 5) && score > best_score) {
                best_score = score;
                best_area = area;
                best_min_y = min_y;
                best_max_y = max_y;
                best_near_count = near_count;
                best_far_count = far_count;
                best_near_sum = near_sum;
                best_far_sum = far_sum;
            }
        }
    }

    if (best_score > 0) {
        if (best_near_count > 0) {
            result->near_x = (int)(best_near_sum / best_near_count);
        }
        if (best_far_count > 0) {
            result->far_x = (int)(best_far_sum / best_far_count);
        }
        if (result->near_x < 0) result->near_x = result->far_x;
        if (result->far_x < 0) result->far_x = result->near_x;
        const int image_center = (int)width / 2;
        const int near_offset = result->near_x - image_center;
        const int far_offset = result->far_x - image_center;
        result->steering_error = clamp_int(
            (2 * near_offset + far_offset) * 1000 /
                (3 * image_center),
            -1000, 1000);
        const int span = best_max_y - best_min_y + 1;
        result->confidence = clamp_int(best_area * 60 / 500, 0, 60) +
                             clamp_int(span * 40 / 50, 0, 40);
        result->component_area = best_area;
        result->found = result->confidence >= 25;
    }

    const uint16_t roi_color = 0x07e0;
    const uint16_t band_color = 0x07ff;
    const uint16_t target_color = 0xf800;
    const uint16_t detected_color = 0xffe0;
    horizontal_line(pixels, width, height, ROI_Y_MIN, ROI_X_MIN,
                    ROI_X_MAX - 1, roi_color);
    horizontal_line(pixels, width, height, ROI_Y_MAX - 1, ROI_X_MIN,
                    ROI_X_MAX - 1, roi_color);
    horizontal_line(pixels, width, height, FAR_Y_MIN, ROI_X_MIN,
                    ROI_X_MAX - 1, band_color);
    horizontal_line(pixels, width, height, FAR_Y_MAX - 1, ROI_X_MIN,
                    ROI_X_MAX - 1, band_color);
    horizontal_line(pixels, width, height, NEAR_Y_MIN, ROI_X_MIN,
                    ROI_X_MAX - 1, band_color);
    horizontal_line(pixels, width, height, NEAR_Y_MAX - 1, ROI_X_MIN,
                    ROI_X_MAX - 1, band_color);
    vertical_line(pixels, width, height, (int)width / 2,
                  ROI_Y_MIN, ROI_Y_MAX - 1, target_color);
    if (result->found) {
        vertical_line(pixels, width, height, result->far_x,
                      FAR_Y_MIN, FAR_Y_MAX - 1, detected_color);
        vertical_line(pixels, width, height, result->near_x,
                      NEAR_Y_MIN, NEAR_Y_MAX - 1, detected_color);
    }
}
