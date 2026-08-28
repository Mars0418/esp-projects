#include "line_vision.h"

#include <limits.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define ROI_X_MIN  3
#define ROI_X_MAX  77
/* Camera/TFT mounting flips the visible vertical direction in the raw frame. */
#define ROI_Y_MIN  12
#define ROI_Y_MAX  58
#define FAR_Y_MIN  41
#define FAR_Y_MAX  55
#define NEAR_Y_MIN 13
#define NEAR_Y_MAX 28
#define VECTOR_Y_MIN NEAR_Y_MIN
#define VECTOR_Y_MAX 25
#define VECTOR_BAND_COUNT 6
#define LATERAL_WEIGHT 65
#define HEADING_WEIGHT 35
#define CENTER_X_MIN 16
#define CENTER_X_MAX 64
#define COMPONENT_AREA_SCORE_CAP 75
#define DEFAULT_RED_THRESHOLD 105
#define DEFAULT_GREEN_THRESHOLD 105
#define DEFAULT_BLUE_THRESHOLD 105

static const char *TAG = "LINE_VISION";
static uint8_t *s_mask;
static uint8_t *s_morph;
static uint16_t *s_queue;
static uint16_t *s_best_component;
static size_t s_pixel_capacity;
static volatile uint32_t s_rgb_thresholds =
    (DEFAULT_RED_THRESHOLD << 16) |
    (DEFAULT_GREEN_THRESHOLD << 8) |
    DEFAULT_BLUE_THRESHOLD;

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

static void draw_cross(uint8_t *pixels, size_t width, size_t height,
                       int x, int y, uint16_t color)
{
    for (int offset = -2; offset <= 2; ++offset) {
        set_pixel(pixels, width, height, x + offset, y, color);
        set_pixel(pixels, width, height, x, y + offset, color);
    }
}

static void draw_fitted_line(uint8_t *pixels, size_t width, size_t height,
                             int x0, int y0, int x1, int y1,
                             uint16_t color)
{
    const int dy = y1 - y0;
    if (dy <= 0) {
        return;
    }
    for (int y = y0; y <= y1; ++y) {
        const int x = x0 + (x1 - x0) * (y - y0) / dy;
        set_pixel(pixels, width, height, x - 1, y, color);
        set_pixel(pixels, width, height, x, y, color);
        set_pixel(pixels, width, height, x + 1, y, color);
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
            if (keep) {
                destination[(size_t)y * width + x] = 1;
            }
        }
    }
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
            if (selected) {
                destination[(size_t)y * width + x] = 1;
            }
        }
    }
}

static void morph_open_close(size_t width, size_t height)
{
    morph_erode_3x3(s_mask, s_morph, width, height);
    morph_dilate_3x3(s_morph, s_mask, width, height);
    morph_dilate_3x3(s_mask, s_morph, width, height);
    morph_erode_3x3(s_morph, s_mask, width, height);
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
    ESP_RETURN_ON_FALSE(s_mask != NULL && s_morph != NULL &&
                            s_queue != NULL &&
                            s_best_component != NULL,
                        ESP_ERR_NO_MEM,
                        TAG, "vision buffers unavailable");
    const line_vision_rgb_thresholds_t thresholds =
        line_vision_get_rgb_thresholds();
    ESP_LOGI(TAG, "ROI x=%d..%d y=%d..%d; RGB<=%u,%u,%u; morph=open+close",
             ROI_X_MIN, ROI_X_MAX - 1, ROI_Y_MIN, ROI_Y_MAX - 1,
             thresholds.red, thresholds.green, thresholds.blue);
    return ESP_OK;
}

void line_vision_set_rgb_thresholds(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint32_t packed = ((uint32_t)red << 16) |
                            ((uint32_t)green << 8) |
                            blue;
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
    return s_mask != NULL && pixel_index < s_pixel_capacity &&
           s_mask[pixel_index] == 1;
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
    morph_open_close(width, height);

    int best_score = 0;
    int best_area = 0;
    int best_min_y = 0;
    int best_max_y = 0;
    int best_near_count = 0;
    int best_far_count = 0;
    int best_center_count = 0;
    int64_t best_near_weighted_sum = 0;
    int64_t best_near_weight_total = 0;
    int64_t best_far_sum = 0;
    size_t best_pixel_count = 0;
    int vector_x[VECTOR_BAND_COUNT];
    int vector_y[VECTOR_BAND_COUNT];
    for (int band = 0; band < VECTOR_BAND_COUNT; ++band) {
        vector_x[band] = -1;
        vector_y[band] = -1;
    }

    /* Morphological closing handles small gaps; use strict 8-connectivity. */
    static const int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
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
            int center_count = 0;
            int64_t near_weighted_sum = 0;
            int64_t near_weight_total = 0;
            int64_t far_sum = 0;

            while (head < tail) {
                const size_t index = s_queue[head++];
                const int x = index % width;
                const int y = index / width;
                area++;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                if (y >= NEAR_Y_MIN && y < NEAR_Y_MAX) {
                    const int weight = NEAR_Y_MAX - y;
                    near_weighted_sum += (int64_t)x * weight;
                    near_weight_total += weight;
                    near_count++;
                }
                if (y >= FAR_Y_MIN && y < FAR_Y_MAX) {
                    far_sum += x;
                    far_count++;
                }
                if (x >= CENTER_X_MIN && x < CENTER_X_MAX) {
                    center_count++;
                }
                for (size_t direction = 0;
                     direction < sizeof(dx) / sizeof(dx[0]); ++direction) {
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
            const int score = area + near_count * 6 +
                              vertical_span * 10 + center_count * 2;
            if (area >= 9 && area < (int)sample_count * 2 / 5 &&
                vertical_span >= 4 && near_count >= 5 &&
                center_count >= 5 && score > best_score) {
                best_score = score;
                best_area = area;
                best_min_y = min_y;
                best_max_y = max_y;
                best_near_count = near_count;
                best_far_count = far_count;
                best_center_count = center_count;
                best_near_weighted_sum = near_weighted_sum;
                best_near_weight_total = near_weight_total;
                best_far_sum = far_sum;
                best_pixel_count = tail;
                memcpy(s_best_component, s_queue,
                       best_pixel_count * sizeof(s_best_component[0]));
            }
        }
    }

    if (best_score > 0) {
        int64_t band_x_sum[VECTOR_BAND_COUNT] = {0};
        int64_t band_y_sum[VECTOR_BAND_COUNT] = {0};
        int band_count[VECTOR_BAND_COUNT] = {0};
        for (size_t index = 0; index < best_pixel_count; ++index) {
            const size_t pixel_index = s_best_component[index];
            const int x = pixel_index % width;
            const int y = pixel_index / width;
            if (y < VECTOR_Y_MIN || y >= VECTOR_Y_MAX) {
                continue;
            }
            const int band = (y - VECTOR_Y_MIN) * VECTOR_BAND_COUNT /
                             (VECTOR_Y_MAX - VECTOR_Y_MIN);
            band_x_sum[band] += x;
            band_y_sum[band] += y;
            band_count[band]++;
        }

        int closest_band = -1;
        int furthest_band = -1;
        int64_t sum_x = 0;
        int64_t sum_y = 0;
        for (int band = 0; band < VECTOR_BAND_COUNT; ++band) {
            if (band_count[band] == 0) {
                continue;
            }
            vector_x[band] = (int)(band_x_sum[band] / band_count[band]);
            vector_y[band] = (int)(band_y_sum[band] / band_count[band]);
            if (closest_band < 0) {
                closest_band = band;
            }
            furthest_band = band;
            sum_x += vector_x[band];
            sum_y += vector_y[band];
            result->vector_point_count++;
        }

        /* Fallback values for frames that do not contain enough fit points. */
        if (closest_band >= 0) {
            result->near_x = vector_x[closest_band];
            result->far_x = vector_x[furthest_band];
        } else {
            if (best_near_weight_total > 0) {
                result->near_x =
                    (int)(best_near_weighted_sum / best_near_weight_total);
            }
            if (best_far_count > 0) {
                result->far_x = (int)(best_far_sum / best_far_count);
            }
        }
        /*
         * Least-squares axis through six tightly spaced points.  Its position
         * at VECTOR_Y_MIN is the cross-track target; its slope is the heading
         * target.  Driving both to zero overlays the track axis on the image
         * centre line instead of chasing any individual centroid.
         */
        if (result->vector_point_count >= 2) {
            const int mean_x = (int)(sum_x / result->vector_point_count);
            const int mean_y = (int)(sum_y / result->vector_point_count);
            int64_t covariance = 0;
            int64_t y_variance = 0;
            for (int band = 0; band < VECTOR_BAND_COUNT; ++band) {
                if (vector_x[band] < 0) {
                    continue;
                }
                const int dy = vector_y[band] - mean_y;
                covariance += (int64_t)dy * (vector_x[band] - mean_x);
                y_variance += (int64_t)dy * dy;
            }
            if (y_variance > 0) {
                result->heading_error = clamp_int(
                    (int)(covariance * 1000 / y_variance), -1000, 1000);
                result->near_x = clamp_int(
                    mean_x + (int)(covariance *
                                   (VECTOR_Y_MIN - mean_y) / y_variance),
                    ROI_X_MIN, ROI_X_MAX - 1);
                result->far_x = clamp_int(
                    mean_x + (int)(covariance *
                                   (VECTOR_Y_MAX - 1 - mean_y) /
                                   y_variance),
                    ROI_X_MIN, ROI_X_MAX - 1);
            }
        }
        if (result->near_x < 0) result->near_x = result->far_x;
        if (result->far_x < 0) result->far_x = result->near_x;
        const int image_center = (int)width / 2;
        const int near_offset = result->near_x - image_center;
        result->lateral_error = clamp_int(
            near_offset * 1000 / image_center, -1000, 1000);
        /*
         * The fitted near intercept aligns the track with the image centre;
         * the fitted tangent aligns the chassis with the same centre axis.
         * Only the closest 24 raw rows participate, avoiding corner cutting.
         */
        result->steering_error = clamp_int(
            (LATERAL_WEIGHT * result->lateral_error +
             HEADING_WEIGHT * result->heading_error) / 100,
            -1000, 1000);
        const int span = best_max_y - best_min_y + 1;
        const int roi_confidence = clamp_int(
            span * 30 / (ROI_Y_MAX - ROI_Y_MIN), 0, 30);
        const int band_confidence =
            (best_near_count >= 5 ? 20 : 0) +
            (best_far_count >= 5 ? 10 : 0);
        const int area_confidence = clamp_int(
            best_area * 15 / COMPONENT_AREA_SCORE_CAP, 0, 15);
        const int center_confidence = clamp_int(
            best_center_count * 10 / 10, 0, 10);
        const int vector_confidence = clamp_int(
            result->vector_point_count * 25 / VECTOR_BAND_COUNT, 0, 25);
        const int base_confidence = roi_confidence + band_confidence +
                                    area_confidence + center_confidence +
                                    vector_confidence;

        const int edge_distance = result->near_x - ROI_X_MIN <
                                          ROI_X_MAX - 1 - result->near_x
                                      ? result->near_x - ROI_X_MIN
                                      : ROI_X_MAX - 1 - result->near_x;
        const int edge_scale = clamp_int((edge_distance - 2) * 100 / 8,
                                         0, 100);
        result->confidence = clamp_int(
            base_confidence * edge_scale / 100, 0, 100);
        result->component_area = best_area;
        result->found = result->confidence >= 25;
    }

    const uint16_t track_color = 0x07e0;
    const uint16_t roi_color = 0xffff;
    const uint16_t band_color = 0x07ff;
    const uint16_t target_color = 0xf800;
    const uint16_t detected_color = 0xffe0;
    const uint16_t vector_color = 0xf81f;
    memset(s_mask, 0, width * height);
    if (result->found) {
        for (size_t index = 0; index < best_pixel_count; ++index) {
            const size_t pixel_index = s_best_component[index];
            s_mask[pixel_index] = 1;
            set_pixel(pixels, width, height,
                      pixel_index % width, pixel_index / width,
                      track_color);
        }
    }
    horizontal_line(pixels, width, height, ROI_Y_MIN, ROI_X_MIN,
                    ROI_X_MAX - 1, roi_color);
    horizontal_line(pixels, width, height, ROI_Y_MAX - 1, ROI_X_MIN,
                    ROI_X_MAX - 1, roi_color);
    horizontal_line(pixels, width, height, VECTOR_Y_MIN, ROI_X_MIN,
                    ROI_X_MAX - 1, band_color);
    horizontal_line(pixels, width, height, VECTOR_Y_MAX - 1, ROI_X_MIN,
                    ROI_X_MAX - 1, band_color);
    vertical_line(pixels, width, height, (int)width / 2,
                  ROI_Y_MIN, ROI_Y_MAX - 1, target_color);
    if (result->found) {
        for (int band = 0; band < VECTOR_BAND_COUNT; ++band) {
            if (vector_x[band] >= 0) {
                draw_cross(pixels, width, height, vector_x[band],
                           vector_y[band], vector_color);
            }
        }
        draw_fitted_line(pixels, width, height,
                         result->near_x, VECTOR_Y_MIN,
                         result->far_x, VECTOR_Y_MAX - 1,
                         detected_color);
    }
}
