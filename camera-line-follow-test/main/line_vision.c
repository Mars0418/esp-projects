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
/* The mounted camera maps increasing raw y to farther points on the track. */
#define ROI_Y_MIN 0
#define ROI_Y_MAX 52
#define NEAR_Y_MAX 28

/* Raw y grows away from the vehicle.  The entry gate is the central half of
 * the nearest 12 rows; the wider hold gate adds hysteresis while tracking. */
#define FOOT_GATE_Y_MAX 12
#define FOOT_MIN_PIXELS 3
#define FOOT_MIN_PATH_PIXELS 12

#define PATH_SAMPLE_COUNT 20
#define MIN_CORNER_PATH_SAMPLES 7
#define CORNER_MIN_SEGMENT_SAMPLES 3
#define CORNER_LOCAL_RADIUS 3
#define CORNER_MIN_VECTOR_LENGTH_SQUARED 9
#define CORNER_MIN_ANGLE_DEG 25
#define CORNER_MIN_IMPROVEMENT_PERCENT 15
#define STRAIGHT_CORRIDOR_HALF_WIDTH 9
#define STRAIGHT_CORRIDOR_MIN_VECTOR_SQUARED 25
#define MIN_COMPONENT_AREA 8
#define MIN_COMPONENT_EXTENT 5
#define MIN_COMPONENT_NEAR_PIXELS 2
#define MAX_COMPONENT_FILL_PERCENT 35
#define ANCHOR_Y_TOLERANCE 3
#define NEAR_LOOKAHEAD_PATH_PIXELS 14
#define FAR_LOOKAHEAD_PATH_PIXELS 40
#define FAR_PREVIEW_WEIGHT_MIN 25
#define FAR_PREVIEW_WEIGHT_MAX 65
#define MIN_PATH_PIXELS 9
#define MIN_CONTRAST 12
#define LOCAL_CONTRAST_RADIUS 4
#define LOCAL_DARKNESS_MIN 18
#define LOCAL_LUMA_MAX 165

#define DEFAULT_RED_THRESHOLD 125
#define DEFAULT_GREEN_THRESHOLD 125
#define DEFAULT_BLUE_THRESHOLD 125
#define LOW_CONTRAST_LUMA_THRESHOLD 110
#define STEERING_TARGET_X_OFFSET_PX 0

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
static uint8_t *s_luma;
static uint32_t *s_luma_integral;
static size_t s_pixel_capacity;
static size_t s_integral_capacity;
static bool s_previous_path_valid;
static int s_previous_near_x;
static volatile bool s_corridor_lock_requested;
static bool s_corridor_locked;
static int s_corridor_anchor_x;
static int s_corridor_anchor_y;
static int s_corridor_dx;
static int s_corridor_dy;
static bool s_corridor_valid;
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

static int steering_target_x(size_t width)
{
    return clamp_int((int)width / 2 + STEERING_TARGET_X_OFFSET_PX,
                     ROI_X_MIN, ROI_X_MAX - 1);
}

static int foot_entry_x_min(size_t width)
{
    return clamp_int((int)width / 4, ROI_X_MIN, ROI_X_MAX - 1);
}

static int foot_entry_x_max(size_t width)
{
    return clamp_int((int)width * 3 / 4, ROI_X_MIN + 1, ROI_X_MAX);
}

static int foot_hold_x_min(size_t width)
{
    return clamp_int((int)width / 6, ROI_X_MIN, ROI_X_MAX - 1);
}

static int foot_hold_x_max(size_t width)
{
    return clamp_int((int)width * 5 / 6, ROI_X_MIN + 1, ROI_X_MAX);
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

static void *allocate_hot_buffer(size_t bytes)
{
    void *buffer = heap_caps_malloc(bytes,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (buffer == NULL) {
        buffer = heap_caps_malloc(bytes,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    return buffer;
}

static void build_luma_integral(size_t width, size_t height)
{
    const size_t stride = width + 1;
    memset(s_luma_integral, 0,
           stride * (height + 1) * sizeof(s_luma_integral[0]));
    for (size_t y = 0; y < height; ++y) {
        uint32_t row_sum = 0;
        for (size_t x = 0; x < width; ++x) {
            row_sum += s_luma[y * width + x];
            s_luma_integral[(y + 1) * stride + x + 1] =
                s_luma_integral[y * stride + x + 1] + row_sum;
        }
    }
}

static int local_mean_luma(size_t width, int x, int y)
{
    const int x0 = clamp_int(x - LOCAL_CONTRAST_RADIUS,
                             ROI_X_MIN, ROI_X_MAX);
    const int x1 = clamp_int(x + LOCAL_CONTRAST_RADIUS + 1,
                             ROI_X_MIN, ROI_X_MAX);
    const int y0 = clamp_int(y - LOCAL_CONTRAST_RADIUS,
                             ROI_Y_MIN, ROI_Y_MAX);
    const int y1 = clamp_int(y + LOCAL_CONTRAST_RADIUS + 1,
                             ROI_Y_MIN, ROI_Y_MAX);
    const size_t stride = width + 1;
    const uint32_t sum =
        s_luma_integral[(size_t)y1 * stride + x1] -
        s_luma_integral[(size_t)y0 * stride + x1] -
        s_luma_integral[(size_t)y1 * stride + x0] +
        s_luma_integral[(size_t)y0 * stride + x0];
    return (int)(sum / (uint32_t)((x1 - x0) * (y1 - y0)));
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

static void draw_rectangle(uint8_t *pixels, size_t width, size_t height,
                           int x0, int y0, int x1, int y1,
                           uint16_t color)
{
    horizontal_line(pixels, width, height, y0, x0, x1, color);
    horizontal_line(pixels, width, height, y1, x0, x1, color);
    vertical_line(pixels, width, height, x0, y0, y1, color);
    vertical_line(pixels, width, height, x1, y0, y1, color);
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

static void morph_close(size_t width, size_t height)
{
    memset(s_morph, 0, width * height);
    for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
        for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
            bool selected = false;
            for (int dy = -1; dy <= 1 && !selected; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx >= ROI_X_MIN && nx < ROI_X_MAX &&
                        ny >= ROI_Y_MIN && ny < ROI_Y_MAX &&
                        s_mask[(size_t)ny * width + nx] != 0) {
                        selected = true;
                        break;
                    }
                }
            }
            s_morph[(size_t)y * width + x] = selected;
        }
    }

    memset(s_mask, 0, width * height);
    for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
        for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
            bool keep = true;
            for (int dy = -1; dy <= 1 && keep; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    /* Pixels outside the ROI are unknown, not background.
                     * Ignoring them keeps valid track pixels on the image
                     * boundary instead of erasing another one-pixel band. */
                    if (nx < ROI_X_MIN || nx >= ROI_X_MAX ||
                        ny < ROI_Y_MIN || ny >= ROI_Y_MAX) {
                        continue;
                    }
                    if (s_morph[(size_t)ny * width + nx] == 0) {
                        keep = false;
                        break;
                    }
                }
            }
            s_mask[(size_t)y * width + x] = keep;
        }
    }
}

static int find_near_anchor(size_t width, size_t component_count,
                            int minimum_y)
{
    int64_t x_sum = 0;
    int count = 0;
    const int entry_x_min = foot_entry_x_min(width);
    const int entry_x_max = foot_entry_x_max(width);
    const int hold_x_min = foot_hold_x_min(width);
    const int hold_x_max = foot_hold_x_max(width);

    /* Use the centre of the component only in its nearest few rows. Choosing
     * the pixel nearest screen centre systematically hides real lateral
     * error, while averaging all 12 footprint rows lets a later branch drag
     * the anchor. The nearest slice represents the line under the chassis. */
    int central_min_y = INT_MAX;
    for (size_t index = 0; index < component_count; ++index) {
        const int pixel = s_best_component[index];
        const int x = pixel % (int)width;
        const int y = pixel / (int)width;
        if (y < FOOT_GATE_Y_MAX && x >= entry_x_min &&
            x < entry_x_max && y < central_min_y) central_min_y = y;
    }
    if (central_min_y != INT_MAX) {
        x_sum = 0;
        count = 0;
        for (size_t index = 0; index < component_count; ++index) {
            const int pixel = s_best_component[index];
            const int x = pixel % (int)width;
            const int y = pixel / (int)width;
            if (y < FOOT_GATE_Y_MAX &&
                y <= central_min_y + ANCHOR_Y_TOLERANCE &&
                x >= entry_x_min && x < entry_x_max) {
                x_sum += x;
                count++;
            }
        }
        const int target_x = (int)(x_sum / count);
        int best_pixel = s_best_component[0];
        int best_distance = INT_MAX;
        for (size_t index = 0; index < component_count; ++index) {
            const int pixel = s_best_component[index];
            const int x = pixel % (int)width;
            const int y = pixel / (int)width;
            if (y >= FOOT_GATE_Y_MAX ||
                y > central_min_y + ANCHOR_Y_TOLERANCE ||
                x < entry_x_min || x >= entry_x_max) continue;
            const int distance = abs(x - target_x) +
                                 2 * (y - central_min_y);
            if (distance < best_distance) {
                best_distance = distance;
                best_pixel = pixel;
            }
        }
        return best_pixel;
    }

    /* A component touching the vehicle gate must be traced from that contact
     * point.  This prevents a visible outgoing branch from becoming the path
     * origin while the incoming line is still under the vehicle. */
    for (size_t index = 0; index < component_count; ++index) {
        const int pixel = s_best_component[index];
        const int x = pixel % (int)width;
        const int y = pixel / (int)width;
        if (y < FOOT_GATE_Y_MAX && x >= hold_x_min && x < hold_x_max) {
            x_sum += x;
            count++;
        }
    }
    if (count > 0) {
        const int target_x = (int)(x_sum / count);
        int best_pixel = s_best_component[0];
        int best_distance = INT_MAX;
        for (size_t index = 0; index < component_count; ++index) {
            const int pixel = s_best_component[index];
            const int x = pixel % (int)width;
            const int y = pixel / (int)width;
            if (y >= FOOT_GATE_Y_MAX || x < hold_x_min ||
                x >= hold_x_max) {
                continue;
            }
            const int distance = abs(x - target_x) + 2 * y;
            if (distance < best_distance) {
                best_distance = distance;
                best_pixel = pixel;
            }
        }
        return best_pixel;
    }

    x_sum = 0;
    count = 0;
    for (size_t index = 0; index < component_count; ++index) {
        const int pixel = s_best_component[index];
        const int y = pixel / (int)width;
        if (y <= minimum_y + ANCHOR_Y_TOLERANCE) {
            x_sum += pixel % (int)width;
            count++;
        }
    }
    if (count == 0) return s_best_component[0];

    const int target_x = (int)(x_sum / count);
    int best_pixel = s_best_component[0];
    int best_distance = INT_MAX;
    for (size_t index = 0; index < component_count; ++index) {
        const int pixel = s_best_component[index];
        const int y = pixel / (int)width;
        if (y > minimum_y + ANCHOR_Y_TOLERANCE) continue;
        const int distance = abs(pixel % (int)width - target_x) +
                             2 * (y - minimum_y);
        if (distance < best_distance) {
            best_distance = distance;
            best_pixel = pixel;
        }
    }
    return best_pixel;
}

static int trace_component_path(size_t width, size_t height,
                                size_t component_count, int minimum_y,
                                path_point_t samples[PATH_SAMPLE_COUNT],
                                int *near_lookahead_x,
                                int *near_lookahead_y,
                                int *lookahead_x, int *lookahead_y,
                                int *path_length_pixels)
{
    memset(s_mask, 0, width * height);
    for (size_t index = 0; index < component_count; ++index) {
        s_mask[s_best_component[index]] = 1;
    }
    memset(s_parent, 0xff, width * height * sizeof(s_parent[0]));
    memset(s_distance, 0, width * height * sizeof(s_distance[0]));

    const int anchor = find_near_anchor(width, component_count, minimum_y);
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

    const size_t near_lookahead_index =
        path_length > NEAR_LOOKAHEAD_PATH_PIXELS
            ? NEAR_LOOKAHEAD_PATH_PIXELS
            : path_length - 1;
    const size_t far_lookahead_index =
        path_length > FAR_LOOKAHEAD_PATH_PIXELS
            ? FAR_LOOKAHEAD_PATH_PIXELS
            : path_length - 1;
    *near_lookahead_x = s_queue[near_lookahead_index] % width;
    *near_lookahead_y = s_queue[near_lookahead_index] / width;
    *lookahead_x = s_queue[far_lookahead_index] % width;
    *lookahead_y = s_queue[far_lookahead_index] / width;
    *path_length_pixels = (int)path_length;

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
                          int path_length_pixels,
                          line_vision_result_t *result)
{
    if (count < MIN_CORNER_PATH_SAMPLES) return;

    /* Search from the vehicle-facing end of the ordered path so the first
     * confirmed corner wins even when a later corner is also visible. */
    for (int split = CORNER_MIN_SEGMENT_SAMPLES - 1;
         split <= count - CORNER_MIN_SEGMENT_SAMPLES; ++split) {
        const int first = split - CORNER_LOCAL_RADIUS > 0
                              ? split - CORNER_LOCAL_RADIUS
                              : 0;
        const int last = split + CORNER_LOCAL_RADIUS < count - 1
                             ? split + CORNER_LOCAL_RADIUS
                             : count - 1;
        if (split - first + 1 < CORNER_MIN_SEGMENT_SAMPLES ||
            last - split + 1 < CORNER_MIN_SEGMENT_SAMPLES) {
            continue;
        }

        const int64_t single_residual =
            segment_residual(points, first, last);
        const int64_t split_residual =
            segment_residual(points, first, split) +
            segment_residual(points, split, last);
        if (single_residual <= 0 || split_residual >= single_residual) {
            continue;
        }

        int near_dx;
        int near_dy;
        int far_dx;
        int far_dy;
        segment_vector(points, first, split, &near_dx, &near_dy);
        segment_vector(points, split, last, &far_dx, &far_dy);
        const int64_t near_length_squared =
            (int64_t)near_dx * near_dx + (int64_t)near_dy * near_dy;
        const int64_t far_length_squared =
            (int64_t)far_dx * far_dx + (int64_t)far_dy * far_dy;
        if (near_length_squared < CORNER_MIN_VECTOR_LENGTH_SQUARED ||
            far_length_squared < CORNER_MIN_VECTOR_LENGTH_SQUARED) {
            continue;
        }

        const int64_t cross =
            (int64_t)near_dx * far_dy - (int64_t)near_dy * far_dx;
        const int64_t dot =
            (int64_t)near_dx * far_dx + (int64_t)near_dy * far_dy;
        const int angle_deg = (int)lroundf(
            atan2f((float)llabs(cross), (float)dot) *
            180.0f / 3.14159265f);
        const int improvement = clamp_int(
            (int)((single_residual - split_residual) * 100 /
                  single_residual),
            0, 100);
        const int direction = cross < 0 ? 1 : (cross > 0 ? -1 : 0);
        if (angle_deg < CORNER_MIN_ANGLE_DEG ||
            improvement < CORNER_MIN_IMPROVEMENT_PERCENT ||
            direction == 0) {
            continue;
        }

        result->big_turn = true;
        result->turn_direction = direction;
        result->turn_angle_deg = angle_deg;
        result->turn_confidence = clamp_int(
            (angle_deg - 15) * 2 + improvement / 2, 0, 100);
        result->corner_x = points[split].x;
        result->corner_y = points[split].y;
        result->corner_path_distance = clamp_int(
            split * path_length_pixels / count, 0, path_length_pixels);
        return;
    }
}

static bool corridor_vector_from_path(const path_point_t *points, int count,
                                      int *anchor_x, int *anchor_y,
                                      int *dx, int *dy)
{
    if (count < 3) return false;
    const int endpoint = count > 4 ? 4 : count - 1;
    *anchor_x = points[0].x;
    *anchor_y = points[0].y;
    *dx = points[endpoint].x - points[0].x;
    *dy = points[endpoint].y - points[0].y;
    return (int64_t)(*dx) * (*dx) + (int64_t)(*dy) * (*dy) >=
           STRAIGHT_CORRIDOR_MIN_VECTOR_SQUARED;
}

static void evaluate_straight_corridor(const path_point_t *points, int count,
                                       size_t width, size_t component_count,
                                       line_vision_result_t *result)
{
    const bool lock_requested = __atomic_load_n(
        &s_corridor_lock_requested, __ATOMIC_ACQUIRE);
    int current_anchor_x;
    int current_anchor_y;
    int current_dx;
    int current_dy;
    const bool current_valid = corridor_vector_from_path(
        points, count, &current_anchor_x, &current_anchor_y,
        &current_dx, &current_dy);

    if (!lock_requested) {
        s_corridor_locked = false;
        if (current_valid) {
            s_corridor_anchor_x = current_anchor_x;
            s_corridor_anchor_y = current_anchor_y;
            s_corridor_dx = current_dx;
            s_corridor_dy = current_dy;
            s_corridor_valid = true;
        }
    } else if (!s_corridor_locked) {
        /* The controller requests the lock after consuming a frame, so the
         * cached line is the incoming straight from that frame. */
        if (!s_corridor_valid && current_valid) {
            s_corridor_anchor_x = current_anchor_x;
            s_corridor_anchor_y = current_anchor_y;
            s_corridor_dx = current_dx;
            s_corridor_dy = current_dy;
            s_corridor_valid = true;
        }
        s_corridor_locked = s_corridor_valid;
    }

    result->corridor_locked = s_corridor_locked;
    result->corridor_half_width = STRAIGHT_CORRIDOR_HALF_WIDTH;
    if (!s_corridor_valid) return;

    const int64_t length_squared =
        (int64_t)s_corridor_dx * s_corridor_dx +
        (int64_t)s_corridor_dy * s_corridor_dy;
    const float vector_length = sqrtf((float)length_squared);
    int64_t maximum_projection = 0;
    int64_t near_x_sum = 0;
    int support_count = 0;
    int near_support_count = 0;
    for (size_t index = 0; index < component_count; ++index) {
        const int pixel = s_best_component[index];
        const int x = pixel % (int)width;
        const int y = pixel / (int)width;
        const int relative_x = x - s_corridor_anchor_x;
        const int relative_y = y - s_corridor_anchor_y;
        const int64_t projection =
            (int64_t)relative_x * s_corridor_dx +
            (int64_t)relative_y * s_corridor_dy;
        const int64_t cross =
            (int64_t)s_corridor_dx * relative_y -
            (int64_t)s_corridor_dy * relative_x;
        if (projection < -(int64_t)3 * (int64_t)vector_length ||
            cross * cross >
                (int64_t)STRAIGHT_CORRIDOR_HALF_WIDTH *
                STRAIGHT_CORRIDOR_HALF_WIDTH * length_squared) {
            continue;
        }
        support_count++;
        if (projection <= (int64_t)18 * (int64_t)vector_length) {
            near_x_sum += x;
            near_support_count++;
        }
        if (projection > maximum_projection) {
            maximum_projection = projection;
        }
    }
    result->corridor_pixel_count = support_count;
    result->corridor_near_pixel_count = near_support_count;
    result->corridor_length_pixels = clamp_int(
        (int)lroundf((float)maximum_projection / vector_length),
        0, 1000);
    if (near_support_count > 0) {
        const int center_x = steering_target_x(width);
        const int half_width = (ROI_X_MAX - ROI_X_MIN) / 2;
        result->corridor_lateral_error = clamp_int(
            ((int)(near_x_sum / near_support_count) - center_x) *
                1000 / half_width,
            -1000, 1000);
    }
}

esp_err_t line_vision_init(size_t width, size_t height)
{
    ESP_RETURN_ON_FALSE(width * height <= UINT16_MAX, ESP_ERR_INVALID_SIZE,
                        TAG, "frame too large");
    ESP_RETURN_ON_FALSE(width >= ROI_X_MAX && height >= ROI_Y_MAX,
                        ESP_ERR_INVALID_SIZE, TAG, "frame too small");
    s_pixel_capacity = width * height;
    s_mask = allocate_hot_buffer(s_pixel_capacity);
    s_morph = allocate_hot_buffer(s_pixel_capacity);
    s_luma = allocate_hot_buffer(s_pixel_capacity);
    s_integral_capacity = (width + 1) * (height + 1);
    s_luma_integral = allocate_hot_buffer(
        s_integral_capacity * sizeof(uint32_t));
    s_queue = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_best_component = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_parent = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_distance = heap_caps_malloc(s_pixel_capacity * sizeof(uint16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(s_mask && s_morph && s_queue && s_best_component &&
                            s_parent && s_distance && s_luma &&
                            s_luma_integral,
                        ESP_ERR_NO_MEM, TAG, "vision buffers unavailable");
    s_previous_near_x = steering_target_x(width);
    s_previous_path_valid = false;
    s_corridor_locked = false;
    s_corridor_valid = false;
    __atomic_store_n(&s_corridor_lock_requested, false,
                     __ATOMIC_RELEASE);
    ESP_LOGI(TAG,
             "PATH PURSUIT ROI x=%d..%d y=%d..%d lookahead=%d/%dpx local-contrast",
             ROI_X_MIN, ROI_X_MAX - 1, ROI_Y_MIN, ROI_Y_MAX - 1,
             NEAR_LOOKAHEAD_PATH_PIXELS, FAR_LOOKAHEAD_PATH_PIXELS);
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

void line_vision_set_corridor_locked(bool locked)
{
    __atomic_store_n(&s_corridor_lock_requested, locked,
                     __ATOMIC_RELEASE);
}

void line_vision_process(uint8_t *pixels, size_t width, size_t height,
                         line_vision_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->near_x = -1;
    result->far_x = -1;
    result->corner_x = -1;
    result->corner_y = -1;
    result->lookahead_x = -1;
    result->lookahead_y = -1;
    result->near_lookahead_x = -1;
    result->near_lookahead_y = -1;
    result->corridor_locked = s_corridor_locked;
    result->corridor_half_width = STRAIGHT_CORRIDOR_HALF_WIDTH;
    if (!pixels || !s_mask || !s_luma || !s_luma_integral ||
        width * height > s_pixel_capacity ||
        (width + 1) * (height + 1) > s_integral_capacity ||
        width < ROI_X_MAX || height < ROI_Y_MAX) {
        return;
    }

    uint32_t histogram[256] = {0};
    uint32_t roi_pixel_count = 0;
    for (int y = 0; y < (int)height; ++y) {
        for (int x = 0; x < (int)width; ++x) {
            const size_t index = (size_t)y * width + x;
            const uint8_t luma = pixel_luminance(pixels, index);
            s_luma[index] = luma;
            if (x >= ROI_X_MIN && x < ROI_X_MAX &&
                y >= ROI_Y_MIN && y < ROI_Y_MAX) {
                histogram[luma]++;
                roi_pixel_count++;
            }
        }
    }
    const int dark_level = histogram_percentile(histogram, roi_pixel_count, 1);
    const int light_level = histogram_percentile(histogram, roi_pixel_count, 70);
    result->contrast = light_level - dark_level;
    const line_vision_rgb_thresholds_t rgb_caps =
        line_vision_get_rgb_thresholds();
    result->threshold = result->contrast >= MIN_CONTRAST
                            ? clamp_int(dark_level +
                                            result->contrast * 30 / 100,
                                        35, 140)
                            : LOW_CONTRAST_LUMA_THRESHOLD;
    build_luma_integral(width, height);
    memset(s_mask, 0, width * height);
    for (int y = ROI_Y_MIN; y < ROI_Y_MAX; ++y) {
        for (int x = ROI_X_MIN; x < ROI_X_MAX; ++x) {
            const size_t index = (size_t)y * width + x;
            const int luma = s_luma[index];
            bool global_dark = false;
            if (luma <= result->threshold) {
                uint8_t red;
                uint8_t green;
                uint8_t blue;
                pixel_rgb(pixels, index, &red, &green, &blue);
                global_dark = red <= rgb_caps.red &&
                              green <= rgb_caps.green &&
                              blue <= rgb_caps.blue;
            }
            const bool locally_dark =
                luma <= LOCAL_LUMA_MAX &&
                local_mean_luma(width, x, y) - luma >= LOCAL_DARKNESS_MIN;
            if (global_dark || locally_dark) {
                s_mask[index] = 1;
            }
        }
    }
    morph_close(width, height);

    int best_score = INT_MIN;
    int best_area = 0;
    int best_min_y = 0;
    int best_max_y = 0;
    int best_min_x = 0;
    int best_max_x = 0;
    int best_near_count = 0;
    int best_foot_hold_count = 0;
    int best_foot_entry_count = 0;
    int best_anchor_x = steering_target_x(width);
    size_t best_pixel_count = 0;
    const int entry_x_min = foot_entry_x_min(width);
    const int entry_x_max = foot_entry_x_max(width);
    const int hold_x_min = foot_hold_x_min(width);
    const int hold_x_max = foot_hold_x_max(width);
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
            int min_x = start_x;
            int max_x = start_x;
            int min_y = start_y;
            int max_y = start_y;
            int near_count = 0;
            int foot_hold_count = 0;
            int foot_entry_count = 0;
            int64_t foot_x_sum = 0;
            while (head < tail) {
                const size_t index = s_queue[head++];
                const int x = index % width;
                const int y = index / width;
                area++;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                if (y < NEAR_Y_MAX) near_count++;
                if (y < FOOT_GATE_Y_MAX &&
                    x >= hold_x_min && x < hold_x_max) {
                    foot_hold_count++;
                    foot_x_sum += x;
                    if (x >= entry_x_min && x < entry_x_max) {
                        foot_entry_count++;
                    }
                }
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

            int64_t anchor_sum = 0;
            int anchor_count = 0;
            for (size_t index = 0; index < tail; ++index) {
                const int pixel = s_queue[index];
                if (pixel / (int)width <= min_y + ANCHOR_Y_TOLERANCE) {
                    anchor_sum += pixel % (int)width;
                    anchor_count++;
                }
            }
            const int anchor_x = foot_hold_count > 0
                                     ? (int)(foot_x_sum / foot_hold_count)
                                     : (anchor_count > 0
                                            ? (int)(anchor_sum / anchor_count)
                                            : start_x);
            const int extent = (max_x - min_x) > (max_y - min_y)
                                   ? max_x - min_x + 1
                                   : max_y - min_y + 1;
            const int tracking_x = s_previous_path_valid
                                       ? s_previous_near_x
                                       : steering_target_x(width);
            const int tracking_penalty =
                abs(anchor_x - tracking_x) *
                (s_previous_path_valid ? 3 : 1);
            const int score = clamp_int(area, 0, 500) +
                              extent * 12 + near_count * 6 +
                              foot_hold_count * 6 +
                              foot_entry_count * 6 - tracking_penalty;
            const bool tracked_far_candidate =
                s_previous_path_valid && min_y < 40 &&
                abs(anchor_x - s_previous_near_x) <= 15;
            const bool candidate =
                area >= MIN_COMPONENT_AREA &&
                area * 100 <=
                    (int)roi_pixel_count * MAX_COMPONENT_FILL_PERCENT &&
                extent >= MIN_COMPONENT_EXTENT &&
                (near_count >= MIN_COMPONENT_NEAR_PIXELS ||
                 tracked_far_candidate);
            if (candidate && score > best_score) {
                best_score = score;
                best_area = area;
                best_min_x = min_x;
                best_max_x = max_x;
                best_min_y = min_y;
                best_max_y = max_y;
                best_near_count = near_count;
                best_foot_hold_count = foot_hold_count;
                best_foot_entry_count = foot_entry_count;
                best_anchor_x = anchor_x;
                best_pixel_count = tail;
                memcpy(s_best_component, s_queue,
                       tail * sizeof(s_best_component[0]));
            }
        }
    }

    path_point_t path[PATH_SAMPLE_COUNT] = {0};
    if (best_pixel_count > 0) {
        result->path_point_count = trace_component_path(
            width, height, best_pixel_count, best_min_y, path,
            &result->near_lookahead_x, &result->near_lookahead_y,
            &result->lookahead_x, &result->lookahead_y,
            &result->path_length_pixels);
        result->vector_point_count = result->path_point_count;
        result->component_area = best_area;
        result->steering_band_pixel_count = best_area;
        result->steering_band_fill_percent =
            best_area * 100 / (int)roi_pixel_count;
        result->foot_pixel_count = best_foot_hold_count;
        result->foot_center_pixel_count = best_foot_entry_count;

        if (result->path_point_count >= 2 &&
            result->path_length_pixels >= MIN_PATH_PIXELS) {
            detect_corner(path, result->path_point_count,
                          result->path_length_pixels, result);
            const int center_x = steering_target_x(width);
            const int half_width = (ROI_X_MAX - ROI_X_MIN) / 2;
            result->near_x = path[0].x;
            result->far_x = path[result->path_point_count - 1].x;
            result->lateral_error = clamp_int(
                (result->near_x - center_x) * 1000 / half_width,
                -1000, 1000);
            result->foot_lateral_error = result->lateral_error;
            result->foot_path_length_pixels = result->path_length_pixels;
            result->foot_track_valid =
                best_foot_hold_count >= FOOT_MIN_PIXELS &&
                result->foot_path_length_pixels >= FOOT_MIN_PATH_PIXELS;
            result->foot_track_centered =
                result->foot_track_valid &&
                best_foot_entry_count >= FOOT_MIN_PIXELS;
            const int heading_dy = clamp_int(
                abs(result->lookahead_y - result->near_lookahead_y),
                8, 1000);
            result->heading_error = clamp_int(
                (result->lookahead_x - result->near_lookahead_x) *
                    1000 / heading_dy,
                -1000, 1000);

            const int near_preview_error = clamp_int(
                (result->near_lookahead_x - center_x) * 1000 /
                    half_width,
                -1000, 1000);
            const int far_preview_error = clamp_int(
                (result->lookahead_x - center_x) * 1000 / half_width,
                -1000, 1000);
            const int preview_disagreement =
                abs(result->lookahead_x - result->near_lookahead_x);
            result->far_preview_weight = clamp_int(
                FAR_PREVIEW_WEIGHT_MIN + preview_disagreement * 2,
                FAR_PREVIEW_WEIGHT_MIN, FAR_PREVIEW_WEIGHT_MAX);
            const int preview_error =
                (near_preview_error *
                     (100 - result->far_preview_weight) +
                 far_preview_error * result->far_preview_weight) /
                100;
            result->steering_error = clamp_int(
                (85 * preview_error + 15 * result->lateral_error) / 100,
                -1000, 1000);
            result->steering_band_error = result->steering_error;
            result->steering_band_valid = true;

            int left_pixels = 0;
            for (size_t index = 0; index < best_pixel_count; ++index) {
                if (s_best_component[index] % width < (size_t)center_x) {
                    left_pixels++;
                }
            }
            result->steering_band_left_percent =
                left_pixels * 100 / best_area;
            result->steering_band_right_percent =
                100 - result->steering_band_left_percent;

            const int vertical_extent = best_max_y - best_min_y + 1;
            const int horizontal_extent = best_max_x - best_min_x + 1;
            const int extent = vertical_extent > horizontal_extent
                                   ? vertical_extent : horizontal_extent;
            const int extent_confidence = clamp_int(extent * 40 / 70, 0, 40);
            const int path_confidence = clamp_int(
                result->path_length_pixels * 30 / 70, 0, 30);
            const int near_confidence = clamp_int(
                best_near_count * 20 / 20, 0, 20);
            const int contrast_confidence = clamp_int(
                result->contrast * 10 / 50, 0, 10);
            result->confidence = clamp_int(
                extent_confidence + path_confidence + near_confidence +
                    contrast_confidence,
                0, 100);
            const bool tracked_low_confidence =
                s_previous_path_valid &&
                abs(best_anchor_x - s_previous_near_x) <= 12 &&
                result->confidence >= 20;
            result->found = result->confidence >= 25 ||
                            tracked_low_confidence;
            evaluate_straight_corridor(
                path, result->path_point_count, width,
                best_pixel_count, result);
        }
    }

    if (result->found) {
        s_previous_near_x = best_anchor_x;
        s_previous_path_valid = true;
    }

    const uint16_t track_color = 0x07e0;
    const uint16_t path_color = 0xf81f;
    const uint16_t near_lookahead_color = 0x07ff;
    const uint16_t lookahead_color = 0xffe0;
    const uint16_t corner_color = 0xfd20;
    const uint16_t roi_color = 0xffff;
    const uint16_t target_color = 0xf800;
    const uint16_t foot_gate_color = result->foot_track_valid
                                         ? 0x07ff
                                         : 0xfd20;
    if (result->found) {
        for (size_t index = 0; index < best_pixel_count; ++index) {
            const size_t pixel_index = s_best_component[index];
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
        draw_cross(pixels, width, height,
                   result->near_lookahead_x, result->near_lookahead_y,
                   near_lookahead_color);
        draw_cross(pixels, width, height,
                   result->lookahead_x, result->lookahead_y,
                   lookahead_color);
        if (result->big_turn) {
            draw_corner_marker(pixels, width, height,
                               result->corner_x, result->corner_y,
                               corner_color);
        }
    } else {
        memset(s_mask, 0, width * height);
    }
    horizontal_line(pixels, width, height, ROI_Y_MIN,
                    ROI_X_MIN, ROI_X_MAX - 1, roi_color);
    horizontal_line(pixels, width, height, ROI_Y_MAX - 1,
                    ROI_X_MIN, ROI_X_MAX - 1, roi_color);
    vertical_line(pixels, width, height, steering_target_x(width),
                  ROI_Y_MIN, ROI_Y_MAX - 1, target_color);
    draw_rectangle(pixels, width, height,
                   foot_entry_x_min(width), ROI_Y_MIN,
                   foot_entry_x_max(width) - 1, FOOT_GATE_Y_MAX - 1,
                   foot_gate_color);
}
