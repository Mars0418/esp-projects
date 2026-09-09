#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Same cascade as obstacle forward/left: speed PI inside, accumulated encoder
 * progress balance outside. Forward uses equal A/D travel, not strafe's 1:2:1. */
typedef struct {
    bool active;
    int32_t start_a, start_d;
    int pwm_a, pwm_d;
} straight_sync_t;
static inline int64_t straight_sync_distance(int32_t now, int32_t start)
{
    uint32_t wrapped = (uint32_t)now - (uint32_t)start;
    int64_t delta = wrapped <= INT32_MAX ? (int64_t)wrapped
                                       : (int64_t)wrapped - 4294967296LL;
    return delta < 0 ? -delta : delta;
}
static inline void straight_sync_update(straight_sync_t *s, bool enabled,
                                        int32_t a, int32_t d)
{
    s->pwm_a = s->pwm_d = 0;
    if (!enabled) { s->active = false; return; }
    if (!s->active) {
        s->active = true; s->start_a = a; s->start_d = d;
    }
    const int64_t pa = straight_sync_distance(a, s->start_a);
    const int64_t pd = straight_sync_distance(d, s->start_d);
    /* (average - progress) / 4, clamped as in obstacle forward. */
    int64_t correction = (pd - pa) / 8;
    if (correction > 25) correction = 25;
    if (correction < -25) correction = -25;
    s->pwm_a = (int)correction;
    s->pwm_d = -(int)correction;
}

static inline int straight_sync_push_duty(int pi, int sync)
{
    int duty = 250 + pi + sync;
    if (duty < 165) duty = 165;
    if (duty > 335) duty = 335;
    return duty;
}

static inline int straight_sync_speed_target(int delta_a, int delta_d)
{
    return (int)(((int64_t)delta_a + delta_d) / 2);
}

/* Positive heading error requests left rotation: more A forward, less D forward. */
static inline int straight_push_heading_pwm(float target_rad, float heading_rad)
{
    float error = remainderf(target_rad - heading_rad, 6.2831853f);
    if (!isfinite(error) || fabsf(error) < 0.008726646f) return 0;
    int correction = (int)lroundf(error * (180.0f / 3.14159265f) * 3.0f);
    if (correction > 30) correction = 30;
    if (correction < -30) correction = -30;
    return correction;
}
