#pragma once
#include <stdbool.h>
#include <stdint.h>

static inline bool tour_station_departure_allowed(bool requested, int64_t now_us,
                                                   int64_t earliest_us, bool speaking)
{
    return requested && now_us >= earliest_us && !speaking;
}

/* Re-arm only on distinct straight frames after leaving the stopped corner. */
static inline bool tour_corner_rearm_ready(int64_t now_us, int64_t not_before_us,
                                           bool new_frame, bool straight,
                                           int *straight_frames)
{
    if (!new_frame) return false;
    if (now_us < not_before_us || !straight) {
        *straight_frames = 0;
        return false;
    }
    if (*straight_frames < 3) ++*straight_frames;
    return *straight_frames >= 3;
}
