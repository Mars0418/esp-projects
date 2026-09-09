#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum { SIMPLE_WAIT, SIMPLE_SEARCH, SIMPLE_APPROACH, SIMPLE_PUSH,
               SIMPLE_DONE, SIMPLE_STOPPED, SIMPLE_BRAKE_NEXT, SIMPLE_RETRACE, SIMPLE_BRAKE_TURN } simple_push_state_t;
typedef struct {
    int64_t now_us;
    /* red/red_x/red_y carry the currently selected ball in either round. */
    bool fresh, start, emergency, red, goal;
    bool retrace_complete;
    int red_x, red_y, goal_x;
    float goal_distance_mm;
    float goal_gap_mm;
    bool ball_ground_valid, goal_ground_valid;
    float ball_right_mm, ball_forward_mm, goal_right_mm, goal_forward_mm;
} simple_push_input_t;
typedef struct {
    simple_push_state_t state;
    int ball_index; /* 0: red, 1: purple; only successful red stop advances. */
    const char *reason;
    int64_t entered_us;
    bool backing_away;
    int64_t next_turn_us;
    int pending_turn_sign, pending_turn_frames;
    int confirmed, lost, scored;
} simple_push_t;
typedef struct {
    float forward, right, turn;
    float speed_scale;
    bool rear_only;
    int duration_ms;
} simple_push_motion_t;
const char *simple_push_state_name(simple_push_state_t state);
simple_push_motion_t simple_push_update(simple_push_t *s,
                                        const simple_push_input_t *in);

const char *simple_push_status_name(const simple_push_t *state);
