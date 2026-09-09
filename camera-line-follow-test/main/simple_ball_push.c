#include "simple_ball_push.h"
#include <math.h>
#include <stdlib.h>

const char *simple_push_state_name(simple_push_state_t state)
{
    static const char *const names[] = {
        "WAIT_START", "SEARCH_RED", "ALIGN_SHOT", "PUSH_RED", "DONE", "STOPPED", "BRAKE_FOR_PURPLE", "RETRACE_RED_PUSH", "BRAKE_BEFORE_LEFT"
    };
    return (unsigned)state < 9 ? names[state] : "INVALID";
}
const char *simple_push_status_name(const simple_push_t *s)
{
    if (s->ball_index == 1) {
        if (s->state == SIMPLE_SEARCH) return "SEARCH_PURPLE";
        if (s->state == SIMPLE_PUSH) return "PUSH_PURPLE";
    }
    return simple_push_state_name(s->state);
}
static void enter(simple_push_t *s, simple_push_state_t state,
                  int64_t now, const char *reason)
{
    s->state = state;
    s->entered_us = now;
    s->confirmed = s->lost = s->scored = 0;
    s->next_turn_us = 0;
    s->backing_away = false;
    s->pending_turn_sign = s->pending_turn_frames = 0;
    s->reason = reason;
}
simple_push_motion_t simple_push_update(simple_push_t *s,
                                        const simple_push_input_t *in)
{
    simple_push_motion_t motion = {.duration_ms = 120, .speed_scale = 0.25f};
    if (s->state == SIMPLE_DONE || s->state == SIMPLE_STOPPED) return motion;
    if (in->emergency) {
        enter(s, SIMPLE_STOPPED, in->now_us, "EMERGENCY_OR_CAMERA_STOP");
        return motion;
    }
    if (s->state == SIMPLE_WAIT) {
        if (in->start && in->fresh)
            enter(s, SIMPLE_SEARCH, in->now_us, "SEARCHING");
        return motion;
    }
    if (!in->fresh) {
        enter(s, SIMPLE_STOPPED, in->now_us, "STALE_CAMERA_FRAME");
        return motion;
    }
    if (s->state == SIMPLE_BRAKE_NEXT) {
        /* Main pauses the motors with 200 ms active braking, then settles.
         * Never interpret the red transition frame as a purple observation. */
        if (in->now_us - s->entered_us < 500000) return motion;
        enter(s, SIMPLE_RETRACE, in->now_us, "REVERSE_RED_PUSH_DISTANCE");
        return motion;
    }
    if (s->state == SIMPLE_RETRACE) {
        if (in->retrace_complete)
            enter(s, SIMPLE_BRAKE_TURN, in->now_us, "RETRACE_COMPLETE_SETTLE");
        else if (in->now_us - s->entered_us >= 15000000)
            enter(s, SIMPLE_STOPPED, in->now_us, "RETRACE_TIMEOUT");
        return motion;
    }
    if (s->state == SIMPLE_BRAKE_TURN) {
        if (in->now_us - s->entered_us < 500000) return motion;
        enter(s, SIMPLE_SEARCH, in->now_us, "TURN_LEFT_FIND_PURPLE");
        motion.turn = 0.25f;
        motion.duration_ms = 90;
        s->next_turn_us = in->now_us + 650000;
        return motion;
    }
    /* Alignment may wait or adjust indefinitely; search and push stay bounded. */
    if (s->state != SIMPLE_APPROACH &&
        in->now_us - s->entered_us >= 15000000) {
        enter(s, SIMPLE_STOPPED, in->now_us, "STATE_TIMEOUT");
        return motion;
    }
    if (s->state == SIMPLE_SEARCH) {
        if (s->ball_index == 1 && in->now_us < s->next_turn_us) return motion;
        s->confirmed = in->red ? s->confirmed + 1 : 0;
        if (s->confirmed >= 3)
            enter(s, SIMPLE_APPROACH, in->now_us, "ALIGN_BALL_GOAL");
        else if (!in->red && in->now_us >= s->next_turn_us) {
            motion.turn = s->ball_index == 0 ? -0.25f : 0.25f;
            motion.duration_ms = 70;
            s->next_turn_us = in->now_us + 650000;
        }
        return motion;
    }
    if (!in->red) {
        s->confirmed = s->scored = 0;
        s->pending_turn_frames = 0;
        if (++s->lost >= 3)
            enter(s, SIMPLE_STOPPED, in->now_us, s->ball_index == 0 ? "RED_LOST" : "PURPLE_LOST");
        return motion; /* Stop on the first missing frame; never push blind. */
    }
    s->lost = 0;
    const int error = in->red_x - 80;
    if (s->state == SIMPLE_PUSH && abs(error) > 40) {
        enter(s, SIMPLE_STOPPED, in->now_us, "SHOT_BALL_OFF_AXIS");
        return motion; /* Never sidestep against a ball already being pushed. */
    }
    if (s->state == SIMPLE_APPROACH) {
        motion.speed_scale = 0.50f;
        const float bx = in->ball_right_mm, by = in->ball_forward_mm;
        const float gx = in->goal_right_mm, gy = in->goal_forward_mm;
        if (!in->ball_ground_valid || !isfinite(bx) || !isfinite(by) || by <= 0) {
            s->confirmed = 0;
            s->reason = "WAIT_BALL_GEOMETRY";
            return motion;
        }
        /* Clearance is measured from chassis centre; keep room for fine alignment.
         * Hysteresis prevents alternating reverse/align at the clearance threshold. */
        if (by < 130 || in->red_y < 39) s->backing_away = true;
        if (s->backing_away && (by < 160 || in->red_y < 44)) {
            s->confirmed = 0;
            s->reason = "BACK_AWAY_FROM_BALL";
            motion.forward = -0.4f;
            motion.duration_ms = 500;
            return motion;
        }
        s->backing_away = false;
        if (!in->goal || !in->goal_ground_valid || !isfinite(gx) ||
            !isfinite(gy) || gy < by + 60) {
            s->confirmed = 0;
            s->pending_turn_frames = 0;
            s->reason = in->goal && !in->goal_ground_valid
                ? "WAIT_GOAL_CORNER" : "WAIT_GOAL_BEHIND_BALL";
            return motion;
        }
        if (in->now_us < s->next_turn_us) return motion;
        const float bearing = atan2f(bx, by);
        /* Ground geometry, not equal image x: extend ball--goal line to y=0. */
        const float intercept = (bx * gy - gx * by) / (gy - by);
        const float miss_at_goal = gx - bx * gy / by;
        if (fabsf(bearing) > 0.17f ||
            (fabsf(miss_at_goal) <= 20 && (fabsf(bx) > 10 || fabsf(gx) > 20))) {
            s->confirmed = 0;
            s->reason = "TURN_TO_SHOT";
            const int direction = bx > 0 ? -1 : 1;
            s->pending_turn_frames = direction == s->pending_turn_sign
                ? s->pending_turn_frames + 1 : 1;
            s->pending_turn_sign = direction;
            if (s->pending_turn_frames < 2) return motion;
            motion.turn = direction * 0.25f;
            motion.duration_ms = fabsf(bearing) > 0.25f ? 65 : (fabsf(bearing) > 0.08f ? 35 : 20);
            s->next_turn_us = in->now_us + 650000;
            s->pending_turn_frames = 0;
            return motion;
        }
        /* Upright y=119-raw_y: target centre-to-lower band y=60..80.
         * Approach only after pointing toward the ball; reobserve before aligning. */
        if (in->red_y > 59) {
            s->confirmed = 0;
            s->pending_turn_frames = 0;
            s->reason = "APPROACH_TO_AIM_BAND";
            motion.forward = 0.4f;
            motion.duration_ms = 160;
            return motion;
        }
        s->pending_turn_frames = 0;
        if (fabsf(miss_at_goal) > 20) {
            s->confirmed = 0;
            s->reason = "SIDESTEP_BEHIND_BALL";
            motion.right = intercept > 0 ? 0.45f : -0.45f;
            motion.duration_ms = fabsf(intercept) > 50 ? 200 : (fabsf(intercept) > 20 ? 100 : 40);
            s->next_turn_us = in->now_us + 350000;
            return motion;
        }
        s->reason = "CONFIRM_SHOT_LINE";
        if (++s->confirmed < 4) return motion;
        enter(s, SIMPLE_PUSH, in->now_us, "STRAIGHT_SHOT");
    }
    /* Stop before entry, independent of ball disappearance or corner fitting.
     * The nearest goal surface is measured along the ball's forward corridor. */
    const bool near_goal = in->goal && in->ball_ground_valid &&
        isfinite(in->goal_gap_mm) && in->goal_gap_mm >= 0 && in->goal_gap_mm <= 180;
    if (near_goal) {
        if (s->ball_index == 0) {
            s->ball_index = 1;
            enter(s, SIMPLE_BRAKE_NEXT, in->now_us, "RED_GOAL_STOP_SWITCH_PURPLE");
        } else {
            enter(s, SIMPLE_DONE, in->now_us, "PURPLE_GOAL_APPROACH_STOP");
        }
        return motion;
    }
    /* Refresh a bounded straight drive each frame, without inter-frame braking.
     * Once aligned, the goal does not steer the ball away. */
    motion.speed_scale = 1.00f;
    motion.forward = 1.0f;
    motion.duration_ms = 500;
    return motion;
}
