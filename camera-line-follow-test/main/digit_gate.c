#include "digit_gate.h"
void digit_gate_reset(digit_gate_t *s)
{
    *s=(digit_gate_t){.candidate=-1};
}
int digit_gate_update(digit_gate_t *s, int digit, bool has_region,
                       float score, float margin, int64_t now_ms)
{
    (void)margin; /* Kept in telemetry; confirmation uses the requested score rule. */
    if (s->last_ms && now_ms-s->last_ms>600) {
        /* A dropped frame cannot release a previously issued command. */
        s->streak=0; s->candidate=-1; s->empty_frames=0;
    }
    s->last_ms=now_ms;
    if (!has_region) {
        s->streak=0; s->candidate=-1;
        if (!s->empty_frames) s->empty_since_ms=now_ms;
        if (s->empty_frames<100) ++s->empty_frames;
        if (s->empty_frames>=5 && now_ms-s->empty_since_ms>=500) s->latched=false;
        return -1;
    }
    s->empty_frames=0;
    if (digit<0 || digit>9 || !(score>=DIGIT_MIN_SCORE)) {
        s->candidate=-1; s->streak=0; return -1;
    }
    if (digit!=s->candidate) {
        s->candidate=digit; s->streak=1; s->since_ms=now_ms;
    } else if(s->streak<100) ++s->streak;
    if (!s->latched && s->streak>=DIGIT_CONFIRM_FRAMES) {
        s->latched=true; return digit;
    }
    return -1;
}
