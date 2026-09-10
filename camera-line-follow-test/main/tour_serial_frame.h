#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef struct { char data[192]; size_t length; bool active, overflow; } tour_serial_frame_t;
/* 0: legacy byte, 1: consumed, 2: complete framed command. Ctrl-C always escapes. */
static inline int tour_serial_feed(tour_serial_frame_t *s, uint8_t c)
{
    if(c==3) { memset(s,0,sizeof(*s)); return 0; }
    if(c=='@' && !s->active) { memset(s,0,sizeof(*s)); s->active=true; return 1; }
    if(!s->active) return 0;
    if(c=='\r' || c=='\n') {
        s->active=false; s->data[s->length]=0;
        return s->overflow ? 1 : 2;
    }
    if(s->length+1<sizeof(s->data)) s->data[s->length++]=(char)c;
    else s->overflow=true;
    return 1;
}
