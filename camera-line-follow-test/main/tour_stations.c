#include "tour_speech.h"
int tour_station_clip(int route, int corner, bool finished)
{
    if (route == 1) {
        if (finished) return corner == 2 ? 3 : -1;
        return corner == 1 ? 1 : corner == 2 ? 2 : -1;
    }
    if (route == 2) {
        if (finished) return corner == 1 ? 5 : -1;
        return corner == 1 ? 4 : -1;
    }
    return -1;
}
