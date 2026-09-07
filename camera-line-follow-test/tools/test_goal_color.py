"""RGB565 reference-model regression using firmware thresholds.

Checks the color rule, not C execution or real-scene component detection.
Run with the IDF Python; no extra packages or device access required.
"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = (ROOT / "main/black_marker_vision.c").read_text(encoding="utf-8")
    purple_source = (ROOT / "main/purple_ball_vision.c").read_text(encoding="utf-8")
    defines = re.findall(r"^#define (\w+) (\d+)\s*$", source + "\n" + purple_source, re.M)
    c = {name: int(value) for name, value in defines}
    limits = {name: int(value) for name, value in re.findall(r"\.(luminance_max|channel_max|rgb_spread_max) = (\d+)", source)}
    removed = overlap = retained = 0
    for color in range(65536):
        r = ((color >> 11) & 31) * 255 // 31
        g = ((color >> 5) & 63) * 255 // 63
        b = (color & 31) * 255 // 31
        before = (((77*r + 150*g + 29*b) >> 8) <= limits['luminance_max'] and
                  max(r,g,b) <= limits['channel_max'] and
                  max(r,g,b)-min(r,g,b) <= limits['rgb_spread_max'])
        purple = (b >= c['PURPLE_BLUE_MIN'] and r >= c['PURPLE_RED_MIN'] and
                  g >= c['PURPLE_GREEN_MIN'] and r <= c['PURPLE_RED_MAX'] and
                  g <= c['PURPLE_GREEN_MAX'] and b >= r+c['PURPLE_BLUE_OVER_RED'] and
                  b >= g+c['PURPLE_BLUE_OVER_GREEN'])
        tinted = (b >= c['GOAL_PURPLE_BLUE_MIN'] and
                  b-r >= c['GOAL_PURPLE_BLUE_EXCESS'] and
                  b-g >= c['GOAL_PURPLE_BLUE_EXCESS'])
        after = before and not tinted
        assert not after or before
        assert not (after and purple), ('purple/black overlap', r,g,b)
        # Independent preservation checks: neutral, slight-cast and deep black.
        if b < 40 or b-r < 12 or b-g < 12:
            assert after == before, ('neutral black changed', r,g,b)
        removed += before and not after
        overlap += before and purple
        retained += after
    assert removed > 0 and overlap > 0 and retained > 0
    print(f'PASS reference model: 65536 RGB565 colors; removed={removed}, '
          f'previous purple overlaps={overlap}, retained black colors={retained}')


if __name__ == '__main__':
    main()
