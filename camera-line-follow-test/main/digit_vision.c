#include "digit_vision.h"
#include <math.h>
#include <string.h>

static int imin(int a, int b) { return a < b ? a : b; }
static int imax(int a, int b) { return a > b ? a : b; }

digit_region_t digit_vision_prepare(digit_vision_workspace_t *s,
                                    const uint8_t rgb[160 * 120 * 2], unsigned rotation,
                                    bool mirror, uint8_t input[784])
{
    digit_region_t r = {.reason = "NO_DIGIT"};
    memset(input, 0, 784);
    memset(s->labels, 0, sizeof(s->labels));
    unsigned hist[256] = {0};
    for (int sy = 0; sy < 120; ++sy)
        for (int sx = 0; sx < 160; ++sx) {
            int p = (sy * 160 + sx) * 2;
            unsigned v = ((unsigned)rgb[p] << 8) | rgb[p + 1];
            unsigned red = ((v >> 11) & 31) * 255 / 31;
            unsigned green = ((v >> 5) & 63) * 255 / 63;
            unsigned blue = (v & 31) * 255 / 31;
            int x = 119 - sy, y = sx;
            s->gray[y * 120 + x] = (77 * red + 150 * green + 29 * blue) >> 8;
        }
    const int x0 = DIGIT_ROI_X, y0 = DIGIT_ROI_Y;
    const int x1 = x0 + DIGIT_ROI_SIZE, y1 = y0 + DIGIT_ROI_SIZE;
    unsigned total = 0, count = 0;
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) {
            unsigned v = s->gray[y * 120 + x];
            hist[v]++;
            total += v;
            count++;
        }
    unsigned cum = 0;
    int low = 0, high = 255;
    bool found_low = false;
    for (int i = 0; i < 256; ++i) {
        cum += hist[i];
        if (!found_low && cum >= count / 100) { low = i; found_low = true; }
        if (cum >= count * 9 / 10) { high = i; break; }
    }
    r.contrast = high - low;
    if (r.contrast < 45 || high < 100) { r.reason = "LOW_CONTRAST"; return r; }
    unsigned n0 = 0, sum0 = 0;
    double best_var = -1;
    int threshold = 0;
    for (int t = 0; t < 255; ++t) {
        n0 += hist[t]; sum0 += hist[t] * t;
        if (!n0 || n0 == count) continue;
        double delta = (double)sum0 / n0 - (double)(total - sum0) / (count - n0);
        double var = (double)n0 * (count - n0) * delta * delta;
        if (var > best_var) { best_var = var; threshold = t; }
    }
    r.threshold = threshold;
    typedef struct {int l,r,t,b,area;unsigned label;} component_t;
    component_t pieces[128];
    int piece_count=0;
    unsigned next_label = 0, selected = 0;
    int best_area = 0, second_area = 0;
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) {
            int p = y * 120 + x;
            if (s->labels[p] || s->gray[p] > threshold) continue;
            unsigned label = ++next_label;
            int head = 0, tail = 1;
            s->queue[0] = p; s->labels[p] = label;
            int left=x, right=x, top=y, bottom=y;
            while (head < tail) {
                int q = s->queue[head++], qx = q % 120, qy = q / 120;
                left=imin(left,qx); right=imax(right,qx);
                top=imin(top,qy); bottom=imax(bottom,qy);
                for (int dy=-1; dy<=1; ++dy)
                    for (int dx=-1; dx<=1; ++dx) {
                        int nx=qx+dx, ny=qy+dy;
                        if (nx<x0 || nx>=x1 || ny<y0 || ny>=y1) continue;
                        int np=ny*120+nx;
                        if (!s->labels[np] && s->gray[np]<=threshold) {
                            s->labels[np]=label; s->queue[tail++]=np;
                        }
                    }
            }
            int w=right-left+1, h=bottom-top+1;
            if(tail>=4 && left>x0 && right<x1-1 && top>y0 && bottom<y1-1) {
                if(piece_count>=128) {r.reason="NOISY";return r;}
                pieces[piece_count++]=(component_t){left,right,top,bottom,tail,label};
            }
            /* Reject cut-off objects, paper edges, tiny specks, and solid patches. */
            if (tail<20 || h<12 || w<2 || h>90 || w>90 || w>h*2 ||
                left<=x0 || right>=x1-1 || top<=y0 || bottom>=y1-1 ||
                tail*100 < w*h*7 || (w>h/3 && tail*100>w*h*85)) continue;
            if (tail > best_area) {
                second_area=best_area; best_area=tail; selected=label;
                r.x=left; r.y=top; r.width=w; r.height=h; r.area=tail;
            } else if (tail>second_area) second_area=tail;
        }
    if (!selected) return r;
    /* Handwriting can contain disconnected strokes (notably the top bar of 5).
     * Merge vertically adjacent pieces with substantial horizontal overlap.
     * Side-by-side digits remain separate and are rejected below. */
    (void)second_area;
    memset(s->queue,0,(next_label+1)*sizeof(s->queue[0]));
    s->queue[selected]=1;
    int base_l=r.x,base_r=r.x+r.width-1,base_t=r.y,base_b=r.y+r.height-1;
    for(int i=0;i<piece_count;++i) {
        component_t c=pieces[i];
        if(c.label==selected)continue;
        int overlap=imin(base_r,c.r)-imax(base_l,c.l)+1;
        int gap=imax(0,imax(c.t-base_b-1,base_t-c.b-1));
        if(overlap*3>=imin(r.width,c.r-c.l+1) && overlap>0 &&
           gap<=imax(3,r.height/4) &&
           (c.b<=base_t+imax(2,r.height/10) || c.t>=base_b-imax(2,r.height/10))) {
            s->queue[c.label]=1;
            int right=imax(r.x+r.width-1,c.r),bottom=imax(r.y+r.height-1,c.b);
            r.x=imin(r.x,c.l);r.y=imin(r.y,c.t);
            r.width=right-r.x+1;r.height=bottom-r.y+1;r.area+=c.area;
        }
    }
    for(int i=0;i<piece_count;++i)
        if(!s->queue[pieces[i].label] && pieces[i].area*3>r.area) {
            r.reason="MULTIPLE_OBJECTS";return r;
        }
    if(r.width>90||r.height>90) {r.reason="TOO_LARGE";return r;}
    uint8_t normalized[784] = {0};
    float scale = 20.0f / imax(r.width,r.height);
    int nw=imax(1,(int)lroundf(r.width*scale)), nh=imax(1,(int)lroundf(r.height*scale));
    int ox=(28-nw)/2, oy=(28-nh)/2;
    /* Area supersampling preserves thin strokes when downscaling to MNIST size. */
    for (int y=0;y<nh;++y)
        for (int x=0;x<nw;++x) {
            float sum=0;
            for(int yy=0;yy<4;++yy)
                for(int xx=0;xx<4;++xx) {
                    int sx=r.x+imin(r.width-1,(int)((x+(xx+0.5f)/4)*r.width/nw));
                    int sy=r.y+imin(r.height-1,(int)((y+(yy+0.5f)/4)*r.height/nh));
                    int p=sy*120+sx;
                    if(s->queue[s->labels[p]])
                        sum+=fminf(1.0f,fmaxf(0,(float)(high-s->gray[p])/r.contrast));
                }
            normalized[(oy+y)*28+ox+x]=(uint8_t)lroundf(sum*255/16);
        }
    /* Match MNIST's centre of mass, with clipping avoided by the four-pixel margin. */
    float mass=0,mx=0,my=0;
    for(int y=0;y<28;++y) for(int x=0;x<28;++x) {
        int v=normalized[y*28+x]; mass+=v; mx+=x*v; my+=y*v;
    }
    if (mass<255*5) { r.reason="TOO_SMALL"; return r; }
    int dx=(int)lroundf(13.5f-mx/mass), dy=(int)lroundf(13.5f-my/mass);
    dx=imax(-ox,imin(28-ox-nw,dx)); dy=imax(-oy,imin(28-oy-nh,dy));
    for(int y=0;y<28;++y) for(int x=0;x<28;++x) {
        int tx=x+dx,ty=y+dy;
        if(tx<0||tx>=28||ty<0||ty>=28) continue;
        if(mirror) tx=27-tx;
        for(unsigned k=0;k<(rotation&3);++k) { int z=tx; tx=27-ty; ty=z; }
        input[ty*28+tx]=normalized[y*28+x];
    }
    r.valid=true; r.reason="OK";
    return r;
}
