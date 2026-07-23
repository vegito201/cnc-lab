/* main_scurve -- toan tuyen theo paper Chen 2013:
   G-code -> segments -> bo goc -> scurve_lookahead (Step 1-2)
          -> scurve_plan_all (Step 3: B/A/C) -> vong tick eq (17) -> CSV.
   Build: gcc -Wall -Wextra main_scurve.c profile.c motion.c gcode.c -o main_scurve -lm */
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "gcode.h"
#include "motion.h"
#include "profile.h"

int main(int argc, char *argv[])
{
    const char *file = (argc > 1) ? argv[1] : "test.gcode";
    struct GCommand cmds[MAX_SEGMENTS];
    struct Segment  segs[MAX_SEGMENTS];
    struct Block    blocks[MAX_SEGMENTS];
    struct SegPlan  plans[MAX_SEGMENTS];

    int n  = read_gcode_file(file, cmds, MAX_SEGMENTS);
    int ns = build_segments(cmds, n, segs);
    ns = apply_corner_rounding(segs, ns);
    int nbl = scurve_lookahead(segs, ns, LOOKAHEAD_PATHS, blocks);
    float lech = scurve_plan_all(segs, ns, blocks, nbl, plans);

    printf("%d doan, %d block, sai so quang duong lon nhat: %.6f mm\n",
           ns, nbl, lech);
    printf("%-4s %6s->%6s->%6s %4s %4s %4s %2s %5s | J1' J2' (tran %.0f)\n",
           "seg", "vs", "vm'", "ve", "na", "nb", "nc", "k", "tick", MAX_JERK);
    int i;
    for (i = 0; i < ns; i++)
        printf("%-4d %6.2f->%6.2f->%6.2f %4d %4d %4d %2d %5d | %7.0f %7.0f\n",
               i+1, plans[i].vs, plans[i].vmp, plans[i].ve,
               plans[i].na, plans[i].nb, plans[i].nc, plans[i].k, plans[i].N,
               plans[i].J1p, plans[i].J2p);

    FILE *csv = fopen("trajectory.csv", "w");
    if (csv) fprintf(csv, "t_ms,x,y,v\n");
    long tick = 0;
    for (i = 0; i < ns; i++) {
        struct Segment *sg = &segs[i];
        struct SegPlan *p  = &plans[i];
        float cx=0, cy=0, R=0, a0=0, sw=0, s=0;
        if (sg->is_arc) {
            cx = sg->x0 + sg->i_off;  cy = sg->y0 + sg->j_off;
            R  = sqrtf(sg->i_off*sg->i_off + sg->j_off*sg->j_off);
            arc_sweep(sg->x0, sg->y0, sg->x1, sg->y1, cx, cy, sg->clockwise, &a0, &sw);
        }
        int mtick;
        for (mtick = 0; mtick < p->N; mtick++) {
            float v = scurve_v_tick(mtick, p->vs, p->vmp, p->ve,
                                    p->na, p->nb, p->nc, p->k,
                                    p->J1p, p->J2p);
            s += v * TS;
            float alpha = s / sg->length;  if (alpha > 1.0f) alpha = 1.0f;
            float x, y;
            if (sg->is_arc) { float g = a0 + alpha*sw; x = cx + R*cosf(g); y = cy + R*sinf(g); }
            else { x = sg->x0 + alpha*(sg->x1 - sg->x0); y = sg->y0 + alpha*(sg->y1 - sg->y0); }
            if (csv) fprintf(csv, "%.1f,%.4f,%.4f,%.3f\n", (float)tick, x, y, v);
            tick++;
        }
    }
    if (csv) fclose(csv);
    printf("Total time: %.1f ms (%.2f sec), %ld ticks\nSaved: trajectory.csv\n",
           (float)tick, tick/1000.0f, tick);
    return 0;
}
