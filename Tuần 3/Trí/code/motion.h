#ifndef MOTION_H
#define MOTION_H

#include "gcode.h"

/* =============================================================
   SEGMENT GEOMETRY
   ============================================================= */

/* Mot doan chuyen dong da resolve day du hinh hoc.
   Duoc xay TRUOC khi noi suy, de look-ahead nhin duoc toan bo duong chay
   (biet doan sau re huong nao thi moi biet doan truoc duoc phep ra nhanh bao nhieu). */
struct Segment {
    int   is_arc;
    int   clockwise;             /* chi co nghia khi is_arc: G2 = 1, G3 = 0 */
    float x0, y0, x1, y1;        /* diem dau, diem cuoi */
    float i_off, j_off;          /* offset tam cung (da resolve tu R neu can) */
    float feedrate;              /* mm/min */
    float length;                /* do dai doan (mm) */
    float tan_in_x,  tan_in_y;   /* vector tiep tuyen don vi tai diem DAU */
    float tan_out_x, tan_out_y;  /* vector tiep tuyen don vi tai diem CUOI */
    float corner_r;              /* ban kinh bo goc tai DIEM CUOI doan (0 = khong bo) */
    float v_entry, v_exit;       /* ket qua look-ahead (mm/s) */
};

void arc_sweep(float x0, float y0, float x1, float y1,
               float cx, float cy, int clockwise,
               float *out_angle_start, float *out_sweep);
int arc_center_from_r(float x0, float y0, float x1, float y1,
                      float r, int clockwise,
                      float *out_i, float *out_j);
float junction_velocity(float t0x, float t0y, float t1x, float t1y);

/* Section 3, Step 2 cua paper: block = day cac doan nam giua 2 break path
   (break path = doan Type 7, ngan toi muc khong noi suy noi).
   perfect = chi so doan "perfect path" dai nhat trong block (Type 1-6,
   duoc phep chinh v_m/J ma khong dung 2 dau) -- noi hap thu sai so;
   perfect = -1 neu block khong co doan nao Type 1-6. */
struct Block { int dau, cuoi; int perfect; };
int divide_blocks(struct Segment *segs, int n, struct Block *blocks);
int scurve_lookahead(struct Segment *segs, int n, int j_window,
                     struct Block *blocks);

/* Ke hoach 1 doan sau Step 3: du bo so de scurve_v_tick phat tung tick.
   N = (2+k)*(na+nb) + nc = tong tick cua doan. */
struct SegPlan {
    int   na, nb, nc, k, N;
    float vs, vmp, ve;        /* van toc vao / dinh (da va) / ra */
    float J1p, J2p;           /* jerk tinh lai (eq 12/13/21/24) */
};
/* Section 3 Step 3: trong moi block quet XUOI bang Algorithm B toi truoc
   perfect path, quet NGUOC bang Algorithm A ve sau perfect path, rieng
   perfect path chot bang Algorithm C. Tra ve sai so quang duong lon nhat
   |dist - L| (mm) de nguoi goi tu cham. */
float scurve_plan_all(struct Segment *segs, int n,
                      const struct Block *blocks, int nblocks,
                      struct SegPlan *plans);
int build_segments(struct GCommand *commands, int cmd_count, struct Segment *segs);
int apply_corner_rounding(struct Segment *segs, int n);

#endif /* MOTION_H */
