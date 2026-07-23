#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "motion.h"
#include "profile.h"

/*
 * arc_sweep: goc bat dau va goc quet cua cung tron.
 * angle_start = goc cua diem dau nhin tu tam (atan2).
 * sweep = goc quay tu dau den cuoi; ep dau theo chieu quay:
 *   G2 (CW, thuan kim dong ho)  -> sweep phai AM
 *   G3 (CCW, nguoc kim dong ho) -> sweep phai DUONG
 * (atan2 tra ve goc trong [-pi, pi] nen hieu 2 goc co the sai dau,
 *  phai cong/tru 2*pi de dung chieu.)
 */
void arc_sweep(float x0, float y0, float x1, float y1,
               float cx, float cy, int clockwise,
               float *out_angle_start, float *out_sweep) {
    float angle_start = atan2f(y0 - cy, x0 - cx);
    float sweep = atan2f(y1 - cy, x1 - cx) - angle_start;
    if (clockwise)  { if (sweep > 0) sweep -= 2.0f * 3.14159265f; }
    else            { if (sweep < 0) sweep += 2.0f * 3.14159265f; }
    *out_angle_start = angle_start;
    *out_sweep       = sweep;
}

/*
 * arc_center_from_r: tinh offset tam (I,J) khi G-code cho ban kinh R.
 *
 * Co 2 duong tron ban kinh R di qua 2 diem -> 2 tam ung vien.
 * Tam nam tren duong trung truc cua day cung, cach trung diem mot khoang
 *   h = sqrt(R^2 - (d/2)^2)   (Pythagore, d = do dai day cung)
 * Chon tam ben nao: quy uoc R>0 = cung nho hon 180 do, R<0 = lon hon 180 do,
 * ket hop voi chieu quay G2/G3 (gop trong bien 'sign').
 *
 * Tra ve 0 neu hinh hoc vo ly (2 diem xa hon duong kinh 2R).
 */
int arc_center_from_r(float x0, float y0, float x1, float y1,
                      float r, int clockwise,
                      float *out_i, float *out_j) {
    float dx = x1 - x0, dy = y1 - y0;
    float d  = sqrtf(dx*dx + dy*dy);
    if (d < 0.001f || d > 2.0f * fabsf(r) + 0.01f) {
        printf("  [WARN] arc geometry invalid: d=%.2f R=%.2f\n", d, fabsf(r));
        return 0;
    }
    float h_sq = r*r - (d/2.0f)*(d/2.0f);
    float h    = (h_sq > 0.0f) ? sqrtf(h_sq) : 0.0f;
    float sign = ((r > 0) ? 1.0f : -1.0f) * (clockwise ? 1.0f : -1.0f);
    /* (px,py) = phap tuyen don vi cua day cung (quay day cung 90 do) */
    float px = -dy / d, py = dx / d;
    *out_i = (x0 + x1) / 2.0f + sign * h * px - x0;
    *out_j = (y0 + y1) / 2.0f + sign * h * py - y0;
    return 1;
}

/* ====== LOOK-AHEAD: eq (34) van toc diem cua; 2 luot quet nam o scurve_lookahead ====== */
float junction_velocity(float t0x, float t0y, float t1x, float t1y) {
    /* goc giua 2 tiep tuyen: cos(alpha) = t0 . t1 (2 vector don vi) */
    float dot = t0x*t1x + t0y*t1y;
    if (dot >  1.0f) dot =  1.0f;   /* chong loi lam tron truoc acos */
    if (dot < -1.0f) dot = -1.0f;
    float alpha = acosf(dot);
    if (alpha < 0.001f) return 1.0e9f;  /* gan thang: tra so lon, feedrate se cap sau */
    return MAX_ACCELERATION * TS / (2.0f * sinf(alpha * 0.5f));
}

int build_segments(struct GCommand *commands, int cmd_count, struct Segment *segs) {
    float current_x = 0.0f, current_y = 0.0f;
    float feedrate  = RAPID_FEEDRATE;
    int   incremental = 0;   /* 0 = G90 absolute, 1 = G91 incremental */
    int   n = 0, i;

    for (i = 0; i < cmd_count; i++) {
        struct GCommand *cmd = &commands[i];

        if      (cmd->mode_change == 90) incremental = 0;
        else if (cmd->mode_change == 91) incremental = 1;
        if (cmd->code[0] == '\0') continue;   /* dong chi co G90/G91 */
        if (cmd->has_f) feedrate = cmd->f;    /* feedrate modal */

        /* toa do dich: G91 cong don, G90 thay the; thieu X/Y thi giu nguyen */
        float target_x = current_x, target_y = current_y;
        if (incremental) {
            if (cmd->has_x) target_x += cmd->x;
            if (cmd->has_y) target_y += cmd->y;
        } else {
            if (cmd->has_x) target_x = cmd->x;
            if (cmd->has_y) target_y = cmd->y;
        }

        struct Segment *s = &segs[n];
        memset(s, 0, sizeof(*s));
        s->is_arc   = (cmd->code[1] == '2' || cmd->code[1] == '3');
        s->x0 = current_x;  s->y0 = current_y;
        s->x1 = target_x;   s->y1 = target_y;
        s->feedrate = feedrate;

        /* cap nhat vi tri ngay: neu doan bi bo qua (continue ben duoi)
           thi vi tri van phai nhay den dich */
        current_x = target_x;
        current_y = target_y;

        if (s->is_arc) {
            s->clockwise = (cmd->code[1] == '2');
            float i_off = cmd->has_i ? cmd->i : 0.0f;
            float j_off = cmd->has_j ? cmd->j : 0.0f;
            if (cmd->has_r &&
                !arc_center_from_r(s->x0, s->y0, s->x1, s->y1,
                                   cmd->r, s->clockwise, &i_off, &j_off))
                continue;  /* arc loi hinh hoc: bo qua */
            s->i_off = i_off;
            s->j_off = j_off;

            float radius = sqrtf(i_off*i_off + j_off*j_off);
            if (radius < 0.001f) continue;  /* khong co tam hop le */

            float angle_start, sweep;
            arc_sweep(s->x0, s->y0, s->x1, s->y1,
                      s->x0 + i_off, s->y0 + j_off, s->clockwise,
                      &angle_start, &sweep);
            s->length = fabsf(sweep) * radius;  /* do dai cung = R * goc quet */

            float angle_end = angle_start + sweep;
            float dir = s->clockwise ? -1.0f : 1.0f;
            s->tan_in_x  = dir * -sinf(angle_start);
            s->tan_in_y  = dir *  cosf(angle_start);
            s->tan_out_x = dir * -sinf(angle_end);
            s->tan_out_y = dir *  cosf(angle_end);
        } else {
            float dx = s->x1 - s->x0, dy = s->y1 - s->y0;
            float L  = sqrtf(dx*dx + dy*dy);
            if (L < 0.001f) continue;  /* doan khong di chuyen (vd G90 G0 X0 Y0 tu goc) */
            s->length    = L;
            s->corner_r  = cmd->has_r ? fabsf(cmd->r) : 0.0f;
            s->tan_in_x  = dx / L;
            s->tan_in_y  = dy / L;
            s->tan_out_x = s->tan_in_x;
            s->tan_out_y = s->tan_in_y;
        }

        if (++n >= MAX_SEGMENTS) break;
    }
    return n;
}

/* =============================================================
   CORNER ROUNDING (bo goc bang cung tron)
   =============================================================

   Cu phap kieu Fanuc: "G01 X.. Y.. R8" -> goc tai DIEM CUOI cua doan
   nay duoc bo tron ban kinh 8mm. Vi du trong sach CNC Programming
   Tutorials (Thanh Tran, trang 13): R8/R10 tai P1, P2, P10, P13.

   Hinh hoc: goc tai dinh P giua 2 doan thang, huong vao u, huong ra w,
   goc doi huong alpha (cos alpha = u.w nhu eq 36). Cung tron ban kinh R
   tiep xuc ca 2 canh, hai tiep diem cach P mot doan:
       d = R * tan(alpha/2)
   Doan truoc bi cat ngan d o cuoi, doan sau cat ngan d o dau,
   chen 1 segment cung tron (dai R*alpha) vao giua.

   Loi ich voi look-ahead: 2 diem noi moi (thang->cung, cung->thang)
   co tiep tuyen LIEN TUC (alpha = 0) -> junction_velocity() tra ve
   "khong gioi han" -> dao giu nguyen toc do qua goc thay vi phai
   giam gan ve 0 nhu goc nhon. Luu y gia toc huong tam v^2/R phai
   <= a_max: voi R8, v=50 -> 312 mm/s^2 < 500, on (chua enforce
   trong code, ghi nhan viec tuong lai).
*/
int apply_corner_rounding(struct Segment *segs, int n) {
    int i;
    for (i = 0; i + 1 < n; i++) {
        struct Segment *a = &segs[i];
        float r = a->corner_r;
        if (r <= 0.0f || a->is_arc || segs[i+1].is_arc) continue;
        if (n >= MAX_SEGMENTS) break;

        /* goc doi huong tai dinh goc (nhu junction_velocity) */
        float ux = a->tan_out_x,      uy = a->tan_out_y;
        float wx = segs[i+1].tan_in_x, wy = segs[i+1].tan_in_y;
        float dot = ux*wx + uy*wy;
        if (dot >  1.0f) dot =  1.0f;
        if (dot < -1.0f) dot = -1.0f;
        float alpha = acosf(dot);
        if (alpha < 0.01f) continue;   /* gan thang: khong co goc de bo */

        /* tiep diem cach dinh d = R*tan(alpha/2); phai vua trong 2 doan */
        float d = r * tanf(alpha * 0.5f);
        if (d > 0.9f * a->length || d > 0.9f * segs[i+1].length) {
            printf("  [WARN] bo goc R%.1f khong du cho (d=%.2f mm), giu goc nhon\n", r, d);
            continue;
        }

        float px = a->x1, py = a->y1;               /* dinh goc cu */
        float ax = px - ux * d, ay = py - uy * d;   /* tiep diem VAO cung */
        float bx = px + wx * d, by = py + wy * d;   /* tiep diem RA cung */

        /* chieu quay: cross > 0 = re trai = CCW (G3), cross < 0 = CW (G2) */
        float cross = ux*wy - uy*wx;
        int cw = (cross < 0.0f);
        /* tam = tiep diem vao + R * phap tuyen (phap tuyen huong ve phia re) */
        float nx = cw ? uy : -uy;
        float ny = cw ? -ux : ux;
        float cx = ax + r * nx, cy = ay + r * ny;

        /* don cho trong mang: day cac doan tu i+1 lui 1 vi tri */
        memmove(&segs[i+2], &segs[i+1], (size_t)(n - i - 1) * sizeof(*segs));
        n++;

        /* cat ngan doan truoc (giu tangent) */
        a->x1 = ax;  a->y1 = ay;
        a->length  -= d;
        a->corner_r = 0.0f;

        /* cat ngan doan sau */
        segs[i+2].x0 = bx;  segs[i+2].y0 = by;
        segs[i+2].length -= d;

        /* chen cung tron tiep tuyen lien tuc 2 dau */
        struct Segment *arc = &segs[i+1];
        memset(arc, 0, sizeof(*arc));
        arc->is_arc    = 1;
        arc->clockwise = cw;
        arc->x0 = ax;  arc->y0 = ay;
        arc->x1 = bx;  arc->y1 = by;
        arc->i_off = cx - ax;
        arc->j_off = cy - ay;
        arc->feedrate  = a->feedrate;
        arc->length    = r * alpha;
        arc->tan_in_x  = ux;  arc->tan_in_y  = uy;
        arc->tan_out_x = wx;  arc->tan_out_y = wy;

        printf("  bo goc R%.1f tai (%.2f, %.2f): cung %s dai %.2f mm\n",
               r, px, py, cw ? "G2" : "G3", arc->length);
        i++;   /* nhay qua cung vua chen */
    }
    return n;
}



/* divide_blocks -- Section 3 Step 2:
   1. quet tung doan, phan loai Type (bang v_entry/v_exit tu look-ahead);
   2. Type 7 = break path: cat block tai do, va ep van toc 2 dau break
      theo eq (33) (vs + ve <= L/Ts, chia doi moi ben) -- ep luon sang
      doan hang xom de moi noi lien mach;
   3. moi block ghi nhan perfect path dai nhat (Type 1-6).
   Tra ve so block. VI DU: test.gcode khong co Type 7 -> 1 block [0..17],
   perfect = seg 7 (L=75.27, Type 1). */
int divide_blocks(struct Segment *segs, int n, struct Block *blocks)
{
    int so_block = 0, dau = 0, i;

    for (i = 0; i <= n; i++) {
        int la_break = 0;
        if (i < n) {
            float vm = segs[i].feedrate / 60.0f;
            int na, nb, k;
            scurve_steps(segs[i].v_entry, vm, segs[i].v_exit, &na, &nb, &k);
            la_break = (scurve_type(segs[i].length, segs[i].v_entry, vm,
                                    segs[i].v_exit, na, nb, k) == 7);
        }
        if (i == n || la_break) {
            if (i > dau) {                       /* chot block [dau .. i-1] */
                struct Block *b = &blocks[so_block];
                b->dau = dau;  b->cuoi = i - 1;  b->perfect = -1;
                float dai_nhat = -1.0f;
                int j;
                for (j = dau; j < i; j++) {
                    float vm = segs[j].feedrate / 60.0f;
                    int na, nb, k;
                    scurve_steps(segs[j].v_entry, vm, segs[j].v_exit, &na, &nb, &k);
                    int t = scurve_type(segs[j].length, segs[j].v_entry, vm,
                                        segs[j].v_exit, na, nb, k);
                    if (t >= 1 && t <= 6 && segs[j].length > dai_nhat) {
                        dai_nhat = segs[j].length;
                        b->perfect = j;
                    }
                }
                so_block++;
            }
            if (i < n && la_break) {
                /* eq (33) NGUYEN VAN (Tri chinh): rang buoc tren TONG vs+ve <= L/Ts;
                   chi ra tay khi TONG vuot, cat theo TI LE (dau cao ganh nhieu) */
                float ngan_sach = segs[i].length / TS;
                float tong = segs[i].v_entry + segs[i].v_exit;
                if (tong > ngan_sach) {
                    float ti_le = ngan_sach / tong;
                    segs[i].v_entry *= ti_le;
                    segs[i].v_exit  *= ti_le;
                    if (i > 0)     segs[i-1].v_exit  = segs[i].v_entry;
                    if (i + 1 < n) segs[i+1].v_entry = segs[i].v_exit;
                }
            }
            dau = i + 1;
        }
    }
    return so_block;
}


/* scurve_lookahead -- Section 3 Step 1 + 2 luot quet, ban S-curve.
   Khac compute_lookahead (trapezoid, giu nguyen de doi chieu) o 3 diem:
   1. reach dung scurve_reach (nhi phan nguoc eq (8)) thay sqrt(v^2+2aL);
   2. cua so truot j doan: tai bien cua so ep v=0 (chua thay tuong lai
      thi phai du suc dung) -- j = LOOKAHEAD_PATHS trong config.h;
   3. sau 2 luot quet goi divide_blocks: cat block tai Type 7 + ep eq (33).
   Tra ve so block (blocks do nguoi goi cap). */
int scurve_lookahead(struct Segment *segs, int n, int j_window,
                     struct Block *blocks)
{
    float vj[MAX_SEGMENTS + 1];
    int i;

    /* Step 1: van toc diem cua (eq 34), chan boi feedrate 2 doan ke */
    vj[0] = 0.0f;  vj[n] = 0.0f;
    for (i = 1; i < n; i++) {
        float v_cua = junction_velocity(segs[i-1].tan_out_x, segs[i-1].tan_out_y,
                                        segs[i].tan_in_x,   segs[i].tan_in_y);
        float tran = segs[i-1].feedrate / 60.0f;
        if (segs[i].feedrate / 60.0f < tran) tran = segs[i].feedrate / 60.0f;
        vj[i] = (v_cua < tran) ? v_cua : tran;
    }
    /* bien cua so truot: chua biet gi sau do -> phai dung duoc */
    for (i = j_window; i < n; i += j_window) vj[i] = 0.0f;

    /* quet LUI (tinh than Algorithm A): doan i phai GIAM kip xuong vj[i+1] */
    for (i = n - 1; i >= 0; i--) {
        float v_max = scurve_reach(vj[i+1], segs[i].feedrate / 60.0f, segs[i].length);
        if (vj[i] > v_max) vj[i] = v_max;
    }
    /* quet TOI (tinh than Algorithm B): doan i phai TANG kip tu vj[i] */
    for (i = 0; i < n; i++) {
        float v_max = scurve_reach(vj[i], segs[i].feedrate / 60.0f, segs[i].length);
        if (vj[i+1] > v_max) vj[i+1] = v_max;
    }
    for (i = 0; i < n; i++) {
        segs[i].v_entry = vj[i];
        segs[i].v_exit  = vj[i+1];
    }
    return divide_blocks(segs, n, blocks);   /* Step 2 */
}


/* --- eq (8) voi bo so cua plan: quang duong profile thuc di --- */
static float plan_dist(const struct SegPlan *p)
{
    return 0.5f*(p->vs + p->vmp)*(2 + p->k)*p->na*TS
         + 0.5f*(p->ve + p->vmp)*(2 + p->k)*p->nb*TS
         + p->vmp*p->nc*TS;
}

/* Quet XUOI (Algorithm B): vs da chot, don sai so lam tron ve phia ra.
   Truong hop nb = 0 (ve == v_m: Type 4 tang / deu): khong co vung giam
   de hap thu -> v_m' va ve' DI CUNG NHAU (eq (11) voi nb = 0, nc lam
   tron LEN de v_m' <= v_m); moi noi phia sau nhan ve' -- day chinh la
   tinh than Algorithm B ("adjusting the end point feedrate"). */
/* =====================================================================
   plan_C_doan -- dong so MOI doan bang 2.1.2 (phep cua Algorithm C):
   hai dau lay tu look-ahead (Step 1-2, da <= tran eq34, chi-duoc-HA),
   KHONG NANG; phan du lam tron nc do DINH + JERK cua chinh doan nuot
   (eq 9 tron LEN + eq 11-13). Can cu: dinh nghia Algorithm A/B/C trong
   3.1 khong trich 2.1.3/2.1.4 (Tri phat hien); Table 2 cua paper khong
   co dau vet nang (khop gat = dung tran eq34).
   2.1.3/2.1.4 chi con vai du phong chieu-HA khi 2.1.2 bo tay
   (nc am / dinh tut / jerk no): ket-o-dinh roi cruise. */
static void plan_C_doan(struct Segment *sg, struct SegPlan *p)
{
    float vm = sg->feedrate/60.0f, vs = sg->v_entry, ve = sg->v_exit;
    float L = sg->length;
    vm = scurve_peak(vs, ve, vm, L);            /* Type 2/3/5/6: dinh kha thi */
    scurve_steps(vs, vm, ve, &p->na, &p->nb, &p->k);

    int hong = 0;
    if (p->nb > 0) {                            /* co vung phanh that: 2.1.2 day du */
        p->nc = scurve_nc(L, vs, vm, ve, p->na, p->nb, p->k);   /* eq (9) tron LEN */
        if (p->nc < 0) hong = 1;
        if (!hong) {
            p->vmp = scurve_vm_fix(L, vs, ve, p->na, p->nb, p->nc, p->k,
                                   &p->J1p, &p->J2p);           /* eq (11)-(13) */
            /* van do TRUC TIEP trieu chung (bai hoc P2): dinh lech nhe
               duoi vs/ve la gon lan tan hop le -- benh that la J' no,
               da co coi J' rieng bat. */
            if (p->vmp > vm + 1e-3f)              hong = 1;   /* doi qua kha nang voi toi */
            if (fabsf(p->J1p) > MAX_JERK*1.001f)  hong = 1;   /* jerk leo no */
            if (fabsf(p->J2p) > MAX_JERK*1.001f)  hong = 1;   /* jerk phanh no */
        }
        if (!hong) {
            p->vs = vs;  p->ve = ve;            /* HAI DAU GIU NGUYEN - khong nang */
            if (p->na == 0) p->vs = p->vmp;     /* decel thuan: so ghi so thuc thi */
        }
    }
    if (p->nb == 0 || hong) {
        /* nb = 0 (khong co vung phanh -> ve thuc = DINH) hoac 2.1.2 bo tay:
           ket-o-dinh = 2.1.2 dang suy bien (chinh dinh, ve di theo dinh).
           Chi duoc HA so voi muc tieu. */
        scurve_steps(vs, vm, ve, &p->na, &p->nb, &p->k);
        p->nb = 0;
        int   nc0 = (int)((2.0f*L - (2+p->k)*p->na*(vs+vm)*TS) / (2.0f*vm*TS));
        float tot = -1.0f;  int nc_tot = nc0 + 1;  int thu;
        for (thu = nc0; thu <= nc0 + 1; thu++) {
            if (thu < 0) continue;
            float v = (2.0f*L - (2+p->k)*p->na*vs*TS) / ((2.0f*thu + (2+p->k)*p->na)*TS);
            if (v <= vm + 1e-4f && v > tot) { tot = v; nc_tot = thu; }
        }
        p->nc  = nc_tot;
        p->vmp = (2.0f*L - (2+p->k)*p->na*vs*TS) / ((2.0f*p->nc + (2+p->k)*p->na)*TS);
        if (p->na > 0 && p->vmp < vs) {         /* van cruise: buoc noi nho nhat */
            p->na = 0;  p->k = 0;
            int   nc1 = (int)(L / (vm*TS)) + 1;
            float v1 = L / (nc1*TS), v2 = L / ((nc1+1)*TS);
            if (fabsf(v2 - vs) < fabsf(v1 - vs)) { p->nc = nc1+1; p->vmp = v2; }
            else                                 { p->nc = nc1;   p->vmp = v1; }
        }
        p->vs  = vs;
        p->ve  = p->vmp;
        p->J2p = 0.0f;
        p->J1p = (p->na > 0) ? (p->vmp - vs)/((1+p->k)*(float)p->na*p->na*TS*TS) : 0.0f;
    }
    if (p->na == 0 && p->nb == 0) {             /* cruise: so sach = so thuc thi */
        p->vs = p->vmp;  p->ve = p->vmp;
    }
    if (p->nc < 0) p->nc = 0;
    p->N = (2+p->k)*(p->na + p->nb) + p->nc;
    sg->v_exit = p->ve;
}

float scurve_plan_all(struct Segment *segs, int n,
                      const struct Block *blocks, int nblocks,
                      struct SegPlan *plans)
{
    int i;
    (void)blocks; (void)nblocks;   /* block van dung o divide (Type 7 cat, eq 33);
                                      plan gio la mot luot xuoi 2.1.2 moi doan */
    for (i = 0; i < n; i++) {
        plan_C_doan(&segs[i], &plans[i]);
        if (i + 1 < n) segs[i+1].v_entry = plans[i].ve;
    }

    /* CHUONG eq (34) hau-plan (Tri yeu cau): binh thuong phai IM LANG. */
    for (i = 1; i < n; i++) {
        float tran_goc = junction_velocity(segs[i-1].tan_out_x, segs[i-1].tan_out_y,
                                           segs[i].tan_in_x,   segs[i].tan_in_y);
        if (plans[i].vs > tran_goc * 1.001f)
            printf("  [CANH BAO eq34] khop %d: v=%.3f vuot tran goc %.3f (+%.1f%%)\n",
                   i + 1, plans[i].vs, tran_goc, (plans[i].vs / tran_goc - 1) * 100);
    }

    float lech_max = 0.0f;
    for (i = 0; i < n; i++) {
        float d = plan_dist(&plans[i]) - segs[i].length;
        if (d < 0) d = -d;
        if (d > lech_max) lech_max = d;
    }
    return lech_max;
}
