/*
 * =============================================================
 * GCode Trajectory Calculator -- Trapezoid Velocity + Look-ahead
 * =============================================================
 *
 * LUONG XU LY:
 *   file.gcode
 *      |
 *      v  parse_line()          <- doc tung dong thanh GCommand
 *   GCommand[]
 *      |
 *      v  build_segments()      <- resolve hinh hoc (toa do, tam arc, do dai)
 *   Segment[]
 *      |
 *      v  compute_lookahead()   <- tinh van toc tai cac diem noi
 *      |
 *      v  interpolate_segment() <- tinh quy dao theo thoi gian
 *   trajectory.csv              <- t_ms, x, y, v (moi 1ms 1 dong)
 *      |
 *      v  visualize.py          <- ve do thi
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

/* =============================================================
   HANG SO (cac knob hieu chinh)
   ============================================================= */

/* Gia toc toi da cua may (mm/s^2).
   Nho -> tang/giam toc cham, em may. Lon -> nhanh nhung rung.
   500 la muc trung binh cho CNC hobby. */
#define MAX_ACCELERATION  500.0f

/* Feedrate khoi tao (mm/min). 3000 mm/min = 50 mm/s. */
#define RAPID_FEEDRATE   3000.0f

/* Buoc thoi gian noi suy: 1ms = 1 diem CSV.
   Tren phan cung thuc te day la chu ky ngat cua Interpolator ISR. */
#define DT_MS               1.0f

/* Thoi gian cho phep doi huong tai goc re (s) -- xem junction_velocity().
   Dat = chu ky mau Ts (1ms) dung nhu eq 34 cua paper Chen 2013:
   toan bo cu doi huong phai xong trong 1 tick -> bao thu tuyet doi,
   goc sac tuyet doi nhung qua goc rat cham (goc 90 do ~ 0.35 mm/s).
   Tang len (vd 0.02f = 20ms) = qua goc nhanh hon nhung goc bi bo tron. */
#define JUNCTION_T         0.001f  /* 1ms = Ts, giong paper */

#define MAX_SEGMENTS        100

/* =============================================================
   G-CODE PARSING
   ============================================================= */

/* Mot lenh G-code sau khi parse.
   Lenh thang (G0/G1) chi dung x,y,f; lenh cung (G2/G3) dung them i,j,r.
   ponytail: 1 struct phang thay cho union linear/arc nhu ban goc --
   vai float khong dung (i,j,r nam im khi lenh thang) re hon viec
   moi cho doc du lieu phai viet code 2 nhanh arc/linear. */
struct GCommand {
    char  code[4];      /* "G0".."G3", "" neu dong khong co lenh di chuyen */
    int   mode_change;  /* 0 = khong doi, 90 = G90 absolute, 91 = G91 incremental */
    float x, y;         /* toa do dich */
    float i, j;         /* offset tu diem dau den tam cung: tam = (x0+I, y0+J) */
    float r;            /* ban kinh cung (cach viet thay cho I,J) */
    float f;            /* feedrate (mm/min) */
    int   has_x, has_y, has_i, has_j, has_r, has_f;  /* 1 neu dong co tham so do */
};

/*
 * parse_line: doc 1 dong G-code thanh GCommand.
 *
 * Cac dac diem cua G-code phai xu ly:
 *   1. N number: so thu tu dong, tuy chon ("N40 G01 X10") -> bo qua.
 *   2. Comment trong ngoac don: "G01 X10 (di den diem A)" -> xoa truoc khi parse.
 *   3. Modal: dong khong co G0-G3 thi dung lenh G cuoi cung con hieu luc.
 *      Vi du "N70 Y10" sau mot dong G1 -> hieu la G1 Y10.
 *   4. Nhieu G tren 1 dong: "G90 G01 X10" -> xu ly tat ca.
 *
 * Tham so:
 *   line       - chuoi 1 dong G-code
 *   modal_code - G code dang hieu luc tu dong truoc
 */
struct GCommand parse_line(char *line, const char *modal_code) {
    struct GCommand cmd;
    char clean[256];
    char *p, *w;
    int depth, g_num, k;

    memset(&cmd, 0, sizeof(cmd));

    /* buoc 1: bo N number dau dong */
    p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == 'N') {
        p++;
        while (*p >= '0' && *p <= '9') p++;
    }

    /* buoc 2: copy sang buffer 'clean', bo ki tu nam trong ngoac don.
       depth dem so ngoac mo chua dong (ho tro ngoac long nhau). */
    w = clean;
    depth = 0;
    for (; *p; p++) {
        if (*p == '(') { depth++; continue; }
        if (*p == ')') { depth--; continue; }
        if (depth == 0) *w++ = *p;
    }
    *w = '\0';

    /* buoc 3: tim tat ca G code tren dong.
       strchr nhay den tung chu 'G', sscanf doc so nguyen ngay sau no. */
    for (p = clean; (p = strchr(p, 'G')) != NULL; p++) {
        sscanf(p, "G%d", &g_num);
        if (g_num == 90 || g_num == 91) cmd.mode_change = g_num;
        else if (g_num >= 0 && g_num <= 3) snprintf(cmd.code, 4, "G%d", g_num);
    }

    /* buoc 4: dong khong co G0-G3 -> ke thua lenh modal cua dong truoc */
    if (cmd.code[0] == '\0' && modal_code && modal_code[0] != '\0')
        memcpy(cmd.code, modal_code, 4);
    if (cmd.code[0] == '\0') return cmd;

    /* buoc 5: doc cac tham so. Voi moi chu cai trong "XYIJRF":
       strchr tim vi tri chu cai do, sscanf doc so thap phan ngay sau.
       vals[k]/flags[k] tro den field tuong ung trong cmd -- them tham so
       moi (vd Z) chi can them 1 chu cai va 1 cap con tro. */
    {
        const char letters[]  = "XYIJRF";
        float *vals[]  = { &cmd.x, &cmd.y, &cmd.i, &cmd.j, &cmd.r, &cmd.f };
        int   *flags[] = { &cmd.has_x, &cmd.has_y, &cmd.has_i,
                           &cmd.has_j, &cmd.has_r, &cmd.has_f };
        for (k = 0; k < 6; k++) {
            p = strchr(clean, letters[k]);
            if (p && sscanf(p + 1, "%f", vals[k]) == 1) *flags[k] = 1;
        }
    }
    return cmd;
}

/* read_gcode_file: doc toan bo file, tra ve so lenh doc duoc.
   Bo qua dong comment (bat dau bang ';') va dong trong.
   modal_code luu lenh G cuoi cung de dong sau ke thua (dac tinh modal). */
int read_gcode_file(const char *filepath, struct GCommand *commands, int max_cmd) {
    FILE *file = fopen(filepath, "r");
    if (!file) { printf("Error: Cannot open %s\n", filepath); return 0; }

    char line[256];
    int  cmd_count = 0;
    char modal_code[4] = "";

    while (fgets(line, 256, file)) {
        if (line[0] == ';' || line[0] == '\n') continue;
        struct GCommand cmd = parse_line(line, modal_code);
        if (cmd.code[0] != '\0') memcpy(modal_code, cmd.code, 4);
        if (cmd.code[0] == '\0' && cmd.mode_change == 0) continue;  /* dong rong */
        commands[cmd_count++] = cmd;
        if (cmd_count >= max_cmd) break;
    }
    fclose(file);
    return cmd_count;
}

/* =============================================================
   TRAPEZOID PROFILE
   =============================================================

   Chia moi doan duong thanh 3 pha van toc:

       v (mm/s)
       |           ___________
   v_peak         /           \
       |         /             \
   v_entry _____/               \_____ v_exit
       |
       +-------|-----------|-------|---> t (s)
         t_accel   t_cruise   t_decel

   Khi doan du dai: v_peak = feedrate (hinh thang day du).
   Khi doan ngan, khong kip tang den feedrate: profil thanh tam giac,
   dinh tam giac tinh tu phuong trinh dong hoc v^2 = v0^2 + 2as:
     - quang duong tang toc: s_acc = (v_peak^2 - v_entry^2) / 2a
     - quang duong giam toc: s_dec = (v_peak^2 - v_exit^2)  / 2a
     - tong s_acc + s_dec = do dai doan s, giai ra:
         v_peak = sqrt(a*s + (v_entry^2 + v_exit^2)/2)

   Vi du: doan 50mm, F3000 (=50mm/s), v_entry = v_exit = 0:
     v_peak kha thi = sqrt(500*50) = 158 > 50 -> chay o 50 mm/s
     t_accel = 50/500 = 0.1s, s_accel = 2.5mm
     s_cruise = 50 - 2.5 - 2.5 = 45mm -> t_cruise = 0.9s
     tong = 0.1 + 0.9 + 0.1 = 1.1s
*/
struct TrapezoidProfile {
    float v_entry, v_peak, v_exit;      /* van toc vao / dinh / ra (mm/s) */
    float t_accel, t_cruise, t_decel;   /* thoi gian 3 pha (s) */
    float total_time;                   /* tong thoi gian doan (s) */
    float s_accel, s_cruise;            /* quang duong pha 1, pha 2 (mm) */
};

struct TrapezoidProfile compute_trapezoid(float segment_length, float feedrate,
                                          float v_entry, float v_exit) {
    struct TrapezoidProfile prof;
    float v_max = feedrate / 60.0f;  /* mm/min -> mm/s */
    float a     = MAX_ACCELERATION;

    /* van toc vao/ra khong duoc vuot feedrate */
    if (v_entry > v_max) v_entry = v_max;
    if (v_exit  > v_max) v_exit  = v_max;

    /* v_peak kha thi theo cong thuc o tren; cap boi feedrate */
    float v_peak = sqrtf(a * segment_length
                         + (v_entry*v_entry + v_exit*v_exit) / 2.0f);
    if (v_peak > v_max) v_peak = v_max;

    prof.v_entry = v_entry;
    prof.v_peak  = v_peak;
    prof.v_exit  = v_exit;
    prof.t_accel = (v_peak - v_entry) / a;               /* tu v = v0 + at */
    prof.t_decel = (v_peak - v_exit)  / a;
    prof.s_accel = (v_peak*v_peak - v_entry*v_entry) / (2.0f * a);  /* v^2 = v0^2 + 2as */

    float s_decel = (v_peak*v_peak - v_exit*v_exit) / (2.0f * a);
    prof.s_cruise = segment_length - prof.s_accel - s_decel;
    if (prof.s_cruise < 0.0f) prof.s_cruise = 0.0f;      /* profil tam giac */
    prof.t_cruise   = (prof.s_cruise > 0.0f) ? prof.s_cruise / v_peak : 0.0f;
    prof.total_time = prof.t_accel + prof.t_cruise + prof.t_decel;
    return prof;
}

/*
 * trapezoid_state: vi tri s (mm) va van toc v (mm/s) tai thoi diem t_sec.
 *
 * Tinh analytic (truc tiep tu t bang cong thuc dong hoc) thay vi
 * cong don dv moi 1ms -- cong don se tich luy loi lam tron theo thoi gian,
 * analytic thi khong.
 *   Pha 1 tang toc: v = v_entry + a*t,  s = v_entry*t + a*t^2/2
 *   Pha 2 di deu:   v = v_peak,         s = s_accel + v_peak*t
 *   Pha 3 giam toc: v = v_peak - a*t,   s = s_accel + s_cruise + v_peak*t - a*t^2/2
 * (t trong moi pha tinh tu dau pha do)
 */
void trapezoid_state(float t_sec, const struct TrapezoidProfile *prof,
                     float *out_s, float *out_v) {
    float a = MAX_ACCELERATION;

    if (t_sec <= prof->t_accel) {
        /* pha 1: tang toc */
        *out_v = prof->v_entry + a * t_sec;
        *out_s = prof->v_entry * t_sec + 0.5f * a * t_sec * t_sec;
    } else if (t_sec <= prof->t_accel + prof->t_cruise) {
        /* pha 2: di deu */
        float t = t_sec - prof->t_accel;
        *out_v = prof->v_peak;
        *out_s = prof->s_accel + prof->v_peak * t;
    } else {
        /* pha 3: giam toc */
        float t = t_sec - prof->t_accel - prof->t_cruise;
        *out_v = prof->v_peak - a * t;
        if (*out_v < prof->v_exit) *out_v = prof->v_exit;  /* khong tut duoi v_exit */
        *out_s = prof->s_accel + prof->s_cruise
                 + prof->v_peak * t - 0.5f * a * t * t;
    }
}

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
    float v_entry, v_exit;       /* ket qua look-ahead (mm/s) */
};

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

/* =============================================================
   LOOK-AHEAD (Chen et al. 2013)
   =============================================================

   Khong co look-ahead: moi doan phai dung han (v=0) o hai dau -> may
   giat cuc khi chay nhieu doan ngan. Look-ahead doc truoc toan bo duong
   chay de tinh van toc CHO PHEP tai moi diem noi giua 2 doan, roi dam bao
   cac van toc do dat duoc voi gia toc gioi han. 3 buoc:

   1. junction_velocity (eq 34): khi re goc alpha voi toc do v, vector
      van toc doi huong dot ngot, bien thien |dv| = 2*v*sin(alpha/2).
      Bien thien nay phai thuc hien duoc voi gia toc a_max trong khoang
      thoi gian T, suy ra:
        v_junction <= a_max * T / (2*sin(alpha/2))
      Goc cang gat -> phai qua cang cham. Duong gan thang -> khong gioi han.
      (Hien dat JUNCTION_T = Ts = 1ms dung nhu paper: goc 90 do chi cho
       phep 0.35 mm/s, coi nhu van dung tai goc gat -- doi lai goc sac
       tuyet doi. Muon nhanh hon co the noi T len, goc se bi bo tron.)

   2. Backward pass (cuoi -> dau): doan i phai GIAM kip tu v[i] xuong
      v[i+1] trong do dai L[i]. Neu v[i] cao qua thi ha xuong
        v[i] <= sqrt(v[i+1]^2 + 2*a*L[i])
   3. Forward pass (dau -> cuoi): doan i phai TANG kip, doi xung voi (2).

   Sau 2 pass, moi cap (v[i], v[i+1]) thoa |v[i+1]^2 - v[i]^2| <= 2*a*L[i]
   nen compute_trapezoid luon dung duoc profil hop le.
*/
float junction_velocity(float t0x, float t0y, float t1x, float t1y) {
    /* goc giua 2 tiep tuyen: cos(alpha) = t0 . t1 (2 vector don vi) */
    float dot = t0x*t1x + t0y*t1y;
    if (dot >  1.0f) dot =  1.0f;   /* chong loi lam tron truoc acos */
    if (dot < -1.0f) dot = -1.0f;
    float alpha = acosf(dot);
    if (alpha < 0.001f) return 1.0e9f;  /* gan thang: tra so lon, feedrate se cap sau */
    return MAX_ACCELERATION * JUNCTION_T / (2.0f * sinf(alpha * 0.5f));
}

void compute_lookahead(struct Segment *segs, int n) {
    /* vj[i] = van toc tai diem noi TRUOC doan i.
       vj[0] = 0 (xuat phat dung yen), vj[n] = 0 (ket thuc dung yen). */
    float vj[MAX_SEGMENTS + 1];
    float a = MAX_ACCELERATION;
    int i;

    /* buoc 1: junction velocity tu goc doi huong, cap boi feedrate 2 doan ke */
    vj[0] = 0.0f;
    vj[n] = 0.0f;
    for (i = 1; i < n; i++) {
        float v = junction_velocity(segs[i-1].tan_out_x, segs[i-1].tan_out_y,
                                    segs[i].tan_in_x,    segs[i].tan_in_y);
        float vmax_prev = segs[i-1].feedrate / 60.0f;
        float vmax_next = segs[i].feedrate   / 60.0f;
        if (v > vmax_prev) v = vmax_prev;
        if (v > vmax_next) v = vmax_next;
        vj[i] = v;
    }

    /* buoc 2: backward pass -- dam bao giam toc kip */
    for (i = n - 1; i >= 0; i--) {
        float v_reachable = sqrtf(vj[i+1]*vj[i+1] + 2.0f * a * segs[i].length);
        if (vj[i] > v_reachable) vj[i] = v_reachable;
    }
    /* buoc 3: forward pass -- dam bao tang toc kip */
    for (i = 0; i < n; i++) {
        float v_reachable = sqrtf(vj[i]*vj[i] + 2.0f * a * segs[i].length);
        if (vj[i+1] > v_reachable) vj[i+1] = v_reachable;
    }

    /* ghi ket qua vao tung doan */
    for (i = 0; i < n; i++) {
        segs[i].v_entry = vj[i];
        segs[i].v_exit  = vj[i+1];
    }
}

/*
 * build_segments: chay qua GCommand[], resolve hinh hoc thanh Segment[].
 * Xu ly logic modal: G90/G91 (absolute/incremental), feedrate ke thua,
 * vi tri hien tai cap nhat dan qua tung lenh.
 *
 * Tiep tuyen (dung cho look-ahead):
 *   Line: (dx,dy)/L -- giong nhau o dau va cuoi doan.
 *   Arc:  vuong goc voi ban kinh tai diem do. Diem tren cung la
 *         P(theta) = tam + R*(cos theta, sin theta), dao ham theo theta:
 *         CCW (G3): tiep tuyen = (-sin theta,  cos theta)
 *         CW  (G2): nguoc lai  = ( sin theta, -cos theta)  (bien 'dir')
 */
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
   INTERPOLATION
   =============================================================

   Noi suy 1 doan (thang hoac cung) theo thoi gian:
   1. Tinh profil hinh thang cho do dai doan
   2. Vong lap moi 1ms:
        a. s(t), v(t) tu profil (trapezoid_state)
        b. alpha = s / L  = phan tram quang duong da di (0..1)
        c. doi alpha thanh toa do:
             thang: P = P0 + alpha*(P1-P0)
             cung:  goc = angle_start + alpha*sweep, P = tam + R*(cos,sin)
        d. ghi CSV: t_ms, x, y, v
   3. Ghi diem cuoi chinh xac (x1,y1) de tranh loi lam tron float
   Tra ve thoi gian doan (ms) de main() cong don.
*/
float interpolate_segment(const struct Segment *seg, float t_start_ms, FILE *csv) {
    /* hinh hoc cung: chi tinh 1 lan truoc vong lap */
    float cx = 0, cy = 0, radius = 0, angle_start = 0, sweep = 0;
    if (seg->is_arc) {
        cx = seg->x0 + seg->i_off;
        cy = seg->y0 + seg->j_off;
        radius = sqrtf(seg->i_off*seg->i_off + seg->j_off*seg->j_off);
        arc_sweep(seg->x0, seg->y0, seg->x1, seg->y1, cx, cy,
                  seg->clockwise, &angle_start, &sweep);
    }

    struct TrapezoidProfile prof = compute_trapezoid(seg->length, seg->feedrate,
                                                     seg->v_entry, seg->v_exit);
    float total_ms = prof.total_time * 1000.0f;

    float t_ms;
    for (t_ms = 0.0f; t_ms < total_ms; t_ms += DT_MS) {
        float s, v, x, y;
        trapezoid_state(t_ms / 1000.0f, &prof, &s, &v);

        float alpha = s / seg->length;
        if (alpha > 1.0f) alpha = 1.0f;

        if (seg->is_arc) {
            float angle = angle_start + alpha * sweep;
            x = cx + radius * cosf(angle);
            y = cy + radius * sinf(angle);
        } else {
            x = seg->x0 + alpha * (seg->x1 - seg->x0);
            y = seg->y0 + alpha * (seg->y1 - seg->y0);
        }
        if (csv) fprintf(csv, "%.1f,%.4f,%.4f,%.3f\n", t_start_ms + t_ms, x, y, v);
    }

    /* diem cuoi chinh xac */
    if (csv) fprintf(csv, "%.1f,%.4f,%.4f,%.3f\n",
                     t_start_ms + total_ms, seg->x1, seg->y1, prof.v_exit);

    printf("  v_entry=%.1f  v_peak=%.1f  v_exit=%.1f mm/s  |  %.0f ms\n",
           prof.v_entry, prof.v_peak, prof.v_exit, total_ms);
    return total_ms;
}

/* =============================================================
   MAIN
   ============================================================= */
int main(int argc, char *argv[]) {
    const char *filepath = (argc > 1) ? argv[1] : "test.gcode";
    printf("Reading file: %s\n\n", filepath);

    struct GCommand commands[MAX_SEGMENTS];
    int cmd_count = read_gcode_file(filepath, commands, MAX_SEGMENTS);
    printf("Found %d commands:\n\n", cmd_count);

    /* file CSV ket qua: moi dong 1 diem mau cach nhau DT_MS */
    FILE *csv = fopen("trajectory.csv", "w");
    if (csv) fprintf(csv, "t_ms,x,y,v\n");

    /* buoc 1: resolve hinh hoc */
    struct Segment segments[MAX_SEGMENTS];
    int seg_count = build_segments(commands, cmd_count, segments);
    printf("Built %d motion segments\n\n", seg_count);

    /* buoc 2: look-ahead dien v_entry/v_exit cho tung doan */
    compute_lookahead(segments, seg_count);

    printf("Look-ahead junction velocities (JUNCTION_T = %.0f ms):\n",
           JUNCTION_T * 1000.0f);
    int i;
    for (i = 0; i < seg_count; i++) {
        printf("  seg %2d  %s  L=%6.2f mm  v_entry=%5.1f  v_exit=%5.1f mm/s\n",
               i + 1,
               segments[i].is_arc ? (segments[i].clockwise ? "G2 " : "G3 ") : "lin",
               segments[i].length, segments[i].v_entry, segments[i].v_exit);
    }
    printf("\n");

    /* buoc 3: noi suy tung doan, cong don thoi gian */
    float t_ms = 0.0f;
    for (i = 0; i < seg_count; i++) {
        printf("--- Segment %d:\n", i + 1);
        t_ms += interpolate_segment(&segments[i], t_ms, csv);
    }

    printf("Total time: %.1f ms (%.2f sec)\n", t_ms, t_ms / 1000.0f);
    if (csv) {
        fclose(csv);
        printf("Saved: trajectory.csv  ->  python visualize.py trajectory.csv\n");
    }
    return 0;
}
