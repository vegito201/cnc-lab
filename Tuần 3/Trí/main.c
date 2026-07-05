/*
 * =============================================================
 * GCode Trajectory Calculator -- S-curve Velocity + Look-ahead
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

/* Jerk toi da (mm/s^3) -- toc do bien thien cua gia toc (S-curve).
   Nho -> gia toc len/xuong tu tu, cuc em nhung cham.
   Lon -> tien ve trapezoid (gia toc nhay bac).
   Voi a=500: thoi gian keo gia toc tu 0 len max = a/J = 0.1s. */
#define MAX_JERK         5000.0f

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
   S-CURVE PROFILE (7 pha, gioi han jerk -- Chen 2013 section 2.1)
   =============================================================

   Trapezoid cu: gia toc nhay bac 0 -> a_max tuc thi (jerk vo han) -> may rung.
   S-curve: gia toc cung phai tang/giam tu tu voi toc do toi da J (jerk):

       v (mm/s)                 ______________
   v_peak                     /:              :\
       |                    /  :              :  \
       |                  /    :              :    \
   v_entry _____________/      :              :      \______ v_exit
       |    [1]   [2]   [3]    :     [4]      :  [5] [6] [7]
       +---------------------------------------------------> t
       a |   /```\             :              :
         |  /     \            :              :
       0 |_/       \___________:______________:_____________
         |                     :              :\        /
    -a   |                     :              :  \____/

   7 pha: [1] jerk+ (a tang 0->A)   [2] a = A khong doi   [3] jerk- (a ve 0)
          [4] di deu v_peak
          [5] jerk- (a giam 0->-A)  [6] a = -A             [7] jerk+ (a ve 0)

   MOI DOAN TANG/GIAM TOC (ramp) doi xung theo thoi gian, nen:
     - thoi gian ramp cho do bien thien dv:
         dv >= A^2/J (du cho de a dat A): T = dv/A + A/J   (co pha giua)
         dv <  A^2/J (tam giac gia toc):  A' = sqrt(J*dv), T = 2*sqrt(dv/J)
     - quang duong trong ramp = van toc trung binh * thoi gian
         s_ramp = (v_dau + v_cuoi)/2 * T
       (dung chinh xac vi profil gia toc doi xung quanh trung diem ramp)

   Tim v_peak: khac trapezoid, khong co cong thuc dong (paper phai chia
   7 loai profile theo L -- section 2.2). O day tan dung tinh DON DIEU:
   v_peak cang cao thi tong quang duong 2 ramp cang dai -> binary search
   v_peak sao cho s_up + s_down <= L, phan du la pha di deu [4].
   (Cach nay tu bao het 7 type cua paper: type nao thi pha tuong ung
   tu dong co thoi gian = 0.)
*/

/* Mot ramp (doan tang toc lien tuc tu v0 len v1, gom pha 1-2-3).
   Giam toc dung CHINH struct nay phat nguoc thoi gian (xem scurve_state). */
struct Ramp {
    float v0, v1;   /* van toc thap / cao (mm/s), v1 >= v0 */
    float A;        /* gia toc dinh thuc dat (mm/s^2), co the < MAX_ACCELERATION */
    float t_j;      /* thoi gian keo jerk (pha dau = pha cuoi) (s) */
    float t_c;      /* thoi gian gia toc khong doi o giua (s) */
    float T;        /* tong thoi gian ramp = 2*t_j + t_c (s) */
    float dist;     /* quang duong di het ramp (mm) */
};

struct Ramp make_ramp(float v_low, float v_high) {
    struct Ramp r;
    float dv = v_high - v_low;
    float J  = MAX_JERK;

    r.v0 = v_low;
    r.v1 = v_high;
    if (dv <= 1e-6f) {  /* khong doi van toc: ramp rong */
        r.A = 0.0f; r.t_j = 0.0f; r.t_c = 0.0f; r.T = 0.0f; r.dist = 0.0f;
        return r;
    }
    r.A = sqrtf(J * dv);                    /* gia toc dinh neu tam giac */
    if (r.A > MAX_ACCELERATION) r.A = MAX_ACCELERATION;
    r.t_j = r.A / J;
    r.t_c = dv / r.A - r.t_j;               /* am -> khong co pha giua */
    if (r.t_c < 0.0f) r.t_c = 0.0f;
    r.T    = 2.0f * r.t_j + r.t_c;
    r.dist = 0.5f * (v_low + v_high) * r.T; /* van toc trung binh * thoi gian */
    return r;
}

/*
 * ramp_state: s (mm) va v (mm/s) tai thoi diem t trong ramp TANG toc.
 * Analytic tung pha (tich phan jerk):
 *   Pha 1 (0..t_j):      a = J*t         v = v0 + J*t^2/2       s = v0*t + J*t^3/6
 *   Pha 2 (t_c):         a = A           v = v1e + A*t          s = ... + A*t^2/2
 *   Pha 3 (t_j):         a = A - J*t     v = v2e + A*t - J*t^2/2
 * (t trong moi pha tinh tu dau pha do; v1e/v2e = van toc cuoi pha truoc)
 */
void ramp_state(const struct Ramp *r, float t, float *out_s, float *out_v) {
    float J = MAX_JERK;

    if (t < 0.0f)   t = 0.0f;
    if (t > r->T)   t = r->T;
    if (r->T <= 0.0f) { *out_s = 0.0f; *out_v = r->v0; return; }

    /* trang thai cuoi pha 1 */
    float v1e = r->v0 + 0.5f * J * r->t_j * r->t_j;
    float s1e = r->v0 * r->t_j + J * r->t_j * r->t_j * r->t_j / 6.0f;

    if (t <= r->t_j) {
        /* pha 1: jerk duong, gia toc tang dan */
        *out_v = r->v0 + 0.5f * J * t * t;
        *out_s = r->v0 * t + J * t * t * t / 6.0f;
    } else if (t <= r->t_j + r->t_c) {
        /* pha 2: gia toc khong doi A */
        float u = t - r->t_j;
        *out_v = v1e + r->A * u;
        *out_s = s1e + v1e * u + 0.5f * r->A * u * u;
    } else {
        /* pha 3: jerk am, gia toc giam ve 0 */
        float v2e = v1e + r->A * r->t_c;
        float s2e = s1e + v1e * r->t_c + 0.5f * r->A * r->t_c * r->t_c;
        float u   = t - r->t_j - r->t_c;
        *out_v = v2e + r->A * u - 0.5f * J * u * u;
        *out_s = s2e + v2e * u + 0.5f * r->A * u * u - J * u * u * u / 6.0f;
    }
}

/* Profil S-curve day du cua 1 doan: ramp len + di deu + ramp xuong */
struct SCurveProfile {
    float v_entry, v_peak, v_exit;   /* van toc vao / dinh / ra (mm/s) */
    struct Ramp up;                  /* v_entry -> v_peak */
    struct Ramp down;                /* v_exit -> v_peak, PHAT NGUOC khi giam toc */
    float t_cruise, s_cruise;        /* pha di deu */
    float total_time;                /* tong thoi gian doan (s) */
};

struct SCurveProfile compute_scurve(float segment_length, float feedrate,
                                    float v_entry, float v_exit) {
    struct SCurveProfile prof;
    float v_max = feedrate / 60.0f;  /* mm/min -> mm/s */
    int k;

    /* van toc vao/ra khong duoc vuot feedrate */
    if (v_entry > v_max) v_entry = v_max;
    if (v_exit  > v_max) v_exit  = v_max;

    /* binary search v_peak: quang duong 2 ramp don dieu tang theo v_peak.
       48 vong lap du dua sai so ve duoi do phan giai float. */
    float lo = (v_entry > v_exit) ? v_entry : v_exit;
    float hi = v_max;
    if (make_ramp(v_entry, hi).dist + make_ramp(v_exit, hi).dist <= segment_length) {
        lo = hi;  /* doan du dai de dat feedrate */
    } else {
        for (k = 0; k < 48; k++) {
            float mid = 0.5f * (lo + hi);
            float need = make_ramp(v_entry, mid).dist + make_ramp(v_exit, mid).dist;
            if (need <= segment_length) lo = mid; else hi = mid;
        }
        /* neu ngay ca v_peak = max(v_entry,v_exit) cung khong vua thi
           look-ahead da tinh sai -- khong xay ra sau 2 pass scurve_reach */
    }

    prof.v_entry = v_entry;
    prof.v_peak  = lo;
    prof.v_exit  = v_exit;
    prof.up      = make_ramp(v_entry, lo);
    prof.down    = make_ramp(v_exit,  lo);
    prof.s_cruise = segment_length - prof.up.dist - prof.down.dist;
    if (prof.s_cruise < 0.0f) prof.s_cruise = 0.0f;
    prof.t_cruise = (prof.v_peak > 1e-6f) ? prof.s_cruise / prof.v_peak : 0.0f;
    prof.total_time = prof.up.T + prof.t_cruise + prof.down.T;
    return prof;
}

/*
 * scurve_state: vi tri s (mm) va van toc v (mm/s) tai thoi diem t_sec.
 * Van analytic nhu trapezoid_state cu -- khong cong don, khong tich luy loi.
 *
 * Meo phan giam toc: giam toc v_peak -> v_exit chinh la ramp
 * v_exit -> v_peak chay NGUOC thoi gian (profil doi xung):
 *   v(t) = v_ramp(T_down - t)
 *   s(t) = dist_down - s_ramp(T_down - t)   (quang duong con lai cua ramp)
 */
void scurve_state(float t_sec, const struct SCurveProfile *prof,
                  float *out_s, float *out_v) {
    if (t_sec <= prof->up.T) {
        /* pha 1-3: tang toc */
        ramp_state(&prof->up, t_sec, out_s, out_v);
    } else if (t_sec <= prof->up.T + prof->t_cruise) {
        /* pha 4: di deu */
        float t = t_sec - prof->up.T;
        *out_v = prof->v_peak;
        *out_s = prof->up.dist + prof->v_peak * t;
    } else {
        /* pha 5-7: giam toc = ramp 'down' phat nguoc */
        float t = t_sec - prof->up.T - prof->t_cruise;
        if (t > prof->down.T) t = prof->down.T;
        float s_rev, v_rev;
        ramp_state(&prof->down, prof->down.T - t, &s_rev, &v_rev);
        *out_v = v_rev;
        *out_s = prof->up.dist + prof->s_cruise + (prof->down.dist - s_rev);
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
    float corner_r;              /* ban kinh bo goc tai DIEM CUOI doan (0 = khong bo) */
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
      v[i+1] trong do dai L[i]. Neu v[i] cao qua thi ha xuong muc voi toi.
   3. Forward pass (dau -> cuoi): doan i phai TANG kip, doi xung voi (2).

  voi trapezoid la cong thuc dong sqrt(v0^2 + 2*a*L).
   Voi S-curve thi doi van toc CAN NHIEU QUANG DUONG HON (mat them
   thoi gian keo gia toc len/xuong) va khong co cong thuc dong --
   scurve_reach() binary search tren make_ramp().dist (don dieu).
   Sau 2 pass, moi cap (v[i], v[i+1]) deu vua trong L[i]
   nen compute_scurve luon dung duoc profil hop le.
*/

/* van toc cao nhat dat duoc tu v0 sau quang duong L (S-curve, doi xung
   cho ca tang lan giam toc). Can tren = cong thuc trapezoid (S-curve
   luon dat thap hon), binary search giua v0 va can tren. */
float scurve_reach(float v0, float L) {
    float lo = v0;
    float hi = sqrtf(v0 * v0 + 2.0f * MAX_ACCELERATION * L);
    int k;
    for (k = 0; k < 40; k++) {
        float mid = 0.5f * (lo + hi);
        if (make_ramp(v0, mid).dist <= L) lo = mid; else hi = mid;
    }
    return lo;
}
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

    /* buoc 2: backward pass -- dam bao giam toc kip (S-curve) */
    for (i = n - 1; i >= 0; i--) {
        float v_reachable = scurve_reach(vj[i+1], segs[i].length);
        if (vj[i] > v_reachable) vj[i] = v_reachable;
    }
    /* buoc 3: forward pass -- dam bao tang toc kip (S-curve) */
    for (i = 0; i < n; i++) {
        float v_reachable = scurve_reach(vj[i], segs[i].length);
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

/* =============================================================
   INTERPOLATION
   =============================================================

   Noi suy 1 doan (thang hoac cung) theo thoi gian:
   1. Tinh profil S-curve cho do dai doan
   2. Vong lap moi 1ms:
        a. s(t), v(t) tu profil (scurve_state)
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

    struct SCurveProfile prof = compute_scurve(seg->length, seg->feedrate,
                                               seg->v_entry, seg->v_exit);
    float total_ms = prof.total_time * 1000.0f;

    float t_ms;
    for (t_ms = 0.0f; t_ms < total_ms; t_ms += DT_MS) {
        float s, v, x, y;
        scurve_state(t_ms / 1000.0f, &prof, &s, &v);

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
    printf("Built %d motion segments\n", seg_count);

    /* bo goc: chen cung tron tai cac goc co R tren lenh G01 */
    seg_count = apply_corner_rounding(segments, seg_count);
    printf("After corner rounding: %d segments\n\n", seg_count);

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
