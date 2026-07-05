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
   S-CURVE PROFILE (7 vung -- Chen 2013 Section 2.1, Figure 1)
   =============================================================

   KY HIEU DAT THEO PAPER de doc code doi chieu truc tiep voi bai bao:
     v_start, v_m, v_end : van toc diem dau / mong muon / diem cuoi (Figure 1)
     T1..T7              : thoi gian 7 vung
     A, D                : gia toc dinh vung tang / vung giam (o day D = A)
     J                   : jerk toi da (paper co J1, J2 rieng; o day J1 = J2 = J)
     L                   : chieu dai doan duong

   KHAC PAPER O 1 DIEM: paper chay theo TICK nguyen (T1 = na*Ts, T2 = k*na*Ts,
   eq (2),(5),(9),(10) tinh na, nb, nc; eq (11)-(25) chinh v_m', v_start',
   v_end', J' de khu sai so lam tron tick). Minh mo phong thoi gian LIEN TUC
   (float) nen na*Ts -> T1 truc tiep va toan bo cum lam tron do khong can.

       v (mm/s)                 ______________
   v_m                        /:              :\
       |                    /  :              :  \
       |                  /    :              :    \
   v_start _____________/      :              :      \______ v_end
       |    T1   T2   T3       :      T4      : T5  T6  T7
       +---------------------------------------------------> t
       a |   /```\             :              :
       A |  /     \            :              :
       0 |_/       \___________:______________:_____________
         |                     :              :\        /
      -D |                     :              :  \____/

   Vung 1: jerk +J (a: 0->A)       Vung 5: jerk -J (a: 0->-D)
   Vung 2: a = A khong doi         Vung 6: a = -D khong doi
   Vung 3: jerk -J (a: A->0)       Vung 7: jerk +J (a: -D->0)
   Vung 4: di deu o v_m            (T1 = T3, T5 = T7: keo len bao lau
                                    thi ha xuong bay lau)
*/

/*
 * solve_zone: giai eq (1) [vung tang toc] / eq (4) [vung giam toc]
 * dang lien tuc. Cho do bien thien van toc dv cua ca vung, tra ve
 * T_jerk (= T1 = T3, hoac T5 = T7), T_const (= T2 hoac T6) va
 * gia toc dinh A (hoac D).
 *
 * eq (1) paper viet theo tick:  v_m = v_start + (1+k)*J1*(na*Ts)^2
 * Thay T1 = na*Ts, T2 = k*T1 thi eq (1) chinh la:
 *     dv = J*T1^2 + J*T1*T2 = J*T1*(T1 + T2)                    (*)
 * (J*T1 = gia toc dinh A; dv = dien tich hinh thang gia toc = A*(T1+T2))
 *
 * 2 truong hop -- tuong ung dieu kien na >= A/(J*Ts) cua eq (2):
 *   dv >= A^2/J : gia toc cham tran A -> T1 = A/J, T2 = dv/A - T1 (giai *)
 *   dv <  A^2/J : khong cham tran     -> T2 = 0,   A' = sqrt(J*dv) (giai * voi T2=0)
 *
 * VI DU SO THAT: dv = 50 (0 -> 50 mm/s), A_max = 500, J = 5000:
 *   A^2/J = 500*500/5000 = 50 = dv -> vua cham tran: A = 500
 *   T1 = T3 = 500/5000 = 0.1s ; T2 = 50/500 - 0.1 = 0s
 *   ca vung: 0.2s  (trapezoid cung dv chi can 0.1s -- gia cua jerk huu han)
 */
void solve_zone(float dv, float *out_T_jerk, float *out_T_const, float *out_A) {
    float J = MAX_JERK;
    if (dv <= 1e-6f) {                    /* khong doi van toc: vung rong */
        *out_T_jerk = 0.0f; *out_T_const = 0.0f; *out_A = 0.0f;
        return;
    }
    float A = sqrtf(J * dv);              /* gia toc dinh neu KHONG cham tran */
    if (A > MAX_ACCELERATION) A = MAX_ACCELERATION;
    *out_A       = A;
    *out_T_jerk  = A / J;                 /* = T1 = T3 (hoac T5 = T7) */
    *out_T_const = dv / A - *out_T_jerk;  /* = T2 (hoac T6); am -> ep ve 0 */
    if (*out_T_const < 0.0f) *out_T_const = 0.0f;
}

/*
 * zone_dist: quang duong di het 1 vung tang/giam toc tu v_low den v_high.
 * Profil gia toc doi xung theo thoi gian nen van toc trung binh cua vung
 * = (v_low + v_high)/2, suy ra:
 *     L_vung = (v_low + v_high)/2 * (T1 + T2 + T3)
 * Day chinh la tung so hang cua eq (8):
 *     2L = (2+k)*((v_start+v_m)*na + (v_end+v_m)*nb)*Ts + 2*v_m*nc*Ts
 * trong do (2+k)*na*Ts = T1+T2+T3 la thoi gian ca vung.
 *
 * VI DU SO THAT (tiep vi du solve_zone): 0 -> 50 mm/s het 0.2s
 *   -> L_vung = (0+50)/2 * 0.2 = 5 mm  (trapezoid: 2.5mm)
 */
float zone_dist(float v_low, float v_high) {
    float T_jerk, T_const, A;
    solve_zone(v_high - v_low, &T_jerk, &T_const, &A);
    return 0.5f * (v_low + v_high) * (2.0f * T_jerk + T_const);
}

/* Profil S-curve day du cua 1 doan. Ngoai T1..T7 con luu san "bang bien":
   thoi diem / van toc / quang duong tai CUOI moi vung, de scurve_state()
   tra cuu -- cac gia tri nay la eq (17) tinh tai cuoi vung. */
struct SCurveProfile {
    float v_start, v_m, v_end;          /* van toc dau / dinh / cuoi (mm/s) */
    float T1, T2, T3, T4, T5, T6, T7;   /* thoi gian 7 vung (s) */
    float A, D;                         /* gia toc dinh vung tang / giam (mm/s^2) */
    float t_bound[8];                    /* t_bound[r] = thoi diem cuoi vung r (t_bound[0]=0) */
    float v_bound[8];                    /* van toc tai cuoi vung r (mm/s) */
    float s_bound[8];                    /* quang duong tich luy den cuoi vung r (mm) */
    float total_time;                   /* = t_bound[7] (s) */
};

/*
 * compute_scurve: dung profil 7 vung cho doan dai L, van toc vao/ra cho truoc.
 *
 * Buoc kho nhat la tim v_m. Paper Section 2.2 phan loai duong di thanh
 * 7 Type theo L (eq (26)-(33)) de biet vung nao ton tai, roi chinh v_m'
 * bang eq (11). Minh KHONG phan loai, thay bang nhan xet don dieu:
 *     v_m cang cao -> zone_dist 2 vung cang dai
 * nen binary search v_m lon nhat sao cho:
 *     zone_dist(v_start, v_m) + zone_dist(v_end, v_m) <= L     [dieu kien eq (8)]
 * Type nao thi vung tuong ung tu dong = 0 (Type 2: T4=0; Type 3: T2=T6=T4=0; ...)
 *
 * Ten bien binary search:
 *   v_feasible : gia tri v_m DA BIET CHAC vua trong L (ket qua nam o day)
 *   v_too_high : gia tri v_m DA BIET CHAC doi hoi nhieu hon L
 *   moi vong thu diem giua, thu hep khoang [v_feasible, v_too_high] con mot nua;
 *   48 vong -> sai so ~ v_max/2^48, duoi do phan giai float.
 *
 * VI DU SO THAT: doan 50mm, F3000 (v_max = 50mm/s), v_start = v_end = 0:
 *   thu v_m = 50: zone_dist = 5mm moi ben (vi du o zone_dist) -> 10 <= 50: OK
 *   -> v_m = 50 ; L con lai cho vung 4: 40mm -> T4 = 40/50 = 0.8s
 *   total = 0.2 + 0.8 + 0.2 = 1.2s   (trapezoid cu: 1.1s)
 * Doan ngan khong dat feedrate thi binary search tu ra v_m thap hon.
 */
struct SCurveProfile compute_scurve(float L, float feedrate,
                                    float v_start, float v_end) {
    struct SCurveProfile p;
    float J = MAX_JERK;
    float v_max = feedrate / 60.0f;   /* mm/min -> mm/s */
    int k;

    /* van toc vao/ra khong duoc vuot feedrate (eq (35) da cap o look-ahead,
       chan them lan nua cho an toan) */
    if (v_start > v_max) v_start = v_max;
    if (v_end   > v_max) v_end   = v_max;

    /* tim v_m: binary search dieu kien eq (8) nhu mo ta o tren */
    float v_feasible   = (v_start > v_end) ? v_start : v_end;
    float v_too_high = v_max;
    if (zone_dist(v_start, v_max) + zone_dist(v_end, v_max) <= L) {
        v_feasible = v_max;   /* doan du dai de dat feedrate (Type 1) */
    } else {
        for (k = 0; k < 48; k++) {
            float v_try = 0.5f * (v_feasible + v_too_high);
            if (zone_dist(v_start, v_try) + zone_dist(v_end, v_try) <= L)
                v_feasible = v_try;
            else
                v_too_high = v_try;
        }
        /* neu ngay ca v_m = max(v_start,v_end) cung khong vua thi look-ahead
           da sai -- khong xay ra vi 2 pass da dung scurve_reach */
    }

    p.v_start = v_start;
    p.v_m     = v_feasible;
    p.v_end   = v_end;

    /* eq (1)/(2): vung tang toc ; eq (4)/(5): vung giam toc */
    solve_zone(p.v_m - v_start, &p.T1, &p.T2, &p.A);  p.T3 = p.T1;
    solve_zone(p.v_m - v_end,   &p.T5, &p.T6, &p.D);  p.T7 = p.T5;

    /* eq (9)/(10) dang lien tuc: thoi gian vung 4 tu chieu dai con du */
    float L_acc = 0.5f * (v_start + p.v_m) * (p.T1 + p.T2 + p.T3);
    float L_dec = 0.5f * (v_end   + p.v_m) * (p.T5 + p.T6 + p.T7);
    float L_cru = L - L_acc - L_dec;
    if (L_cru < 0.0f) L_cru = 0.0f;
    p.T4 = (p.v_m > 1e-6f) ? L_cru / p.v_m : 0.0f;

    /* bang bien: eq (17) tinh tai CUOI tung vung (u = T_vung).
       v_bound: van toc cuoi vung; s_bound: quang duong tich luy. */
    p.t_bound[0] = 0.0f;  p.v_bound[0] = v_start;  p.s_bound[0] = 0.0f;
    {
        float T[8] = { 0.0f, p.T1, p.T2, p.T3, p.T4, p.T5, p.T6, p.T7 };
        int r;
        for (r = 1; r <= 7; r++) p.t_bound[r] = p.t_bound[r-1] + T[r];
    }
    p.v_bound[1] = v_start + 0.5f * J * p.T1 * p.T1;        /* cuoi vung 1 */
    p.v_bound[2] = p.v_bound[1] + p.A * p.T2;                /* cuoi vung 2 */
    p.v_bound[3] = p.v_m;                                   /* cuoi vung 3 */
    p.v_bound[4] = p.v_m;                                   /* cuoi vung 4 */
    p.v_bound[5] = p.v_m - 0.5f * J * p.T5 * p.T5;          /* cuoi vung 5 */
    p.v_bound[6] = p.v_bound[5] - p.D * p.T6;                /* cuoi vung 6 */
    p.v_bound[7] = v_end;                                   /* cuoi vung 7 */

    p.s_bound[1] = v_start * p.T1 + J * p.T1*p.T1*p.T1 / 6.0f;
    p.s_bound[2] = p.s_bound[1] + p.v_bound[1] * p.T2 + 0.5f * p.A * p.T2*p.T2;
    p.s_bound[3] = p.s_bound[2] + p.v_m * p.T3 - J * p.T3*p.T3*p.T3 / 6.0f;
    p.s_bound[4] = p.s_bound[3] + p.v_m * p.T4;
    p.s_bound[5] = p.s_bound[4] + p.v_m * p.T5 - J * p.T5*p.T5*p.T5 / 6.0f;
    p.s_bound[6] = p.s_bound[5] + p.v_bound[5] * p.T6 - 0.5f * p.D * p.T6*p.T6;
    p.s_bound[7] = p.s_bound[6] + v_end * p.T7 + J * p.T7*p.T7*p.T7 / 6.0f;

    p.total_time = p.t_bound[7];
    return p;
}

/*
 * scurve_state: van toc v va quang duong s tai thoi diem t_sec.
 * Day la eq (17)+(18) cua paper viet o dang LIEN TUC: thay tick n
 * bang thoi gian u tinh tu dau vung (n*Ts -> u), moi nhanh switch
 * la 1 dong cua eq (17). Van analytic -- khong cong don, khong
 * tich luy loi lam tron.
 */
void scurve_state(float t_sec, const struct SCurveProfile *p,
                  float *out_s, float *out_v) {
    float J = MAX_JERK;
    int r;
    float u, t_remain;

    if (t_sec >= p->total_time) {         /* het profil: ghim o diem cuoi */
        *out_v = p->v_end;
        *out_s = p->s_bound[7];
        return;
    }
    /* tim vung r chua t_sec (vung rong T=0 tu dong bi nhay qua) */
    r = 1;
    while (r < 7 && t_sec > p->t_bound[r]) r++;
    u = t_sec - p->t_bound[r-1];           /* thoi gian tinh tu DAU vung r */

    switch (r) {
    case 1:  /* eq (17) dong 1 -- jerk +J:   v = v_start + J*u^2/2 */
        *out_v = p->v_start + 0.5f * J * u * u;
        *out_s = p->v_start * u + J * u*u*u / 6.0f;
        break;
    case 2:  /* dong 2 -- gia toc A khong doi */
        *out_v = p->v_bound[1] + p->A * u;
        *out_s = p->s_bound[1] + p->v_bound[1] * u + 0.5f * p->A * u*u;
        break;
    case 3:  /* dong 3 -- jerk -J; viet theo thoi gian CON LAI den cuoi vung
                de thay ro v cham tran v_m: v = v_m - J*(t_remain)^2/2 */
        t_remain = p->T3 - u;
        *out_v = p->v_m - 0.5f * J * t_remain * t_remain;
        *out_s = p->s_bound[2] + p->v_m * u
                 - J * (p->T3*p->T3*p->T3 - t_remain*t_remain*t_remain) / 6.0f;
        break;
    case 4:  /* dong 4 -- di deu o v_m */
        *out_v = p->v_m;
        *out_s = p->s_bound[3] + p->v_m * u;
        break;
    case 5:  /* dong 5 -- jerk -J: bat dau roi khoi v_m */
        *out_v = p->v_m - 0.5f * J * u * u;
        *out_s = p->s_bound[4] + p->v_m * u - J * u*u*u / 6.0f;
        break;
    case 6:  /* dong 6 -- gia toc -D khong doi */
        *out_v = p->v_bound[5] - p->D * u;
        *out_s = p->s_bound[5] + p->v_bound[5] * u - 0.5f * p->D * u*u;
        break;
    default: /* vung 7, dong 7 -- jerk +J: tiep dat em o v_end */
        t_remain = p->T7 - u;
        *out_v = p->v_end + 0.5f * J * t_remain * t_remain;
        *out_s = p->s_bound[6] + p->v_end * u
                 + J * (p->T7*p->T7*p->T7 - t_remain*t_remain*t_remain) / 6.0f;
        break;
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
   scurve_reach() binary search tren zone_dist() (don dieu).
   Sau 2 pass, moi cap (v[i], v[i+1]) deu vua trong L[i]
   nen compute_scurve luon dung duoc profil hop le.
*/

/* Van toc cao nhat dat duoc tu v0 sau quang duong L (S-curve; doi xung
   nen dung duoc cho ca chieu tang lan giam). Trapezoid co cong thuc dong
   v = sqrt(v0^2 + 2aL); S-curve KHONG co (ton them quang duong keo jerk)
   nhung zone_dist don dieu theo v nen binary search, can tren chinh la
   cong thuc trapezoid (S-curve luon dat THAP hon no).
   v_feasible / v_too_high: nhu trong compute_scurve. */
float scurve_reach(float v0, float L) {
    float v_feasible   = v0;
    float v_too_high = sqrtf(v0 * v0 + 2.0f * MAX_ACCELERATION * L);
    int k;
    for (k = 0; k < 48; k++) {
        float v_try = 0.5f * (v_feasible + v_too_high);
        if (zone_dist(v0, v_try) <= L) v_feasible = v_try; else v_too_high = v_try;
    }
    return v_feasible;
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
                     t_start_ms + total_ms, seg->x1, seg->y1, prof.v_end);

    printf("  v_start=%.1f  v_m=%.1f  v_end=%.1f mm/s  |  %.0f ms\n",
           prof.v_start, prof.v_m, prof.v_end, total_ms);
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
