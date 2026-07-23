#include <math.h>
#include "config.h"
#include "profile.h"

/* =====================================================================
   scurve_steps -- eq (1)-(7). Tri uy quyen chinh 2026-07-09, moi cho lech
   mat chu deu neo vao can cu trong CHINH paper (chi tiet + bang chung so:
   GHI_CHU_LOI_BAN_IN_PAPER.md):
   [P1] Delta-v = 0 -> vung 0 tick (mat chu [sqrt(0)+1]=1 -> k2=-2 -> NaN)
   [P2] eq (3)/(6) voi chieu DUNG: na > A/(J1*Ts) = "gia toc cham tran,
        can them nhip giu" (ban in sai chieu bat dang thuc -> ca dv=0
        bi xep nham che do; tuong duong voi do truc tiep a' > A)
   [P3] k1 = ceil(J*dv/A^2 - 1): nghiem nho nhat cua eq (1) thoa [P2]
   [P4] k = max(k1,k2): mot k chung phai thoa CA HAI ben (T2=kT1, T6=kT5)
   ===================================================================== */
void scurve_steps(float v_start, float v_m, float v_end,
                  int *out_na, int *out_nb, int *out_k)
{
    float J1 = MAX_JERK, J2 = MAX_JERK;
    float A  = MAX_ACCELERATION, D = MAX_ACCELERATION;
    float dva = v_m - v_start, dvb = v_m - v_end;
    int   k = 0, na, nb;
    int   k_truoc = k;               /* "If k is changed": so k moi vs k cu */

    na = (dva <= 1e-6f) ? 0                                          /* [P1] */
       : (int)(sqrtf(dva / ((1 + k) * J1 * TS * TS)) + 1.0f);        /* eq (2) */
    nb = (dvb <= 1e-6f) ? 0
       : (int)(sqrtf(dvb / ((1 + k) * J2 * TS * TS)) + 1.0f);        /* eq (5) */
    {
        /* FLOW MAT CHU (Tri chot 22/07): na >= A/(J1*Ts) -> NHAN na, khong
           dung toi k; na < nguong -> tinh lai k theo eq (3)/(6).
           HANG SO -1 (ban in ghi "-2" la loi in -- 3 chung minh, xem
           GHI_CHU; chung minh thu 3 cua Tri: eq (3) phai dong quy voi
           eq (1) tai dv=0 -> k -> -1; hang -2 cho dien tich AM).
           K = MAX cac can duoi vi k DUNG CHUNG 2 ve (eq 10: N2=k*na,
           N6=k*nb): "k nho nhat con hop le" = max; min cua ban in bo
           doi ve doi cao -> vuot tran 41% (demo t2.c). */
        int na_sat = (int)(A / (J1 * TS)), nb_sat = (int)(D / (J2 * TS));
        int k1 = k, k2 = k;
        if (na > 0 && na < na_sat)                                   /* eq (3) chieu "<" */
            k1 = (int)ceilf(J1 * dva / (A * A) - 1.0f);              /* [P3] hang -1 */
        if (nb > 0 && nb < nb_sat)                                   /* eq (6) chieu "<" */
            k2 = (int)ceilf(J2 * dvb / (D * D) - 1.0f);
        if (k1 < 0) k1 = 0;          /* luoi an toan: voi hang -1 gan nhu khong bao gio kich */
        if (k2 < 0) k2 = 0;
        k = (k1 > k2) ? k1 : k2;                                     /* [P4] k = max can duoi */

        /* CHOT AN TOAN ngoai dinh muc (dv > A^2/J = 160000 mm/s): nhanh
           "<" khong phu ca nay; bo trong thi a' = J*na*Ts vuot tran A.
           Nhanh chet trong de bai (dv <= 33.33) -- chi song o test tong hop. */
        if (na > 0 && na > na_sat) {
            int kk = (int)ceilf(J1 * dva / (A * A) - 1.0f);
            if (kk > k) k = kk;
        }
        if (nb > 0 && nb > nb_sat) {
            int kk = (int)ceilf(J2 * dvb / (D * D) - 1.0f);
            if (kk > k) k = kk;
        }
    }
    if (k != k_truoc) {              /* mat chu: "If k is changed, na and nb are calculated by (7)" */
        if (na > 0) na = (int)(sqrtf(dva / ((1 + k) * J1 * TS * TS)) + 1.0f);
        if (nb > 0) nb = (int)(sqrtf(dvb / ((1 + k) * J2 * TS * TS)) + 1.0f);
    }
    *out_na = na;  *out_nb = nb;  *out_k = k;
}

/* =====================================================================
   scurve_nc -- eq (9) nguyen van: giai eq (8) ra so tick vung deu.
   eq (8):  2L = (2+k)*((v_start+v_m)*na + (v_end+v_m)*nb)*Ts + 2*v_m*nc*Ts
   (= tong 3 khuc quang duong: tang + deu + giam, nhan 2 hai ve)
   VI DU SO THAT (seg 1 test.gcode: L=17.42, 0->50->50, na=50 nb=1 k=1):
     vung tang 3.75mm, vung giam 0.15mm, con 13.52mm chay deu
     -> 270.4 tick -> nc = [270.4] + 1 = 271
     -> profile di 17.45mm, LECH +0.03mm so voi L: gia cua ep tick nguyen
        ("nc is rounding" duoi eq (10)); eq (11)-(16) se va sai so nay. */
int scurve_nc(float L, float v_start, float v_m, float v_end,
              int na, int nb, int k)
{
    return (int)((2.0f*L - (2+k)*((v_start+v_m)*na + (v_end+v_m)*nb)*TS)
                 / (2.0f * v_m * TS)) + 1;
}


/* =====================================================================
   scurve_vm_fix -- eq (11)-(13) nguyen van: va sai so lam tron nc bang
   cach ha v_m xuong v_m' de dung nc tick nguyen di VUA KHIT quang duong L.
   eq (11): v_m' = (2L - (2+k)*(na*v_start + nb*v_end)*Ts)
                   / ((2*nc + (2+k)*(na+nb)) * Ts)
   eq (12): J1' = (v_m' - v_start) / ((1+k)*na^2*Ts^2)   (jerk tinh lai)
   eq (13): J2' = (v_m' - v_end ) / ((1+k)*nb^2*Ts^2)
   Rang buoc eq (14): J1' <= J1, J2' <= J2, v_m' <= v_m.
   Sai so tuong doi eq (15): eta = |v_m - v_m'| / v_m,
   chap nhan duoc khi eq (16): eta < 1/(na+nb+nc).                     */
float scurve_vm_fix(float L, float v_start, float v_end,
                    int na, int nb, int nc, int k,
                    float *out_J1p, float *out_J2p)
{
    float vmp = (2.0f*L - (2+k)*(na*v_start + nb*v_end)*TS)
              / ((2.0f*nc + (2+k)*(na+nb)) * TS);               /* eq (11) */
    *out_J1p = (na > 0) ? (vmp - v_start) / ((1+k) * (float)na*na * TS*TS)
                        : 0.0f;                                  /* eq (12) */
    *out_J2p = (nb > 0) ? (vmp - v_end ) / ((1+k) * (float)nb*nb * TS*TS)
                        : 0.0f;                                  /* eq (13) */
    return vmp;
}


/* =====================================================================
   scurve_type -- muc 2.2 paper: phan loai doan thanh 7 Type, eq (26)-(33).
   Doan cang ngan / chenh van toc cang nho thi profile cang rung bot bo phan:
     Type 1: tang - DEU - giam (du do)          eq (26)
     Type 2: tang - giam, mat vung deu           eq (27)
     Type 3: tang - giam, mat luon khuc giu ga   eq (28)
     Type 4: mot dau da o v_m (chi 1 doc + deu)  eq (29)/(30)
     Type 5: mot con doc thang vs -> ve          eq (31)
     Type 6: mot con doc, khong khuc giu ga      eq (32)
     Type 7: qua ngan, 1-2 tick                  eq (33)
   GHI CHU TRUNG THUC: PDF in 2 cot nen dieu kien eq (28)/(31) bi tron dong
   khi trich; to hop lai theo bac thang do dai (dai -> ngan) + doi xung
   tang/giam. Cho nao la suy dien co ghi (*). Doan DEU THUAN (vs=vm=ve,
   qua cung bo goc) paper khong neu -> xep Type 4 suy bien (chi vung deu).
   VI DU SO THAT (test.gcode): seg1 (L=17.42, 0<50=50) -> Type 4;
   seg6 (L=33.57, 0.5->50->0.3): nguong eq (26) = 7.56mm < L -> Type 1.  */
int scurve_type(float L, float v_start, float v_m, float v_end,
                int na, int nb, int k)
{
    float eps = 1e-4f;
    int vs_eq_vm = (v_m - v_start) < eps;
    int ve_eq_vm = (v_m - v_end ) < eps;

    /* Type 4 -- eq (29)/(30): mot dau bang v_m (ke ca deu thuan: ca 2 dau) */
    if (vs_eq_vm || ve_eq_vm) {
        if (vs_eq_vm && ve_eq_vm)
            /* deu thuan: van phai du dai toi thieu 1 tick duong di,
               khong thi la break path -- eq (33) ap chung */
            return (L < (v_start + v_end) * TS) ? 7 : 4;
        if (ve_eq_vm && 2.0f*L >= (2+k)*na*(v_start+v_end)*TS) return 4;
        if (vs_eq_vm && 2.0f*L >= (2+k)*nb*(v_start+v_end)*TS) return 4;
        return -4;  /* ngan hon ca 1 con doc toi v_m: ngoai eq (29)/(30), can hoi */
    }

    /* ca hai dau < v_m: bac thang theo do dai */
    {
        float B1 = 0.5f*(2+k)*((v_start+v_m)*na + (v_end+v_m)*nb)*TS; /* eq (26) */
        float B2 = ((v_start+v_m)*na + (v_end+v_m)*nb)*TS;            /* eq (27) */
        int   n_side = (v_end >= v_start) ? na : nb;   /* ben tang hay ben giam (*) */
        float B3 = 0.5f*(2+k)*n_side*(v_start+v_m)*TS;                /* eq (28)(*) */
        float B5 = n_side*(v_start+v_end)*TS;                         /* eq (31) */
        float B7 = (v_start+v_end)*TS;                                /* eq (32)/(33) */

        if (L >= B1) return 1;
        if (L >= B2) return 2;
        if (L >= B3) return 3;
        if (L >= B5) return 5;
        if (L >= B7) return 6;
        return 7;
    }
}


/* =====================================================================
   ALGORITHM A / B cua Section 3 = muc 2.1.3 / 2.1.4, eq (19)-(25).
   Cung mot y voi Algorithm C (scurve_vm_fix) nhung chon num khac de xoay:
     A: chinh v_start' + J1'  (v_m, v_end giu nguyen)   eq (19)-(22)
     B: chinh v_end'  + J2'   (v_m, v_start giu nguyen) eq (19),(23)-(25)
   eq (19) = eq (9) nhung KHONG cong 1 (lam tron XUONG): phan duong du
   duoc don ve phia endpoint -> chieu chinh luon la NANG len (eq 22/25:
   v' >= v), khong bao gio ha xuong duoi cam ket look-ahead.
   GHI CHU TRUNG THUC: eq (20)/(23) trong PDF bi tron dong 2 cot, dau cua
   so hang nb/na khong doc duoc chac chan -> lay bang cach giai eq (8)
   (1 phep chuyen ve); test thay-nguoc-vao-eq-(8) phai ra dung L la trong
   tai phan xu.                                                          */
int scurve_nc_ab(float L, float v_start, float v_m, float v_end,
                 int na, int nb, int k)                          /* eq (19) */
{
    return (int)((2.0f*L - (2+k)*((v_start+v_m)*na + (v_end+v_m)*nb)*TS)
                 / (2.0f * v_m * TS));
}

float scurve_vstart_fix(float L, float v_m, float v_end,     
                        int na, int nb, int k, int nc, float *out_J1p)
{
    float vsp = (2.0f*L - 2.0f*v_m*nc*TS) / ((2+k)*na*TS)        /* eq (20) */
              - (v_end + v_m)*nb / (float)na - v_m;
    *out_J1p = (v_m - vsp) / ((1+k)*(float)na*na*TS*TS);         /* eq (21) */
    return vsp;
}

float scurve_vend_fix(float L, float v_start, float v_m,        
                      int na, int nb, int k, int nc, float *out_J2p)
{
    float vep = (2.0f*L - 2.0f*v_m*nc*TS) / ((2+k)*nb*TS)        /* eq (23) */
              - (v_start + v_m)*na / (float)nb - v_m;
    *out_J2p = (v_m - vep) / ((1+k)*(float)nb*nb*TS*TS);         /* eq (24) */
    return vep;
}


/* =====================================================================
   scurve_zone_dist -- quang duong toi thieu de doi van toc v_lo -> v_hi
   theo S-curve (1 vung, eq (8) voi nc=0, bo qua ve): = so hang dau eq (8).
   scurve_reach -- nguoc lai: tu v0, trong quang duong L, S-curve len toi
   da duoc bao nhieu (khong vuot v_cap)? Paper khong cho cong thuc dong
   (nguoc eq (8) theo v_hi kho vi na,k nhay bac nguyen) -> tim nhi phan:
   don dieu "v_hi cao hon thi can duong dai hon" nen chac chan hoi tu.
   Day la trai tim 2 luot quet lui/toi cua look-ahead (Step 3).
   VI DU SO THAT: v0=0, L=2mm, cap=33.33: can 2.081mm moi len duoc 33.33
   -> reach tra ve ~32.6 mm/s (len het 2mm thi thieu mot ti).            */
float scurve_zone_dist(float v_lo, float v_hi)
{
    int na, nb, k;
    scurve_steps(v_lo, v_hi, v_hi, &na, &nb, &k);   /* ve=v_hi: chi vung tang */
    return 0.5f * (v_lo + v_hi) * (2 + k) * na * TS;
}

float scurve_reach(float v0, float v_cap, float L)
{
    if (scurve_zone_dist(v0, v_cap) <= L) return v_cap;  /* du duong: cham tran */
    float thap = v0, cao = v_cap;
    int lap;
    for (lap = 0; lap < 40; lap++) {                     /* 40 lan chia doi */
        float giua = 0.5f * (thap + cao);
        if (scurve_zone_dist(v0, giua) <= L) thap = giua; else cao = giua;
    }
    return thap;
}

/* =====================================================================
   scurve_peak -- dinh van toc cao nhat p de CA HAI cu doi van toc
   (v_start -> p roi p -> v_end) lot vua trong quang duong L.
   Day chinh la ban chat Type 2/3/5/6 cua muc 2.2: doan ngan thi dinh
   khong len toi v_m. Tim nhi phan tren zone_dist (don dieu theo p).   */
float scurve_peak(float v_start, float v_end, float v_cap, float L)
{
    float lo = (v_start > v_end) ? v_start : v_end, hi = v_cap;
    if (scurve_zone_dist(v_start, hi) + scurve_zone_dist(v_end, hi) <= L)
        return hi;
    int i;
    for (i = 0; i < 40; i++) {
        float mid = 0.5f * (lo + hi);
        if (scurve_zone_dist(v_start, mid) + scurve_zone_dist(v_end, mid) <= L)
            lo = mid;
        else
            hi = mid;
    }
    return lo;
}

/* =====================================================================
   scurve_v_tick -- eq (17)/(18): van toc TRUNG BINH cua tick thu n,
   tuc quang duong dao di trong tick n chia cho Ts. Xay tu [15]:
   lay s(t) giai tich tru nhau tai 2 mep tick -> ra da thuc theo n.
     p = Ts^2/6   (he so chung trong eq 17)
     n2..n7 (eq 18): chi so CUC BO cua tick trong tung pha:
       n2 = n - na              (pha giu ga,   0 <= n2 < k*na)
       n3 = n - (k+1)*na        (pha nha ga,   0 <= n3 < na)
       n4 = n - (k+2)*na        (pha deu,      0 <= n4 < nc)
       n5 = n4 - nc             (pha dap phanh,0 <= n5 < nb)
       n6 = n5 - nb             (pha giu phanh,0 <= n6 < k*nb)
       n7 = n6 - k*nb           (pha nha phanh,0 <= n7 < nb)
   Tong so tick 1 doan: N = (2+k)*na + nc + (2+k)*nb.

   CANH BAO BAN IN (2 cot bi tron dong): nhanh pha 3, 5, 6 trong PDF
   doc khong chac chan -> DUNG BAN DAO HAM LAI tu [15] (Tri chot
   2026-07-09; bang chung: ban mat chu hut 3.5-4.9mm + giat 99-132
   mm/s, ban dao ham chi lech 0.00003mm; pha 3 ban in ROI mat "+3na").
   Ban nguyen van da go khoi code 2026-07-20 sau khi xong nhiem vu
   doi chung; ho so day du: GHI_CHU_LOI_BAN_IN_PAPER.md.              */
float scurve_v_tick(int n, float v_start, float v_m, float v_end,
                    int na, int nb, int nc, int k,
                    float J1p, float J2p)
{
    float ptick = TS * TS / 6.0f;                     /* paper: p = Ts^2/6 */
    int n2 = n - na, n3 = n - (k+1)*na, n4 = n - (k+2)*na;
    int n5 = n4 - nc, n6 = n5 - nb, n7 = n6 - k*nb;

    if (n < na)                                       /* pha 1: vuot ga */
        return v_start + ptick*J1p*(3.0f*n*n + 3*n + 1);
    if (n2 < k*na)                                    /* pha 2: giu ga */
        return v_start + 3.0f*ptick*J1p*na*(2*n2 + na + 1);
    if (n3 < na) {                                    /* pha 3: nha ga */
        return v_start + ptick*J1p*(3.0f*na*na + 6.0f*k*na*na + 3.0f*na
                                    + 6.0f*na*n3 - 3.0f*n3*n3 - 3*n3 - 1);
    }
    if (n4 < nc)                                      /* pha 4: deu */
        return v_m;
    if (n5 < nb) {                                    /* pha 5: dap phanh */
        return v_m - ptick*J2p*(3.0f*n5*n5 + 3*n5 + 1);
    }
    if (n6 < k*nb) {                                  /* pha 6: giu phanh */
        return v_m - 3.0f*ptick*J2p*nb*(2*n6 + nb + 1);
    }
    /* pha 7: nha phanh -- mat chu PDF khop voi dao ham, dung chung */
    return v_end + ptick*J2p*(3.0f*nb*nb - 6.0f*nb*n7 - 3*nb
                              + 3.0f*n7*n7 + 3*n7 + 1);
}
