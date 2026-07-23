#ifndef PROFILE_H
#define PROFILE_H

/* =====================================================================
   PROFILE -- toan van toc S-curve theo paper Chen 2013 (tick nguyen)

   TU DIEN KY HIEU (dung thong nhat moi ham):
     Ts        chu ky mau = 1 tick = 1ms (config.h)
     v_start   van toc VAO doan (mm/s)   -- look-ahead quyet
     v_m       van toc mong muon (mm/s)  -- tu F cua G-code
     v_end     van toc RA doan (mm/s)    -- look-ahead quyet
     na        so tick pha VUOT ga (T1 = T3 = na*Ts)
     nb        so tick pha VUOT phanh (T5 = T7 = nb*Ts)
     nc        so tick pha chay DEU (T4 = nc*Ts)
     k         pha giu dai gap k lan pha vuot (T2 = k*T1, T6 = k*T5)
     J1, J2    jerk vung tang / vung giam (mm/s^3); ta chon J2 = J1
     A,  D     tran gia toc tang / giam (mm/s^2); ta chon D = A
     J1', J2'  jerk TINH LAI sau khi lam tron tick (eq 12/13/21/24)
     v_m', v_start', v_end'  gia tri da chinh de khop L (eq 11/20/23)

   BAN DO eq -> ham:
     eq (1)-(7)   scurve_steps        na, nb, k
     eq (8)-(9)   scurve_nc           nc (lam tron LEN: [x]+1)
     eq (11)-(13) scurve_vm_fix       Algorithm C - chinh dinh, giu 2 dau
     eq (15)-(16) (test_profile.c)    kiem eta = do ha v_m co dang ke khong
     eq (17)-(18) scurve_v_tick       van toc trung binh tick thu n
     eq (19)      scurve_nc_ab        nc cho A/B (lam tron XUONG)
     eq (20)-(21) scurve_vstart_fix   Algorithm A - chinh van toc vao
     eq (23)-(24) scurve_vend_fix     Algorithm B - chinh van toc ra
     eq (26)-(33) scurve_type         phan loai 7 Type theo do dai
     (suy tu 8)   scurve_zone_dist    duong toi thieu cho 1 cu doi van toc
     (suy tu 8)   scurve_reach        nguoc lai: L cho truoc, len duoc bao nhieu
   ===================================================================== */

/* ---- S-curve Chen 2013, ban tick nguyen ---- */
void scurve_steps(float v_start, float v_m, float v_end,
                  int *out_na, int *out_nb, int *out_k);        /* eq (1)-(7) nguyen van */
int scurve_nc(float L, float v_start, float v_m, float v_end,
              int na, int nb, int k);                           /* eq (8)-(9) */

float scurve_vm_fix(float L, float v_start, float v_end,
                    int na, int nb, int nc, int k,
                    float *out_J1p, float *out_J2p);            /* eq (11)-(13) */

int scurve_type(float L, float v_start, float v_m, float v_end,
                int na, int nb, int k);                         /* eq (26)-(33) */

int   scurve_nc_ab(float L, float v_start, float v_m, float v_end,
                   int na, int nb, int k);                      /* eq (19) */
float scurve_vstart_fix(float L, float v_m, float v_end,
                        int na, int nb, int k, int nc, float *out_J1p); /* Alg A */
float scurve_vend_fix(float L, float v_start, float v_m,
                      int na, int nb, int k, int nc, float *out_J2p);   /* Alg B */

float scurve_zone_dist(float v_lo, float v_hi);   /* so hang eq (8), 1 vung */
float scurve_reach(float v0, float v_cap, float L);/* nguoc eq (8): tim nhi phan */

float scurve_v_tick(int n, float v_start, float v_m, float v_end,
                    int na, int nb, int nc, int k,
                    float J1p, float J2p);                      /* eq (17)-(18) */

float scurve_peak(float v_start, float v_end, float v_cap, float L);
                                    /* Type 2/3/5/6: dinh kha thi cua doan ngan */
#endif /* PROFILE_H */
