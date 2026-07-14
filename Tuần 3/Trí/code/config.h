#ifndef CONFIG_H
#define CONFIG_H

/* =============================================================
   HANG SO (cac knob hieu chinh)
   ============================================================= */
#define MAX_ACCELERATION  40000.0f/*(mm/s^2)*/


#define RAPID_FEEDRATE   2000.0f /*(mm/min)*/


#define MAX_JERK         10000.0f/*(mm/s^3)*/

#define TS               0.001f  /* 1ms */


#define LOOKAHEAD_PATHS     100

#define MAX_SEGMENTS        100

#endif /* CONFIG_H */
