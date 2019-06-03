#ifndef ac_bmp3_h
#define ac_bmp3_h

#include <inttypes.h>

typedef int (*ac_bmp3_rw_func)(uint8_t dev,
                               uint8_t reg,
                               void *buf,
                               size_t len);

extern int ac_bmp3_initalize(ac_bmp3_rw_func r_func,
                             ac_bmp3_rw_func w_func);

extern int ac_bmp3_measure(double *temp, double *press);

#endif /* ac_bmp3_h */
