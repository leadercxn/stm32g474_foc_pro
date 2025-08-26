#ifndef SIX_STEPS_H__
#define SIX_STEPS_H__

#include "stdint.h"

typedef struct
{
    uint16_t    freq_t_ref;         /* 开环根据固定霍尔状态启动频率 */
    uint16_t    voilage_ref;        /* 开环给定参考电压 */
    uint16_t    vvvf_huanx_count;   /* 换相计数 */
    uint16_t    vvvf_count;         /* VF计算 */
    uint16_t    vvvf_state;         /* VF换相状态 */
    uint16_t    vvvf_num[6];        /* VF霍尔换相顺序 */
    uint16_t    old_vf_state;       /* 历史VF换相状态 */
    uint16_t    initial_state;      /* VF初始状态定位 */
    uint16_t    freq_t_ref_count;   /* 六步换相换相计数 */
    uint16_t    count;              /* 换相电压的换相计数 */
}vvvf_start;

void anwerfen_sw(void);             /* 无感六步换相顺序 */
void change_voltage(void);          /* 改变换向电压 */
void zero_ctr_init(void);           /* 无感控制初始化 */
void zero_ctr_loop(void);           /* 无感控制逻辑 */

#endif
