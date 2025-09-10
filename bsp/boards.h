#ifndef BOARDS_H__
#define BOARDS_H__

#define BSP_MINI_STM32G474RE

#if defined(BSP_MINI_STM32G474RE)
    #include "mini_stm32g474re_board.h"
#elif defined(BSP_MINI_STM32G474RE_V2)
    #include "mini_stm32g474re_board_v2.h"
#elif defined(BSP_V0_2)
    #include "board_v0_2.h"
#else
    #error "board is not defined"
#endif

#endif
