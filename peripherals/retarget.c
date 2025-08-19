#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "stdio.h"
#include "stdint.h"

#include "boards.h"
#include "uart.h"

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined(__GNUC__)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif

/**
 * 重定向
 */
PUTCHAR_PROTOTYPE
{
    while ((USART1->ISR & 0X40) == 0);              /* 等待上一个字符发送完成 */

    USART1->TDR = (uint8_t)ch;                      /* 将要发送的字符 ch 写入到TDR寄存器 */
    return ch;
}

GETCHAR_PROTOTYPE
{
    uint8_t  ch = 0;

    usart1_rx(&ch);

    return ch;
}

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
// nothing
#elif defined(__GNUC__)
__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        *ptr++ = __io_getchar();
    }
    return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    (void)file;
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        __io_putchar(*ptr++);
    }
    return len;
}
#endif