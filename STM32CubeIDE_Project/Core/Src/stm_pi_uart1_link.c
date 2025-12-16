#include "stm_pi_uart1_link.h"
#include <string.h>

/**
 * @brief Sets all buffers and flags to their default values, preparing the structure for communication.
 */
void STM_Pi_UART1_Link_Init(stm_pi_uart1_link_t *stm_pi_uart1_link) {
    memset(stm_pi_uart1_link, 0, sizeof(*stm_pi_uart1_link));
    stm_pi_uart1_link->ready_byte = READY_BYTE;
}
