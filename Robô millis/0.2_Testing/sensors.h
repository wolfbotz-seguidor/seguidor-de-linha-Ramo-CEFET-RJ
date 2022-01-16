#include "HAL_atmega328p.h"
#include <stdbool.h>

/* Macros */
#define PWM_RETURN      80
#define NOP() __asm__ __volatile__ ("nop")

void sensors_ADC_maq();
void sensors_laterais(void);
void sensors_sentido_de_giro();
void sensors_frontais(int *erro_sensores, int *speedW, int *speedX, unsigned int *PWM_general, unsigned int *PWMR, unsigned int *PWM_Curva);
//void sensors_volta_pra_pista(void);

// ================================================================================================================================================

/* 
 * Comentarios
 * 
 * sensors_leitura_de_pista <- sensors_frontais
 * 
 * sensors_le_marcadores    <- sensors_laterais
 *  
 */