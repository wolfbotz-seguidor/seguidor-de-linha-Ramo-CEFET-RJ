#include <stdio.h>
#include <stdbool.h>
#include "UART.h"

/*Macros*/
#define tempo           0
#define distancia       1
#define raio            2
#define vel_linear      0       //vel_linear_média
#define pwm_medio       1
#define acel_medio      2

unsigned int dados_valor_pwm(void);
unsigned int dados_distancia_calculo(void);
unsigned int dados_calculo_do_raio();
unsigned int dados_velocid_linear();
unsigned int dados_speed_avrg(void);
void dados_envia();
void dados_coleta(void);
void dados_telemetria(void);