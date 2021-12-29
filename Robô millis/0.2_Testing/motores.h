#include "HAL_atmega328p.h"
#include "PWM.h"

//Lado direito
#define AIN2 PD6
#define AIN1 PD5 // Quando em HIGH, roda direita anda para frente 

//Lado Esquerdo
#define BIN1 PD4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 PD3 

void motores_frente();
void motores_direita_frente();
void motores_esquerda_frente();
void motores_giro_esquerda();
void motores_giro_direita();
void motores_direita_tras();
void motores_esquerda_tras();
void motores_tras();
void motores_off();
void motores_freio();