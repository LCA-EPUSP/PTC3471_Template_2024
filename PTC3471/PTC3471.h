#ifndef Lab_H
#define Lab_H

#include "mbed.h"
#include "QEI.h"

#define min(X, Y)  ((X) < (Y) ? (X) : (Y))
#define max(X, Y)  ((X) > (Y) ? (X) : (Y))

#define pi 3.141592653589793
#define Ctrl_Ts 0.1

#endif

void Ctrl_LerAngulos(void);
void Ctrl_Travar(void);
void Ctrl_ChecarTrava(void);
void Ctrl_BlinkOk(void);
void Protecao_Init(QEI*, Ticker*, double);