#include "PTC3471.h"
#include "mbed.h"
#include "QEI.h"
/////////////////////#include "USBSerial.h"

DigitalOut Ctrl_Status(LED1);

QEI * Ctrl_Encoder_Motor;
Ticker * Controler_Interrupt;

DigitalOut Ctrl_Direita(PA_7);//
DigitalOut Ctrl_Esquerda(PA_4);
PwmOut Ctrl_Motor(PA_6);//

Ticker Ctrl_StatusOk;
Ticker Ctrl_Trava;

double Ctrl_Phi = 0;
double Ctrl_Phi1 = 0;
double Ctrl_dPhi = 0;
double LimitAngle = 0;

void Ctrl_LerAngulos(void){
    Ctrl_Phi1 = Ctrl_Phi;
    Ctrl_Phi = pi*Ctrl_Encoder_Motor->getPulses()/320.0;
    Ctrl_dPhi = (Ctrl_Phi-Ctrl_Phi1);
}

void Ctrl_Travar(void){
    Ctrl_LerAngulos();
    
    if(Ctrl_dPhi>=0.005){
        Ctrl_Motor = 0.0;//min(1.5*Ctrl_dPhi, 0.15);
        Ctrl_Direita = 1;
        Ctrl_Esquerda = 1;
    }
    else if(Ctrl_dPhi<=-0.005){
        Ctrl_Motor = 0.0;//min(-1.5*Ctrl_dPhi, 0.15);
        Ctrl_Direita = 1;
        Ctrl_Esquerda = 1;
    }
    else{
        Ctrl_Direita=1;
        Ctrl_Esquerda=1;
    }
    
    if(Ctrl_Status==0){
        Ctrl_Status = 1;
    }
    else{
        Ctrl_Status = 0;
    }
}

void Ctrl_ChecarTrava(void){
    Ctrl_LerAngulos();
    if(Ctrl_Phi>LimitAngle || Ctrl_Phi<-LimitAngle || Ctrl_dPhi > 1.05 || Ctrl_dPhi < -1.05){
        Ctrl_StatusOk.detach();
        Ctrl_Trava.detach();
        Controler_Interrupt->detach();
        Ctrl_Trava.attach(&Ctrl_Travar, Ctrl_Ts);
    }
}

void Ctrl_BlinkOk(void){
    Ctrl_Status = !Ctrl_Status;
}

void Protecao_Init(QEI* Encoder_Motor, Ticker* Control_Interrupt, double angle){
    
    Ctrl_Encoder_Motor = Encoder_Motor;
    Controler_Interrupt = Control_Interrupt;
    LimitAngle = angle;
    
    Ctrl_Status = 1;
    Ctrl_StatusOk.attach(&Ctrl_BlinkOk, 1);
    
    if(LimitAngle>0)
        Ctrl_Trava.attach(&Ctrl_ChecarTrava, Ctrl_Ts);
    
    Ctrl_Motor = 0.0;
    Ctrl_Direita = 0;
    Ctrl_Esquerda = 1;
}