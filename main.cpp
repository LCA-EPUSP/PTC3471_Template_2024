#include "PTC3471.h" //V. 2024
#include "QEI.h"
#include "math.h"
#include "mbed.h"

#define Ts 0.01 // periodo de amostragem
#define pi 3.141592653589793

/******************************************************************************/
/****************  Definição de Variaveis, Objetos e Funções ******************/
/******************************************************************************/
Serial pc(USBTX, USBRX);

Ticker Control_Interrupt; // Interrupção de Tempo para acionamento do algoritmo
                          // de controle

QEI Encoder_Motor(PB_0, PA_12, NC, 160,QEI::X4_ENCODING); // Objeto de leitura do encoder do motor

DigitalIn AAA(PB_7);
DigitalIn BBB(PB_1);
QEI Encoder_Pendulo(PB_1, PB_7, NC, 600,QEI::X4_ENCODING); // Objeto de leitura do encoder do pêndulo

DigitalOut Horario(PA_4); // DigitalOut que sinaliza se deve virar o motor no sentido horário
DigitalOut AntiHorario(PA_7); // DigitalOut que sinaliza se deve virar o motor
                              // no sentido anti-horário
PwmOut Motor(PA_6); // D.C. do PWM [0, 1]: porcentagem de tensão sobre o motor

bool Flag_Controle = false;
int PlotCount = 0;

double phi0 = 0; // phi0 -> Angulo lido pelo Encoder_Braco
double phi1 = 0; // phi1 -> Angulo lido pelo Encoder_Pendulo

double th0 = 0;  // th0 -> Angulo do braço
double th1 = 0;  // th1 -> Angulo do pêndulo
double dth0 = 0; // dth0 -> Velocidade do braço
double dth1 = 0; // dth1 -> Velocidade do pêndulo

double th0_f = 0;  // th0 -> Angulo do braço filtrado
double th1_f = 0;  // th1 -> Angulo do pêndulo filtrado
double dth0_f = 0; // dth0 -> Velocidade do braço
double dth1_f = 0; // dth1 -> Velocidade do pêndulo

int16_t phi0_int = 0, phi1_int = 0; // Variáveis convertidas para inteiro para serem
int16_t th0_f_int = 0, th1_f_int = 0; // transmitidas via serial
int16_t dth0_f_int = 0, dth1_f_int = 0;

double tau = 4e-2; // Cte de tempo do FPB dos estados

double th0_a = 0;  // Valor de th0 um período de amostragem anterior
double th1_a = 0;  // Valor de th1 um período de amostragem anterior

float K[4] = {0.0, 0.0, 0.0, 0.0};


float u = 0.0; // Inicialização da lei de controle

float t_end = 30.0; // Duração do Experimento
float tempo = 0;    // Acumula o tempo de execução do experimento
                 // Esta variável pode ser usada para temporização da referência

int N_plot = (int)t_end / (Ts * 10); // Numero de amostras armazenadas para plot

void Init(void); // Função de Inicialização
void Control_Function(void); // Função de flag do controle, a ser chamada pela interrupção
void Sensor_Read(void); // Função de leitura dos sensores
void Controle_Algoritmo(void); // Função que implementa o algoritmo de controle escolhido

/******************************************************************************/
/*************************** Corpo de Funções *********************************/
/******************************************************************************/

/*************************** Função Principal *********************************/
// A main chama todas as inicializações e então aguarda o sinal de que deve
// realizar controle. Esse sinal é dado pela flag "Controle" e é setada por uma
// interrupção de tempo.
//
// Para garantir a execução imediata do algoritmo de controle nenhum wait deve
// ser chamado durante a execução do controle e o uso de printf's deve ser esporádico.
int main() {
  AAA.mode(PullUp);
  BBB.mode(PullUp);
  pc.baud(9600); // set terminal speed, default 9600
  int ap = 0;    // Indice dos vetores de amostras
  int ii = 0;    // Indice para plot das amostras
  int16_t th0_f_int[N_plot], th1_f_int[N_plot]; // Vetores para armazenar dados e
  int16_t dth0_f_int[N_plot], dth1_f_int[N_plot],  u_inte[N_plot]; // serem transmitidos via serial

  /*********************************************************************************/
  /** Inicialização do algoritmo de proteção. NUNCA DEVE SER RETIRADO DO PROGRAMA **/
  /**/                                wait(5);                                   /**/
  /**/                                                                           /**/
  /**/          Protecao_Init(&Encoder_Motor, &Control_Interrupt, pi);           /**/
  /** Inicialização do algoritmo de proteção. NUNCA DEVE SER RETIRADO DO PROGRAMA **/
  /*********************************************************************************/

  Init();

  while (1) {

    if (Flag_Controle) {

      Sensor_Read();        // Executa a leitura dos sensores
      Controle_Algoritmo(); // Execução do seu algoritmo de controle

      Flag_Controle = false; // Sinaliza que deve-se esperar o próximo sinal da interrupção
                 // de tempo para executar o próximo passo de controle
                 
      PlotCount++;
      if (PlotCount >=10) { // Controla para que o printf ocorra a cada 10 iterações

        // As variáveis serão multiplicadas por 1000 e convertidas para inteiro
        // antes de serem trasmitidas. Ao receber, deve-se dividir por 1000
        // antes de fazer o plot. OBS: a precisão no gráfico será de 3 casas
        // decimais
        th0_f_int[ap] = th0_f * 1000;
        th1_f_int[ap] = th1_f * 1000;
        dth0_f_int[ap] = dth0_f * 1000;
        dth1_f_int[ap] = dth1_f * 1000;
        u_inte[ap] = u * 1000;

        ap = ap + 1; // Prepara para a próxima amostra
        PlotCount = 0;
      }


    }
    // Após t_end segundos, o experimento é interrompido e os dados são
    // transmitidos via serial
    if (tempo >= t_end) {
      Control_Interrupt.detach();
      Motor = 0;
      Horario = 0;
      AntiHorario = 0;
      for (ii = 0; ii < N_plot; ii++){
        printf("%d \t %d \t %d \t %d \t %d\n\r", th0_f_int[ii], th1_f_int[ii],
               dth0_f_int[ii], dth1_f_int[ii], u_inte[ii]);
        }
        break;
    }
  }
}

/************** Função de implementação do algoritmo de controle **************/
// Nesta função você deve escrever a implementação do algoritmo de controle es-
// colhido e do algoritmo de estimação das velocidades.
// Caso necessite acesso a alguma variavel não medida ou alguma cons-
// tante não definida sinta-se livre para passa-las como argumento, definir
// como variavel global ou com um #define
void Controle_Algoritmo(void) {

  dth0 = (th0 - th0_a) / Ts; // Calculo das velocidades por backward
  dth1 = (th1 - th1_a) / Ts; // É interessante propor outro método

  // Filtro (1/tau*s +1) nos derivadas
  dth0_f = (tau / (Ts + tau)) * dth0_f + (Ts / (Ts + tau)) * dth0;
  dth1_f = (tau / (Ts + tau)) * dth1_f + (Ts / (Ts + tau)) * dth1;

  u = -(K[0] * th0_f + K[1] * th1_f + K[2] * dth0_f + K[3] * dth1_f);
  

  if (u > 0.5)
    u = 0.5;
  if (u < -0.5)
    u = -0.5;

  if (u < 0) {
    Motor = -u;
    Horario = 1;
    AntiHorario = 0;
  } else if (u > 0) {
    Motor = u;
    Horario = 0;
    AntiHorario = 1;
  } else {
    Motor = 0;
    Horario = 0;
    AntiHorario = 0;
  }
}

/************************* Função de Inicialização
 * *****************************/
// Esta função concentra todas as inicializações do sistema
void Init(void) {
  Motor.period(0.0001);
  Horario = 0;
  AntiHorario = 0;
  Motor = 0.0;
  Control_Interrupt.attach(&Control_Function, Ts);
}

/********************** Função de leitura dos sensores
 * *************************/
// Cada vez que esta função é chamada deve-se calcular os ângulos e velocidades
// angulares por algum método conhecido
void Sensor_Read(void) {
  th0_a = th0;
  th1_a = th1;

  /** Leituras cruas dos ângulos do encoder **/
  phi0 = pi * Encoder_Motor.getPulses() / 320.0;
  phi1 = pi * Encoder_Pendulo.getPulses() / 1200.0;

  th0 = phi0;
  /** Tratamento do ângulo lido para ser zero na vertical para cima **/
  // Como o encoder é incremental quando inicializamos o programa com o pêndulo
  // na posição
  if (phi1 > 0) // vertical para baixo esta passa a ser lida como 0º. Porém,
                // para o algoritmo de controle
    th1 = phi1 - pi; // funcionar corretamente 0º deve ser o pêndulo na posição
                    // vertical para cima. Para garantir que isso aconteça subido
                    // o pêndulo no sentido horário ou anti-horário fazemos
  else if (phi1 <= 0) // th1 = th1-sgn(th1)*pi, onde sgn(x) é o sinal de x.
    th1 = phi1 + pi;

  // Filtro (1/tau*s +1) nos angulos
  th0_f = (tau / (Ts + tau)) * th0_f + (Ts / (Ts + tau)) * th0;
  th1_f = (tau / (Ts + tau)) * th1_f + (Ts / (Ts + tau)) * th1;
  th1_f = atan2(sin(th1_f), cos(th1_f));
}

/**************** Função de flag do algoritmo de controle ******************/
// Esta função avisa a main quando executar o próximo passo do algoritmo de
// controle. O uso de uma interrupção para o acionamento da flag garante que
// haja exatamente Ts segundos entre execuções.
void Control_Function(void) {

  Flag_Controle = true;
  tempo = tempo + Ts;
}