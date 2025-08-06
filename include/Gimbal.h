/*------------------------------------------------
2025/05/19
DCSP Gimbal Project
22100252 Ryu-Minseo
-------------------------------------------------*/

#ifndef Gimbal_H
#define Gimbal_H

#include "NIDAQmx.h"

#define   N_STEP		    (int)   ( FINAL_TIME * SAMPLING_FREQ )
#define   FINAL_TIME		(double)(				100.0 )
#define   SAMPLING_FREQ		(double)(               200.0 )
#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )
#define   UNIT_PI			(double)( 3.14159265358979  )
#define   STEP_PERIOD		(double)( 1.00  )
#define	  NUM_POTENTIO		(int)   ( 21 )
#define	  NUM_DOA	    	(int)   ( 5 )
#define   FREQ_POINT		(int)	( 10 )

typedef struct LPF_Tustin{
    double Ts, wc;        // Sampling time and cutoff frequency (optional)

    double b0, b1, b2;     // Numerator coefficients
    double a1, a2;         // Denominator coefficients (a0 = 1 assumed)

    double x1, x2;         // Previous inputs
    double y1, y2;         // Previous outputs
} LPF_Tustin;

// Real Time
double GetWindowTime(void);

// Stop
void Stop(TaskHandle outputVc, TaskHandle outputVw);

void Stop_Vc(TaskHandle outputVc);

//
double Get_Offset(TaskHandle Input_V);

//Write
void Write_Vc(TaskHandle outputVc, TaskHandle outputVw, double Vcmd);


/* ========= 1. Read Sensor Data ============ */

// Read All  Sensor
void Read_Sensor(TaskHandle inputTask, float64* Vg, float64* Vp,float64* V_DIR,float64* V_DOA);
// Only Gyro
void Read_Vg(TaskHandle inputTask, float64* Vg);
// Only Potentiometer
void Read_Vp(TaskHandle inputTask, float64* Vp);
// Only DOA
void Read_V_DOA(TaskHandle inputTask, float64* V_DOA);

void Read_Vp_DOA(TaskHandle inputTask, float64* Vp, float64* V_DOA);

/* ========= 2. Linearization ============ */

// Vcmd to Vs
double fcmd(double Vcmd);

// Step Signal Vs to Wg
double Vs2Vg(double time);

// Step Signal Vcm to Wg for Validation
double Vc2Vg(double time);


/* ========= 3. Motor Modeling ============ */

// Make Freq List Array
void MakeFreqArray(double* freq_array, double start, double step);

// Make Sin Singnal
double Freq_Signal(double freq, double time, double amp);

// Step Response for Motor Validation
double Step_Response_Motor(double time, double amp);

/* ========= 4. Potentio System ============ */

double Potentio(double Vp);
void Run_CW(TaskHandle outputVc)  ;
void Run_CCW(TaskHandle outputVc) ;
void SetZero(TaskHandle inputTask, TaskHandle outputVc);

/* ========= 5. LPF ============ */
void init_LPF(LPF_Tustin* lpf);
double apply_LPF(LPF_Tustin* lpf, double x);


double DAO2VP (double V_DIR,double V_DOA);

#endif