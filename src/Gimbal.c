#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>

#include "Gimbal.h"


// Real time
double GetWindowTime(void)
{
	LARGE_INTEGER	liEndCounter, liFrequency;

	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);

	return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
};

// Stop
void Stop(TaskHandle outputVc, TaskHandle outputVw) {

	int32 err1 = DAQmxWriteAnalogScalarF64(outputVc, 1.0, 5.0, 2.5, NULL);
	int32 err2 = DAQmxWriteAnalogScalarF64(outputVw, 1.0, 5.0, 0, NULL);

	if (err1 != 0 || err2 != 0) {
		printf("[STOP ERROR] Failed to stop system properly.\n");
		if (err1 != 0) printf(" - Vc Error Code: %d\n", err1);
		if (err2 != 0) printf(" - Vw Error Code: %d\n", err2);
	}
	else {
		printf("[STOP] System safely stopped.\n");
	}
}

void Stop_Vc(TaskHandle outputVc)
{
	double Vcmd = 2.5;
	DAQmxWriteAnalogScalarF64(outputVc, 1.0, 5.0, Vcmd, NULL);
	printf(">>> Vcmd STOP (%.2f V)\n", Vcmd);
}

//
double Get_Offset(TaskHandle Input_V)
{
	printf("Wait for the Vgyro Offset Calculating....\n");
	getchar();

	double V_offset = 0.0;
	double Vg = 0.0;
	int count_offset = 0;
	double time = 0.0;
	double time_prev = 0.0;
	double time_curr = GetWindowTime();


	do {
		time_prev = time_curr;
		time = count_offset * SAMPLING_TIME;

		Read_Vg(Input_V, &Vg);
		V_offset += Vg;
		count_offset++;


		// 샘플링 간격 유지
		while (1)
		{
			time_curr = GetWindowTime();
			if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
		}

	} while (time < 3.0);  // 5초 동안 반복

	V_offset = V_offset / count_offset;
	printf("V_offset: %.6f V \n", V_offset);

	return V_offset;

}

// Write
void Write_Vc(TaskHandle outputVc, TaskHandle outputVw, double Vcmd)
{
	DAQmxWriteAnalogScalarF64(outputVw, 1.0, 5.0, 5.0, NULL); // Wg 채널 항상 5V 출력
	DAQmxWriteAnalogScalarF64(outputVc, 1.0, 5.0, Vcmd, NULL);  // Vs 출력

}

/* ========= 1. Read Sensor Data ============ */
// All
void Read_Sensor(TaskHandle inputTask, float64* Vg, float64* Vp,float64* V_DIR,float64* V_DOA) {
	float64 data[4] = { 0.0 };
	int32 read = 0;

	int32 err = DAQmxReadAnalogF64(inputTask, -1, 1.0,DAQmx_Val_GroupByChannel,data, 4, &read, NULL);

	if (err != 0) {
		printf("[DAQ ERROR] DAQmxReadAnalogF64 failed with code %d\n", err);
		*V_DIR = 0.0;
		*V_DOA = 0.0;
		*Vg = 0.0;
		*Vp = 0.0;


		return;
	}
	*V_DIR = data[0];
	*V_DOA = data[1];
	*Vg = data[2];
	*Vp = data[3];
}

// Only Gyro
void Read_Vg(TaskHandle inputTask, float64* Vg)
{
	float64 data[4] = { 0.0 };
	int32 read = 0;

	int32 err = DAQmxReadAnalogF64(inputTask, -1, 1.0, DAQmx_Val_GroupByChannel, data, 4, &read, NULL);

	if (err != 0) {
		printf("[ERROR] readGyroOnly() failed: %d\n", err);
		*Vg = 0.0;
	}
	else {
		*Vg = data[2];
	}
}

// Only Potentiometer
void Read_Vp(TaskHandle inputTask, float64* Vp)
{
	float64 data[4] = { 0.0 };
	int32 read = 0;

	int32 err = DAQmxReadAnalogF64(inputTask, -1, 1.0,
		DAQmx_Val_GroupByChannel, data, 4, &read, NULL);

	if (err != 0) {
		printf("[ERROR] ReadPotentiometer() failed: %d\n", err);
		*Vp = 0.0;
	}
	else {
		*Vp = data[3];  // ai3만 사용
	}
}

void Read_V_DOA(TaskHandle inputTask, float64* V_DOA)
{
	float64 data[4] = { 0.0 };
	int32 read = 0;

	int32 err = DAQmxReadAnalogF64(inputTask, -1, 1.0,
		DAQmx_Val_GroupByChannel, data, 4, &read, NULL);

	if (err != 0) {
		printf("[ERROR] ReadPotentiometer() failed: %d\n", err);
		*V_DOA = 0.0;
	}
	else {
		*V_DOA = data[1];  // ai3만 사용
	}
}

void Read_Vp_DOA(TaskHandle inputTask, float64* Vp, float64* V_DOA)
{
	float64 data[4] = { 0.0 };
	int32 read = 0;

	int32 err = DAQmxReadAnalogF64(inputTask, -1, 1.0,
		DAQmx_Val_GroupByChannel, data, 4, &read, NULL);

	if (err != 0) {
		printf("[ERROR] Read_Vp_DOA() failed: %d\n", err);
		*Vp = 0.0;
		*V_DOA = 0.0;
	}
	else {
		*V_DOA = data[1];  // Dev3/ai1
		*Vp    = data[3];  // Dev3/ai3
	}
}

/* ========= 2. Linearization ============ */

// Vcmd to Vs (Linearization Function) (2025.07.28)
double fcmd(double Vcmd) {

	double Vs;

	double p1 = 0.015251896578492 ;
	double p2 = 0.003749626029857 ;
	double p3 = 0.444434833382093 ;
	double p4 = 2.691902398957750 ;

	double m1 = 0.058633342691436 ;
	double m2 = 0.142924728135780 ;
	double m3 = 0.595400261220998 ;
	double m4 = 2.263640957987132 ;

	if (Vcmd > 0.1)
	{
		// Vs = p1 * pow(Vcmd, 2) + p2 * Vcmd + p3;
		Vs = p1 * pow(Vcmd, 3) + p2 * pow(Vcmd, 2) + p3 *Vcmd + p4;
	}
	else if (Vcmd < -0.1)
	{
		// Vs = m1 * pow(Vcmd, 2) + m2 * Vcmd + m3;
		Vs = m1 * pow(Vcmd, 3) + m2 * pow(Vcmd, 2) + m3 *Vcmd + m4;
	}
	else
	{
		Vs = 2.5;
	}

	if (Vs >= 5) Vs = 5;
	if (Vs <= 0) Vs = 0;

	return Vs;
}

// Step Signal Vs to Wg (Linearization Data)
double Vs2Vg(double time)
{

	double time_revised = time / STEP_PERIOD;

	// Vstate (0 ~ 3)
	int Vstate = ((int)time_revised) % 4;

	// Sign Determination (0, +1, -1)
	double Vsign = 0.0;
	if (Vstate == 0 || Vstate == 2) {
		Vsign = 0.0;
	}
	else if (Vstate == 1) {
		Vsign = 1.0;
	}
	else if (Vstate == 3) {
		Vsign = -1.0;
	}

	// Scale Adjustment
	int Vscale = ((int)time_revised) / 4 + 1;

	// Vs
	double Vs = 2.5 + Vsign * Vscale * 0.1;

	return Vs;
}

// Step Signal Vcm to Wg for Validation
double Vc2Vg(double time){

	double time_revised = time / STEP_PERIOD;

	/* Making Vcmd */
	double Vstate = ((int)time_revised) % 4;

	double Vsign = 0.0;

	if (Vstate == 0 || Vstate == 2) {
		Vsign = 0.0;
	}
	else if (Vstate == 1) {
		Vsign = 1.0;
	}
	else if (Vstate == 3) {
		Vsign = -1.0;
	}

	double Vscale = ((int)time_revised) / 4 + 1;

	double Vcmd = 0.0 + Vsign * Vscale * 0.1;

	return Vcmd;
}


/* ========= 3. Motor Modeling ============ */


// 1. 주파수 배열 만드는 함수

// Make Freq List Array
void MakeFreqArray(double* freq_array, double start, double interval)
{
	for (int i = 0; i < FREQ_POINT; i++) {
		freq_array[i] = start + interval * i;
	}
}

// Make Sin Singnal
double Freq_Signal(double freq, double time, double amp)
{
	return amp * sin(2 * UNIT_PI * freq * time);
}

// Step Response for Motor Validation
double Step_Response_Motor(double time,double amp )
{

	double time_revised = time / STEP_PERIOD ;

	// Vstate (0 ~ 3)
	int Vstate = ((int)time_revised) % 4;

	double Vc = 0; ;

	if (Vstate == 0 || Vstate == 2) {
		Vc = 0.0;
	}
	else if (Vstate == 1) {
		Vc = amp;
	}
	else if (Vstate == 3) {
		Vc = - amp;
	}

	// Scale Adjustment
	int Vscale = ((int)time_revised) / 4 + 1;

	// Vs


	return Vc;

}


/* ========= 4. Potentio System ============ */

double Potentio(double Vp)
{
	double p1 =  70.2843588016516  ;  //  70.45
	double p2 = -175.710897004129  ;  // -177.2

	double degree = p1 * Vp + p2 ;

	return degree;
}


void Run_CW(TaskHandle outputVc) {

	double Vcmd = 2.75;
	DAQmxWriteAnalogScalarF64(outputVc, 1.0, 5.0, Vcmd, NULL);
}


void Run_CCW(TaskHandle outputVc) {
	double Vcmd = 2.2;
	DAQmxWriteAnalogScalarF64(outputVc, 1.0, 5.0, Vcmd, NULL);
}


void SetZero(TaskHandle inputTask, TaskHandle outputVc)
{
	float64 Vp = 0.0;
	double Vcmd = 0.0;

	while (1) {
		Read_Vp(inputTask, &Vp);
		printf("Input_Vp : %20.10f \n", Vp);

		if (Vp > 2.5) {
			Run_CCW(outputVc);
		}
		else if (Vp < 2.5) {
			Run_CW(outputVc);
		}

		if (Vp < 2.5005 && Vp > 2.4995) {
			Stop_Vc(outputVc);
			printf("Setting End \n");
			break;
		}
	}
}



/* ========= 4. Controller  ============ */


// 1. Stabilization

/* ========= 4. LPF  ============ */

void init_LPF(LPF_Tustin* lpf) {
	// 대역폭 60Hz 기준 Butterworth LPF
	// Ts에 따라 바뀔 수 있음
	lpf->Ts = 1.0 / 1000.0;     // 예: 1kHz 샘플링
	lpf->wc = 2 * UNIT_PI * 60.0;  // rad/s

	// 이미 MATLAB에서 얻은 계수 사용
	lpf->b0 = 0.2758;
	lpf->b1 = 0.5515;
	lpf->b2 = 0.2758;

	lpf->a1 = -0.06938;
	lpf->a2 = 0.1724;

	// 상태 초기화
	lpf->x1 = lpf->x2 = 0.0;
	lpf->y1 = lpf->y2 = 0.0;
}

double apply_LPF(LPF_Tustin* lpf, double x) {
	// 차례대로 y[n] 계산
	double y = lpf->b0 * x
			 + lpf->b1 * lpf->x1
			 + lpf->b2 * lpf->x2
			 - lpf->a1 * lpf->y1
			 - lpf->a2 * lpf->y2;

	// 상태 업데이트
	lpf->x2 = lpf->x1;
	lpf->x1 = x;

	lpf->y2 = lpf->y1;
	lpf->y1 = y;

	return y;
}

/* ========= 4. Potentio System ============ */
double DAO2VP (double V_DIR,double V_DOA) {


	double p1 =  0.0905 ;
	double p2 =  2.4929 ;

	double m1 =  -0.1079;
	double m2 = 2.5376 ;

	if (V_DIR > 2.5)
	{
		return m1 * V_DOA + m2;
	}
	else
	{
		return p1 * V_DOA + p2;
	}

}