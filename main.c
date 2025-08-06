#define _CRT_SECURE_NO_WARNINGS


#include <conio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>

#include "Gimbal.h"


double		Out_Time[N_STEP]  = { 0.0, };
double		Out_Vs[N_STEP]    = { 0.0, };
double		Out_Vc[N_STEP]    = { 0.0, };
double		Out_Vg[N_STEP]    = { 0.0, };
double		Out_Wg[N_STEP]    = { 0.0, };
double		Out_Vp[N_STEP]    = { 0.0, };

double		Out_Ai[N_STEP]    = { 0.0, };
double		Out_Ao[N_STEP]    = { 0.0, };

double		Out_Ki[N_STEP]    = { 0.0, };
double		Out_Kp[N_STEP]    = { 0.0, };
double		Out_Er_1[N_STEP]    = { 0.0, };
double		Out_Er_2[N_STEP]    = { 0.0, };




int main(void)
{
	LPF_Tustin LPF;
	init_LPF(&LPF);

	printf("Start \n");

	double Vflag = 5.0;
	double Vcmd = 0.0;
	double Vs = 0.0;

	float64 Vp = 0.0;
	float64 Vg = 0.0;
	float64 Wg = 0.0 ;
	float64 V_DIR = 0.0 ;
	float64 V_DOA = 0.0 ;


	TaskHandle Output_Vw = 0;
	TaskHandle Output_Vc = 0;
	TaskHandle	Input_V = 0;

	// {Vg, Vp}
	double data[4] = { 0, };
	int32 read = 0;
	double READ = 1;


	double		time_prev = 0.0;
	double		time_curr = 0.0;
	double		time ;

	int			count = 0;
	double		cur_count = 0;

	char key;
	char check;

	// ============= Controller ================ //

	double num_m = 10.88 ;
	double den_m[2] = { 1, 31.1403 };

	double Pm = den_m[1] ;
	double tau_m = 1 / den_m[1] ;
	double Km = num_m * tau_m ;
	double Kg = 1 / 0.00067 ;

	double T = SAMPLING_TIME ;

	// ============= Designation Controller ================ //

	double zeta_D = 0.707 ;
	double wn_D   = 35.5  ;

	double Ai_scale = 0.0 ;
	double Ai = 0.0 ;
	double Ah = 0.0 ;

	double Kd_D = (tau_m * 2 * wn_D * zeta_D - 1) / (Km * Kg) ;
	double Kp_D = (pow(wn_D,2) * tau_m) / (Km * Kg) ;

	double K2 = Kd_D ;
	double K1 = Kp_D / Kd_D ;

	// ============= Stabilization Controller ================ //

	double zeta_S = 0.707 ;
	double wn_S   = 35.5  ;

	double Kp_S = ((2 * wn_S * zeta_S / Pm) - 1) / (Km * Kg) ;
	double Ki_S = (pow(wn_S,2) / (Km * Kg * Pm)) ;

	double E1 = 0.0 ;
	double E2 = 0.0 ;
	double Signal_D = 0.0 ;
	double Signal_W = 0.0 ;

	/* Controller Buffer */
	double signal_Ki = 0.0 ;
	double signal_Kp = 0.0 ;

	double Integral = 0.0;

	double Wi_scale = 0.0 ;
	double Wi = 0.0 ;


	/* Controller Buffer */
	double Error = 0.0 ;
	double Error_Cur = 0.0 ;
	double Error_Pre = 0.0 ;


	// ============= Tracking Controller ================ //
	double Deg = 0.0 ;
	double Kt = 4 ;

	// ============= File ================ //

	// 2. Potentio
	FILE* File_Potentio;
	char FileName_Potentio[100] = { "" };

	double DGR[NUM_POTENTIO] = { 0.0 };
	double VP[NUM_POTENTIO] = { 0.0 };

	for (int i = 0; i < NUM_POTENTIO; i++) {
		DGR[i] = -60 + i * 6;
	}

	// 3. DOA
	FILE* File_DOA;
	char FileName_DOA[100] = { "" };

	double DOA[NUM_DOA] = { 0.0 };
	double VP_DOA[NUM_DOA] = { 0.0 };


	// ============= Basic Setting ============== //

	DAQmxCreateTask("", &Output_Vw);
	DAQmxCreateTask("", &Output_Vc);
	DAQmxCreateTask("", &Input_V);

	DAQmxCreateAOVoltageChan(Output_Vw, "Dev3/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(Output_Vc, "Dev3/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, "");
	DAQmxCreateAIVoltageChan(Input_V, "Dev3/ai0:3", "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, "");

	DAQmxStartTask(Output_Vw);
	DAQmxStartTask(Output_Vc);
	DAQmxStartTask(Input_V);

	printf("Press any key to start the program.... \n");
	getchar();

	Stop(Output_Vc, Output_Vw);

	printf("Calculate Vg Offset (wait 3 second) \n");
	double V_Offset = Get_Offset(Input_V);

	printf("Enter Key \n");

	while (1)
	{
		// Settting (Vc = stop / Vw = ready to work)
		DAQmxWriteAnalogScalarF64(Output_Vc, 1.0, 5.0, 2.5, NULL);
		DAQmxWriteAnalogScalarF64(Output_Vw, 1.0, 5.0, 5.0, NULL);

		Read_Sensor(Input_V, &Vg, &Vp,&V_DIR, &V_DOA);

		cur_count++;

		if (_kbhit())
		{
			key = _getch();

			// 1. Linearization
			if (key == 'l' || key == 'L')
			{

				printf("Get Linearization Data \n");

				time_curr = GetWindowTime();

				for (int i = 0; i < N_STEP; i++) {

					time_prev = time_curr;
					time = SAMPLING_TIME * i;

					Vs = Vs2Vg(time);

					Write_Vc(Output_Vc, Output_Vw, Vs);
					Read_Vg(Input_V, &Vg);

					Out_Time[i] = time ;
					Out_Vs[i] = Vs ;
					Out_Vg[i] = Vg ;


					// 샘플링 타임 조절
					while (1) {
						time_curr = GetWindowTime();
						if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
					}
				}

				// Stop
				Stop(Output_Vc, Output_Vw);
				printf("[Linearization End]\n");


				// File

				FILE* File_Linearization = fopen("../data/linearization/Linearization.out", "w+t");
				if (File_Linearization == NULL) {

					printf("Fail to Open File \n");  // 에러 출력
					exit(EXIT_FAILURE);        // 프로그램 종료
				}
				for (int i = 0; i < N_STEP; i++) {
					fprintf(File_Linearization, "%20.10f %20.10f %20.10f\n", Out_Time[i], Out_Vs[i], Out_Vg[i]);
				}
				fclose(File_Linearization);

				break ;
			}

			// 1. Linearization Validation
			if (key == 'v' || key == 'V')
			{

				printf("Start Linearization Validation \n");
				printf("V_offset: %.6f V", V_Offset);

				time_curr = GetWindowTime();

				for (int i = 0; i < N_STEP; i++) {

					time_prev = time_curr;
					time = SAMPLING_TIME * i;

					Vcmd = Vc2Vg(time);

					Vs = fcmd(Vcmd);

					Write_Vc(Output_Vc, Output_Vw, Vs);
					Read_Vg(Input_V, &Vg);

					Out_Time[i] = time;
					Out_Vs[i] = Vs;
					Out_Vc[i] = Vcmd;
					Out_Vg[i] = Vg;
					Out_Wg[i] = (Vg-V_Offset)* Kg ;


					// 샘플링 타임 조절
					while (1) {
						time_curr = GetWindowTime();
						if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
					}
				}

				// Stop
				Stop(Output_Vc, Output_Vw);
				printf("[Validation End]\n");

				// File

				FILE* File_Validation = fopen("../data/linearization/Validation_Result.out", "w+t");
				for (int i = 0; i < N_STEP; i++) {
					fprintf(File_Validation, "%20.10f %20.10f %20.10f %20.10f %20.10f\n", Out_Time[i], Out_Vs[i], Out_Vc[i], Out_Vg[i], Out_Wg[i]);
				}
				fclose(File_Validation);

				break;
			}

			// 2. Clock Wise
			else if (key == 'd' || key == 'D')
			{
				printf("Input_Vp : %20.10f \n", Vp);

				while (1)
				{
					Run_CW(Output_Vc);

					if (_kbhit())
					{
						check = _getch();

						if (check == 'e' || check == 'E')
						{
							Stop_Vc(Output_Vc);
							printf("Input_Vp : %20.10f \n", Vp);
							break;
						}

					}
				}
			}

			// 2. Count Clock Wise
			else if (key == 'a' || key == 'A')
			{
				printf("Input_Vp : %20.10f \n", Vp);

				while (1)
				{
					Run_CCW(Output_Vc);

					if (_kbhit())
					{
						check = _getch();

						if (check == 'e' || check == 'E')
						{
							Stop_Vc(Output_Vc);
							printf("Input_Vp : %20.10f \n", Vp);
							break;

						}

					}
				}


			}

			// 2. 0[degree] set
			else if (key == 'w' || key == 'W')
			{
				SetZero(Input_V, Output_Vc);
			}

			// 2. Save Data for Potentiometer Modeling
			else if (key == 'k' || key == 'K')
			{
				Read_Vp(Input_V, &Vp);
				printf("Input_Vp : %20.10f \n", Vp);

				VP[count] = Vp;
				printf("DGR : %20.10f ,VP : %20.10f  \n", DGR[count], VP[count]);
				count++;
			}

			// 3. Motor Modeling
			else if (key == 'm' || key == 'M')
			{
				FILE* File_Freq_Response[FREQ_POINT];
				char fileNames[FREQ_POINT][100];

				double init_freq = 3.5 ; // [Hz]
				double intv_freq = 0.4 ; // [Hz]
				double duration  = 2.0 ; // [sec]
				double amplitude = 1.3 ;  // [V]

				double Freq_List[FREQ_POINT];

				MakeFreqArray(Freq_List, init_freq, intv_freq);

				printf("Get Frequency Response Data \n");



				// FREQ_POINT Loop
				for (int idx = 0; idx < FREQ_POINT ; idx++)
				{
					// File Open
					sprintf(fileNames[idx], "../data/freq_response/Frequency_Response_%.2fHz.out", Freq_List[idx]);
					File_Freq_Response[idx] = fopen(fileNames[idx], "w+t");

					if (File_Freq_Response[idx] == NULL) {
						printf("파일 열기 실패: %s\n", fileNames[idx]);
						exit(1);
					}

					printf(">> Start Measure %.2f Hz \n", Freq_List[idx]);
					getchar();

					time_curr = GetWindowTime();

					// Duration Loop
					for (int i = 0; i < SAMPLING_FREQ * duration ; i++)
					{

						time_prev = time_curr;
						time = SAMPLING_TIME * i;

						Vcmd = Freq_Signal(Freq_List[idx], time, amplitude);
						Vs = fcmd(Vcmd);

						Write_Vc(Output_Vc, Output_Vw, Vs);
						Read_Vg(Input_V, &Vg);

						fprintf(File_Freq_Response[idx], "%20.10f %20.10f %20.10f %20.10f\n", time, Vcmd, Vs, Vg);

						if (i % 20 == 0) { // 0.1초 간격으로만 출력
							printf("t: %6.3f | Vcmd: %7.4f | Vs: %7.4f | Vin: %7.4f\n", time, Vcmd, Vs, Vg);
						}

						if (_kbhit())
						{
							check = _getch();

							if (check == 's' || check == 'S')
							{
								Stop(Output_Vc, Output_Vw);
								goto END_FREQ_RESPONSE ;
							}

						}

						// 샘플링 타임 조절
						while (1) {
							time_curr = GetWindowTime();
							if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
						}

					}

					fclose(File_Freq_Response[idx]);
					Stop(Output_Vc, Output_Vw);

					printf(">> %.2f Hz 측정 완료 \n", Freq_List[idx]);

				}

					END_FREQ_RESPONSE:
					printf("[Frequency Response End]\n");
					break;
			}

			// 3. Step Response for Motor
			else if (key == 'r' || key == 'R')
			{
				printf("Step Response for Motor Validation\n");
				double amplitude = 1.3 ;

				time_curr = GetWindowTime();

				for (int i = 0; i < N_STEP; i++) {

					time_prev = time_curr;
					time = SAMPLING_TIME * i;

					Vcmd = Step_Response_Motor(time,amplitude);
					Vs = fcmd(Vcmd);

					Write_Vc(Output_Vc, Output_Vw, Vs);
					Read_Vg(Input_V, &Vg);

					Out_Time[i] = time;
					Out_Vs[i] = Vs;
					Out_Vc[i] = Vcmd;
					Out_Vg[i] = Vg;

					printf("Vs : %f [V] \n",Vs);

					// 샘플링 타임 조절
					while (1) {
						time_curr = GetWindowTime();
						if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
					}
				}

				// Stop
				Stop(Output_Vc, Output_Vw);
				printf("[Step Response End]\n");


				// File
				FILE* File_Motor_Validation = fopen("../data/freq_response/Motor_Validation.out", "w+t");
				for (int i = 0; i < N_STEP; i++) {
					fprintf(File_Motor_Validation, "%20.10f %20.10f %20.10f %20.10f\n", Out_Time[i], Out_Vs[i],Out_Vc[i],Out_Vg[i]);
				}
				fclose(File_Motor_Validation);

				break;
			}

			// 4. Designation Controller
			else if (key == 'z' || key == 'Z') {

				printf("Step Response for Designation Controller \n");

				printf("Insert Target Degree [deg]: ");
				scanf("%lf", &Ai_scale);

				time_curr = GetWindowTime();

				for (int i = 0; i < SAMPLING_FREQ * 5; i++) {

					time_prev = time_curr;

					time = SAMPLING_TIME * i;

					/* Making Ai */
					if (time < 2.0) {
						Ai = 0.0;
					}
					else {
						Ai = Ai_scale;
					}

					Read_Sensor(Input_V, &Vg, &Vp,&V_DIR, &V_DOA);

					Wg = (apply_LPF(&LPF,Vg) - V_Offset) * Kg; // [deg/sec]

					Ah = Potentio(Vp);	// [deg]

					E1 = Ai - Ah; // [deg]
					Signal_D = Kp_D * E1; // [deg/s]
					// signal_deg = Kp * E1; // [deg/s]

					E2 = Signal_D - Kd_D * Wg ;  // [deg/s]

					Vcmd =  E2 ; // [V]

					Vs = fcmd(Vcmd);

					Write_Vc(Output_Vc, Output_Vw, Vs);

					if (Vs > 5 || Vs < 0) {
						printf("Out of Range!!\n");
						Vs = 2.5 ;
					}

					// 샘플링 타임 조절
					while (1) {
						time_curr = GetWindowTime();
						if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
					}

					Out_Time[count]  = time ;
					Out_Ai[count]    = Ai   ;
					Out_Vc[count]    = Vcmd ;
					Out_Ao[count]    = Ah   ;
					Out_Er_1[count]  = E1   ;
					Out_Er_2[count]  = E2   ;


					count++;
				}

				// Stop
				Stop(Output_Vc, Output_Vw);
				printf("[Designation End]\n");

				// File
				FILE* File_Designation = fopen("../data/designation/Designation.out", "w+t");
				for (int i = 0; i < N_STEP; i++) {
					fprintf(File_Designation, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f \n", Out_Time[i], Out_Ai[i],Out_Vc[i],Out_Ao[i],Out_Er_1[i],Out_Er_2[i]);
				}
				fclose(File_Designation);

			}

			// 4. Stabilization Controller
			else if (key == 'x' || key == 'X') {

				printf("Step Response for Stabilization Controller \n");

				printf("Insert Target Angular Velocity [deg/sec]: ");
				scanf("%lf", &Wi_scale);

				time_curr = GetWindowTime();

				for (int i = 0; i < SAMPLING_FREQ * 20; i++) {

					time_prev = time_curr;

					time = SAMPLING_TIME * i;

					/* Making Ai */
					if (time < 2.0) {
						Wi = 0.0;
					}
					else {
						Wi = Wi_scale;
					}

					Read_Vg(Input_V, &Vg);

					Wg = (apply_LPF(&LPF,Vg) - V_Offset) * Kg; // [deg/sec]

					Error  = Wi - Wg ;   // [deg/sec]

					signal_Kp = Error * Kp_S ;

					// Turstin Method
					Error_Cur = Error;
					Integral += (T / 2) * (Error_Cur + Error_Pre) ;
					Error_Pre = Error_Cur ;
					signal_Ki = Ki_S * Integral;

					// Command
					Vcmd = signal_Kp + signal_Ki;
					Vs = fcmd(Vcmd);

					Write_Vc(Output_Vc, Output_Vw, Vs);

					if (Vs > 5 || Vs < 0) {
						printf("Out of Range!!\n");
						Vs = 2.5 ;
					}

					// 샘플링 타임 조절
					while (1) {
						time_curr = GetWindowTime();
						if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
					}

					Out_Time[count]  = time ;
					Out_Vc[count]    = Vcmd ;
					Out_Vs[count]    = Vs ;
					Out_Wg[count]    = Wg ;
					Out_Ki[count]    = signal_Ki;
					Out_Kp[count]    = signal_Kp;
					Out_Er_1[count]    = Error;

					count++;
				}

				// Stop
				Stop(Output_Vc, Output_Vw);
				printf("[Stabilization End]\n");

				// File
				FILE* File_Stabilization = fopen("../data/stabilization/Stabilization.out", "w+t");
				for (int i = 0; i < N_STEP; i++) {
					fprintf(File_Stabilization, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f \n", Out_Time[i], Out_Vc[i],Out_Vs[i],Out_Wg[i],Out_Ki[i],Out_Kp[i],Out_Er_1[i]);
				}
				fclose(File_Stabilization);

				break;
			}

			// 5. Tracking Controller
			else if (key == 'c' || key == 'C') {

				printf("Start Tracking Controller \n");

				time_curr = GetWindowTime();

				for (int i = 0; i < SAMPLING_FREQ * 50; i++) {

					time_prev = time_curr;

					time = SAMPLING_TIME * i;

					Read_Sensor(Input_V, &Vg, &Vp,&V_DIR, &V_DOA);

					Deg = Potentio(DAO2VP(V_DIR,V_DOA));

					printf("Vp : %f [V] \n", DAO2VP(V_DIR,V_DOA));
					printf("Deg : %f [deg] \n",Deg);

					Wi = Deg * Kt;

					Wg = (apply_LPF(&LPF,Vg) - V_Offset) * Kg; // [deg/sec]

					Error  = Wi - Wg;   // [deg/sec]

					signal_Kp = Error * Kp_S ;

					// Turstin Method
					Error_Cur = Error;
					Integral += (T / 2) * (Error_Cur + Error_Pre) ;
					Error_Pre = Error_Cur ;
					signal_Ki = Ki_S * Integral;

					// Command
					Vcmd = signal_Kp + signal_Ki;
					Vs = fcmd(Vcmd);

					Write_Vc(Output_Vc, Output_Vw, Vs);

					if (Vs > 5 || Vs < 0) {
						printf("Out of Range!!\n");
						Vs = 2.5 ;
					}

					// 샘플링 타임 조절
					while (1) {
						time_curr = GetWindowTime();
						if (time_curr - time_prev >= (SAMPLING_TIME * 1000.0)) break;
					}

					Out_Time[count]  = time ;
					Out_Vc[count]    = Vcmd ;
					Out_Vs[count]    = Vs ;
					Out_Wg[count]    = Wg ;
					Out_Ki[count]    = signal_Ki;
					Out_Kp[count]    = signal_Kp;
					Out_Er_1[count]    = Error;
				}

				// Stop
				Stop(Output_Vc, Output_Vw);
				printf("[Tracking End]\n");

				// File
				FILE* File_Tracking = fopen("../data/tracking/Tracking.out", "w+t");
				for (int i = 0; i < N_STEP; i++) {
					fprintf(File_Tracking, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n", Out_Time[i], Out_Vc[i],Out_Vs[i],Out_Wg[i],Out_Ki[i],Out_Kp[i],Out_Er_1[i]);
				}
				fclose(File_Tracking);

				break;
			}

			// 5. DAO Modeling
			else if (key == 't' || key == 'T') {
				Read_Vp_DOA(Input_V, &Vp, &V_DOA);

				printf("Vp : %20.10f \n", Vp);
				printf("V_DOA : %20.10f \n", V_DOA);

				VP_DOA[count] = Vp;
				DOA[count] = V_DOA;

				printf("count : %d \n", count+1);
				count++;
			}



			// 6. Stop
			else if (key == 's' || key == 'S')
			{
				Stop(Output_Vc, Output_Vw);
				break;
			}
		}
	}

	DAQmxStopTask(Output_Vw);
	DAQmxStopTask(Output_Vc);
	DAQmxStopTask(Input_V);


	// ========== File =========== //



	// 1. Linearization


	// 2. Potentio Modeling

	// File_Potentio = fopen(strcat(OutFileName, "Potentiometer_Model.out"), "w+t");

	strcpy(FileName_Potentio, "../data/potentio/Potentiometer_Model.out");
	File_Potentio = fopen(FileName_Potentio, "w+t");

	if (File_Potentio == NULL)
	{
		printf("Fail to Open Potentio ! \n");
	}

	for (int idx = 0; idx < NUM_POTENTIO; idx++)
	{
		fprintf(File_Potentio, "%20.10f %20.10f \n", DGR[idx], VP[idx]);

	}

	fclose(File_Potentio);

	// 2. Potentio Modeling
	strcpy(FileName_DOA, "../data/DOA/DOA_Model.out");
	File_DOA = fopen(FileName_DOA, "w+t");

	if (File_DOA == NULL)
	{
		printf("Fail to Open Potentio ! \n");
	}

	for (int idx = 0; idx < NUM_DOA; idx++)
	{
		fprintf(File_DOA, "%20.10f %20.10f \n", DOA[idx], VP_DOA[idx]);

	}

	fclose(File_DOA);

}
