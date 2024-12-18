#ifndef DEFINES_FUNCTIONS_H
#define DEFINES_FUNCTIONS_H

#include <math.h>
#include "main.h"
extern I2C_HandleTypeDef hi2c1;

//Registers Address (MPU6500)
#define MPU6050_ADDR 0xD0
#define TEMP_OUT_H_REG 0x41
#define WHO_AM_I_REG 0x75
#define	XG_OFFSET_H			(uint8_t)0x13	//cancelamento de offsets do girosc�pio
#define XG_OFFSET_L			(uint8_t)0x14	//
#define YG_OFFSET_H			(uint8_t)0x15	//
#define	YG_OFFSET_L			(uint8_t)0x16	//
#define	ZG_OFFSET_H			(uint8_t)0x17	//
#define ZG_OFFSET_L			(uint8_t)0x18	//
#define	SMPLRT_DIV			(uint8_t)0x19	//sample rate divider
#define	CONFIG				(uint8_t)0x1A	//configura��es gerais
#define	GYRO_CONFIG			(uint8_t)0x1B	//configura��es do girosc�pio
#define	ACCEL_CONFIG		(uint8_t)0x1C	//configura��es do aceler�metro
#define	ACCEL_CONFIG2		(uint8_t)0x1D	//configura��es do aceler�metro
#define	WOM_THR				(uint8_t)0x1F	//configura��es de threshold do WakeOnMotion
#define INT_PIN_CFG			(uint8_t)0x37	//configura��es do pino de interrup��o
#define	INT_ENABLE			(uint8_t)0x38	//habilita��o de interrup��es
#define	INT_STATUS			(uint8_t)0x3A	//status das interrup��es
#define DATA_ARRAY_POINTER	(uint8_t)0x3B	//ponteiro para o in�cio dos registradores de dados do sensor
#define	SIGNAL_PATH_RESET	(uint8_t)0x68	//reset do caminho de dados digitais
#define	ACCEL_INTEL_CTRL	(uint8_t)0x69	//controle de inteligencia do aceler�metro
#define USER_CTRL			(uint8_t)0x6A	//controles do usu�rio
#define	PWR_MGMT_1			(uint8_t)0x6B	//gerenciamento de energia
#define	PWR_MGMT_2			(uint8_t)0x6C	//gerenciamento de energia
#define	WHO_AM_I			(uint8_t)0x75	//ID do MPU-6500 (ID = 0x70)
#define XA_OFFSET_H			(uint8_t)0x77	//cancelamento de offsets do aceler�metro
#define	XA_OFFSET_L			(uint8_t)0x78	//
#define	YA_OFFSET_H			(uint8_t)0x7A	//
#define	YA_OFFSET_L			(uint8_t)0x7B	//
#define	ZA_OFFSET_H			(uint8_t)0x7D	//
#define	ZA_OFFSET_L			(uint8_t)0x7E	//
#define	accelScalingFactor	(float)2/32768		//fator de escala do aceler�metro �2g
#define	gyroScalingFactor	(float)250/32768	//fator de escala do girosc�pio �250 �/s

//Defini��es dos bits dos registradores
#define	DEVICE_RESET		(uint8_t)(1 << 7)	//bit 7 do registrador PWR_MGMT_1
#define	I2C_IF_DIS			(uint8_t)(1 << 4)	//bit 4 do registrador USER_CTRL
#define	SIG_COND_RST		(uint8_t)(1 << 0)	//bit 0 do registrador USER_CTRL
#define	RAW_RDY_EN			(uint8_t)(1 << 0)	//bit 0 do registrador INT_ENABLE
#define	WOM_EN				(uint8_t)(1 << 6)	//bit 6 do registrador INT_ENABLE
#define	ACCEL_INTEL_EN		(uint8_t)(1 << 7)	//bit 7 do registrador ACCEL_INTEL_CTRL
#define	ACCEL_INTEL_MODE	(uint8_t)(1 << 6)	//bit 6 do registrador ACCEL_INTEL_CTRL

//Defines for de Ultrasonic Sensor
#define TRIG_PIN GPIO_PIN_6
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_7
#define ECHO_PORT GPIOA

//Registers Address (BMP280)
#define BMP280_ADDR 	0xEC
#define Po				1013.25f
#define CALIB_REGS		0x88
#define	RESET			0xE0
#define CTRL_MEAS		0xF4
#define CONFIG_BMP280	0xF5
#define DATA_REGS		0xF7

//Defines for tbe buttons
#define UP_BUTTON GPIO_PIN_9
#define ENTER_BUTTON GPIO_PIN_10
#define DOWN_BUTTON GPIO_PIN_11

//Variables for MPU6500 Configuration
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
int16_t Temp_RAW = 0;
float Ax, Ay, Az;
float gx, gy, gz;
float temp;

//Variables for BMP280 Configuration
uint16_t BMP280_dig_T1;
int16_t BMP280_dig_T2;
int16_t BMP280_dig_T3;
uint16_t BMP280_dig_P1;
int16_t BMP280_dig_P2;
int16_t BMP280_dig_P3;
int16_t BMP280_dig_P4;
int16_t BMP280_dig_P5;
int16_t BMP280_dig_P6;
int16_t BMP280_dig_P7;
int16_t BMP280_dig_P8;
int16_t BMP280_dig_P9;
int32_t t_fine;
float t, p;

//Variables for Kalman filter
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

//Variable for Tim3 interrupt
int Data_Ready = 0;

//Variables for Ultrasonic Sensor
int adc_value = 0;
int Limit_Ultrasonic = 0;
int intermediary_screen = 1;
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;  // cm
uint8_t x_distance, x_limit;

//Low pass filter
float alpha = 0.3; // Taxa de suavização (0.0 - 1.0, quanto menor, menos a amostra atual influencia no valor filtrado)
uint16_t filtered_value = 0;
uint16_t ultrasonic_filtered = 0;

/* Functions */

uint16_t low_pass_filter(uint16_t new_value) {
	filtered_value = (alpha * new_value) + ((1 - alpha) * filtered_value);
	return filtered_value;
}

void MPU6050_Init(void) {

	uint8_t check;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000); // check device ID WHO_AM_I

	if (check == 0x70) {  // 0x68 will be returned by the sensor if OK

		HAL_Delay(100);
		uint8_t Data;
		Data = DEVICE_RESET;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &check, 1, 1000); //reset do sensor

		Data = 0b00000111;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SIGNAL_PATH_RESET, 1, &Data, 1,
				1000);	//reset dos filtros digitais
		HAL_Delay(100);

		Data = SIG_COND_RST;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, &Data, 1, 1000);//reseta os registradores de dados do sensor
		HAL_Delay(100);

		Data = 0b001;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, 1000);	//seleciona a melhor fonte de clock dispon�vel (PLL do girosc�pio)
		HAL_Delay(100);

		Data = 0b100;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG, 1, &Data, 1, 1000);	//par�metros de amostragem e filtragem (Fs=1kHz, gyro_BW=20Hz, temp_BW=20Hz)

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, 1000);//escala de leitura do girosc�pio (�250�/s)

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1,
				1000);		//escala da de leitura do aceler�metro (�2g)

		Data = 0b100;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG2, 1, &Data, 1,
				1000);		//par�metros do LPF (Fs=1kHz, accel_BW=20Hz)

		Data = 9;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, 1000);	//taxa de amostragem de sa�da [1kHz/(1+9) = 100sps]
	}
}

void menu(void) {
	ssd1306_Fill(0);
	ssd1306_SetCursor(25, 5);
	ssd1306_WriteString(menu_items[item_sel_previous], Font_7x10, 1);
	ssd1306_DrawBitmap(4, 2, bitmap_icons[item_sel_previous], 16, 16, 1);
	ssd1306_SetCursor(25, 5 + 20 + 2);
	ssd1306_WriteString(menu_items[item_selected], Font_7x10, 1);
	ssd1306_DrawBitmap(4, 24, bitmap_icons[item_selected], 16, 16, 1);
	ssd1306_SetCursor(25, 5 + 20 + 20 + 2 + 2);
	ssd1306_WriteString(menu_items[item_sel_next], Font_7x10, 1);
	ssd1306_DrawBitmap(4, 46, bitmap_icons[item_sel_next], 16, 16, 1);

	ssd1306_DrawBitmap(0, 22, bitmap_item_sel_outline, 128, 21, 1);
	ssd1306_DrawBitmap(128 - 8, 0, bitmap_scrollbar_background, 8, 64, 1);
	ssd1306_DrawRectangle(125, 64 / NUM_ITEMS * item_selected, 128,
			(64 / NUM_ITEMS * item_selected + (64 / NUM_ITEMS)), 1);
	ssd1306_DrawRectangle(126, 64 / NUM_ITEMS * item_selected, 127,
			(64 / NUM_ITEMS * item_selected + (64 / NUM_ITEMS)), 1);
}

void print_accel(void) {
	uint8_t check;
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
	if (check == 0x70) {

		char buffer_float[7];
		ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
		ssd1306_SetCursor(13, 1);
		ssd1306_WriteString("ACCELEROMETER: ", Font_7x10, 1);
		ssd1306_FillRectangle(1, 15, 128, 16, 1);
		ssd1306_DrawRectangle(1, 20, 127, 63, 1);

		ssd1306_SetCursor(7, 25); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("ACCEL.X: ", Font_6x8, 1); //Escreve o texto no buffer
		sprintf(buffer_float, "%.1f G", Ax);
		ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
		ssd1306_SetCursor(7, 39); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("ACCEL.Y: ", Font_6x8, 1); //Escreve o texto no buffer
		sprintf(buffer_float, "%.1f G", Ay);
		ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
		ssd1306_SetCursor(7, 53); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("ACCEL.Z: ", Font_6x8, 1); //Escreve o texto no buffer
		sprintf(buffer_float, "%.1f G", Az);
		ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer

	} else {
		ssd1306_SetCursor(5, 30); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("INERCIAL OFF", Font_6x8, 1); //Escreve o texto no buffer
	}
}

void print_gyro(void) {
	uint8_t check;
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
	if (check == 0x70) {

		char buffer_floats[7];
		ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
		ssd1306_SetCursor(35, 1);
		ssd1306_WriteString("GYROSCOPE", Font_7x10, 1);
		ssd1306_FillRectangle(1, 15, 128, 16, 1);
		ssd1306_DrawRectangle(1, 20, 127, 63, 1);

		ssd1306_SetCursor(7, 25); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("GYRO X: ", Font_6x8, 1); //Escreve o texto no buffer
		sprintf(buffer_floats, "%.0f", RateRoll);
		ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no buffer
		ssd1306_SetCursor(7, 39); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("GYRO Y: ", Font_6x8, 1); //Escreve o texto no buffer
		sprintf(buffer_floats, "%.0f", RatePitch);
		ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no buffer
		ssd1306_SetCursor(7, 53); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("GYRO Z: ", Font_6x8, 1); //Escreve o texto no buffer
		sprintf(buffer_floats, "%.0f", RateYaw);
		ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no bufferr

	} else {
		ssd1306_SetCursor(5, 30); //Posiciona o "cursor" no pixel correspondente
		ssd1306_WriteString("INERCIAL OFF", Font_6x8, 1); //Escreve o texto no buffer
	}
}

void calibration(void) {

	const uint16_t MAX_SAMPLE = 1000; //quantidade de amostras usadas no c�lculo da m�dia dos valores de offset
	uint8_t check1; //Variáveis auxiliar para envio dos dados dos registradores
	uint8_t check2; //
	uint8_t rawData[14];			//array para receber os dados do sensor
	int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;//valores crus do aceler�metro
	int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;	//valores crus do girosc�pio
	int32_t ACC_RAW_ACCEL_X, ACC_RAW_ACCEL_Y, ACC_RAW_ACCEL_Z;//valores acumulados do aceler�metro
	int32_t ACC_RAW_GYRO_X, ACC_RAW_GYRO_Y, ACC_RAW_GYRO_Z;	//valores acumulados do girosc�pio
	char buffer_float[5]; //buffer para impressão de variáveis na tela do display

	//acumuladores dos valores crus do sensor
	ACC_RAW_ACCEL_X = 0;
	ACC_RAW_ACCEL_Y = 0;
	ACC_RAW_ACCEL_Z = 0;
	ACC_RAW_GYRO_X = 0;
	ACC_RAW_GYRO_Y = 0;
	ACC_RAW_GYRO_Z = 0;

	ssd1306_Fill(0);

	uint16_t percentual;	//vari�vel para exibir o progresso da calibra��o
	for (uint16_t contador = 0; contador <= MAX_SAMPLE; ++contador) {
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, DATA_ARRAY_POINTER, 1, rawData,
				14, 1000);

		//Separando os dados lidos do sensor
		RAW_ACCEL_X = ((int16_t) rawData[0] << 8) | (rawData[1]);
		RAW_ACCEL_Y = ((int16_t) rawData[2] << 8) | (rawData[3]);
		RAW_ACCEL_Z = ((int16_t) rawData[4] << 8) | (rawData[5]);
		RAW_GYRO_X = ((int16_t) rawData[8] << 8) | (rawData[9]);
		RAW_GYRO_Y = ((int16_t) rawData[10] << 8) | (rawData[11]);
		RAW_GYRO_Z = ((int16_t) rawData[12] << 8) | (rawData[13]);

		//Acumulando os dados
		ACC_RAW_ACCEL_X += RAW_ACCEL_X;
		ACC_RAW_ACCEL_Y += RAW_ACCEL_Y;
		ACC_RAW_ACCEL_Z += RAW_ACCEL_Z;
		ACC_RAW_GYRO_X += RAW_GYRO_X;
		ACC_RAW_GYRO_Y += RAW_GYRO_Y;
		ACC_RAW_GYRO_Z += RAW_GYRO_Z;

		//Imprimindo o percentual da calibração
		if (contador % 10 == 0) {  //Imprimirá a cada 1%
			percentual = (contador * 100) / MAX_SAMPLE;

			ssd1306_Fill(0);
			ssd1306_SetCursor(27, 1);
			ssd1306_WriteString("CALIBRATION: ", Font_7x10, 1);
			ssd1306_FillRectangle(1, 15, 128, 17, 1);

			ssd1306_SetCursor(57, 28);
			snprintf(buffer_float, sizeof(buffer_float), "%d\n", percentual);
			ssd1306_WriteString(buffer_float, Font_7x10, 1);
			ssd1306_WriteString("%", Font_7x10, 1);
			ssd1306_DrawRectangle(11, 40, 117, 55, 1);
			ssd1306_FillRectangle(11, 40,
					(11 + (percentual * (117 - 11)) / 100), 55, 1);
			ssd1306_UpdateScreen();
		}

		HAL_Delay(15);
	}

	//M�dia dos valores lidos dos registradores do sensor
	RAW_ACCEL_X = (ACC_RAW_ACCEL_X / MAX_SAMPLE);
	RAW_ACCEL_Y = (ACC_RAW_ACCEL_Y / MAX_SAMPLE);
	RAW_ACCEL_Z = (ACC_RAW_ACCEL_Z / MAX_SAMPLE);
	RAW_GYRO_X = (ACC_RAW_GYRO_X / MAX_SAMPLE);
	RAW_GYRO_Y = (ACC_RAW_GYRO_Y / MAX_SAMPLE);
	RAW_GYRO_Z = (ACC_RAW_GYRO_Z / MAX_SAMPLE);

	//Valores inteiros dos offsets mensurados
	RAW_ACCEL_X = round(-(RAW_ACCEL_X * accelScalingFactor ) * 1024.0);
	RAW_ACCEL_Y = round(-(RAW_ACCEL_Y * accelScalingFactor ) * 1024.0);
	RAW_ACCEL_Z = round(-(1 - (RAW_ACCEL_Z * accelScalingFactor )) * 1024.0);
	RAW_GYRO_X = round(-(RAW_GYRO_X * gyroScalingFactor ) * 32.768);
	RAW_GYRO_Y = round(-(RAW_GYRO_Y * gyroScalingFactor ) * 32.768);
	RAW_GYRO_Z = round(-(RAW_GYRO_Z * gyroScalingFactor ) * 32.768);

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, XA_OFFSET_H, 1, &check1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, XA_OFFSET_L, 1, &check2, 1, 1000);
	int16_t Cancel_XA_Offset = (((((uint16_t) check1 << 8) | check2) >> 1)
			+ RAW_ACCEL_X) << 1;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, YA_OFFSET_H, 1, &check1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, YA_OFFSET_L, 1, &check2, 1, 1000);
	int16_t Cancel_YA_Offset = (((((uint16_t) check1 << 8) | check2) >> 1)
			+ RAW_ACCEL_Y) << 1;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ZA_OFFSET_H, 1, &check1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ZA_OFFSET_L, 1, &check2, 1, 1000);
	int16_t Cancel_ZA_Offset = (((((uint16_t) check1 << 8) | check2) >> 1)
			+ RAW_ACCEL_Z) << 1;

	//Escrevendo os valores de cancelamento de offset do aceler�metro
	check1 = Cancel_XA_Offset >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XA_OFFSET_H, 1, &check1, 1, 1000);
	check1 = Cancel_XA_Offset & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XA_OFFSET_L, 1, &check1, 1, 1000);

	check1 = Cancel_YA_Offset >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YA_OFFSET_H, 1, &check1, 1, 1000);
	check1 = Cancel_YA_Offset & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YA_OFFSET_L, 1, &check1, 1, 1000);

	check1 = Cancel_ZA_Offset >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZA_OFFSET_H, 1, &check1, 1, 1000);
	check1 = Cancel_ZA_Offset & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZA_OFFSET_L, 1, &check1, 1, 1000);

	//Escrevendo os valores de cancelamento de offset do girosc�pio
	check1 = RAW_GYRO_X >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XG_OFFSET_H, 1, &check1, 1, 1000);
	check1 = RAW_GYRO_X & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, XG_OFFSET_L, 1, &check1, 1, 1000);

	check1 = RAW_GYRO_Y >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YG_OFFSET_H, 1, &check1, 1, 1000);
	check1 = RAW_GYRO_Y & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, YG_OFFSET_L, 1, &check1, 1, 1000);

	check1 = RAW_GYRO_Z >> 8;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZG_OFFSET_H, 1, &check1, 1, 1000);
	check1 = RAW_GYRO_Z & 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ZG_OFFSET_L, 1, &check1, 1, 1000);

	current_screen = !current_screen;
}

void kalman_1d(float *KalmanState, float *KalmanUncertainty, float *KalmanInput,
		float *KalmanMeasurement) {

	*KalmanState = *KalmanState + 0.004 * *KalmanInput;
	*KalmanUncertainty = *KalmanUncertainty + 0.004 * 0.004 * 4.0 * 4.0;

//  float KalmanGain = *KalmanUncertainty * 1.0/(1.0**KalmanUncertainty + 3.0 * 3.0);
	float KalmanGain = 0.1;

	*KalmanState = *KalmanState
			+ KalmanGain * (*KalmanMeasurement - *KalmanState);
	*KalmanUncertainty = (1.0 - KalmanGain) * *KalmanUncertainty;
	Kalman1DOutput[0] = *KalmanState;
	Kalman1DOutput[1] = *KalmanUncertainty;
}

void MPU6050_Read_Measures(void) {

	uint8_t check;
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
	if (check == 0x70) {
		uint8_t Rec_Data[14];
		// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, 1000);
		Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
		Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
		Temp_RAW = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
		Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
		Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
		Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);
		/*** convert the RAW values into acceleration in 'g'
		 we have to divide according to the Full scale value set in FS_SEL
		 I have configured FS_SEL = 0. So I am dividing by 16384.0
		 for more details check ACCEL_CONFIG Register ****/
		Ax = (float) Accel_X_RAW / 16384.0;
		Ay = (float) Accel_Y_RAW / 16384.0;
		Az = (float) Accel_Z_RAW / 16384.0;
		RateRoll = (float) Gyro_X_RAW / 131.0;
		RatePitch = (float) Gyro_Y_RAW / 131.0;
		RateYaw = (float) Gyro_Z_RAW / 131.0;
		temp = ((float) Temp_RAW) / 333.87 + 21.0;
		AngleRoll = atan(Ay / sqrt(Ax * Ax + Az * Az)) * 1 / (3.142 / 180.0);
		AnglePitch = -atan(Ax / sqrt(Ay * Ay + Az * Az)) * 1 / (3.142 / 180.0);
	}
}

void print_kalman(void) {
	kalman_1d(&KalmanAngleRoll, &KalmanUncertaintyAngleRoll, &RateRoll,
			&AngleRoll);
	KalmanAngleRoll = Kalman1DOutput[0];
	KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
	kalman_1d(&KalmanAnglePitch, &KalmanUncertaintyAnglePitch, &RatePitch,
			&AnglePitch);
	KalmanAnglePitch = Kalman1DOutput[0];
	KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

	char buffer_float[7];
	ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
	ssd1306_SetCursor(14, 1);
	ssd1306_WriteString("KALMAN FILTER: ", Font_7x10, 1);
	ssd1306_FillRectangle(1, 15, 128, 16, 1);
	ssd1306_DrawRectangle(1, 20, 127, 63, 1);

	ssd1306_SetCursor(7, 25); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("ROLL: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f", KalmanAngleRoll);
	ssd1306_WriteString(buffer_float, Font_7x10, 1); //Escreve o texto no buffer
	ssd1306_SetCursor(7, 39); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("PITCH: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f", KalmanAnglePitch);
	ssd1306_WriteString(buffer_float, Font_7x10, 1); //Escreve o texto no buffer

}

void BMP280_Init(void) {

	uint8_t Data;
	HAL_Delay(5);                //aguarda o start-up time (mínimo de 2ms)

	//Reseta o sensor e aguarda o start-up time novamente (mínimo de 2ms)
	Data = 0xB6;
	HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, RESET, 1, &Data, 1, 1000);
	HAL_Delay(5);                //aguarda 5 ms

	//Coletando os parâmetros de calibração
	uint8_t param[24];
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, CALIB_REGS, 1, param, 24, 1000);

	//Extraindo os dados de calibração da temperatura
	BMP280_dig_T1 = (param[1] << 8) | param[0];
	BMP280_dig_T2 = (param[3] << 8) | param[2];
	BMP280_dig_T3 = (param[5] << 8) | param[4];

	//Extraindo os dados de calibração da pressão
	BMP280_dig_P1 = (param[7] << 8) | param[6];
	BMP280_dig_P2 = (param[9] << 8) | param[8];
	BMP280_dig_P3 = (param[11] << 8) | param[10];
	BMP280_dig_P4 = (param[13] << 8) | param[12];
	BMP280_dig_P5 = (param[15] << 8) | param[14];
	BMP280_dig_P6 = (param[17] << 8) | param[16];
	BMP280_dig_P7 = (param[19] << 8) | param[18];
	BMP280_dig_P8 = (param[21] << 8) | param[20];
	BMP280_dig_P9 = (param[23] << 8) | param[22];

	Data = 0x1C;
	HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, CONFIG, 1, &Data, 1, 1000);
	//IIR Filter(coeficient = 16)

	//Sensor output in sleep mode(Standard Mode activity)
	//20 bits de resolução, oversampling x16 na pressão, resolução de 0.16 Pa
	//20 bits de resolução, oversampling x16 na temperatura, resolução de 0.0003 °C

	Data = 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, CTRL_MEAS, 1, &Data, 1, 1000);
}

void BMP280_Read_Measures(float *t, float *p) {
	//Lendo os registradores com os valores crus das grandezas
	uint8_t RawData[6];
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, DATA_REGS, 1, RawData, 6, 1000);

	//Extraindo os dados crus da pressão e temperatura
	int32_t adc_P = ((RawData[0] << 16) | (RawData[1] << 8) | RawData[2]) >> 4; //pressão raw
	int32_t adc_T = ((RawData[3] << 16) | (RawData[4] << 8) | RawData[6]) >> 4; //temperatura raw

	//calculando a temperatura
	float var1, var2, T;
	var1 = (((float) adc_T) / 16384 - ((float) BMP280_dig_T1) / 1024)
			* ((float) BMP280_dig_T2);
	var2 = ((((float) adc_T) / 131072 - ((float) BMP280_dig_T1) / 8192)
			* (((float) adc_T) / 131072 - ((float) BMP280_dig_T1) / 8192))
			* ((float) BMP280_dig_T3);
	t_fine = (var1 + var2);
	T = (var1 + var2) / 5120;
	*t = T;        //retorna T em °C (retorno em float)

	//Measurement of Atmospheric Pressure
	float P;
	var1 = ((float) t_fine / 2) - 64000;
	var2 = var1 * var1 * ((float) BMP280_dig_P6) / 32768;
	var2 = var2 + var1 * ((float) BMP280_dig_P5) * 2;
	var2 = (var2 / 4) + (((float) BMP280_dig_P4) * 65536);
	var1 = (((float) BMP280_dig_P3) * var1 * var1 / 524288
			+ ((float) BMP280_dig_P2) * var1) / 524288;
	var1 = (1 + var1 / 32768) * ((float) BMP280_dig_P1);
	if (var1 == 0)
		P = 0; //evita exceção causada por divisão por zero
	else {
		P = 1048576 - (float) adc_P;
		P = (P - (var2 / 4096)) * 6250 / var1;
		var1 = ((float) BMP280_dig_P9) * P * P / 2147483648;
		var2 = P * ((float) BMP280_dig_P8) / 32768;
		P = P + (var1 + var2 + ((float) BMP280_dig_P7)) / 16;
		*p = P / 100;    //retorna P em hPa (retorno em float)
	}
}

void print_BMP280(void) {

	//Lendo os registradores com os valores crus das grandezas
	uint8_t RawData[6];
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, DATA_REGS, 1, RawData, 6, 1000);

	//Extraindo os dados crus da pressão e temperatura
	int32_t adc_P = ((RawData[0] << 16) | (RawData[1] << 8) | RawData[2]) >> 4; //pressão raw
	int32_t adc_T = ((RawData[3] << 16) | (RawData[4] << 8) | RawData[6]) >> 4; //temperatura raw

	//calculando a temperatura
	float var1, var2, T;
	var1 = (((float) adc_T) / 16384 - ((float) BMP280_dig_T1) / 1024)
			* ((float) BMP280_dig_T2);
	var2 = ((((float) adc_T) / 131072 - ((float) BMP280_dig_T1) / 8192)
			* (((float) adc_T) / 131072 - ((float) BMP280_dig_T1) / 8192))
			* ((float) BMP280_dig_T3);
	t_fine = (var1 + var2);
	T = (var1 + var2) / 5120;
	t = T;        //retorna T em °C (retorno em float)

	//Measurement of Atmospheric Pressure
	float P;
	var1 = ((float) t_fine / 2) - 64000;
	var2 = var1 * var1 * ((float) BMP280_dig_P6) / 32768;
	var2 = var2 + var1 * ((float) BMP280_dig_P5) * 2;
	var2 = (var2 / 4) + (((float) BMP280_dig_P4) * 65536);
	var1 = (((float) BMP280_dig_P3) * var1 * var1 / 524288
			+ ((float) BMP280_dig_P2) * var1) / 524288;
	var1 = (1 + var1 / 32768) * ((float) BMP280_dig_P1);
	if (var1 == 0)
		P = 0; //evita exceção causada por divisão por zero
	else {
		P = 1048576 - (float) adc_P;
		P = (P - (var2 / 4096)) * 6250 / var1;
		var1 = ((float) BMP280_dig_P9) * P * P / 2147483648;
		var2 = P * ((float) BMP280_dig_P8) / 32768;
		P = P + (var1 + var2 + ((float) BMP280_dig_P7)) / 16;
		p = P / 100;    //retorna P em hPa (retorno em float)
	}
	float altitude = 44330.0f * (1.0 - pow((p) / Po, (1.0f / 5.255f))); //equação barométrica

	char buffer_float[7];
	ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
	ssd1306_SetCursor(5, 16); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Pressao: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f hPa", p);
	ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
	ssd1306_SetCursor(5, 30); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Temp: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f°C", t);
	ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
	ssd1306_SetCursor(5, 44); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Altitude: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f m", altitude);
	ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer

}

#endif /*DEFINES_FUNCTIONS_H*/
