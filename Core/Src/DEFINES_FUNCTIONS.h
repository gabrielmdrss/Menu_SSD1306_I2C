#ifndef DEFINES_FUNCTIONS_H
#define DEFINES_FUNCTIONS_H

#include "png.h"

//Registers Address (MPU6500)
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

//Registers Address (BMP280)
#define BMP280_ADDR 0xEC
#define Po			1013.25f
#define CALIB_REGS	0x88
#define	RESET		0xE0
#define CTRL_MEAS	0xF4
#define CONFIG		0xF5
#define DATA_REGS	0xF7

#define UP_BUTTON GPIO_PIN_9
#define DOWN_BUTTON GPIO_PIN_11
#define ENTER_BUTTON GPIO_PIN_10

//Variables for MPU6500 Configuration
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
float Ax, Ay, Az;
float gx, gy, gz;

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

/* Functions */
void MPU6050_Init(void) {
	uint8_t check;
	uint8_t Data;
// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
	if (check == 0x68) // 0x68 will be returned by the sensor if OK
			{
// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &Data, 1, 1000);
// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &Data, 1, 1000);
// Set Gyroscopic configuration in GYRO_CONFIG Register
		Data = 0x00; // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 ̐/s
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &Data, 1, 1000);
// Set accelerometer configuration in ACCEL_CONFIG Register
		Data = 0x00; // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &Data, 1, 1000);
	}
}

void MPU6500_Read_Values(void) {
}

void BMP280_Init(void) {

	uint8_t Data;
	HAL_Delay(5);				//aguarda o start-up time (mínimo de 2ms)

	//Reseta o sensor e aguarda o start-up time novamente (mínimo de 2ms)
	Data = 0xB6;
	HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, RESET, 1, &Data, 1, 1000);
	HAL_Delay(5);				//aguarda 5 ms

	//Coletando os parâmetros de calibração
	uint8_t param[24];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, CALIB_REGS, 1, param, 24, 1000);

	//Extraindo os dados de calibração da temperatura
	BMP280_dig_T1 = (param[0] << 8) | param[1];
	BMP280_dig_T2 = (param[2] << 8) | param[3];
	BMP280_dig_T3 = (param[4] << 8) | param[5];

	//Extraindo os dados de calibração da pressão
	BMP280_dig_P1 = (param[6] << 8) | param[7];
	BMP280_dig_P2 = (param[8] << 8) | param[9];
	BMP280_dig_P3 = (param[10] << 8) | param[11];
	BMP280_dig_P4 = (param[12] << 8) | param[13];
	BMP280_dig_P5 = (param[14] << 8) | param[15];
	BMP280_dig_P6 = (param[16] << 8) | param[17];
	BMP280_dig_P7 = (param[18] << 8) | param[19];
	BMP280_dig_P8 = (param[20] << 8) | param[21];
	BMP280_dig_P9 = (param[22] << 8) | param[23];

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
	int32_t adc_P = ((RawData[0] << 16) | (RawData[1] << 8) | RawData[2]) >> 4;	//pressão raw
	int32_t adc_T = ((RawData[3] << 16) | (RawData[4] << 8) | RawData[6]) >> 4;	//temperatura raw

	//calculando a temperatura
	float var1, var2, T;
	var1 = (((float) adc_T) / 16384 - ((float) BMP280_dig_T1) / 1024)
			* ((float) BMP280_dig_T2);
	var2 = ((((float) adc_T) / 131072 - ((float) BMP280_dig_T1) / 8192)
			* (((float) adc_T) / 131072 - ((float) BMP280_dig_T1) / 8192))
			* ((float) BMP280_dig_T3);
	t_fine = (var1 + var2);
	T = (var1 + var2) / 5120;
	*t = T;		//retorna T em °C (retorno em float)

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
		*p = P / 100;	//retorna P em hPa (retorno em float)
	}
}

void animation(void) {
	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse1, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse2, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse3, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse4, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse5, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse6, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse7, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse8, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse9, 128, 64, 0);
	ssd1306_UpdateScreen();

	ssd1306_Fill(1);
	ssd1306_DrawBitmap(0, 0, horse10, 128, 64, 0);
	ssd1306_UpdateScreen();
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

void read_accel(void) {
	uint8_t Rec_Data[6];
	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);
	Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
	/*** convert the RAW values into acceleration in 'g'
	 we have to divide according to the Full scale value set in FS_SEL
	 I have configured FS_SEL = 0. So I am dividing by 16384.0
	 for more details check ACCEL_CONFIG Register ****/
	Ax = (float) Accel_X_RAW / 16384.0;
	Ay = (float) Accel_Y_RAW / 16384.0;
	Az = (float) Accel_Z_RAW / 16384.0;

	char buffer_float[7];
	ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
	ssd1306_SetCursor(5, 16); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Accel x: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f", Ax);
	ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
	ssd1306_SetCursor(5, 30); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Accel y: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f", Ay);
	ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
	ssd1306_SetCursor(5, 44); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Accel z: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_float, "%.1f", Az);
	ssd1306_WriteString(buffer_float, Font_6x8, 1); //Escreve o texto no buffer
}

void read_gyro(void){
	uint8_t Rec_Data[6];
	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 12, 1000);
	Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
	Gyro_X_RAW = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
	Gyro_Y_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
	Gyro_Z_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
	/*** convert the RAW values into acceleration in 'g'
	 we have to divide according to the Full scale value set in FS_SEL
	 I have configured FS_SEL = 0. So I am dividing by 16384.0
	 for more details check ACCEL_CONFIG Register ****/
	gx = (float) Gyro_X_RAW / 131.0;
	gy = (float) Gyro_Y_RAW / 131.0;
	gz = (float) Gyro_Z_RAW / 131.0;
	char buffer_floats[7];
	ssd1306_Fill(0); //Seta todos os pixels do buffer para branco
	ssd1306_SetCursor(5, 16); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Gyro x: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_floats, "%.1f", gx);
	ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no buffer
	ssd1306_SetCursor(5, 30); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Gyro y: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_floats, "%.1f", gy);
	ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no buffer
	ssd1306_SetCursor(5, 44); //Posiciona o "cursor" no pixel correspondente
	ssd1306_WriteString("Gyro z: ", Font_6x8, 1); //Escreve o texto no buffer
	sprintf(buffer_floats, "%.1f", gz);
	ssd1306_WriteString(buffer_floats, Font_6x8, 1); //Escreve o texto no bufferr
}

#endif /*DEFINES_FUNCTIONS_H*/
