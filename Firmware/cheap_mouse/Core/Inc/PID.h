#include "main.h"
float Error, pre_Error, pre_pre_Error;
float P_part, I_part, D_part, Out, pre_out;
float Kp = 8;
float Ki = 10;
float Kd = 1;
float T = 0.01;


void Motor1(int pulse_width)
{
	if(pulse_width<0){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_SET);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999 + pulse_width);
	}
	else{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_width);
	}

}

void Motor2(int pulse_width)
{
	if(pulse_width<0){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999 + pulse_width);
	}
	else{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_width);
	}

}



void PID(float setpoint, float input, int velocity){
		Error = setpoint - input;
		P_part = Kp*(Error);
		I_part += Ki*T*Error;
		if(I_part > 300){
			I_part = 300;
		}
		D_part = Kd*(Error - pre_Error)/T;
		Out = P_part + I_part + D_part ;
		pre_Error = Error;
		pre_out = Out;

		Motor1( velocity - round_pid(Out));
		Motor2( velocity + round_pid(Out));


	 	return Out;
}
int round_pid(double x)
	{
	    if (x < 0.0)
	    	if(x > -500){
	        return (int)(x - 0.5);
	    	}
	    	else{
	    		return -500;
	    	}
	    else
	    	if(x < 500){
		        return (int)(x + 0.5);
	    	}
	    	else{
	    		return 500;
	    	}
	}

typedef struct{
	float Kp1;
	float Ki1;
	float Kd1;

	float Kp2;
	float Ki2;
	float Kd2;

	float Kp3;
	float Ki3;
	float Kd3;

	float Kp4;
	float Ki4;
	float Kd4;
    uint8_t crc;
}PID_value;

typedef struct{

	uint16_t data[12];
    uint8_t crc;
}PID_raw;

PID_raw convert(PID_value pid){
	PID_raw raw;
	raw.data[0] = pid.Kp1;
	raw.data[1] = pid.Ki1;
	raw.data[2] = pid.Kd1;

	raw.data[3] = pid.Kp2;
	raw.data[4] = pid.Ki2;
	raw.data[5] = pid.Kd2;

	raw.data[6] = pid.Kp3;
	raw.data[7] = pid.Ki3;
	raw.data[8] = pid.Kd3;

	raw.data[9] = pid.Kp4;
	raw.data[10] = pid.Ki4;
	raw.data[11] = pid.Kd4;

	return raw;
}
PID_raw pid_decode(uint8_t data[24]){
	PID_raw pid;

	pid.data[0] = (data[0] << 6) | ((data[1]&0xFC) >> 2);
	pid.data[1] = ((data[1] & 0x03) <<12 ) |(data[2] <<4)|((data[3]&0xF0 )>> 4);
	pid.data[2] = ((data[3]&0x0F )<< 10 | (data[4] << 2) |  (data[5] &0xC0)>>6);

	pid.data[3] = ((data[5] & 0x3F)<<8|data[6]);
	pid.data[4] = ((data[7] << 6)|((data[8]&0xFC) >> 2));
	pid.data[5] = ((data[8] & 0x03) <<12)| (data[9] <<4) |((data[10]&0xF0 )>> 4);

	pid.data[6] = ((data[10]&0x0F) << 10) | (data[11] << 2) |  ((data[12] &0xC0)>>6);
	pid.data[7] = ((data[12] & 0x3F)<<8)|(data[13]);
	pid.data[8] = ((data[14] << 6) | ((data[15]&0xFC) >> 2));

	pid.data[9] = ((data[15] & 0x03) << 12) | (data[16] << 4) | ((data[17]&0xF0) >> 4);
	pid.data[10] = ((data[17]&0x0F) << 10) | (data[18]<<2)|((data[19] &0xC0)>>6);
	pid.data[11] = ((data[19] & 0x3F)<<8) | data[20];
    pid.crc = data[21];
	return pid;
}
