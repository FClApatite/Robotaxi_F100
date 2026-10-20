#include "stm32f1xx.h"
#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "string.h"
uint8_t Serial_rx_buf[10];
class System_t{
	public:
	float steering_angle=0;
	float forward_velocity=0;
	float filtered_forward_velocity=0;
	float gain=225;
	float iir=0.8;
	float integration_zero_resistance=19;
	float ls_integration_zero_resistance=56;
	float integration_max=25;	
	float integration=0;
	float slew_rate=10;//10/pulse;
	float last_output=0;
	uint8_t state;
	uint8_t buzzer_mode=3;
	uint8_t last_buzzer_mode=3;
};
class System_define_t{
	public:
	uint16_t servo_mid=1060;
	float servo_deg_to_pulse=10.5f;
	uint16_t wheel_full_scale=1024;
	uint16_t wheel_encoder_pulses=1024;
	float cycles=1000;
	float wheel_perimeter=0.06504f;
	float wheel_max_current=10.0f;//limited by TIM1_BRK
};
System_t taxi;
System_define_t taxi_defs;
bool alive=0;
int alive_timer=0;
int buzzer_timer=0;
void Serial_Depack();
void Speed_PID();
void Buzzer_Handle();
void LED_Handle();
void TIM6_Task();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM6){
		TIM6_Task();
	}
}
void TIM6_Task(){
	uint16_t servo1_pwm=taxi_defs.servo_mid;
	servo1_pwm-=taxi_defs.servo_deg_to_pulse*taxi.steering_angle;
	htim3.Instance->CCR1=servo1_pwm;
	alive_timer++;
	if(alive==1){
		alive=0;
		alive_timer=0;
	}
	if(alive_timer>3000){
		taxi.forward_velocity=0;
		taxi.steering_angle=0;
		alive_timer=3000;
		GPIOD->BRR|=0x7F00;
		GPIOD->BSRR=0x3F00;//"0"
	}else{
			GPIOD->BRR|=0x7F00;
		GPIOD->BSRR=0x7700;//"A"
	}
	Buzzer_Handle();
	LED_Handle();
	Speed_PID();
}
void Speed_PID(){
	float now_forward_velocity;
	int16_t encoder_steps=htim2.Instance->CNT;
	htim2.Instance->CNT=0;
	now_forward_velocity=((float)encoder_steps/(float)taxi_defs.wheel_encoder_pulses)*taxi_defs.wheel_perimeter*taxi_defs.cycles;
	taxi.filtered_forward_velocity+=taxi.iir*now_forward_velocity;
	taxi.filtered_forward_velocity/=(1.0f+taxi.iir);
	float speed_error=taxi.forward_velocity-taxi.filtered_forward_velocity;
	float prop_output=speed_error*taxi.gain;
	float int_output=0;
	if(taxi.forward_velocity>0.55){
	taxi.integration-=taxi.integration/taxi.integration_zero_resistance;
	taxi.integration+=speed_error;
	if(taxi.integration>taxi.integration_max)taxi.integration=taxi.integration_max;
	if(taxi.integration<-taxi.integration_max)taxi.integration=-taxi.integration_max;
	int_output=((taxi.gain/100.0f)*taxi.integration_zero_resistance)*taxi.integration;
	}else{
		taxi.integration-=taxi.integration/taxi.ls_integration_zero_resistance;
	taxi.integration+=speed_error;
	if(taxi.integration>taxi.integration_max)taxi.integration=taxi.integration_max;
	if(taxi.integration<-taxi.integration_max)taxi.integration=-taxi.integration_max;
	int_output=((taxi.gain/250.0f)*taxi.ls_integration_zero_resistance)*taxi.integration;
	}
	float output=prop_output+int_output;
	if(output-taxi.last_output>taxi.slew_rate)output=taxi.last_output+taxi.slew_rate;
	if(output-taxi.last_output<-taxi.slew_rate)output=taxi.last_output-taxi.slew_rate;
	if(output>1023)output=1023;
	if(output<-1023)output=-1023;
	taxi.last_output=output;
	if(output>0){
		htim1.Instance->CCR2=1023;
		htim1.Instance->CCR1=1023-(uint16_t)output;
	}
	if(output<0){
		htim1.Instance->CCR1=1023;
		htim1.Instance->CCR2=1023+(int16_t)output;
	}
	if(output==0){
		htim1.Instance->CCR1=1023;
		htim1.Instance->CCR2=1023;
	}
}
void Buzzer_Handle(){
   buzzer_timer++;
	if(buzzer_timer>1211)buzzer_timer=0;
	switch(taxi.buzzer_mode){
		case 0:
		{ if(taxi.buzzer_mode!=taxi.last_buzzer_mode){
			buzzer_timer=0;
			taxi.last_buzzer_mode=taxi.buzzer_mode;
		}
			if(buzzer_timer%70==0){
			htim16.Instance->CCR1=htim16.Instance->ARR/2;
		  }
		if(buzzer_timer%70==5){
			htim16.Instance->CCR1=0;
		  }
		if(buzzer_timer>=300){taxi.buzzer_mode=taxi.last_buzzer_mode=0xff;buzzer_timer=300;}
			break;
		}
		case 1:
		{ 
			if(taxi.buzzer_mode!=taxi.last_buzzer_mode){
			buzzer_timer=0;
			taxi.last_buzzer_mode=taxi.buzzer_mode;
		}
			if(buzzer_timer%100==0){
			htim16.Instance->CCR1=htim16.Instance->ARR/2;
		  }
		if(buzzer_timer%100==5){
			htim16.Instance->CCR1=0;
		  }
		if(buzzer_timer>=1006){taxi.buzzer_mode=taxi.last_buzzer_mode=taxi.last_buzzer_mode=0xff;buzzer_timer=1006;}
			break;
		}
		case 2:
		{ 
			if(taxi.buzzer_mode!=taxi.last_buzzer_mode){
			buzzer_timer=0;
			taxi.last_buzzer_mode=taxi.buzzer_mode;
		}
			if(buzzer_timer%60==0){
			htim16.Instance->CCR1=htim16.Instance->ARR/2;
		  }
		if(buzzer_timer%60==5){
			htim16.Instance->CCR1=0;
		  }
		if(buzzer_timer>=367){taxi.buzzer_mode=taxi.last_buzzer_mode=0xff;buzzer_timer=367;}
			break;
		}
		case 3:
		{ 
			if(taxi.buzzer_mode!=taxi.last_buzzer_mode){
			buzzer_timer=0;
			taxi.last_buzzer_mode=taxi.buzzer_mode;
		}
			if(buzzer_timer%30==0){
			htim16.Instance->CCR1=htim16.Instance->ARR/2;
		  }
		if(buzzer_timer%30==5){
			htim16.Instance->CCR1=0;
		  }
		if(buzzer_timer>=67){taxi.buzzer_mode=taxi.last_buzzer_mode=0xff;buzzer_timer=67;}
			break;
		}
		case 4:
		{ 
			if(taxi.buzzer_mode!=taxi.last_buzzer_mode){
			buzzer_timer=0;
			taxi.last_buzzer_mode=taxi.buzzer_mode;
		}
			if(buzzer_timer%200==0){
			htim16.Instance->CCR1=htim16.Instance->ARR/2;
		  }
		if(buzzer_timer%200==5){
			htim16.Instance->CCR1=0;
		  }
		if(buzzer_timer>=1209){taxi.buzzer_mode=taxi.last_buzzer_mode=0xff;buzzer_timer=1209;}
			break;
		}
	}
}
uint8_t LED_timer=0;
void LED_Handle(){

	LED_timer++;
	if(LED_timer>200){
		LED_timer=0;
	}
		if(LED_timer%200==0){
		GPIOD->BRR|=GPIO_PIN_15;
		}else if(LED_timer%100==0){
			GPIOD->BSRR|=GPIO_PIN_15;
		}
	}
bool Serial_Verify(){
	uint16_t sum=0;
	sum+=Serial_rx_buf[0];
	sum+=Serial_rx_buf[1];
	sum+=Serial_rx_buf[2];
	for(int i=0;i<Serial_rx_buf[2]-4;i++){
		sum+=Serial_rx_buf[i+3];
	}
	uint8_t verify_sum=0;
	verify_sum=(sum&0x00FF);
	if(verify_sum==Serial_rx_buf[Serial_rx_buf[2]-1]){
		return true;
	}else{
		return false;
	}
}
void Serial_Clear(){
	for(int i=0;i<=9;i++){
		Serial_rx_buf[i]=0;
	}
}
void Serial_RxStart(void){
	(void)HAL_UART_Receive_IT(&huart1, Serial_rx_buf, sizeof(Serial_rx_buf));
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if (huart->Instance == USART1)
{
if (Serial_rx_buf[0] == 0x42) {
            Serial_Depack();
        } else {
          
            for (int i = 1; i < 10; i++) {
                if (Serial_rx_buf[i] == 0x42) {
                    // ?????,??????,??????
                    // ?????????
                    memmove(Serial_rx_buf, &Serial_rx_buf[i], 10 - i);
                    // ???????
                    (void)HAL_UART_Receive_IT(&huart1, &Serial_rx_buf[10 - i], i);
                    return;
                }
            }
        }
Serial_RxStart(); // ????????
}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
if (huart->Instance == USART1)
{
	uint32_t error=HAL_UART_GetError(huart);
	if(((error&HAL_UART_ERROR_ORE)!=0U)||huart->RxState==HAL_UART_STATE_READY){
		__HAL_UART_CLEAR_OREFLAG(huart);
		Serial_Clear();
		Serial_RxStart();
	}
}
}
void Serial_Depack(){
//	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
//__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//}else {
//	return;
//}
	//if(HAL_UART_Receive(&huart1, Serial_rx_buf, 10, 0) == HAL_OK){
		if(Serial_rx_buf[0]==0x42){//package header
			 alive=1;
			switch(Serial_rx_buf[1]){//address switch
				case 0x00://handshake. However, nothing has been defined in SASU's manual.
					Serial_Clear();
				 
					break;
				case 0x01://velocity and direction
				{
					if(Serial_rx_buf[2]==10&&Serial_Verify()==true){
					    memcpy(&taxi.forward_velocity,&Serial_rx_buf[3],4);
					    int16_t pwm_tempo=0;
					    memcpy(&pwm_tempo,&Serial_rx_buf[7],2);
					    pwm_tempo-=1500;
				      float angle=(float)pwm_tempo/10.0f;
						  taxi.steering_angle=angle;
					}
					Serial_Clear();
					break;
				}
				case 0x02://velocity control mode
					Serial_Clear();
					break;//just skip it. always use closed-loop to avoid destroying the board (:
				case 0x03://servo mid set
				{
					if(Serial_rx_buf[3]<3&&Serial_rx_buf[3]>1){
					if(Serial_rx_buf[3]==1){
						Serial_rx_buf[4]=(taxi_defs.servo_mid-500)&0XFF00;
						Serial_rx_buf[5]=(taxi_defs.servo_mid-500)&0X00FF;
					}
					if(Serial_rx_buf[3]==2){
						Serial_rx_buf[4]=(taxi_defs.servo_mid+500)&0XFF00;
						Serial_rx_buf[5]=(taxi_defs.servo_mid+500)&0X00FF;
					}
					if(Serial_rx_buf[3]==3){
						Serial_rx_buf[4]=(taxi_defs.servo_mid)&0XFF00;
						Serial_rx_buf[5]=(taxi_defs.servo_mid)&0X00FF;
					}
					HAL_UART_Transmit(&huart1,Serial_rx_buf,10,1);
				}
					Serial_Clear();
					break;
				}
				case 0x04://beeper mode
				{
					if(Serial_rx_buf[2]==5&&Serial_Verify()==true){
						taxi.buzzer_mode=Serial_rx_buf[3];
					}
					Serial_Clear();
					break;
				}
				case 0x05://lamp color set in SASU's manual:This is digit tube display setting on our board
				{
					if(Serial_rx_buf[2]==8&&Serial_Verify()==true){
						taxi.state=Serial_rx_buf[3];
					}
					Serial_Clear();
					break;
				}
				case 0x06://button
				{
					Serial_Clear();
					break;
				}
				case 0x07://battery
				{
					Serial_Clear();
					break;
			  }
								
			}
		}
//	}
}
			
