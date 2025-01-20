#include "lcd.h"
#include "pic.h"
#include "display.h"
#include "dm4310_ctrl.h"
#include "key_modlue.h"
#include "dm4310_drv.h"

/**
************************************************************************
* @brief:      	display: ��ʾ���溯��
* @param:      	void
* @retval:     	void
* @details:    	����cmd_selection()��motor_id_display()��������ʾ�û����档
*               ����ʾ�����ϻ��ƴ�ֱֱ�ߣ��ָ������Ҳ�����
************************************************************************
**/
void display(void)
{
	cmd_selection();
	motor_id_display();
	
	LCD_DrawLine(10, 104, 10, 240,WHITE);
	LCD_DrawLine(270, 104, 270, 240,WHITE);
}
/**
************************************************************************
* @brief:      	motor_start_display: ��ʾ�������״̬��Ϣ����
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬�����������״̬��Ϣ
* @retval:     	void
* @details:    	���ݴ���ĵ���ṹָ�룬��ʾ���������״̬��Ϣ��
*               ��������������־Ϊ0������ʾ"[READY]"��������ʾ"[START]"��
************************************************************************
**/
void motor_start_display(motor_t *motor)
{
	if(motor->para.state == 0)
	{
		if (motor->start_flag == 0)
			LCD_ShowString(20, 10,(uint8_t *)"[READY]", GREEN, BLACK, 32, 0);
		else 
			LCD_ShowString(20, 10,(uint8_t *)"[START]", GREEN, BLACK, 32, 0);
	}
	else
		LCD_ShowString(20, 10,(uint8_t *)"[ERROR]", RED, BLACK, 32, 0);
}
/**
************************************************************************
* @brief:      	motor_id_display: ��ʾ���ID�����Ϣ����
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID����ʾ��Ӧ�����������Ϣ�����ID�Ͳ�����Ϣ��
*               ֧�ֶ���������ʾ��ͨ��ѡ��ͬ�ĵ��ID�����л���
************************************************************************
**/
void motor_id_display(void)
{
	switch(motor_id)
	{
		case 1:
			motor_start_display(&motor[Motor1]);
			LCD_ShowString(200, 10,(uint8_t *)"MOTOR1", WHITE, BLACK, 24, 0);
			motor_para_display(&motor[Motor1]);
			break;
		case 2:
			motor_start_display(&motor[Motor2]);
			LCD_ShowString(200, 10,(uint8_t *)"MOTOR2", WHITE, BLACK, 24, 0);
			motor_para_display(&motor[Motor2]);
			break;
		case 3:
			motor_start_display(&motor[Motor3]);
			LCD_ShowString(200, 10,(uint8_t *)"MOTOR3", WHITE, BLACK, 24, 0);
			motor_para_display(&motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	motor_para_display: ��ʾ��������Ϳ���ģʽ��Ϣ����
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ͳ���
* @retval:     	void
* @details:    	���ݴ���ĵ���ṹָ�룬��ʾ��������Ϳ���ģʽ��Ϣ��
*               �������������ת�ء��������Լ�����ģʽ��Ϣ��λ��ģʽ���ٶ�ģʽ�ȡ�
************************************************************************
**/
void motor_para_display(motor_t *motor)
{
	LCD_ShowString(150, 40,(uint8_t *)"Tmos:", WHITE, BLACK, 24, 0);
	LCD_ShowFloatNum1(200, 40, motor->para.Tmos, 2, 1, GREEN, BLACK, 24);
	LCD_ShowString(150, 70,(uint8_t *)"Tdri:", WHITE, BLACK, 24, 0);
	LCD_ShowFloatNum1(200, 70, motor->para.Tcoil, 2, 1, GREEN, BLACK, 24);
	
	LCD_ShowFloatNum(92, 104, motor->cmd.pos_set, 2, 1, WHITE, BLACK, 24);
	LCD_ShowString(164, 104,(uint8_t *)"->", WHITE, BLACK, 24, 0);
	LCD_ShowFloatNum(200, 104, motor->para.pos, 2, 1, WHITE, BLACK, 24);
	
	
	LCD_ShowFloatNum(92, 128, motor->cmd.vel_set, 2, 1, WHITE, BLACK, 24);
	LCD_ShowString(164, 128,(uint8_t *)"->", WHITE, BLACK, 24, 0);
	LCD_ShowFloatNum(200, 128, motor->para.vel, 2, 1, WHITE, BLACK, 24);
	
	
	LCD_ShowFloatNum(92, 152, motor->cmd.tor_set, 2, 1, WHITE, BLACK, 24);
	LCD_ShowString(164, 152,(uint8_t *)"->", WHITE, BLACK, 24, 0);
	LCD_ShowFloatNum(200, 152, motor->para.tor, 2, 1, WHITE, BLACK, 24);
	
	LCD_ShowFloatNum(92, 176, motor->cmd.kp_set, 2, 1, WHITE, BLACK, 24);
	LCD_ShowFloatNum(92, 200, motor->cmd.kd_set, 2, 1, WHITE, BLACK, 24);
	
	switch(motor->ctrl.mode)
	{
		case 0:
			LCD_ShowString(70, 55,(uint8_t *)"[MIT.]", WHITE, BLACK, 24, 0);
			break;
		case 1:
			LCD_ShowString(70, 55,(uint8_t *)"[Pos.]", WHITE, BLACK, 24, 0);
			break;
		case 2:
			LCD_ShowString(70, 55,(uint8_t *)"[Vel.]", WHITE, BLACK, 24, 0);
			break;
	}
}
/**
************************************************************************
* @brief:      	cmd_selection: ��ʾLCD�ϵ�����ѡ����溯��
* @param:      	void
* @retval:     	void
* @details:    	����lcd_flag��enter_flag��ֵ����ʾLCD�ϵ�����ѡ����档
*               ����lcd_flag�Ĳ�ֵͬ��������ʾ��Ӧ������ѡ��ṩ�û�������
************************************************************************
**/
void cmd_selection(void)
{
		switch(lcd_flag)
	{
		case 0: 
			if(enter_flag == 0)
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, LIGHTBLUE, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			else
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLUE, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			break;
		case 1: 
			if(enter_flag == 0)
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, LIGHTBLUE, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			else
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLUE, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			break;
		case 2: 
			if(enter_flag == 0)
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, LIGHTBLUE, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			else
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLUE, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			break;
		case 3: 
			if(enter_flag == 0)
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, LIGHTBLUE, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			else
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLUE, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			break;
		case 4: 
			if(enter_flag == 0)
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, LIGHTBLUE, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			else
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLUE, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			break;
		case 5: 
			if(enter_flag == 0)
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, LIGHTBLUE, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			else
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLUE, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLACK, 24, 0);
			}
			break;
		case 6: 
			if(enter_flag == 0)
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, LIGHTBLUE, 24, 0);
			}
			else
			{
				LCD_ShowString(150, 10,(uint8_t *)"ID:", WHITE, BLACK, 24, 0);
				LCD_ShowString(10, 55,(uint8_t *)"Mode:", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 104,(uint8_t *)"Pos :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 128,(uint8_t *)"Vel :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 152,(uint8_t *)"Tor :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 176,(uint8_t *)"KP  :", WHITE, BLACK, 24, 0);
				LCD_ShowString(20, 200,(uint8_t *)"KD  :", WHITE, BLUE, 24, 0);
			}
			break;
	}
}
