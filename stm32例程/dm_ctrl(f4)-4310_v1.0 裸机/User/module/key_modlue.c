#include "key_modlue.h"
#include "drivers.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"

int8_t lcd_flag;
uint8_t enter_flag;

/**
************************************************************************
* @brief:      	motor_start_keyfun: ������������ص�����
* @param:      	void
* @retval:     	void
* @details:    	����ctrl_enable������������ǰѡ�������
************************************************************************
**/
void motor_start_keyfun(void)
{
	ctrl_enable();
}
/**
************************************************************************
* @brief:      	motor_stop_keyfun: ���ֹͣ�����ص�����
* @param:      	void
* @retval:     	void
* @details:    	����ctrl_disable������ֹͣ��ǰѡ�������
************************************************************************
**/
void motor_stop_keyfun(void)
{
	ctrl_disable();
}
/**
************************************************************************
* @brief:      	motor_enter_set: �������ð����ص�����
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰenter_flag״̬���л���ֵ��
*               ��enter_flagΪ1��������Ϊ0����Ϊ0��������Ϊ1��
************************************************************************
**/
void motor_enter_set(void)
{
	if (enter_flag)
		enter_flag = 0;
	else
		enter_flag = 1;
}
/**
************************************************************************
* @brief:      	motor_set: ���õ�����������ص�����
* @param:      	void
* @retval:     	void
* @details:    	����ctrl_set���������õ�ǰѡ������Ĳ�����
************************************************************************
**/
void motor_set(void)
{
	ctrl_set();
} 
/**
************************************************************************
* @brief:      	motor_clear_para: ���������������ص�����
* @param:      	void
* @retval:     	void
* @details:    	����ctrl_clear�����������ǰѡ������Ĳ�����
************************************************************************
**/
void motor_clear_para(void)
{
	ctrl_clear_para();
} 
/**
************************************************************************
* @brief:      	motor_clear_err: ������������Ϣ�����ص�����
* @param:      	void
* @retval:     	void
* @details:    	����ctrl_clear�����������ǰѡ������Ĳ�����
************************************************************************
**/
void motor_clear_err(void)
{
	ctrl_clear_err();
}
/**
************************************************************************
* @brief:      	add_key: ���Ӽ�����
* @param:      	void
* @retval:     	void
* @details:    	���δȷ�ϣ�enter_flagΪ0��������Խ��в���ѡ��
*               ��ʱlcd_flag�ݼ�����lcd_flagС��0����������Ϊ6��
*               ��ȷ�ϣ�enter_flagΪ1���������ctrl_add���������ӵ�ǰѡ������Ĳ�����
************************************************************************
**/
void add_key(void)
{
	if(!enter_flag) // û��ȷ�ϲſ��Խ��в���ѡ��
	{
		lcd_flag--;
		if(lcd_flag < 0)
		{
			lcd_flag = 6;
		}
	}
	else
	{
		ctrl_add();
	}
}
/**
************************************************************************
* @brief:      	minus_key: ���ټ�����
* @param:      	void
* @retval:     	void
* @details:    	���δȷ�ϣ�enter_flagΪ0��������Խ��в���ѡ��
*               ��ʱlcd_flag��������lcd_flag����6����������Ϊ0��
*               ��ȷ�ϣ�enter_flagΪ1���������ctrl_minus���������ٵ�ǰѡ������Ĳ�����
************************************************************************
**/
void minus_key(void)
{
	if(!enter_flag) // û��ȷ�ϲſ��Խ��в���ѡ��
	{
		lcd_flag++;
		if(lcd_flag > 6)
		{
			lcd_flag = 0;
		}
	}
	else
	{
		ctrl_minus();
	}
}
