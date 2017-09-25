/*
*********************************************************************************************************
*
*	ģ������ : ������������ģ��
*	�ļ����� : bsp_key.c
*	��    �� : V1.0
*	˵    �� : ɨ�������������������˲����ƣ����а���FIFO�����Լ�������¼���
*				(1) ��������
*				(2) ��������
*				(3) ������
*				(4) ����ʱ�Զ�����
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*		V1.1    2013-06-29 armfly  ����1����ָ�룬����bsp_Idle() ������ȡϵͳ������ϼ���������
*								   ���� K1 K2 ��ϼ� �� K2 K3 ��ϼ�������ϵͳ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "../../bsp_stm32f4/inc/bsp_key.h"

#include "stm32f4xx.h"

/*
	�ó��������ڰ�����STM32-X3��STM32-V5������

	�����������Ӳ�������޸�GPIO����� IsKeyDown1 - IsKeyDown8 ����

	����û��İ�������С��8��������Խ�����İ���ȫ������Ϊ�͵�1������һ��������Ӱ�������
	#define KEY_COUNT    8	  ����� bsp_key.h �ļ��ж���
*/

#ifdef STM32_X3		/* ������ STM32-X3 ������ */
	/*
		������STM32-X �������߷��䣺
			K1��       : PC13 (�͵�ƽ��ʾ����)
			K2��       : PC0  (�͵�ƽ��ʾ����)
			K3��       : PC1  (�͵�ƽ��ʾ����)
			ҡ��UP��   : PC2  (�͵�ƽ��ʾ����)
			ҡ��DOWN�� : PC3  (�͵�ƽ��ʾ����)
			ҡ��LEFT�� : PC4  (�͵�ƽ��ʾ����)
			ҡ��RIGHT��: PA0  (�͵�ƽ��ʾ����)
			ҡ��OK��   : PA1  (�͵�ƽ��ʾ����)
	*/
	#define RCC_ALL_KEY 	(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC)	/* �����ڶ�Ӧ��RCCʱ�� */

	#define GPIO_PORT_K1    GPIOC
	#define GPIO_PIN_K1	    GPIO_Pin_13

	#define GPIO_PORT_K2    GPIOC
	#define GPIO_PIN_K2	    GPIO_Pin_0

	#define GPIO_PORT_K3    GPIOC
	#define GPIO_PIN_K3	    GPIO_Pin_1

	#define GPIO_PORT_K4    GPIOC
	#define GPIO_PIN_K4	    GPIO_Pin_3

	#define GPIO_PORT_K5    GPIOC
	#define GPIO_PIN_K5	    GPIO_Pin_4

	#define GPIO_PORT_K6    GPIOC
	#define GPIO_PIN_K6	    GPIO_Pin_2

	#define GPIO_PORT_K7    GPIOA
	#define GPIO_PIN_K7	    GPIO_Pin_1

	#define GPIO_PORT_K8    GPIOA
	#define GPIO_PIN_K8	    GPIO_Pin_0

#else	/* STM32_V5 */
	/*
		������STM32-V5 �������߷��䣺
			K1 ��      : PI8   (�͵�ƽ��ʾ����)
			K2 ��      : PC13  (�͵�ƽ��ʾ����)
			K3 ��      : PI11  (�͵�ƽ��ʾ����)
			ҡ��UP��   : PH2   (�͵�ƽ��ʾ����)
			ҡ��DOWN�� : PH3   (�͵�ƽ��ʾ����)
			ҡ��LEFT�� : PF11  (�͵�ƽ��ʾ����)
			ҡ��RIGHT��: PG7   (�͵�ƽ��ʾ����)
			ҡ��OK��   : PH15  (�͵�ƽ��ʾ����)
	*/

	/* �����ڶ�Ӧ��RCCʱ�� */
	#define RCC_ALL_KEY 	(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI | RCC_AHB1Periph_GPIOG)

	#define GPIO_PORT_K1    GPIOI
	#define GPIO_PIN_K1	    GPIO_Pin_8

	#define GPIO_PORT_K2    GPIOC
	#define GPIO_PIN_K2	    GPIO_Pin_13

	#define GPIO_PORT_K3    GPIOI
	#define GPIO_PIN_K3	    GPIO_Pin_11

	#define GPIO_PORT_K4    GPIOH
	#define GPIO_PIN_K4	    GPIO_Pin_2

	#define GPIO_PORT_K5    GPIOH
	#define GPIO_PIN_K5	    GPIO_Pin_3

	#define GPIO_PORT_K6    GPIOF
	#define GPIO_PIN_K6	    GPIO_Pin_11

	#define GPIO_PORT_K7    GPIOG
	#define GPIO_PIN_K7	    GPIO_Pin_7

	#define GPIO_PORT_K8    GPIOH
	#define GPIO_PIN_K8	    GPIO_Pin_15
#endif

static KEY_T s_tBtn[KEY_COUNT];
static KEY_FIFO_T s_tKey;		/* ����FIFO����,�ṹ�� */

static void bsp_InitKeyVar(void);
static void bsp_InitKeyHard(void);
static void bsp_DetectKey(uint8_t i);

/*
*********************************************************************************************************
*	�� �� ��: IsKeyDownX
*	����˵��: �жϰ����Ƿ���
*	��    ��: ��
*	�� �� ֵ: ����ֵ1 ��ʾ���£�0��ʾδ����
*********************************************************************************************************
*/
#ifdef STM32_X3		/* ������ STM32-X3 ������ */
	static uint8_t IsKeyDown1(void) {if ((GPIO_PORT_K1->IDR & GPIO_PIN_K1) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown2(void) {if ((GPIO_PORT_K2->IDR & GPIO_PIN_K2) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown3(void) {if ((GPIO_PORT_K3->IDR & GPIO_PIN_K3) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown4(void) {if ((GPIO_PORT_K4->IDR & GPIO_PIN_K4) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown5(void) {if ((GPIO_PORT_K5->IDR & GPIO_PIN_K5) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown6(void) {if ((GPIO_PORT_K6->IDR & GPIO_PIN_K6) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown7(void) {if ((GPIO_PORT_K7->IDR & GPIO_PIN_K7) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown8(void) {if ((GPIO_PORT_K8->IDR & GPIO_PIN_K8) == 0) return 1;else return 0;}
#else				/* ������ STM32-V5 ������ */
	static uint8_t IsKeyDown1(void) {if ((GPIO_PORT_K1->IDR & GPIO_PIN_K1) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown2(void) {if ((GPIO_PORT_K2->IDR & GPIO_PIN_K2) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown3(void) {if ((GPIO_PORT_K3->IDR & GPIO_PIN_K3) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown4(void) {if ((GPIO_PORT_K4->IDR & GPIO_PIN_K4) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown5(void) {if ((GPIO_PORT_K5->IDR & GPIO_PIN_K5) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown6(void) {if ((GPIO_PORT_K6->IDR & GPIO_PIN_K6) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown7(void) {if ((GPIO_PORT_K7->IDR & GPIO_PIN_K7) == 0) return 1;else return 0;}
	static uint8_t IsKeyDown8(void) {if ((GPIO_PORT_K8->IDR & GPIO_PIN_K8) == 0) return 1;else return 0;}
#endif
	static uint8_t IsKeyDown9(void) {if (IsKeyDown1() && IsKeyDown2()) return 1;else return 0;}
	static uint8_t IsKeyDown10(void) {if (IsKeyDown1() && IsKeyDown2()) return 1;else return 0;}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKey
*	����˵��: ��ʼ������. �ú����� bsp_Init() ���á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitKey(void)
{
	bsp_InitKeyVar();		/* ��ʼ���������� */
	bsp_InitKeyHard();		/* ��ʼ������Ӳ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PutKey
*	����˵��: ��1����ֵѹ�밴��FIFO��������������ģ��һ��������
*	��    ��:  _KeyCode : ��������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PutKey(uint8_t _KeyCode)
{
	s_tKey.Buf[s_tKey.Write] = _KeyCode;

	if (++s_tKey.Write  >= KEY_FIFO_SIZE)
	{
		s_tKey.Write = 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKey
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ��
*	��    ��:  ��
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
uint8_t bsp_GetKey(void)
{
	uint8_t ret;

	if (s_tKey.Read == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read];

		if (++s_tKey.Read >= KEY_FIFO_SIZE)
		{
			s_tKey.Read = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKey2
*	����˵��: �Ӱ���FIFO��������ȡһ����ֵ�������Ķ�ָ�롣
*	��    ��:  ��
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
uint8_t bsp_GetKey2(void)
{
	uint8_t ret;

	if (s_tKey.Read2 == s_tKey.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tKey.Buf[s_tKey.Read2];

		if (++s_tKey.Read2 >= KEY_FIFO_SIZE)
		{
			s_tKey.Read2 = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKeyState
*	����˵��: ��ȡ������״̬
*	��    ��:  _ucKeyID : ����ID����0��ʼ
*	�� �� ֵ: 1 ��ʾ���£� 0 ��ʾδ����
*********************************************************************************************************
*/
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
	return s_tBtn[_ucKeyID].State;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetKeyParam
*	����˵��: ���ð�������
*	��    �Σ�_ucKeyID : ����ID����0��ʼ
*			_LongTime : �����¼�ʱ��
*			 _RepeatSpeed : �����ٶ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed)
{
	s_tBtn[_ucKeyID].LongTime = _LongTime;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
	s_tBtn[_ucKeyID].RepeatSpeed = _RepeatSpeed;			/* �����������ٶȣ�0��ʾ��֧������ */
	s_tBtn[_ucKeyID].RepeatCount = 0;						/* ���������� */
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearKey
*	����˵��: ��հ���FIFO������
*	��    �Σ���
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
void bsp_ClearKey(void)
{
	s_tKey.Read = s_tKey.Write;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyHard
*	����˵��: ���ð�����Ӧ��GPIO
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyHard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��1������GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_ALL_KEY, ENABLE);

	/* ��2�����������еİ���GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* �������������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K1;
	GPIO_Init(GPIO_PORT_K1, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K2;
	GPIO_Init(GPIO_PORT_K2, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K3;
	GPIO_Init(GPIO_PORT_K3, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K4;
	GPIO_Init(GPIO_PORT_K4, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K5;
	GPIO_Init(GPIO_PORT_K5, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K6;
	GPIO_Init(GPIO_PORT_K6, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K7;
	GPIO_Init(GPIO_PORT_K7, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K8;
	GPIO_Init(GPIO_PORT_K8, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyVar
*	����˵��: ��ʼ����������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyVar(void)
{
	uint8_t i;

	/* �԰���FIFO��дָ������ */
	s_tKey.Read = 0;
	s_tKey.Write = 0;
	s_tKey.Read2 = 0;

	/* ��ÿ�������ṹ���Ա������һ��ȱʡֵ */
	for (i = 0; i < KEY_COUNT; i++)
	{
		s_tBtn[i].LongTime = KEY_LONG_TIME;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
		s_tBtn[i].Count = KEY_FILTER_TIME / 2;		/* ����������Ϊ�˲�ʱ���һ�� */
		s_tBtn[i].State = 0;							/* ����ȱʡ״̬��0Ϊδ���� */
		//s_tBtn[i].KeyCodeDown = 3 * i + 1;				/* �������µļ�ֵ���� */
		//s_tBtn[i].KeyCodeUp   = 3 * i + 2;				/* ��������ļ�ֵ���� */
		//s_tBtn[i].KeyCodeLong = 3 * i + 3;				/* �������������µļ�ֵ���� */
		s_tBtn[i].RepeatSpeed = 0;						/* �����������ٶȣ�0��ʾ��֧������ */
		s_tBtn[i].RepeatCount = 0;						/* ���������� */
	}

	/* �����Ҫ��������ĳ�������Ĳ����������ڴ˵������¸�ֵ */
	/* ���磬����ϣ������1���³���1����Զ��ط���ͬ��ֵ */
	s_tBtn[KID_JOY_U].LongTime = 100;
	s_tBtn[KID_JOY_U].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	s_tBtn[KID_JOY_D].LongTime = 100;
	s_tBtn[KID_JOY_D].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	s_tBtn[KID_JOY_L].LongTime = 100;
	s_tBtn[KID_JOY_L].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	s_tBtn[KID_JOY_R].LongTime = 100;
	s_tBtn[KID_JOY_R].RepeatSpeed = 5;	/* ÿ��50ms�Զ����ͼ�ֵ */

	/* �жϰ������µĺ��� */
	s_tBtn[0].IsKeyDownFunc = IsKeyDown1;
	s_tBtn[1].IsKeyDownFunc = IsKeyDown2;
	s_tBtn[2].IsKeyDownFunc = IsKeyDown3;
	s_tBtn[3].IsKeyDownFunc = IsKeyDown4;
	s_tBtn[4].IsKeyDownFunc = IsKeyDown5;
	s_tBtn[5].IsKeyDownFunc = IsKeyDown6;
	s_tBtn[6].IsKeyDownFunc = IsKeyDown7;
	s_tBtn[7].IsKeyDownFunc = IsKeyDown8;

	/* ��ϼ� */
	s_tBtn[8].IsKeyDownFunc = IsKeyDown9;
	s_tBtn[9].IsKeyDownFunc = IsKeyDown10;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_DetectKey
*	����˵��: ���һ��������������״̬�����뱻�����Եĵ��á�
*	��    ��:  �����ṹ����ָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_DetectKey(uint8_t i)
{
	KEY_T *pBtn;

	/*
		���û�г�ʼ�������������򱨴�
		if (s_tBtn[i].IsKeyDownFunc == 0)
		{
			printf("Fault : DetectButton(), s_tBtn[i].IsKeyDownFunc undefine");
		}
	*/

	pBtn = &s_tBtn[i];
	if (pBtn->IsKeyDownFunc())
	{
		if (pBtn->Count < KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count < 2 * KEY_FILTER_TIME)
		{
			pBtn->Count++;
		}
		else
		{
			if (pBtn->State == 0)
			{
				pBtn->State = 1;

				/* ���Ͱ�ť���µ���Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 1));
			}

			if (pBtn->LongTime > 0)
			{
				if (pBtn->LongCount < pBtn->LongTime)
				{
					/* ���Ͱ�ť�������µ���Ϣ */
					if (++pBtn->LongCount == pBtn->LongTime)
					{
						/* ��ֵ���밴��FIFO */
						bsp_PutKey((uint8_t)(3 * i + 3));
					}
				}
				else
				{
					if (pBtn->RepeatSpeed > 0)
					{
						if (++pBtn->RepeatCount >= pBtn->RepeatSpeed)
						{
							pBtn->RepeatCount = 0;
							/* ��������ÿ��10ms����1������ */
							bsp_PutKey((uint8_t)(3 * i + 1));
						}
					}
				}
			}
		}
	}
	else
	{
		if(pBtn->Count > KEY_FILTER_TIME)
		{
			pBtn->Count = KEY_FILTER_TIME;
		}
		else if(pBtn->Count != 0)
		{
			pBtn->Count--;
		}
		else
		{
			if (pBtn->State == 1)
			{
				pBtn->State = 0;

				/* ���Ͱ�ť�������Ϣ */
				bsp_PutKey((uint8_t)(3 * i + 2));
			}
		}

		pBtn->LongCount = 0;
		pBtn->RepeatCount = 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyScan
*	����˵��: ɨ�����а���������������systick�ж������Եĵ���
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_KeyScan(void)
{
	uint8_t i;

	for (i = 0; i < KEY_COUNT; i++)
	{
		bsp_DetectKey(i);
	}
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
