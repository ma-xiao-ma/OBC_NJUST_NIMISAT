#include <stdint.h>
#include <sys/types.h>

#include "stm32f4xx.h"

#include "bsp_fsmc_sram.h"

// ---------------------------------------------------------------------------- //

extern unsigned int _data_lma;
extern unsigned int _sdata;
extern unsigned int _edata;

extern unsigned int _hk_lma;
extern unsigned int _shk;
extern unsigned int _ehk;

extern unsigned int _sbss;
extern unsigned int _ebss;

extern void main (void);
void _start (void);

void __attribute__ ((section(".after_vectors"),noreturn))
_start (void)
{
	/* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
	RCC->AHB1ENR = 0x00000078;
	/* Connect PDx pins to FSMC Alternate function */
	GPIOD->AFR[0]  = 0xc0cc00cc;
	GPIOD->AFR[1]  = 0xcccccccc; //0xcc0ccccc;
	/* Configure PDx pins in Alternate function mode */
	GPIOD->MODER   = 0xaaaa8a0a;
	/* Configure PDx pins speed to 100 MHz */
	GPIOD->OSPEEDR = 0xffffcf0f;
	/* Configure PDx pins Output type to push-pull */
	GPIOD->OTYPER  = 0x00000000;
	/* No pull-up, pull-down for PDx pins */
	GPIOD->PUPDR   = 0x00000000;
	/* Connect PEx pins to FSMC Alternate function */
	GPIOE->AFR[0]  = 0xc0ccc0cc;
	GPIOE->AFR[1]  = 0xcccccccc;
	/* Configure PEx pins in Alternate function mode */
	GPIOE->MODER   = 0xaaaa8a8a; //0xaaaa828a;
	/* Configure PEx pins speed to 100 MHz */
	GPIOE->OSPEEDR = 0xffffcfcf;
	/* Configure PEx pins Output type to push-pull */
	GPIOE->OTYPER  = 0x00000000;
	/* No pull-up, pull-down for PEx pins */
	GPIOE->PUPDR   = 0x00000000;
	/* Connect PFx pins to FSMC Alternate function */
	GPIOF->AFR[0]  = 0x00cccccc;
	GPIOF->AFR[1]  = 0xcccc0000;
	/* Configure PFx pins in Alternate function mode */
	GPIOF->MODER   = 0xaa000aaa;
	/* Configure PFx pins speed to 100 MHz */
	GPIOF->OSPEEDR = 0xff000fff;
	/* Configure PFx pins Output type to push-pull */
	GPIOF->OTYPER  = 0x00000000;
	/* No pull-up, pull-down for PFx pins */
	GPIOF->PUPDR   = 0x00000000;
	/* Connect PGx pins to FSMC Alternate function */
	GPIOG->AFR[0]  = 0x00cccccc;
	GPIOG->AFR[1]  = 0x880c0cc8; //0x000000c0;
	/* Configure PGx pins in Alternate function mode */
	GPIOG->MODER   = 0xa22a0aaa; //0x00080aaa;
	/* Configure PGx pins speed to 100 MHz */
	GPIOG->OSPEEDR = 0xa33e0fff; //0x000c0fff;
	/* Configure PGx pins Output type to push-pull */
	GPIOG->OTYPER  = 0x00000000;
	/* No pull-up, pull-down for PGx pins */
	GPIOG->PUPDR = 0x00000000; //0x00000000;
	/*-- FSMC Configuration ------------------------------------------------------*/
	/* Enable the FSMC interface clock */
	RCC->AHB3ENR = 0x00000001;
	/* Configure and enable Bank1_SRAM1 */
	FSMC_Bank1->BTCR[0]  = 0x00001085; //1015
	FSMC_Bank1->BTCR[1]  = 0x00000102;
	FSMC_Bank1E->BWTR[0] = 0x0fffffff;
	/* Configure and enable Bank1_SRAM4 */
	FSMC_Bank1->BTCR[6]  = 0x00001085; //1015
	FSMC_Bank1->BTCR[7]  = 0x00000102;
	FSMC_Bank1E->BWTR[6] = 0x0fffffff;
	/* Configure and enable Bank1_SRAM3 */
	FSMC_Bank1->BTCR[4]  = 0x00001090; //1015
	FSMC_Bank1->BTCR[5]  = 0x00010a03;
	FSMC_Bank1E->BWTR[4] = 0x0fffffff;


	// Copy the data sections from flash to SRAM.
	int * src, * dst;

	src = (int *)&_data_lma;
	dst = (int *)&_sdata;
	while (dst < (int *)&_edata)
	  {*dst++ = *src++;}

	// Zero fill all bss segments
	dst = (int *)&_sbss;
	while (dst < (int *)&_ebss)
		{*dst++ = 0;}

	SystemInit();

	SystemCoreClockUpdate();

	bsp_InitExtSRAM();

	src = (int *)&_hk_lma;
	dst = (int *)&_shk;
	while (dst < (int *)&_ehk)
	  {*dst++ = *src++;}

	main();
}

