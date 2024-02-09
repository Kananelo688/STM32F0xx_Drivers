/*
*************************************************************************************************
 * STM32F411RE Start up file.The start up file does the following:								*
 *								1. Creates a Vector table										*
 *								2. Initialise .data and .bss section							*
 *								3. Call main()													*
 * 							 																	*	
 *																								*
 *Author: Kananelo Chabeli																		*
 *@version: 02/11/2023
 * **********************************************************************************************
*/

/*************************************Includes**************************************************/

#include <stdint.h>

/*************************************Defines***************************************************/

#define SRAM_START 0x20000000U //address where the SRAM start in STM32F4
#define SRAM_SIZE (128U*1024U) //128KB(Size of the SRAM)
#define SRAM_END  ((SRAM_START)+ (SRAM_SIZE)-1)
#define STACK_START  SRAM_END

/************************************External Variables*****************************************/

extern uint32_t _etext;/*linker symbol that marks end of the .text section of the final .elf   */
extern uint32_t _edata;/*linker symbol that marks end of the .data section of the final .elf   */
extern uint32_t _sdata;/*linker symbol that marks start of the .data section of the final .elf */
extern uint32_t _ebss;/*linker symbol that marks end of the .bss section of the final .elf     */
extern uint32_t _sbss;/*linker symbol that marks start of the .bss section of the final .elf   */

/***********************************Fuction Declarations****************************************/

int main(void);				/*The main function which is to be called by the Reset_IRQHandler    */

void Reset_Handler(void); /*Reset_IRQHandler is the entry point to the program on start up or boot*/

/**
 * *********************************************************************************************
 * The following function declaration illustrate the use of the function attributes weak&alias *
 * 		1. WEAK: let's the programmer to override the already defineed weak function with the  *
 * 					same name.																   *
 * 		2. ALIAS: Let's the programmer to provide alias name for the function, The function    *
 * 					will be replaced with the alias function code. 							   *
 * *********************************************************************************************
 * */

void NMI_IRQHandler						(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void HardFault_IRQHandler				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void MemManage_IRQHandler				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void BusFault_IRQHandler				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void UsageFault_IRQHandler				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SVCall_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DebugMonitor_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void PendSV_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SysTick_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void WWDG_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI16_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI21_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI22_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void Flash_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void RCC_IRQHandler                     (void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI0_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI1_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI2_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI3_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI4_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream0_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream1_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream2_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream3_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream4_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream5_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream6_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void ADC_IRQHandler						(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI9_5_IRQHandler 				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM1_BRK_TIM9_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM1_UP_TIM10_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM1_TRG_COM_TIM11_IRQHandler      (void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM1_CC_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM2_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM3_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM4_IRQHandler    				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void I2C1_ER_IRQHandler 				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void I2C1_EV_IRQHandler 				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void I2C2_EV_IRQHandler 				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void I2C2_ER_IRQHandler 				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SPI1_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SPI2_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void USART1_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void USART2_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI15_10_IRQHandler 				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI17_Hanlder 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void EXTI18_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SDIO_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void TIM5_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SPI3_IRQHandler 					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream0_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream1_IRQHandler 			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream2_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream3_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream4_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream5_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream6_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA2_Stream7_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void OTG_FS_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void USART6_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void I2C3_EV_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void I2C3_ER_IRQHandler 				(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void FPU_IRQHandler						(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SPI4_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void SPI5_IRQHandler					(void) __attribute__ ((weak,alias("Default_IRQHandler")));
void DMA1_Stream7_IRQHandler			(void) __attribute__ ((weak,alias("Default_IRQHandler")));

/**
 * **********************************************************************************************
 * Vector Table: The following array is the vector table which was made hand-in-hand with       *
 * 				 the datasheet of STM32F4xx microcontrollers. The reversed space on the vector	*
 * 				 table is marked by 0 															*
 * 																								*
 * Note: Because this is an array of ints, compiler would place this is .data section. However, *
 * The vector table need to be place at some specific address as specified by the STM32. The 	*
 * section attibute is used here to instruct the compiler to create a different section called 	*
 * '.isr_vector', and place this array in that section.											*
 * 																								*
 * **********************************************************************************************
 * */

uint32_t* vectors[] __attribute__((section(".isr_vector")))=
{
(uint32_t*) STACK_START,					/*ADDRESS	0x00000000*/
(uint32_t*) &Reset_Handler,					/*ADDRESS	0x00000004*/
(uint32_t*) &NMI_IRQHandler, 					/*ADDRESS	0x00000008*/
(uint32_t*) &HardFault_IRQHandler,				/*ADDRESS	0x0000000c*/
(uint32_t*) &MemManage_IRQHandler,				/*ADDRESS	0x00000010*/
(uint32_t*) &BusFault_IRQHandler,				/*ADDRESS	0x00000014*/
(uint32_t*) &UsageFault_IRQHandler,			/*ADDRESS	0x00000018*/
0,											/*ADDRESS	0x0000001c*/
0,											/*ADDRESS	0x00000020*/
0,											/*ADDRESS	0x00000024*/
0,											/*ADDRESS	0x00000028*/
(uint32_t*) &SVCall_IRQHandler,				/*ADDRESS	0x0000002c*/
(uint32_t*) &DebugMonitor_IRQHandler,			/*ADDRESS	0x00000030*/
0,											/*ADDRESS	0x00000034*/
(uint32_t*) &PendSV_IRQHandler,				/*ADDRESS	0x00000038*/
(uint32_t*) &SysTick_IRQHandler,				/*ADDRESS	0x0000003c*/
(uint32_t*) &WWDG_IRQHandler,					/*ADDRESS	0x00000040*/	
(uint32_t*) &EXTI16_IRQHandler,				/*ADDRESS	0x00000044*/
(uint32_t*) &EXTI21_IRQHandler,				/*ADDRESS	0x00000048*/
(uint32_t*) &EXTI22_IRQHandler,				/*ADDRESS	0x0000004c*/
(uint32_t*) &Flash_IRQHandler,					/*ADDRESS	0x00000050*/
(uint32_t*) &RCC_IRQHandler,					/*ADDRESS	0x00000054*/
(uint32_t*) &EXTI0_IRQHandler,					/*ADDRESS	0x00000058*/
(uint32_t*) &EXTI1_IRQHandler,					/*ADDRESS	0x0000005c*/
(uint32_t*) &EXTI2_IRQHandler,					/*ADDRESS	0x00000060*/
(uint32_t*) &EXTI3_IRQHandler,					/*ADDRESS	0x00000064*/
(uint32_t*) &EXTI4_IRQHandler,					/*ADDRESS	0x00000068*/
(uint32_t*) &DMA1_Stream0_IRQHandler,			/*ADDRESS	0x0000006c*/
(uint32_t*) &DMA1_Stream1_IRQHandler,			/*ADDRESS	0x00000070*/
(uint32_t*) &DMA1_Stream2_IRQHandler,			/*ADDRESS	0x00000074*/
(uint32_t*) &DMA1_Stream3_IRQHandler,			/*ADDRESS	0x00000078*/
(uint32_t*) &DMA1_Stream4_IRQHandler,			/*ADDRESS	0x0000007c*/
(uint32_t*) &DMA1_Stream5_IRQHandler,			/*ADDRESS	0x00000080*/
(uint32_t*) &DMA1_Stream6_IRQHandler,			/*ADDRESS	0x00000084*/
(uint32_t*) &ADC_IRQHandler,					/*ADDRESS	0x00000088*/
0,											/*ADDRESS	0x0000008c*/	
0,											/*ADDRESS	0x00000090*/
0,											/*ADDRESS	0x00000094*/
0,											/*ADDRESS	0x00000098*/
(uint32_t*) &EXTI9_5_IRQHandler,				/*ADDRESS	0x0000009c*/
(uint32_t*) &TIM1_BRK_TIM9_IRQHandler, 		/*ADDRESS	0x000000A0*/
(uint32_t*) &TIM1_UP_TIM10_IRQHandler,			/*ADDRESS	0x000000A4*/
(uint32_t*) &TIM1_TRG_COM_TIM11_IRQHandler,  	/*ADDRESS	0x000000A8*/
(uint32_t*) &TIM1_CC_IRQHandler, 				/*ADDRESS	0x000000AC*/
(uint32_t*) &TIM2_IRQHandler, 					/*ADDRESS	0x000000B0*/
(uint32_t*) &TIM3_IRQHandler, 					/*ADDRESS	0x000000B4*/
(uint32_t*) &TIM4_IRQHandler, 					/*ADDRESS	0x000000B8*/
(uint32_t*) &I2C1_EV_IRQHandler, 				/*ADDRESS	0x000000BC*/
(uint32_t*) &I2C1_ER_IRQHandler, 				/*ADDRESS	0x000000C0*/
(uint32_t*) &I2C2_EV_IRQHandler, 				/*ADDRESS	0x000000C4*/
(uint32_t*) &I2C2_ER_IRQHandler, 				/*ADDRESS	0x000000C8*/
(uint32_t*) &SPI1_IRQHandler, 					/*ADDRESS	0x000000CC*/
(uint32_t*) &SPI2_IRQHandler, 					/*ADDRESS	0x000000D0*/
(uint32_t*) &USART1_IRQHandler, 				/*ADDRESS	0x000000D4*/
(uint32_t*) &USART2_IRQHandler, 				/*ADDRESS	0x000000D8*/
0,											/*ADDRESS	0x000000DC*/
(uint32_t*) &EXTI15_10_IRQHandler, 			/*ADDRESS	0x000000E0*/
(uint32_t*) &EXTI17_Hanlder, 				/*ADDRESS	0x000000E4*/
(uint32_t*) &EXTI18_IRQHandler, 				/*ADDRESS	0x000000E8*/
0,											/*ADDRESS	0x000000EC*/
0,											/*ADDRESS	0x000000F0*/
0,											/*ADDRESS	0x000000F4*/	
0,											/*ADDRESS	0x000000F8*/
(uint32_t*) &DMA1_Stream7_IRQHandler, 			/*ADDRESS	0x000000FC*/	
0, 											/*ADDRESS	0x00000100*/
(uint32_t*) &SDIO_IRQHandler, 					/*ADDRESS	0x00000104*/
(uint32_t*) &TIM5_IRQHandler, 					/*ADDRESS	0x00000108*/
(uint32_t*) &SPI3_IRQHandler, 					/*ADDRESS	0x0000010C*/
(uint32_t*) &DMA2_Stream0_IRQHandler, 			/*ADDRESS	0x00000120*/
(uint32_t*) &DMA2_Stream1_IRQHandler, 			/*ADDRESS	0x00000124*/
(uint32_t*) &DMA2_Stream2_IRQHandler,			/*ADDRESS	0x00000128*/
(uint32_t*) &DMA2_Stream3_IRQHandler,			/*ADDRESS	0x0000012C*/
(uint32_t*) &DMA2_Stream4_IRQHandler,			/*ADDRESS	0x00000130*/
0,											/*ADDRESS	0x00000134*/
0,											/*ADDRESS	0x00000138*/
0,											/*ADDRESS	0x0000013C*/
0,											/*ADDRESS	0x00000140*/
0,											/*ADDRESS	0x00000144*/	
0,											/*ADDRESS	0x00000148*/
(uint32_t*) &OTG_FS_IRQHandler, 				/*ADDRESS	0x0000014C*/
(uint32_t*) &DMA2_Stream5_IRQHandler,			/*ADDRESS	0x00000150*/
(uint32_t*) &DMA2_Stream6_IRQHandler,			/*ADDRESS	0x00000154*/
(uint32_t*) &DMA2_Stream7_IRQHandler,			/*ADDRESS	0x00000158*/
(uint32_t*) &USART6_IRQHandler, 				/*ADDRESS	0x0000015C*/
(uint32_t*) &I2C3_EV_IRQHandler, 				/*ADDRESS	0x00000160*/
(uint32_t*) &I2C3_ER_IRQHandler, 				/*ADDRESS	0x00000164*/
0, 											/*ADDRESS	0x00000168*/
0,											/*ADDRESS	0x0000016C*/
0,											/*ADDRESS	0x00000170*/
0,											/*ADDRESS	0x00000174*/
0,											/*ADDRESS	0x00000178*/
0,											/*ADDRESS	0x0000017C*/
0,											/*ADDRESS	0x00000180*/
(uint32_t*) &FPU_IRQHandler, 					/*ADDRESS	0x00000184*/
0,											/*ADDRESS	0x00000188*/
0,											/*ADDRESS	0x0000018C*/
(uint32_t*) &SPI4_IRQHandler,					/*ADDRESS	0x00000190*/
(uint32_t*) &SPI5_IRQHandler					/*ADDRESS	0x00000194*/	
};

/**
 * *****************************************************************************************
 * Reset_IRQHandler, is the brain of the start_up file. It copies the data from the Flash into
 *  the SRAM during the start up and initialised the .bss section(unitialised variables) to
 * 0 and then call main(). All of this is done on start up, boot or reset.
 * *****************************************************************************************
 * */

void Reset_Handler(void){

    __asm("ldr      r0,=    _etext"); /*Load address where the .data section starts in FLASH*/
    __asm("ldr      r1, =   _sdata");  /*Load address where .data section starts in SRAM    */
    __asm("ldr      r2, =   _edata"); /*Load address where .data ends in SRAM               */

    __asm("ldr      r3, = 0x01");       /*Load 1 into r2                                    */

    __asm("data_loop:");            /*Loop and copy initialised data from flash into SRAM   */
    __asm("ldrb r4, [r0]");         /*load byte from flash*/
    __asm("cmp      r2, r1");       /*Check if _sdata == _edata                             */
    __asm("beq  _init_bss");         /*If so, jump to code thta initialised the .bss section */
   
    __asm("strb     r4, [r1]");      /*Otherwise copy data from flash (r0), into flash (r1)  */
    __asm("add      r0, r3");        /*incremet _etext                                       */
    __asm("add      r1,r3");        /*increment _sdata*/
    __asm("b data_loop"); 
   
    __asm("_init_bss:");        /*Initialized .bss section                                   */
    __asm("ldr      r0, = _sbss"); /*load start address of .bss section                      */
    __asm("ldr      r1, =_ebss"); /*Load end of .bss section                                 */

    __asm("ldrb      r2, = 0x00"); /*Loading the byte 0 into r2 */

    __asm("_bss_loop:");
    
    __asm("cmp      r1,     r0");
    __asm("beq Rst_rtn");
    __asm("strb    r2,      [r0]");
    __asm("add      r0,     r3");     /*incement _sbss*/
    __asm("b    _bss_loop");
    __asm("Rst_rtn:");      /*Returning put of the reset IRQHandler*/
    __asm("bl   main");       /*Call main*/
    __asm("bx       lr");   /*Return from the function*/
}


void  Default_IRQHandler(void){
	while(1);
}