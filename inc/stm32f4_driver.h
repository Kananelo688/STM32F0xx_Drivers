/**
 ************************************************************************************************************ 
 * Header file that specifies General peripherals of STM32F4 (cortex-M4 microcontrollers). This intereface 	*
 * provides specifications to peripherals such Reset and Clock Conrtol, System Configuration, Real Time clcok
 * and so on.																								*
 * 																											*
 * STM32F4xx peripherals are named from 0 starting from Address 0x40000000. This naming is used to determine*
 * which peripheral is connected which BUS when enabling clock for such peripheral.							*
 * 																											*
 * @author: Kananelo Chabeli																				*		
 * 																											*	
 ************************************************************************************************************
 * */

#ifndef STM32F4_HEADER

#define STM32F4_HEADER

/************************************************************************************************************
 * 										INCLUDES															*
 ************************************************************************************************************ 
 */
#include <stdint.h>
/************************************************************************************************************
 * 										DEFINES																*
 ************************************************************************************************************ 
 * */

#define CRC_BASE				0x40023000U
#define RCC_BASE 				0x40023800U
#define DMA1_BASE				0x40026000U
#define DMA2_BASE 				0x40026400U

#define USB_OTG_FS_BASE 		0x50000000U

#define TIM2_BASE				0x40000000U
#define TIM3_BASE				0x40000400U
#define TIM4_BASE 				0x40000800U
#define TIM5_BASE				0x40000C00U
#define RTC_BKP_BASE 			0x40002800U
#define WWDG_BASE				0x40002C00U
#define IWDG_BASE 				0x40003000U
#define I2Sext_BASE 			0x40003400U
#define SPI2_BASE 				0x40003800U
#define SPI3_BASE 				0x40003C00U
#define I2S3ext_BASE 			0x40004000U
#define USART2_BASE				0x40004400U
#define I2C1_BASE 				0x40005400U
#define I2C2_BASE				0x40005800U
#define I2C3_BASE 				0x40005C00U
#define  PWR_BASE 				0x40007000U

#define TIM1_BASE				0x40010000U
#define USART1_BASE 			0x40011000U
#define USART6_BASE 			0x40011400U
#define ADC1_BASE 				0x40012000U
#define SDIO_BASE 				0x40012C00U
#define SPI1_BASE 				0x40013000U
#define SPI4_BASE 				0x40013400U
#define SYSCFG_BASE 			0x40013800U
#define EXTI_BASE				0x40013C00U
#define TIM9_BASE 				0X40014000U
#define TIM10_BASE 				0X40014400U					
#define TIM11_BASE 				0X40014800U					
#define SPI5_BASE 				0X40015000U

/************************************************************************************************************
 * 					MACROS FOR ENABLING AHB1 BUS PERIPHERALS												*
 ************************************************************************************************************ 
*/

#define RCC_AHB1ENR_GPIOAEN				0x00000001U			/*GPIOA Clock Is enabled at bit 0 				*/
#define RCC_AHB1ENR_GPIOBEN				0x00000002U			/*GPIOB Clock Is enabled at bit 1 				*/
#define RCC_AHB1ENR_GPIOCEN				0x00000004U			/*GPIOC Clock Is Enabled at bit 2 				*/
#define RCC_AHB1ENR_GPIODEN				0x00000008U			/*GPIOD Clock Is enabled at bit 3 		 		*/
#define RCC_AHB1ENR_GPIOEEN				0x00000010U			/*GPIOE Clock Is enabled at bit 4 (16) 			*/
#define RCC_AHB1ENR_GPIOHEN				0x00000080U			/*GPIOH Clock Is Enabled at bit 7 (128 			*/
#define RCC_AHB1ENR_CRCEN				0x00001000U			/*CRC 	Clock Is Enabled at bit 12 				*/
#define RCC_AHB1ENR_DMA1EN				0x00200000U			/*DMA1 	Clock Is Enabled at bit 21				*/	
#define RCC_AHB1ENR_DMA2EN				0x00400000U			/*DMA2 	Clock Is Enabled at bit 22				*/

/************************************************************************************************************
 * 					MACROS FOR ENABLING AHB2 BUS PERIPHERALS												*
 ************************************************************************************************************ 
*/

#define RCC_AHB2ENR_OTGFSEN				0x00000080U		/*USB_OTG_FS Clock Enable is at bit 7 of RCC_AHB2ENR*/

/************************************************************************************************************
 * 					MACROS FOR ENABLING APB1 BUS PERIPHERALS												*
 ************************************************************************************************************ 
*/

#define	RCC_APB1ENR_TIM2EN				0x00000001U		/*TIM2 Clock Is Enabled at bit 0 of RCC_APB1ENR 	*/
#define	RCC_APB1ENR_TIM3EN				0x00000002U		/*TIM3 Clock Is Enabled at bit 1 of RCC_APB1ENR 	*/
#define	RCC_APB1ENR_TIM4EN				0x00000004U		/*TIM4 Clock Is Enabled at bit 2 of RCC_APB1ENR 	*/
#define	RCC_APB1ENR_TIM5EN				0x00000008U		/*TIM5 Clock Is Enabled at bit 3 of RCC_APB1ENR 	*/
#define RCC_APB1ENR_WWDGEN				0x00000800U		/*WWDG Clock Is Enabled at bit 11 of RCC_APB1ENR 	*/
#define RCC_APB1ENR_SPI2EN				0x00004000U		/*SPI2 CLock Is Enabled at bit 14 of RCC_APB1ENR 	*/
#define RCC_APB1ENR_SPI3EN				0x00008000U		/*SPI3 Clock Is Enabled at bit 16 of RCC_APB1ENR 	*/
#define RCC_APB1ENR_USART2EN			0x00020000U		/*USART2 Clock is enabled at bit 17 of RCC_APB1ENR  */
#define RCC_APB1ENR_I2C1EN				0x00200000U		/*I2C1 Clock is enabled at bit 21 of RCC_APB1ENR 	*/
#define RCC_APB1ENR_I2C2EN				0x00400000U		/*I2C2 Clock is Enabled at bit 22 of RCC_APB1ENR 	*/
#define RCC_APB1ENR_I2C3EN				0x00800000U		/*I2C3 Clock is enabled at bit 23 of RCC_APB1ENR 	*/
#define RCC_APB1ENR_PWREN				0x10000000U		/*PWR Clock is enabled at bit 28 of RCC_APB1ENER 	*/

/************************************************************************************************************
 * 					MACROS FOR ENABLING APB2 BUS PERIPHERALS												*
 ************************************************************************************************************ 
*/


#define RCC_APB2ENR_TIM1EN				0x00000001U
#define RCC_APB2ENR_USART2EN			0x00000010U
#define RCC_APB2ENR_USART6EN			0x00000020U
#define RCC_APB2ENR_ADC1EN				0x00000100U
#define RCC_APB2ENR_SDIOEN 				0x00000800U
#define RCC_APB2ENR_SPI1EN 				0x00001000U
#define RCC_APB2ENR_SPI4EN				0x00002000U
#define RCC_APB2ENR_SYSCFGEN			0x00004000U
#define RCC_APB2ENR_TIM9EN				0x00010000U
#define RCC_APB2ENR_TIM10EN				0x00020000U
#define RCC_APB2ENR_TIM11EN				0x00040000U
#define RCC_APB2ENR_SPI5EN				0x00100000U

/************************************************************************************************************
 * 								RESET AND CLOCK CONTROL STRUCTURE											*
 ************************************************************************************************************ 
 **/
typedef struct 
{

	uint32_t CR; 				/**RCC CLOCK CONTROL REGISTER					ADDRESS OFFSET		0x00 	*/
	uint32_t PLLCFGR; 			/*RCC PLL CONFIGURATION REGISTER				ADDRESS OFFSET 		0x04 	*/
	uint32_t CFGR;				/*RCC CLOCK CONFIGURATION REGISTER				ADDRESS OFFSET      0X08 	*/
	uint32_t CIR; 				/*RCC CLOCK INTERRUPT REGISTER					ADDRESS OFFSET		0x0C 	*/
	uint32_t AHB1RSTR; 			/*RCC ABH1 RESET REGISTER						ADDRESS OFFSET		0X10	*/
	uint16_t AHB2RSTR; 			/*RCC ABH2 RESET REGISTER 						ADDRESS OFFSET 		0X14	*/
	
	uint16_t __Reserved__0;		/*			Reserved Space 						    						*/
	uint32_t __Reserved__2[2]; 	/*			Reserved Space: 0x18 and 0x1C				    				*/
	
	uint32_t APB1RSTR; 			/*RCC APB1 RESET REGISTER 						ADDRESS OFFSET		0x20	*/
	uint32_t APB2RSTR; 			/*RCC APB2 RESET REGISTER 						ADDRESS OFFSET		0x24	*/
	
	uint32_t __Reserved__3[2]; 	/*			Reserved Space: 0x28, and 0x2C 				  					*/

	uint32_t AHB1ENR; 			/*RCC AHB1 CLOCK ENABLE REGISTER 				ADDRESS OFFSET  	0x30	*/
	uint32_t AHB2ENR; 			/*RCC AHB2 CLOCK ENABLE REGISTER 				ADDRESS OFFSET  	0x34	*/
	
	uint32_t __Reserved__4[2]; 	/*			Reserved Space: 0x38 and 0x3C				    				*/
	
	uint32_t APB1ENR; 			/*RCC APB1 CLOCK ENABLE REGISTER 				ADDRESS OFFSET  	0x40 	*/
	uint32_t APB2ENR; 			/*RCC APB2 CLOCK ENABLE REGISTER 				ADDRESS OFFSET  	0x44 	*/
	
	uint32_t __Reserved__5[2]; 	/*			Reserved Space: 0x48 and 0x4C				    				*/
	
	uint32_t AHB1LPENR; 		/*RCC ABH1 low power mode clock enable register ADDRESS OFFSET		0x50	*/
	uint32_t AHB2LPENR; 		/*RCC ABH2 low power mode clock enable register ADDRESS OFFSET		0x54	*/
	
	uint32_t __Reserved__6[2]; 	/*			Reserved Space: 0x58 and 0x5C				    				*/
	
	uint32_t APB1LPENR; 		/*RCC APB1 low power mode clock enable register ADDRESS OFFSET		0x60	*/
	uint32_t APB2LPENR; 		/*RCC APB2 low power mode clock enable register ADDRESS OFFSET		0x64	*/
	
	uint32_t __Reserved__7[2]; 	/*			Reserved Space: 0x68 and 0x6C				    				*/
	
	uint32_t BDCR; 				/*RCC Back domain control register 				ADDRESS OFFSET 		0x70 	*/
	uint32_t CSR; 				/*RCC Control and Status Register 				ADDRESS OFFSET 		0x74 	*/
	
	uint32_t __Reserved__8[2]; 	/*			Reserved Space: 0x78 and 0x7C				    				*/
	
	uint32_t SSCGR; 			/*Spread Spectrum Clock Generation Register 	ADDRESS OFFSET      0x80	*/
	uint32_t PLLI2SCFGR; 		/*RCC PLLI2S CONFIGURATION REGISTER 			ADDRESS OFSET 		0x84	*/
	uint32_t DCKCFGR; 			/*Dedicated clocks configuration register 		ADDRESS OFFSET 		0x8C	*/

}RCC_Typedef;

/************************************************************************************************************
 * 								SYSTEM CONFIGURATION STRUCTURE												*
 ************************************************************************************************************																		
 */

typedef struct
{
	uint16_t MEMRMP; 			/*SYSCFG Memory remap register 							ADDRESS OFFSET 0x00 */
	uint16_t __reserved__0;		/*						Reserved space										*/
	uint32_t PCM; 				/*SYSCFG Peripheral configuration register 				ADDRESS OFFSET 0x04 */
	uint16_t EXTICR1; 			/*SYSCG external interrrupt configuratio registr 1 		ADDRESS OFFSET 0x08 */
	uint16_t __reserved__1;		/*						Reserved space										*/
	uint16_t EXTICR2; 			/*SYSCG external interrrupt configuratio registr 2 		ADDRESS OFFSET 0x0C */
	uint16_t __reserved__2;		/*						Reserved space										*/
	uint16_t EXTICR3; 			/*SYSCG external interrrupt configuratio registr 3 		ADDRESS OFFSET 0x10 */
	uint16_t __reserved__3;		/*						Reserved space										*/
	uint16_t EXTICR4; 			/*SYSCG external interrrupt configuratio registr 4 		ADDRESS OFFSET 0x14 */
	uint16_t __reserved__4;		/*						Reserved space										*/

	uint32_t __Reserved__5[2];	/*	Two 4 bytes of reserved space: from 0x18 ro 0x1C 						*/	

	uint16_t COMPCR; 			/*SYSCG compensation cell control register 		 		ADDRESS OFFSET 0x20 */
	uint16_t __reserved__6;		/*						Reserved space										*/

}SYSCFG_Typedef;

/************************************************************************************************************
 * 								POWER CONTROLLER STRUCTURE													*
 ************************************************************************************************************ 
 */

typedef struct 
{	

	uint32_t CR; 				/*Power Control Register 								ADDRESS OFFSET 0x00*/
	uint32_t CSR; 				/*Power Control / Status Register 						ADDRESS OFFSET 0x04*/

}PWR_Typedef;

/************************************************************************************************************
 * 								CYCLIC REDANDECY CHECK UNIT STRUCTURE										*
 ************************************************************************************************************
 *  */

typedef struct
{

	uint32_t DR; 				/*CRC  Data Register 									ADDRESS OFFSET 0x00*/
	uint32_t IDR; 				/*CRC Independent Data Register 						ADDRESS OFFSET 0x04*/
	uint32_t CR; 				/*CRC Control Register 									ADDRESS OFFSET 0x08*/
	
}CRC_Typedef;

/************************************************************************************************************
 * 								EXTERNAL INTERRUPT (EXTI) STRUCTURE											*
 ************************************************************************************************************ 	
 * */

typedef struct
{
	uint32_t IMR; 				/*EXTI Interrupt Mask Register 							ADDRESS OFFSET 0x00*/
	uint32_t EMR; 				/*EXTI Event Mask Register 								ADDRESS OFFSET 0x04*/
	uint32_t RTSR; 				/*EXTI Rising Trigger Selection Register 				ADDRESS OFFSET 0x08*/
	uint32_t FTSR; 				/*EXTI Falling Trigger Selection Register 				ADDRESS OFFSET 0x0C*/
	uint32_t SWIER;				/*EXTI Software Interrupt Event Register 				ADDRESS OFFSET 0x10*/
	uint32_t PR;				/*EXTI Pendin Register 									ADDRESS OFFSET 0x14*/

}EXTI_Typedef;

/************************************************************************************************************
 * 								PERIPHERAL POINTERS															*
 ************************************************************************************************************
 **/
	
#define RCC 			((RCC_Typedef*) 	RCC_BASE)
#define SYSCFG 			((SYSCFG_Typedef*) 	SYSCFG_BASE)
#define PWR 			((PWR_Typedef*)		PWR_BASE)
#define CRC 			((CRC_Typedef*)		CRC_BASE)
#define EXTI 			((EXTI_Typedef*)	EXTI_BASE)

/************************************************************************************************************
 * 									FUNCTION PROTOTYPES 													*
 ************************************************************************************************************ 
 * */

/************************************************************************************************************
 * 									RCC_AHB1_Clock_Enable													*
 * @brief: Enable clock for all peripherals on the Advanced High Speed peripheral bus. 						*
 * 																											*
 * @param: position - a integer that writes 1 to the dedicated bit position on the RCC_AHB1ENR register, for*
 * 			enabling clock to the required GPIO. 															*
 * 																											*
 * **********************************************************************************************************
 * */

__attribute__((always_inline)) inline static void RCC_AHB1_Clock_Enable(uint32_t position)
{
	RCC -> AHB1ENR |= position;
}

/************************************************************************************************************
 * 									RCC_AHB2_Clock_Enable													*
 * @brief: Enable clock for USB_FS_OTG, which is the only peripheral on the AHB2 peripheral. 				*
 * 																											*
 * @param: position - a integer that writes 1 to the dedicated bit position on the RCC_AHB1ENR register, for*
 * 			enabling clock to the required GPIO. 															*
 * 																											*
 * **********************************************************************************************************
 * */

__attribute__((always_inline)) inline static void RCC_AHB2_Clock_Enable(void)
{
	RCC -> AHB2ENR |=128;
}

/************************************************************************************************************
 * 									RCC_APB1_Clock_Enable													*
 * @brief: Enable clock for all peripherals on the Advanced  Peripheral bus. 								*
 * 																											*
 * @param: position - a integer that writes 1 to the dedicated bit position on the RCC_APB1ENR register, for*
 * 			enabling clock to the required GPIO. 															*
 * 																											*
 * **********************************************************************************************************
 * */

__attribute__((always_inline)) inline static void RCC_APB1_Clock_Enable(uint32_t position)
{
	RCC -> APB1ENR |= position;
}
/************************************************************************************************************
 * 									RCC_APB2_Clock_Enable													*
 * @brief: Enable clock for all peripherals on the Advanced High Speed peripheral bus. 						*
 * 																											*
 * @param: position - a integer that writes 1 to the dedicated bit position on the RCC_APB2ENR register, for*
 * 			enabling clock to the required GPIO. 															*
 * 																											*
 * **********************************************************************************************************
 * */
__attribute__((always_inline)) inline static void RCC_APB2_Clock_Enable(uint32_t position)
{
	RCC -> APB2ENR |= position;
}

#endif 