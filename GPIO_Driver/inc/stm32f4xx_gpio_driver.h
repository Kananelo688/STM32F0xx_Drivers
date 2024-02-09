/************************************************************************************************************
 * 																											*
 * AN STM32F4xx GPIO Driver Interface that defines the function prototypes of common GPIO functionalities	*
 * 																											*
 * 																											*
 * @author: Kananelo Chabeli																				*
 * 																											*
 * @Verison: 20/01/2024																						*
 * 																											*	
 ************************************************************************************************************
 * */

#ifndef STM32F4_GPIO_DRIVER

#define STM32F4_GPIO_DRIVER



/************************************************************************************************************
 * 									INCLUDES																*
 * **********************************************************************************************************
 */

#include <stdint.h>					/*for defining integers with specific bits 								*/

/************************************************************************************************************
 * 									USER-DEFINED prototypes													*				
 ************************************************************************************************************
 */
typedef uint8_t GPIO_Pin_Number; 				//Defines type for GPIO pin numbers from pin 0 - pin 15    //

typedef uint8_t GPIO_Mode;				//Defines type for GPIO_Mode:INPUT, OUTPUT, ALTERNATE, and ANALOG 	//

typedef uint8_t GPIO_Resistor; 					//Defines a type for GPIO pull/ pull down resistor

typedef uint8_t GPIO_State; 					//Defines the state of the GPIO pin: ON=0x01 or OFF=0x00

typedef uint8_t GPIO_Output_Type;

typedef uint8_t GPIO_Output_Speed;

typedef uint8_t GPIO_Alternate_Function;

typedef uint8_t Power_Mode;

/************************************************************************************************************
 * 									GPIO PORT BASE ADDRESSES												*
 ************************************************************************************************************
 */

#define GPIOA_BASE				0x40020000U							//GPIOA Base Address
#define GPIOB_BASE				0x40020400U							//GPIOB Bae  Address
#define GPIOC_BASE				0x40020800U							//GPIOC Base Address
#define GPIOD_BASE				0x40020C00U							//GPIOD Base Address
#define GPIOE_BASE				0x40021000U							//GPIOE Base Address
#define GPIOH_BASE				0x40021C00U

/************************************************************************************************************
 * 									GPIO PORT PIN DEFINES 													*
 ************************************************************************************************************
 */

#define GPIO_PIN_0 						0x00U
#define GPIO_PIN_1 						0x01U
#define GPIO_PIN_2 						0x02U
#define GPIO_PIN_3 						0x03U
#define GPIO_PIN_4 						0x04U
#define GPIO_PIN_5 						0x05U
#define GPIO_PIN_6 						0x06U
#define GPIO_PIN_7 						0x07U
#define GPIO_PIN_8 						0x08U
#define GPIO_PIN_9 						0x09U
#define GPIO_PIN_10 					0x0AU
#define GPIO_PIN_11						0x0BU
#define GPIO_PIN_12						0x0CU
#define GPIO_PIN_13						0x0DU
#define GPIO_PIN_14						0x0EU
#define GPIO_PIN_15 					0x0FU

/************************************************************************************************************
 * 									GPIO PORT MODES 														*
 ************************************************************************************************************
 */

#define GPIO_INPUT_MODE 				0x00U
#define GPIO_OUTPUT_MODE				0x01U
#define GPIO_ALTERNATE_MODE				0x02U
#define GPIO_ANALOG_MODE 				0x03U

/************************************************************************************************************
 * 									GPIO PULL-UP PULL DOWN RESISTORS										*
 ************************************************************************************************************
 */

#define GPIO_NO_PUPDR					0x00U
#define GPIO_PULL_UP					0x01U
#define GPIO_PULL_DOWN					0x02U

/************************************************************************************************************
 * 									GPIO PORT OUTPUT TYPE DEFINES											*
 ************************************************************************************************************
 */

#define GPIO_PUSH_PULL 					0x00U
#define GPIO_OPEN_DRAIN 				0x01U

/************************************************************************************************************
 * 									GPIO ON AND OFF STATES 													*
 ************************************************************************************************************
 */

#define GPIO_ON 						0x01U
#define GPIO_OFF 						0x00U

/************************************************************************************************************
 * 									GPIO PORT ALTERNATE FUNCTIONS											*
 ************************************************************************************************************
 */

#define GPIO_AF0 						0x00U
#define GPIO_AF1 						0x01U
#define GPIO_AF2 						0x02U
#define GPIO_AF3 						0x03U
#define GPIO_AF4 						0x04U
#define GPIO_AF5 						0x05U
#define GPIO_AF6 						0x06U
#define GPIO_AF7 						0x07U
#define GPIO_AF8 						0x08U
#define GPIO_AF9 						0x09U
#define GPIO_AF10 						0x0AU
#define GPIO_AF11						0x0BU
#define GPIO_AF12						0x0CU
#define GPIO_AF13 						0x0DU
#define GPIO_AF14						0x0EU
#define GPIO_AF15						0x0FU

/************************************************************************************************************
 * 									GPIO PORT PIN SPEED DEFINES												*
 ************************************************************************************************************
 */
#define GPIO_LOW_SPEED 					0x00U
#define GPIO_MEDIUM_SPEED 				0x01U
#define GPIO_FAST_SPEED					0x02U
#define GPIO_HIGH_SPEED 				0x03U

/************************************************************************************************************
 * 						GPIO STRUCTURE DECLARATION															*	
 ************************************************************************************************************
 */

typedef struct 

{	
	uint32_t MODER; 			/*GPIO port Mode Register					ADDRESS OFFSET		0x00		*/
	uint16_t OTYPER; 			/*GPIO port output type register			ADDRESS OFFSET 		0x04		*/
	uint16_t	__RESERVED__0;	/*				RESERVED SPACE												*/
	uint32_t OSPEEDR;			/*GPIO Port output speed register 			ADDRESS OFFSET		0x08		*/
	uint32_t PUPDR;				/*GPIO pull or pull down register			ADDRESS OFFSET      0x0C		*/
	uint16_t IDR;				/*GPIO port input data register				ADDRESS OFFSET 		0x10		*/
	uint16_t	__RESERVED__1;	/*					RESERVED SPACE											*/
	uint16_t ODR;				/*GPIO port Output Data Register 			ADDRESS OFFSET		0x14		*/
	uint16_t	__RESERVED__2;	/*					RESERVED SPACE											*/
	uint32_t BSRR; 				/*GPIO Bit set reset register				ADDRESS OFFSET 		0x18		*/
	uint16_t LCKR;				/*GPIO port Lock register 					ADDRESS OFFSET 		0X1C		*/
	uint16_t	__RESERVED__3;	/*					RESERVED SPACE											*/
	uint32_t AFLR;				/*GPIO alternate function low register 		ADDRESS OFFSET 		0x20		*/	
	uint32_t AFHR;				/*GPIO alternate function high register		ADDRESS OFFSET		0x24		*/

} GPIO_Typedef; //END OF GPIO_Typedef structure declaration

/************************************************************************************************************
 * 										GPIO POINTERS														*
 ************************************************************************************************************
 * */

#define GPIOA 		((GPIO_Typedef*) GPIOA_BASE)
#define GPIOB 		((GPIO_Typedef*) GPIOB_BASE)
#define GPIOC 		((GPIO_Typedef*) GPIOC_BASE)
#define GPIOD 		((GPIO_Typedef*) GPIOD_BASE)
#define GPIOE 		((GPIO_Typedef*) GPIOE_BASE)
#define GPIOH 		((GPIO_Typedef*) GPIOH_BASE)


/************************************************************************************************************
 * 							FUNCTION PROTOTYPES																*	
 ************************************************************************************************************
 */


/************************************************************************************************************
 * 																											*
 * Function Name:		GPIO_Set_Pin_Mode																	*
 * 																											*
 * Description: 	Sets the given GPIO pin number's mode to either INPUT, OUTPUT, ANALOG, or 				*
 * 					ALTERNATE FUNCTION MODES.																*
 * 																											*		
 ************************************************************************************************************
 */

__attribute__((always_inline)) static inline void 
GPIO_Set_Pin_Mode(GPIO_Typedef* gpio,GPIO_Pin_Number pin, GPIO_Mode mode)
{
	if(mode == GPIO_INPUT_MODE){

		gpio->MODER &=(0b11<<pin*2);
		return;
	}
	
	gpio->MODER|=(mode<<pin*2);
}


/************************************************************************************************************
 * 																											*
 * Function Name:		GPIO_Set_Pin_Type																	*
 * 																											*
 * Description: Sets the type of the given GPIO pin to either push-pull or open-drain, specified by macros	*
 *			GPIO_PUSH_PULL and GPIO_OPEN_DRAIN, repectively.												*
 * 																											*		
 ************************************************************************************************************
 */
	
__attribute__((always_inline)) static inline void 
GPIO_Set_Pin_Type (GPIO_Typedef* gpio,GPIO_Pin_Number pin, GPIO_Output_Type type)
{
	gpio-> OTYPER|=(type<<pin);
}

/************************************************************************************************************
 * 																											*
 * Function Name:		GPIO_Set_Pin_Speed																	*
 * 																											*
 * Description: Configures the speed of the given GPIO pin to either LOW speed(defualt), MEDIUM speed 		*
 * 				FAST speed and HIGH speed. These speeds can be specified with	macros: GPIO_LOW_SPEED,		*
 * 				GPIO_MEDIUM_SPEED, GPIO_FAST_SPEED and GPIO_HIGH_SPEED respectively.						*
 * 																											*		
 ************************************************************************************************************
 */	

__attribute__((always_inline)) static inline void GPIO_Set_Pin_Speed	
(GPIO_Typedef* gpio,GPIO_Pin_Number pin, GPIO_Output_Speed speed)
{
	if(speed==GPIO_LOW_SPEED){
		 gpio-> OSPEEDR&=(0b11<<(pin*2));
	}else{
		 gpio-> OSPEEDR|=(speed<<(pin*2));
	}
}
/************************************************************************************************************
 * 																											*
 * Function Name: GPIO_Pin_PUPDR																			*
 * 																											*	
 * Description: Connects Pull or pull down reistor on the given pin 										*
 *																											* 
 ************************************************************************************************************ 
 */

__attribute__((always_inline)) static inline void 
GPIO_Pin_PUPDR(GPIO_Typedef* gpio,GPIO_Pin_Number pin, GPIO_Resistor res)
{
	gpio->PUPDR |=(res << pin*2);
}

/************************************************************************************************************
 * 																											*
 * Function Name:		GPIO_Set_Pin_State																	*
 * 																											*
 * Description: Sets the State of the given GPIO pin to either ON(true) or OFF(false), which are define by 	*
 * 				macros, GPIO_ON and GPIO_OFF																*
 * 																											*		
 ************************************************************************************************************
 * */

__attribute__((always_inline)) static inline void	
GPIO_Set_Pin_State(GPIO_Typedef* gpio,GPIO_Pin_Number pin, GPIO_State state)
{
	if(state){
		 gpio-> ODR |=(0x01<<pin); //set the given pin to ON state
		return;
	}

	 gpio-> ODR &=~(0x01<<pin); //set the given pin to OFF state(write 0 to the correspondng bit in ODR)
}

/************************************************************************************************************
 * 																											*
 * Function Name:		GPIO_Get_Pin_State																	*
 * 																											*
 * Description: Returns the State of the given GPIO pin to either ON(true) or OFF(false), 					*
 * 					which are define by macros, GPIO_ON and GPIO_OFF.										*																*
 * 																											*		
 ************************************************************************************************************
 * */

__attribute__((always_inline)) static inline GPIO_State 
GPIO_Get_Pin_State(GPIO_Typedef* gpio,GPIO_Pin_Number pin)
{
	if(gpio-> IDR & (1<<pin)){
		return GPIO_ON;
	}
	return GPIO_OFF;
}

/************************************************************************************************************
 * 																											*
 * Function Name:	GPIO_Set_Pin_AF																			*
 * 																											*
 * Description: Configures the given GPIO PIN alternate function mode.										*
 * 																											*		
 ************************************************************************************************************
 * */

__attribute__((always_inline)) static inline void 
GPIO_Set_Pin_AF(GPIO_Typedef* gpio,GPIO_Pin_Number pin, GPIO_Alternate_Function func)
{
	if(pin<=GPIO_PIN_7)
	{//setting alternate function for all pin 0-7 of the GPIO

		if(af==GPIO_AF0)
		{
			gpio -> AFLR &=~(0b1111<<(pin*4));

		}else
		{
			gpio -> AFLR |=(af<<(pin*4));
		}
	}
	else
	{
		//Set alternate function for GPIO Pins 8-15
		if(af==GPIO_AF0){
			
			gpio -> AFHR &=~(0b1111<<(pin*4));

		}else{
			gpio -> AFHR |=(af<<(pin*4));
		}
	}		
}

/************************************************************************************************************
 * 																											*
 * Function Name:		GPIO_Toggle_Pin																		*
 * 																											*
 * Description: Toggles the given GPIO pin. If current value of the pin of ON, its set to OFF or vice versa.*
 * 																											*		
 ************************************************************************************************************
 * */

__attribute__((always_inline)) static inline void GPIO_Toggle_Pin(GPIO_Typedef* gpio,GPIO_Pin_Number pin)
{
	gpio->ODR ^=(0x01<<pin);
}

/************************************************************************************************************
 * 																											*
 * Function Name:		GPIO_Interrupt_Enable																*
 * 																											*
 * Description: Configures and enables the external interrupt on the given GPIO pin number. The second		*
 * 			argument  should be the array of string defining the parameters of the external interrupt line.	*
 * 																											*
 * 			This pointer should point to array with the  following strings in GIVEN order:					*	
 * 																											*
 * 				1. parameters[0]= "masked" or "unmasked" : specifies if the EXTI is masked or unmasked.		*
 * 				2. parameters[1]= "Rising Edge", "Falling Edge", or "Rising and Falling Edges".				*
 * 																											*																			*
 * N:B- Every programmer using this framework is strongly adviced to adhere to the naming convensions and 	*	
 * restrictions for the routines to function as specified.													*
 * 																											*		
 ************************************************************************************************************
 * */

void GPIO_Interrupt_Enable(GPIO_Typedef* gpio,GPIO_Pin_Number pin, char** parameters);

#endif