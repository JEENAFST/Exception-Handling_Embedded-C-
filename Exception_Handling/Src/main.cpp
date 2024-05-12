#include<stdint.h>
#include "stm32f407.h"

#define	LED_PORT GPIOD
#define	SWITCH_PORT GPIOA
//Red Led- PD14
//Green Led - PD12

#define	LED_PORT_CLOCK  (1U<<3)
#define	PA0SWT_PORT_CLOCK  (1U<<0)
#define	PA0SWT_PIN  (1U<<0)
#define PA0SWT_MODE_BIT (1U<<0)
// STM32F4 Discovery Board – GPIO Pins
#define LED_RED_PIN			(1U<<14) //RED
#define LED_GREEN_PIN		(1U<<12)
#define LED_ORANGE_PIN		(1U<<13)
#define LED_BLUE_PIN		(1U<<15)

#define LED_RED_MODE_BIT    (1U<<28)
#define LED_GREEN_MODE_BIT  (1U<<24)
#define LED_ORANGE_MODE_BIT (1U<<26)
#define LED_BLUE_MODE_BIT   (1U<<30)
uint32_t volatile *pNVICIRQEnReg= (uint32_t*)0xE000E100;

typedef enum{
 OFF =0,
 ON = 1
}LedState;



class LED{

private:
		uint8_t Voltage;
		uint8_t Current;
		LedState Status;

public:
		bool pinStatus;
        LED();
        ~LED();
		LED(uint8_t _voltage,uint8_t _current,LedState _status);

		void Led_state(LedState status); //PD12 LED ON/OFF
		void Led_red_toggle(void); //PD12 LED ON/OFF
		void Led_green_toggle(void);
		void Led_orange_toggle(void);
		void Led_blue_toggle(void);
		bool Switch_state(void);

};
LED::LED() //Default constructor
		{
	    RCC->AHB1ENR |= LED_PORT_CLOCK;//PORTD Clock
		LED_PORT->MODER |=LED_RED_MODE_BIT;//Set the 14th bit in output mode
		LED_PORT->MODER |=LED_GREEN_MODE_BIT;//Set the 12th bit in output mode
		LED_PORT->MODER |=LED_ORANGE_MODE_BIT;//Set the 13th bit in output mode
		LED_PORT->MODER |=LED_BLUE_MODE_BIT;//Set the 15th bit in output mode
		}
LED::~LED()
		{
	   LED_PORT->ODR =0;
	   RCC->AHB1ENR =0;

		}
void LED::Led_red_toggle(void)
{
	LED_PORT->ODR^=LED_RED_PIN;
	for(uint32_t i=0 ; i < 300000 ; i++ );
}
void LED::Led_green_toggle(void)
{
	LED_PORT->ODR^=LED_GREEN_PIN;
	for(uint32_t i=0 ; i < 300000 ; i++ );
}
void LED::Led_orange_toggle(void)
{
	LED_PORT->ODR^=LED_ORANGE_PIN;
	for(uint32_t i=0 ; i < 300000 ; i++ );
}
void LED::Led_blue_toggle(void)
{
	LED_PORT->ODR^=LED_BLUE_PIN;
	for(uint32_t i=0 ; i < 300000 ; i++ );
}
 LED::LED(uint8_t _voltage,uint8_t _current,LedState _status) //Parameterized constructor
{
			Voltage=_voltage;
			Current=_current;
			Status=_status;

			RCC->AHB1ENR |= LED_PORT_CLOCK;//PORTD Clock
			LED_PORT->MODER |=LED_RED_MODE_BIT;//Set the 14th pin in output mode

}
void LED::Led_state(LedState status) //PD12 LED ON/OFF

		{
			 Status=status;
			 if(Status==ON )
			 {
			 LED_PORT->ODR |= LED_RED_PIN;
			 }
			 else
			 {
			 LED_PORT->ODR &= ~LED_RED_PIN;

			 }
		}
bool LED::Switch_state(void)
{
   RCC->AHB1ENR |=PA0SWT_PORT_CLOCK;//PORTA Clock
   SWITCH_PORT->MODER &=~(3<<0);//Set the PA0 pin input mode
   bool pinStatus = (uint8_t)(SWITCH_PORT->IDR & (1<<0)); //Checking the status of bit 0
   return pinStatus;
}

uint32_t g1=0; //global variable
uint16_t g2;
uint8_t g3;
LED obj;
int main()
{

   LED* L1=new LED;
  // LED* L2=new LED;


   while(1)
   {

	      L1->Led_green_toggle();

	      try
	      {
	      if(L1->Switch_state())
	      {
	    	  throw obj;// Integer variable
	      }
	      }
	      catch(const uint8_t& g)
	    	     	   	 {
	    	     	   		 L1->Led_orange_toggle();
	    	     	   		 for(uint32_t i=0 ; i < 300000 ; i++ );
	    	     	   		 delete L1;

	    	     	   	 }

	     catch(const uint16_t& g)
	     	   	 {
	     	   		 L1->Led_blue_toggle();
	     	   		 for(uint32_t i=0 ; i < 300000 ; i++ );
	     	   		 delete L1;

	     	   	 }

	   	 catch(const uint32_t& g)
	   	 {
	   		 L1->Led_red_toggle();
	   		 for(uint32_t i=0 ; i < 300000 ; i++ );
	   		 delete L1;

	   	 }
	   	 catch(const LED& g)
	   		   	 {
	   		   		 L1->Led_red_toggle();
	   		   	     L1->Led_blue_toggle();
	   		   	     L1->Led_orange_toggle();
	   		   		 for(uint32_t i=0 ; i < 300000 ; i++ );
	   		   		 delete L1;

	   		   	 }

	   	 catch(...)// catch all
	   		   	   {
	   		       delete L1;

	   		   	   }


	 }


	return 0;

}



