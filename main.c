#include "MKL46Z4.h"
#include "Sensor.h"



int main(void)
{
	uint8_t Dana;
	
	uint16_t clear_light;
	uint16_t red_light;
	uint16_t green_light;
	uint16_t blue_light;
	//uint8_t adres;
	
	InitI2C0();
	InitUART0();
	InitLED();
	
	APDS9960_init();			//inicjalizacja domyslnymi wartosciami
	enableLightSensor();		//ustaw funkcjonalnosc sensora na czujnik kolorow
	
//	adres = APDS9960_read_reg(APDS9960_DEVICE_ID);
//	UART0Transmit(adres);
//	
//	if(adres == 0xAB || adres==0x9C){
//	
//		while(1){
//			BlinkLED();
//		}
//	}
	while(1)
	{
		delay(5000000);				//zaczekaj na zebranie danych
	
		clear_light=readAmbientLight();			//otaczajace swiatlo (Clear)
		red_light=readRedLight();							//czerwone
		green_light=readGreenLight();					//zielone
		blue_light=readBlueLight();						//niebieskie
		
		//Wyslij przez UART w kolejnosci: otaczajace, czerwone, zielone, niebieskie
		Dana=(clear_light&255);		//najpierw mlodszy bajt
		UART0Transmit(Dana);
		Dana=((clear_light & 0xFF00)>>8);		//starszy bajt
		UART0Transmit(Dana);
		
		Dana=(red_light&255);		
		UART0Transmit(Dana);
		Dana=((red_light & 0xFF00)>>8);		
		UART0Transmit(Dana);	

		Dana=(green_light&255);		
		UART0Transmit(Dana);
		Dana=((green_light & 0xFF00)>>8);		
		UART0Transmit(Dana);
	
		Dana=(blue_light&255);		
		UART0Transmit(Dana);
		Dana=((blue_light & 0xFF00)>>8);		
		UART0Transmit(Dana);
		
		UART0Transmit('c');
	}
		
		
	return 0;
}
