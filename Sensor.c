#include "Sensor.h"

/*----------------------------------------------*/
/*-------------------LED------------------------*/
/*----------------------------------------------*/
const uint32_t MaskLED[] = {1UL << 5, 1UL<<29};		//tablica

void InitLED(void)		//Inicjalizacja portów GPIO
{
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;		//Podawanie zegara
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;		// |= ustaw bity, =1 w ..PORTE_MASK pozostawiajac reszte bez zmian
	PORTD->PCR[5] = PORT_PCR_MUX(1UL);		//MUX- Multiplekser
	PORTE->PCR[29]= PORT_PCR_MUX(1UL);
	PTD->PDDR |= MaskLED[GreenLED];		//ustaw Green Led jako wyjscie
	PTE->PDDR |= MaskLED[RedLED];
	
	PTD->PSOR |= MaskLED[GreenLED];	//Wylacz ledy
	PTE->PSOR |= MaskLED[RedLED];
}

void BlinkLED(void)
{
	uint32_t i = 0;

	PTD->PTOR = MaskLED[GreenLED];
	PTE->PTOR = MaskLED[RedLED];
	for(i = 0; i < 3000000; i++){};
}

/*----------------------------------------------*/
/*------------------I2C-------------------------*/
/*----------------------------------------------*/
void InitI2C0(void)		//Inicjalizuje I2C0
{
	 SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;		
		
	 PORTE->PCR[18] |= PORT_PCR_MUX(4);		//port PTE 18 to i2c0 SDA
   PORTE->PCR[19] |= PORT_PCR_MUX(4);		//PTE 19 - I2C0 SCL
		
	 I2C0->F  |= 0x14; // baudrate:	mul=1,ICR=0x14 		??? Dlaczego taka wartosc?
   I2C0->C1 |= I2C_C1_IICEN_MASK; // enable I2C
}

void i2c_set_tx_mode(void)		
{
    I2C0->C1 |= I2C_C1_TX_MASK;

}
void i2c_set_rx_mode(void)
{
    I2C0->C1 &= ~I2C_C1_TX_MASK;
}

void i2c_set_slave_mode(void)
{
    I2C0->C1  &= ~I2C_C1_MST_MASK;
}
void i2c_set_master_mode(void)
{
    I2C0->C1  |=  I2C_C1_MST_MASK;
}

// i2c general

void i2c_give_nack(void)
{
    I2C0->C1 |= I2C_C1_TXAK_MASK;
}
void i2c_give_ack(void)
{
    I2C0->C1 &= ~I2C_C1_TXAK_MASK;
}
void i2c_repeated_start(void)
{
    I2C0->C1     |= 0x04;
}
void i2c_write_byte(uint8_t data)
{
    I2C0->D = data;
}
uint8_t i2c_read_byte(void)
{
    return I2C0->D;
}
void i2c_start(void)
{
    i2c_set_master_mode();
    i2c_set_tx_mode();
}
void i2c_stop(void)
{
    i2c_set_slave_mode();
    i2c_set_rx_mode();
}
void i2c_wait(void)					
{
    
    while((I2C0->S & I2C_S_IICIF_MASK)==0);		//Sprawdz czy wystapilo przerwanie
   
    I2C0->S |= I2C_S_IICIF_MASK;		//wyzeruj flage przerwania
}
uint16_t i2c_get_ack(void)
{
    if((I2C0->S & I2C_S_RXAK_MASK) == 0)	//sprawdz czy ACK/NACK zostalo otrzymane
        return 1;
    else
        return 0;
}

// this delay is very important, it may cause w-r operation failure.
static void pause(void)
{
    int n;
    for(n=0; n<40; n++)
        {}
}
uint8_t APDS9960_read_reg(uint8_t addr)		//cala sekwencja czytania z sensora
{
    uint8_t result;

    i2c_start();
    i2c_write_byte(APDS9960_I2C_ADDRESS | I2C_WRITE);	//wyslij adres i2c sensora, ostatni bit=1(WRITE)
    
    i2c_wait();
    i2c_get_ack();		

    i2c_write_byte(addr);			//adres rejestru
    i2c_wait();
    i2c_get_ack();

    i2c_repeated_start();
    i2c_write_byte(APDS9960_I2C_ADDRESS | I2C_READ);		// znowu adres sensora, teraz z ostatnim bitem =0 (READ)
    i2c_wait();
    i2c_get_ack();

    i2c_set_rx_mode();

    i2c_give_nack();
    result = i2c_read_byte();
    i2c_wait();

    i2c_stop();
    result = i2c_read_byte();
    pause();
    
		return result;
}

void APDS9960_write_reg(uint8_t addr, uint8_t data)		//pisanie do sensora
{
    i2c_start();

    i2c_write_byte(APDS9960_I2C_ADDRESS|I2C_WRITE);		//adres i ostatni bit =1
    i2c_wait();
    i2c_get_ack();

    i2c_write_byte(addr);			//adres rejestru
    i2c_wait();
    i2c_get_ack();

    i2c_write_byte(data);		//dane
    i2c_wait();
    i2c_get_ack();

    i2c_stop();
    pause();
}

/*-----------------------------------------------*/
/*------------------UART0------------------------*/
/*----------------------------------------------*/
void InitUART0(void)
{
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	PORTA->PCR[1] = PORT_PCR_MUX(2);
	PORTA->PCR[2] = PORT_PCR_MUX(2);
	
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(2);		//Wybrac np. OSCERCLK (w naszym przypadku 8MHz - Asynch Module Clock),
	
	UART0->C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);	//zablokowac nadajnik i odbiornik
	UART0->C4 |= UART0_C4_OSR(31);	//ustawic wartosc dzielnika próbkowania nadmiarowego (najlepiej 31),
	
	UART0->BDH = UART_BDH_SBR(0);	//ustawic 13-bitowa wartosc dzielnika, bedacego zródlem zegara dla odbiornika i nadajnika.
	UART0->BDL = UART_BDL_SBR(26);		//czy taka ma byc wartosc (dla 9600 bit/s)???
	
	UART0->BDH &= ~(UART_BDH_SBNS_MASK);	//ustawic np. jeden bit stopu,
	
	UART0->C1 &= ~(UART_C1_M_MASK | UART_C1_PE_MASK);	//ustawic dlugosc danej np. na 8 bitów oraz brak sprzetowej obslugi sprawdzania parzystosci,
	
	UART0->C2	|= (UART_C2_RE_MASK | UART_C2_TE_MASK);//wlaczyc nadajnik i odbiornik
}

void UART0Transmit(uint8_t data)
{
	while(!(UART0->S1 & UART_S1_TDRE_MASK));
	UART0->D = data;
	
}

uint8_t UART0Receive(void)
{
	while (!(UART0->S1 & UART_S1_RDRF_MASK));
	return UART0->D;
}

/*-----------------------------------------------*/
/*------------------APDS-9960--------------------*/
/*-----------------------------------------------*/
void APDS9960_init()		//ustaw domyslne wartosci
{	
	//uint8_t temp;
	
	setMode(ALL,OFF);		//wylacz wszystkie funkcjonalnosci
	
//	temp= APDS9960_read_reg(APDS9960_ENABLE);
//	UART0Transmit(temp);
	
	APDS9960_write_reg(APDS9960_ATIME, DEFAULT_ATIME);	//ustaw domyslny czas integracji ADC
	APDS9960_write_reg(APDS9960_WTIME, DEFAULT_WTIME);		//domyslny czas oczekiwania 
	APDS9960_write_reg(APDS9960_CONFIG1, DEFAULT_CONFIG1);	//rejestr konfiguracji
	//setAmbientLightGain(DEFAULT_AGAIN);						//wzmocnienie (w rejstrze CONTROL)
	APDS9960_write_reg(APDS9960_CONFIG2, DEFAULT_CONFIG2);	//rejestr konfiguracji 2
}
void setMode(uint8_t mode, uint8_t enable)
{
	uint8_t reg_val;
	

	enable = enable & 0x01;
	if( mode <= 6 ) {
		if (enable) {
			reg_val |= (1 << mode);
		} else {
			reg_val &= ~(1 << mode);
		}
	} else if( mode == ALL ) {
		if (enable) {
			reg_val = 0x7F;
		} else {
			reg_val = 0x00;
		}
	}
	APDS9960_write_reg(APDS9960_ENABLE, reg_val);	//rejestr ENABLE
}
void enableLightSensor()
{
	uint8_t temp;
	
	setAmbientLightGain(DEFAULT_AGAIN);		//ustawia wzmocnienie
	setAmbientLightIntEnable(0);			//wylacza przerwanie
	enablePower();										// ustaw bit PON w rejestrze ENABLE
	//setMode(AMBIENT_LIGHT,1);
	temp= APDS9960_read_reg(APDS9960_ENABLE);	// ustaw bit AEN - ALS Enable
	temp |= 2;
	APDS9960_write_reg(APDS9960_ENABLE, temp);
	
//	temp=APDS9960_read_reg(APDS9960_ENABLE);
//	UART0Transmit(temp);
}
void enablePower()
{
	uint8_t temp;
	
	temp= APDS9960_read_reg(APDS9960_ENABLE);
	temp |= 1;
	APDS9960_write_reg(APDS9960_ENABLE, temp);
	//setMode(POWER,1);
	
//	temp= APDS9960_read_reg(APDS9960_ENABLE);
//	UART0Transmit(temp);
}
void setAmbientLightGain(uint8_t drive)
{
	uint8_t val;
	
	val=APDS9960_read_reg(APDS9960_CONTROL);
	
	drive &= 3;
	val &= 252;
	val |= drive;
	//wpisz do CONTROL
	APDS9960_write_reg(APDS9960_CONTROL,val);
}

void setAmbientLightIntEnable(uint8_t enable)
{
	uint8_t val;
	
	val=APDS9960_read_reg(APDS9960_ENABLE);
	
	/* Set bits in register to given value */
	enable &= 1;
	enable = enable << 4;
	val &= 239;
	val |= enable;
	
	APDS9960_write_reg(APDS9960_ENABLE,val);
}
/* Funkcje odczytujace rejestry z danymi */
uint16_t readAmbientLight() 	//Najpierw czytamy mlodszy bajt nastepnie starszy - odczytanie mlodszego skutkujemy automtyczna generacja starszego
{
	uint8_t val_byte;
	uint16_t val = 0;
	
	val_byte=APDS9960_read_reg(APDS9960_CDATAL);	/* mlodszy bajt */
	val=val_byte;
	
	val_byte=APDS9960_read_reg(APDS9960_CDATAH);	/* starszy */
	val=val+((uint16_t)val_byte<<8);
	
	return val;
}
uint16_t readRedLight()
{
	uint8_t val_byte;
	uint16_t val = 0;
	
	val_byte=APDS9960_read_reg(APDS9960_RDATAL);	
	val=val_byte;
	
	val_byte=APDS9960_read_reg(APDS9960_RDATAH);	
	val=val+((uint16_t)val_byte<<8);
	
	return val;
}
uint16_t readGreenLight()
{
	uint8_t val_byte;
	uint16_t val = 0;
	
	val_byte=APDS9960_read_reg(APDS9960_GDATAL);	
	val=val_byte;
	
	val_byte=APDS9960_read_reg(APDS9960_GDATAH);	
	val=val+((uint16_t)val_byte<<8);
	
	return val;
}
uint16_t readBlueLight()
{
	uint8_t val_byte;
	uint16_t val = 0;
	
	val_byte=APDS9960_read_reg(APDS9960_BDATAL);	
	val=val_byte;
	
	val_byte=APDS9960_read_reg(APDS9960_BDATAH);	
	val=val+((uint16_t)val_byte<<8);
	
	return val;
}

void delay(int n)	//prymitywne opoznienie
{
	int i;
	
	for(i=0;i<n;i++)
	{
		
	}
}
