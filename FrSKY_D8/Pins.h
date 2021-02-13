#define	BIND_pin		PA0
#define	LED_pin			PA1
#define	LED2_pin		PA2
//
#define	PPM_pin			PA8								//PPM  5V tolerant
//
#define	S1_pin			PA4								//Dial switch pins
#define	S2_pin			PA5
#define	S3_pin			PA6
#define	S4_pin			PA7
//
#define	RND_pin			PB0
//
#define	PE1_pin			PB4								//PE1
#define	PE2_pin			PB5								//PE2
//CS pins
#define	CC25_CSN_pin	PA15								//CC2500
#define SPI_CSN_pin		PA15
//SPI pins	
#define	SCK_pin			PB13							//SCK
#define	SDO_pin			PB14							//MISO
#define	SDI_pin			PB15							//MOSI
//
#define	TX_INV_pin		PB3
#define	RX_INV_pin		PB1
//
#define	PE1_on  		digitalWrite(PE1_pin,HIGH)
#define	PE1_off		 	digitalWrite(PE1_pin,LOW)
//
#define	PE2_on  		digitalWrite(PE2_pin,HIGH)
#define	PE2_off 		digitalWrite(PE2_pin,LOW)

#define	SCK_on			digitalWrite(SCK_pin,HIGH)
#define	SCK_off			digitalWrite(SCK_pin,LOW)

#define	SDI_on			digitalWrite(SDI_pin,HIGH)
#define	SDI_off			digitalWrite(SDI_pin,LOW)

#define	SDI_1			(digitalRead(SDI_pin)==HIGH)
#define	SDI_0			(digitalRead(SDI_pin)==LOW)

#define	CC25_CSN_on		digitalWrite(CC25_CSN_pin,HIGH)
#define	CC25_CSN_off	digitalWrite(CC25_CSN_pin,LOW)

#define	SPI_CSN_on		digitalWrite(SPI_CSN_pin,HIGH)
#define	SPI_CSN_off		digitalWrite(SPI_CSN_pin,LOW)

#define	SDO_1			(digitalRead(SDO_pin)==HIGH)
#define	SDO_0			(digitalRead(SDO_pin)==LOW)

#define	TX_INV_on		digitalWrite(TX_INV_pin,HIGH)
#define	TX_INV_off		digitalWrite(TX_INV_pin,LOW)

#define	RX_INV_on		digitalWrite(RX_INV_pin,HIGH)
#define	RX_INV_off		digitalWrite(RX_INV_pin,LOW)

#define	LED_on			digitalWrite(LED_pin,HIGH)
#define	LED_off			digitalWrite(LED_pin,LOW)
#define	LED_toggle		digitalWrite(LED_pin ,!digitalRead(LED_pin))
#define	LED_output		pinMode(LED_pin,OUTPUT)
#define	IS_LED_on		( digitalRead(LED_pin)==HIGH)

//iRangeX modules have a second LED
#define	LED2_on			digitalWrite(LED2_pin,HIGH)
#define	LED2_off		digitalWrite(LED2_pin,LOW)
#define	LED2_toggle		digitalWrite(LED2_pin ,!digitalRead(LED2_pin))
#define	LED2_output		pinMode(LED2_pin,OUTPUT)
#define	IS_LED2_on		( digitalRead(LED2_pin)==HIGH)

#define BIND_SET_INPUT		pinMode(BIND_pin,INPUT)
#define BIND_SET_PULLUP		digitalWrite(BIND_pin,HIGH)	
#define BIND_SET_OUTPUT		pinMode(BIND_pin,OUTPUT)
#define IS_BIND_BUTTON_on	(digitalRead(BIND_pin)==LOW)

#define DEBUG_PIN_on
#define DEBUG_PIN_off
#define DEBUG_PIN_toggle

#define	cli() 			noInterrupts()
#define	sei() 			interrupts()
#define	delayMilliseconds(x) delay(x)

//*******************
//***    Timer    ***
//*******************
#define OCR1A TIMER2_BASE->CCR1
#define TCNT1 TIMER2_BASE->CNT
#define TIFR1 TIMER2_BASE->SR
#define OCF1A_bm TIMER_SR_CC1IF
#define UDR0 USART2_BASE->DR
#define UCSR0B USART2_BASE->CR1
#define RXCIE0 USART_CR1_RXNEIE_BIT
#define TXCIE0 USART_CR1_TXEIE_BIT
//#define TIFR1 TIMER2_BASE->SR

//*******************
//***    EEPROM   ***
//*******************
#define EE_ADDR uint16
#define eeprom_write_byte EEPROM.write
#define eeprom_read_byte EEPROM.read
