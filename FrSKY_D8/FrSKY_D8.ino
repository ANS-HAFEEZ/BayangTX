#include <avr/pgmspace.h>

#define DEBUG_SERIAL	// Only for STM32_BOARD, compiled with Upload method "Serial"->usart1, "STM32duino bootloader"->USB serial
#define STM32_BOARD
#include "Multiprotocol.h"
#include "_Config.h"
#include "Pins.h"
#include "TX_Def.h"
#include "Validate.h"

#include <libmaple/usart.h>
#include <libmaple/timer.h>
#include <SPI.h>
#include <EEPROM.h> 
HardwareTimer HWTimer2(2);

void PPM_decode();
extern "C"
{
  void __irq_usart2(void);
  void __irq_usart3(void);
}

//Global constants/variables
uint32_t MProtocol_id;//tx id,
uint32_t MProtocol_id_master;
uint32_t blink=0,last_signal=0;
//
uint16_t counter;
uint8_t  channel;
uint8_t  packet[50];

#define NUM_CHN 8
// Servo data
uint16_t Channel_data[NUM_CHN];
uint8_t  Channel_AUX;

// Protocol variables
uint8_t  rx_tx_addr[5];
uint8_t  rx_id[5];
uint8_t  phase;
uint16_t bind_counter;
uint8_t  bind_phase;
uint8_t  binding_idx;
uint16_t packet_period;
uint8_t  packet_count;
uint8_t  packet_sent;
uint8_t  packet_length;
uint8_t  hopping_frequency[50];
uint8_t  *hopping_frequency_ptr;
uint8_t  hopping_frequency_no=0;
uint8_t  rf_ch_num;
uint8_t  throttle, rudder, elevator, aileron;
uint8_t  flags;
uint16_t crc;
uint8_t  crc8;
uint16_t seed;
uint16_t failsafe_count;
uint16_t state;
uint8_t  len;
uint8_t  armed, arm_flags, arm_channel_previous;
uint8_t  num_ch;
uint32_t pps_timer;
uint16_t pps_counter;

uint8_t calData[50];

//Channel mapping for protocols
uint8_t CH_AETR[]={AILERON, ELEVATOR, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
uint8_t CH_TAER[]={THROTTLE, AILERON, ELEVATOR, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
//uint8_t CH_RETA[]={RUDDER, ELEVATOR, THROTTLE, AILERON, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
uint8_t CH_EATR[]={ELEVATOR, AILERON, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};

// Mode_select variables
uint8_t mode_select;
uint8_t protocol_flags=0,protocol_flags2=0,protocol_flags3=0;


// PPM variable
volatile uint16_t PPM_data[NUM_CHN];
volatile uint8_t  PPM_chan_max=0;
uint32_t chan_order=0;

//Serial protocol
uint8_t sub_protocol;
uint8_t protocol;
uint8_t option;
uint8_t cur_protocol[3];
uint8_t prev_option;
uint8_t prev_power=0xFD; // unused power value
uint8_t  RX_num;

//Serial RX variables
#define BAUD 100000
#define RXBUFFER_SIZE 36	// 26+1+9
volatile uint8_t rx_buff[RXBUFFER_SIZE];
volatile uint8_t rx_ok_buff[RXBUFFER_SIZE];
volatile bool discard_frame = false;
volatile uint8_t rx_idx=0, rx_len=0;


// Telemetry
#define TELEMETRY_BUFFER_SIZE 32
uint8_t packet_in[TELEMETRY_BUFFER_SIZE];//telemetry receiving packets

// Callback
typedef uint16_t (*void_function_t) (void);//pointer to a function with no parameters which return an uint16_t integer
void_function_t remote_callback = 0;

// Init
void setup()
{
	// Setup diagnostic uart before anything else
	#ifdef DEBUG_SERIAL
		Serial.begin(115200,SERIAL_8N1);

		// Wait up to 30s for a serial connection; double-blink the LED while we wait
		unsigned long currMillis = millis();
		unsigned long initMillis = currMillis;
		pinMode(LED_pin,OUTPUT);
		LED_off;
		while (!Serial && (currMillis - initMillis) <= 3000) {
			LED_on;
			delay(100);
			LED_off;
			delay(100);
			LED_on;
			delay(100);
			LED_off;
			delay(500);
			currMillis = millis();
		}

		delay(50);  // Brief delay for FTDI debugging
		debugln("Multiprotocol version: %d.%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION, VERSION_PATCH_LEVEL);
	#endif

  afio_cfg_debug_ports(AFIO_DEBUG_NONE);
  pinMode(LED_pin,OUTPUT);
  pinMode(LED2_pin,OUTPUT);
  
  pinMode(CC25_CSN_pin,OUTPUT);
  pinMode(SPI_CSN_pin,OUTPUT);
  pinMode(PE1_pin,OUTPUT);
  pinMode(PE2_pin,OUTPUT);
  pinMode(TX_INV_pin,OUTPUT);
  pinMode(RX_INV_pin,OUTPUT);

  pinMode(BIND_pin,INPUT_PULLUP);
  pinMode(PPM_pin,INPUT);
  pinMode(S1_pin,INPUT_PULLUP);       // dial switch
  pinMode(S2_pin,INPUT_PULLUP);
  pinMode(S3_pin,INPUT_PULLUP);
  pinMode(S4_pin,INPUT_PULLUP);
  
  pinMode(RND_pin, INPUT_ANALOG);      // set up PB0 pin for analog input

    //Timers
  init_HWTimer();               //0.5us

    //Read module flash size
  #ifndef DISABLE_FLASH_SIZE_CHECK
    unsigned short *flashSize = (unsigned short *) (0x1FFFF7E0);// Address register 
    debugln("Module Flash size: %dKB",(int)(*flashSize & 0xffff));
    if((int)(*flashSize & 0xffff) < MCU_EXPECTED_FLASH_SIZE)  // Not supported by this project
      while (true) { //SOS
        for(uint8_t i=0; i<3;i++){
            LED_on;
            delay(100);
            LED_off;
            delay(100);
        }
        for(uint8_t i=0; i<3;i++){
          LED_on;
          delay(500);
          LED_off;
          delay(100);
        }
        for(uint8_t i=0; i<3;i++){
          LED_on;
          delay(100);
          LED_off;
          delay(100);
        }
        LED_off;
        delay(1000);
      }
   #endif
      
	LED2_on;
  CC25_CSN_on;
//  SPI_CSN_on;
	//	Set SPI lines
	initSPI2();

	//Wait for every component to start
	delayMilliseconds(100);
	
	// Read status of bind button
	if( IS_BIND_BUTTON_on )
	{
		BIND_BUTTON_FLAG_on;	// If bind button pressed save the status
		BIND_IN_PROGRESS;		// Request bind
	}
	else
		BIND_DONE;

	// Read status of mode select binary switch
	// after this mode_select will be one of {0000, 0001, ..., 1111}
	#ifndef ENABLE_PPM
		mode_select = MODE_SERIAL ;	// force serial mode
	#elif defined STM32_BOARD
		mode_select= 0x0F -(uint8_t)(((GPIOA->regs->IDR)>>4)&0x0F);
	#else
		mode_select =
			((PROTO_DIAL1_ipr & _BV(PROTO_DIAL1_pin)) ? 0 : 1) + 
			((PROTO_DIAL2_ipr & _BV(PROTO_DIAL2_pin)) ? 0 : 2) +
			((PROTO_DIAL3_ipr & _BV(PROTO_DIAL3_pin)) ? 0 : 4) +
			((PROTO_DIAL4_ipr & _BV(PROTO_DIAL4_pin)) ? 0 : 8);
	#endif
	mode_select=10;
  debugln("Protocol selection switch reads as %d", mode_select);

  uint8_t bank=bank_switch();

	// Set default channels' value
	for(uint8_t i=0;i<NUM_CHN;i++)
		Channel_data[i]=1024;
	Channel_data[THROTTLE]=0;	//0=-125%, 204=-100%

		// Set default PPMs' value
		for(uint8_t i=0;i<NUM_CHN;i++)
			PPM_data[i]=PPM_MAX_100+PPM_MIN_100;
		PPM_data[THROTTLE]=PPM_MIN_100*2;
	
	// Update LED
	LED_off;
	LED_output;

	//Init RF modules
	modules_reset();

	#ifdef STM32_BOARD
		uint32_t seed=0;
		for(uint8_t i=0;i<4;i++)
		#ifdef RND_pin
			seed=(seed<<8) | (analogRead(RND_pin)& 0xFF);
		#else
		//TODO find something to randomize...
			seed=(seed<<8);
		#endif
		randomSeed(seed);
	#else
		//Init the seed with a random value created from watchdog timer for all protocols requiring random values
		randomSeed(random_value());
	#endif


	// Read or create protocol id
	MProtocol_id_master=random_id(EEPROM_ID_OFFSET,false);
	debugln("Module Id: %lx", MProtocol_id_master);
  const PPM_Parameters *PPM_prot_line=&PPM_prot[bank*14+mode_select-1];
	
	protocol		=	PPM_prot_line->protocol;
	cur_protocol[1] =	protocol;
	sub_protocol   	=	PPM_prot_line->sub_proto;
	RX_num			=	PPM_prot_line->rx_num;
	chan_order		=	PPM_prot_line->chan_order;

		//Forced frequency tuning values for CC2500 protocols
	#if defined(FORCE_FRSKYD_TUNING) && defined(FRSKYD_CC2500_INO)
		if(protocol==PROTO_FRSKYD) 
			option			=	FORCE_FRSKYD_TUNING;		// Use config-defined tuning value for FrSkyD
		else
	#endif
	
	#if defined(FORCE_FRSKYX_TUNING) && defined(FRSKYX_CC2500_INO)
		if(protocol==PROTO_FRSKYX || protocol==PROTO_FRSKYX2)
			option			=	FORCE_FRSKYX_TUNING;		// Use config-defined tuning value for FrSkyX
		else
	#endif 

	option			=	(uint8_t)PPM_prot_line->option;	// Use radio-defined option value

	if(PPM_prot_line->power)		POWER_FLAG_on;
	if(PPM_prot_line->autobind)
	{
		AUTOBIND_FLAG_on;
		BIND_IN_PROGRESS;	// Force a bind at protocol startup
	}
	protocol_init();
  attachInterrupt(PPM_pin,PPM_decode,FALLING);
	debugln("Init complete");
	LED2_on;
}

// Main
// Protocol scheduler
void loop(){ 
	uint16_t next_callback, diff;
	uint8_t count=0;

	while(1){
		while(remote_callback==0 || IS_WAIT_BIND_on || IS_INPUT_SIGNAL_off){
			if(!Update_All()){
				cli();								// Disable global int due to RW of 16 bits registers
				OCR1A=TCNT1;						// Callback should already have been called... Use "now" as new sync point.
				sei();								// Enable global int
			}
		}
		TX_MAIN_PAUSE_on;
		next_callback=remote_callback()<<1;
		TX_MAIN_PAUSE_off;
		
		cli();										// Disable global int due to RW of 16 bits registers
		OCR1A+=next_callback;						// Calc when next_callback should happen
		TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC1IF;  // Clear Timer2/Comp1 interrupt flag		
		diff=OCR1A-TCNT1;							// Calc the time difference
		sei();										// Enable global int
		if((diff&0x8000) && !(next_callback&0x8000))
		{ // Negative result=callback should already have been called... 
			debugln("Short CB:%d",next_callback);
		}
		else
		{
			if(IS_RX_FLAG_on || IS_PPM_FLAG_on)
			{ // Serial or PPM is waiting...
				if(++count>10)
				{ //The protocol does not leave enough time for an update so forcing it
					count=0;
					debugln("Force update");
					Update_All();
				}
			}
      while((TIMER2_BASE->SR & TIMER_SR_CC1IF )==0){
				if(diff>900*2)
				{	//If at least 1ms is available update values 
					if((diff&0x8000) && !(next_callback&0x8000))
					{//Should never get here...
						debugln("!!!BUG!!!");
						break;
					}
					count=0;
					Update_All();
					#ifdef DEBUG_SERIAL
						if(TIMER2_BASE->SR & TIMER_SR_CC1IF )
							debugln("Long update");
					#endif
					if(remote_callback==0)
						break;
					cli();							// Disable global int due to RW of 16 bits registers
					diff=OCR1A-TCNT1;				// Calc the time difference
					sei();							// Enable global int
				}
			}
		}			
	}
}

bool Update_All(){
  if(mode_select!=MODE_SERIAL && IS_PPM_FLAG_on)		// PPM mode and a full frame has been received
	{
	  uint32_t chan_or=chan_order;
		uint8_t ch;		
		uint8_t channelsCount = PPM_chan_max;
						
		for(uint8_t i=0;i<channelsCount;i++)
		{ // update servo data without interrupts to prevent bad read
		  uint16_t val;
			cli();										// disable global int
			val = PPM_data[i];
			sei();										// enable global int
			val=map16b(val,PPM_MIN_100*2,PPM_MAX_100*2,CHANNEL_MIN_100,CHANNEL_MAX_100);
			if(val&0x8000) 					val=CHANNEL_MIN_125;
				else if(val>CHANNEL_MAX_125)	val=CHANNEL_MAX_125;
				if(chan_or)
				{
					ch=chan_or>>28;
					if(ch)
						Channel_data[ch-1]=val;
					else
						Channel_data[i]=val;
					chan_or<<=4;
				}
				else
					Channel_data[i]=val;
			}
			PPM_FLAG_off;									// wait for next frame before update
			#ifdef FAILSAFE_ENABLE
				PPM_failsafe();
			#endif
			update_channels_aux();
			INPUT_SIGNAL_on;								// valid signal received
			last_signal=millis();
		}
	update_led_status();
	#ifdef ENABLE_BIND_CH
		if(IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_off && Channel_data[BIND_CH-1]>CHANNEL_MAX_COMMAND)
		{ // Autobind is on and BIND_CH went up
			CHANGE_PROTOCOL_FLAG_on;							//reload protocol
			BIND_IN_PROGRESS;									//enable bind
			BIND_CH_PREV_on;
		}
		if(IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_on && Channel_data[BIND_CH-1]<CHANNEL_MIN_COMMAND)
		{ // Autobind is on and BIND_CH went down
			BIND_CH_PREV_off;
			//Request protocol to terminate bind
			#if defined(FRSKYD_CC2500_INO) || defined(FRSKYL_CC2500_INO) || defined(FRSKYX_CC2500_INO) || defined(FRSKYV_CC2500_INO) || defined(AFHDS2A_A7105_INO) || defined(FRSKYR9_SX1276_INO)
			if(protocol==PROTO_FRSKYD || protocol==PROTO_FRSKYL || protocol==PROTO_FRSKYX || protocol==PROTO_FRSKYX2 || protocol==PROTO_FRSKYV || protocol==PROTO_AFHDS2A || protocol==PROTO_FRSKY_R9)
				BIND_DONE;
			else
			#endif
			if(bind_counter>2)
				bind_counter=2;
		}
	#endif //ENABLE_BIND_CH
	return false;
}

#if defined(FAILSAFE_ENABLE) && defined(ENABLE_PPM)
void PPM_failsafe()
{
	static uint8_t counter=0;
	
	if(IS_BIND_IN_PROGRESS || IS_FAILSAFE_VALUES_on) 	// bind is not finished yet or Failsafe already being sent
		return;
	BIND_SET_INPUT;
	BIND_SET_PULLUP;
	if(IS_BIND_BUTTON_on)
	{// bind button pressed
		counter++;
		if(counter>227)
		{ //after 5s with PPM frames @22ms
			counter=0;
			for(uint8_t i=0;i<NUM_CHN;i++)
				Failsafe_data[i]=Channel_data[i];
			FAILSAFE_VALUES_on;
		}
	}
	else
		counter=0;
	BIND_SET_OUTPUT;
}
#endif

// Update channels direction and Channel_AUX flags based on servo AUX positions
static void update_channels_aux(void)
{
	//Calc AUX flags
	Channel_AUX=0;
	for(uint8_t i=0;i<8;i++)
		if(Channel_data[CH5+i]>CHANNEL_SWITCH)
			Channel_AUX|=1<<i;
}

// Update led status based on binding and serial
static void update_led_status(void){
	if(IS_INPUT_SIGNAL_on)
		if(millis()-last_signal>70){
			INPUT_SIGNAL_off;							//no valid signal (PPM or Serial) received for 70ms
			debugln("No input signal");
		}
	if(blink<millis()){
		if(IS_INPUT_SIGNAL_off){
			if(mode_select==MODE_SERIAL)
				blink+=BLINK_SERIAL_TIME;				//blink slowly if no valid serial input
			else
				blink+=BLINK_PPM_TIME;					//blink more slowly if no valid PPM input
		}
		else
			if(remote_callback == 0){ // Invalid protocol
				if(IS_LED_on)							//flash to indicate invalid protocol
					blink+=BLINK_BAD_PROTO_TIME_LOW;
				else
					blink+=BLINK_BAD_PROTO_TIME_HIGH;
			}
			else{
				if(IS_WAIT_BIND_on){
					if(IS_LED_on)							//flash to indicate WAIT_BIND
						blink+=BLINK_WAIT_BIND_TIME_LOW;
					else
						blink+=BLINK_WAIT_BIND_TIME_HIGH;
				}
				else{
					if(IS_BIND_DONE)
						LED_off;							//bind completed force led on
					blink+=BLINK_BIND_TIME;					//blink fastly during binding
				}
			}
		LED_toggle;
	}
}

#ifdef ENABLE_PPM
uint8_t bank_switch(void)
{
	uint8_t bank=eeprom_read_byte((EE_ADDR)EEPROM_BANK_OFFSET);
	if(bank>=NBR_BANKS)
	{ // Wrong number of bank
		eeprom_write_byte((EE_ADDR)EEPROM_BANK_OFFSET,0x00);	// set bank to 0
		bank=0;
	}
	debugln("Using bank %d", bank);

	phase=3;
	uint32_t check=millis();
	blink=millis();
	while(mode_select==15)
	{ //loop here if the dial is on position 15 for user to select the bank
		if(blink<millis())
		{
			switch(phase & 0x03)
			{ // Flash bank number of times
				case 0:
					LED_on;
					blink+=BLINK_BANK_TIME_HIGH;
					phase++;
					break;
				case 1:
					LED_off;
					blink+=BLINK_BANK_TIME_LOW;
					phase++;
					break;
				case 2:
					if( (phase>>2) >= bank)
					{
						phase=0;
						blink+=BLINK_BANK_REPEAT;
					}
					else
						phase+=2;
					break;
				case 3:
					LED_output;
					LED_off;
					blink+=BLINK_BANK_TIME_LOW;
					phase=0;
					break;
			}
		}
		if(check<millis())
		{
			bool test_bind=IS_BIND_BUTTON_on;
			if( test_bind )
			{	// Increase bank
				LED_on;
				bank++;
				if(bank>=NBR_BANKS)
					bank=0;
				eeprom_write_byte((EE_ADDR)EEPROM_BANK_OFFSET,bank);
				debugln("Using bank %d", bank);
				phase=3;
				blink+=BLINK_BANK_REPEAT;
				check+=2*BLINK_BANK_REPEAT;
			}
			check+=1;
		}
	}
	return bank;
}
#endif

// Protocol start
static void protocol_init(){
	static uint16_t next_callback;
	if(IS_WAIT_BIND_off){
		remote_callback = 0;			// No protocol
		next_callback=0;				// Default is immediate call back
		LED_off;						// Led off during protocol init
		modules_reset();				// Reset all modules

		binding_idx=0;
		
		//Set global ID and rx_tx_addr
		MProtocol_id = RX_num + MProtocol_id_master;
		set_rx_tx_addr(MProtocol_id);
		
		#ifdef FAILSAFE_ENABLE
			FAILSAFE_VALUES_off;
		#endif
		DATA_BUFFER_LOW_off;
		
		blink=millis();

		PE1_off;	//antenna RF2
		PE2_on;
		next_callback = initFrSky_2way();
		remote_callback = ReadFrSky_2way;
	
//    next_callback = initFrSkyX();
//    remote_callback = ReadFrSkyX;
		debugln("Protocol selected: %d, sub proto %d, rxnum %d, option %d", protocol, sub_protocol, RX_num, option);
		#ifdef MULTI_NAMES
			uint8_t index=0;
			while(multi_protocols[index].protocol != 0)
			{
				if(multi_protocols[index].protocol==protocol)
				{
					multi_protocols_index=index;
					SEND_MULTI_STATUS_on;
					#ifdef DEBUG_SERIAL
						debug("Proto=%s",multi_protocols[multi_protocols_index].ProtoString);
						uint8_t nbr=multi_protocols[multi_protocols_index].nbrSubProto;
						debug(", nbr_sub=%d, Sub=",nbr);
						if(nbr && (sub_protocol&0x07)<nbr)
						{
							uint8_t len=multi_protocols[multi_protocols_index].SubProtoString[0];
							uint8_t offset=len*(sub_protocol&0x07)+1;
							for(uint8_t j=0;j<len;j++)
								debug("%c",multi_protocols[multi_protocols_index].SubProtoString[j+offset]);
						}
						debugln(", Opt=%d",multi_protocols[multi_protocols_index].optionType);
					#endif
					break;
				}
				index++;
			}
		#endif
	}
	
	#if defined(WAIT_FOR_BIND) && defined(ENABLE_BIND_CH)
		if( IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_off && (cur_protocol[1]&0x80)==0 && mode_select == MODE_SERIAL)
		{ // Autobind is active but no bind requested by either BIND_CH or BIND. But do not wait if in PPM mode...
			WAIT_BIND_on;
			return;
		}
	#endif
	WAIT_BIND_off;
	CHANGE_PROTOCOL_FLAG_off;

	if(next_callback>32000)
	{ // next_callback should not be more than 32767 so we will wait here...
		uint16_t temp=(next_callback>>10)-2;
		delayMilliseconds(temp);
		next_callback-=temp<<10;				// between 2-3ms left at this stage
	}
	cli();										// disable global int
	OCR1A = TCNT1 + next_callback*2;			// set compare A for callback
	#ifndef STM32_BOARD
		TIFR1 = OCF1A_bm ;						// clear compare A flag
	#else
		TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC1IF;	// Clear Timer2/Comp1 interrupt flag
	#endif	
	sei();										// enable global int
	BIND_BUTTON_FLAG_off;						// do not bind/reset id anymore even if protocol change
}

void modules_reset()
{
  CC2500_Reset();
	//Wait for every component to reset
	delayMilliseconds(100);
	prev_power=0xFD;		// unused power value
}

void usart3_begin(uint32_t baud,uint32_t config )
{
	usart_init(USART3);
	usart_config_gpios_async(USART3,GPIOB,PIN_MAP[PB11].gpio_bit,GPIOB,PIN_MAP[PB10].gpio_bit,config);
	usart_set_baud_rate(USART3, STM32_PCLK1, baud);
	usart_enable(USART3);
}
void init_HWTimer()
{	
	HWTimer2.pause();									// Pause the timer2 while we're configuring it
	TIMER2_BASE->PSC = 35;								// 36-1;for 72 MHZ /0.5sec/(35+1)
	TIMER2_BASE->ARR = 0xFFFF;							// Count until 0xFFFF
	HWTimer2.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);	// Main scheduler
	TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC2IF;			// Clear Timer2/Comp2 interrupt flag
	TIMER2_BASE->DIER &= ~TIMER_DIER_CC2IE;				// Disable Timer2/Comp2 interrupt
	HWTimer2.refresh();									// Refresh the timer's count, prescale, and overflow
	HWTimer2.resume();
}

// Convert 32b id to rx_tx_addr
static void set_rx_tx_addr(uint32_t id)
{ // Used by almost all protocols
	rx_tx_addr[0] = (id >> 24) & 0xFF;
	rx_tx_addr[1] = (id >> 16) & 0xFF;
	rx_tx_addr[2] = (id >>  8) & 0xFF;
	rx_tx_addr[3] = (id >>  0) & 0xFF;
	rx_tx_addr[4] = (rx_tx_addr[2]&0xF0)|(rx_tx_addr[3]&0x0F);
}

static uint32_t random_id(uint16_t address, uint8_t create_new)
{
	#ifndef FORCE_GLOBAL_ID
		uint32_t id=0;

		if(eeprom_read_byte((EE_ADDR)(address+10))==0xf0 && !create_new)
		{  // TXID exists in EEPROM
			for(uint8_t i=4;i>0;i--)
			{
				id<<=8;
				id|=eeprom_read_byte((EE_ADDR)address+i-1);
			}
			if(id!=0x2AD141A7)	//ID with seed=0
			{
				debugln("Read ID from EEPROM");
				return id;
			}
		}
		// Generate a random ID
		#if defined STM32_BOARD
			#define STM32_UUID ((uint32_t *)0x1FFFF7E8)
			if (!create_new)
			{
				id = STM32_UUID[0] ^ STM32_UUID[1] ^ STM32_UUID[2];
				debugln("Generated ID from STM32 UUID");
			}
			else
		#endif
				id = random(0xfefefefe) + ((uint32_t)random(0xfefefefe) << 16);

		for(uint8_t i=0;i<4;i++)
			eeprom_write_byte((EE_ADDR)address+i,id >> (i*8));
		eeprom_write_byte((EE_ADDR)(address+10),0xf0);//write bind flag in eeprom.
		return id;
	#else
		(void)address;
		(void)create_new;
		return FORCE_GLOBAL_ID;
	#endif
}

// Generate frequency hopping sequence in the range [02..77]
static void __attribute__((unused)) calc_fh_channels(uint8_t num_ch)
{
	uint8_t idx = 0;
	uint32_t rnd = MProtocol_id;
	uint8_t max=(num_ch/3)+2;
	
	while (idx < num_ch)
	{
		uint8_t i;
		uint8_t count_2_26 = 0, count_27_50 = 0, count_51_74 = 0;

		rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization
		// Use least-significant byte. 73 is prime, so channels 76..77 are unused
		uint8_t next_ch = ((rnd >> 8) % 73) + 2;
		// Keep a distance of 5 between consecutive channels
		if (idx !=0)
		{
			if(hopping_frequency[idx-1]>next_ch)
			{
				if(hopping_frequency[idx-1]-next_ch<5)
					continue;
			}
			else
				if(next_ch-hopping_frequency[idx-1]<5)
					continue;
		}
		// Check that it's not duplicated and spread uniformly
		for (i = 0; i < idx; i++) {
			if(hopping_frequency[i] == next_ch)
				break;
			if(hopping_frequency[i] <= 26)
				count_2_26++;
			else if (hopping_frequency[i] <= 50)
				count_27_50++;
			else
				count_51_74++;
		}
		if (i != idx)
			continue;
		if ( (next_ch <= 26 && count_2_26 < max) || (next_ch >= 27 && next_ch <= 50 && count_27_50 < max) || (next_ch >= 51 && count_51_74 < max) )
			hopping_frequency[idx++] = next_ch;//find hopping frequency
	}
}

/**************************/
/**************************/
/**  Interrupt routines  **/
/**************************/
/**************************/

void PPM_decode()
{	// Interrupt on PPM pin
	static int8_t chan=0,bad_frame=1;
	static uint16_t Prev_TCNT1=0;
	uint16_t Cur_TCNT1;

	Cur_TCNT1 = TCNT1 - Prev_TCNT1 ;	// Capture current Timer1 value
	if(Cur_TCNT1<1600)
		bad_frame=1;					// bad frame
	else
		if(Cur_TCNT1>4400)
		{  //start of frame
			if(chan>=MIN_PPM_CHANNELS)
			{
				PPM_FLAG_on;			// good frame received if at least 4 channels have been seen
				if(chan>PPM_chan_max) PPM_chan_max=chan;	// Saving the number of channels received
			}
			chan=0;						// reset channel counter
			bad_frame=0;
		}
		else
			if(bad_frame==0)			// need to wait for start of frame
			{  //servo values between 800us and 2200us will end up here
				PPM_data[chan]=Cur_TCNT1;
				if(chan++>=MAX_PPM_CHANNELS)
					bad_frame=1;		// don't accept any new channels
			}
	Prev_TCNT1+=Cur_TCNT1;
}
