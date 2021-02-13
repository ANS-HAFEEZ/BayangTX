/*******************/
/*** TX SETTINGS ***/
/*******************/
//Modify the channel order based on your TX: AETR, TAER, RETA...
//Examples: Flysky & DEVO is AETR, JR/Spektrum radio is TAER, Multiplex is AERT...
//Default is AETR.
#define AETR

/*****************/
/*** AUTO BIND ***/  // Also referred as "Bind on powerup"
/*****************/
//Bind from channel enables you to bind when a specified channel is going from low to high. This feature is only active
// if you specify AUTOBIND in PPM mode or set AutoBind to YES for serial mode.
//Comment to globaly disable the bind feature from a channel.
//#define ENABLE_BIND_CH
//Set the channel number used for bind. Default is 16.
//#define BIND_CH	16

//Comment to disable the wait for bind feature. If Autobind is enabled in the model config, this feature will not activate
// the selected protocol unless a bind is requested using bind from channel or the GUI "Bind" button.
//The goal is to prevent binding other people's model when powering up the TX, changing model or scanning through protocols.
#define WAIT_FOR_BIND


/****************/
/*** RF CHIPS ***/
/****************/
#define CC2500_INSTALLED

/** CC2500 Fine Frequency Tuning **/
//For optimal performance the CC2500 RF module used by the CORONA, FrSkyD, FrSkyV, FrSkyX, Hitec, HoTT, Futaba/SFHSS and Redpine protocols needs to be tuned for each protocol.
//Initial tuning should be done via the radio menu with a genuine CORONA/FrSky/Hitec/HoTT/Futaba/Redpine receiver.  
//Once a good tuning value is found it can be set here and will override the radio's 'option' setting for all existing and new models which use that protocol.
//For more information: https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/tree/master/docs/Frequency_Tuning.md
//Uncomment the lines below (remove the "//") and set an appropriate value (replace the "0") to enable. Valid range is -127 to +127.
#define FORCE_FRSKYD_TUNING	0
//#define FORCE_FRSKYX_TUNING	0


/** Low Power **/
//Low power is reducing the transmit power of the multi module. This setting is configurable per model in PPM (table below) or Serial mode (radio GUI).
//It can be activated when flying indoor or small models since the distance is short or if a model is causing issues when flying closed to the TX.
//By default low power selection is enabled on all rf chips, but you can disable it by commenting (add //) the lines below if you don't want to risk
//flying a model with low power.
//#define A7105_ENABLE_LOW_POWER
//#define CYRF6936_ENABLE_LOW_POWER
//#define CC2500_ENABLE_LOW_POWER
//#define NRF24L01_ENABLE_LOW_POWER


/*****************/
/*** GLOBAL ID ***/
/*****************/
//A global ID is used by most protocols to bind and retain the bind to models. To prevent duplicate IDs, it is automatically
// generated using a random 32 bits number the first time the eeprom is initialized.
//If you have 2 Multi modules which you want to share the same ID so you can use either to control the same RC model
// then you can force the ID to a certain known value using the lines below.
//Default is commented, you should uncoment only for test purpose or if you know exactly what you are doing!!!
//The 8 numbers below can be anything between 0...9 and A..F
//#define FORCE_GLOBAL_ID	0x12345678

//Protocols using the CYRF6936 (DSM, Devo, Walkera...) are using the CYRF ID instead which should prevent duplicated IDs.
//If you have 2 Multi modules which you want to share the same ID so you can use either to control the same RC model
// then you can force the ID to a certain known value using the lines below.
//Default is commented, you should uncoment only for test purpose or if you know exactly what you are doing!!!
//#define FORCE_CYRF_ID	"\x12\x34\x56\x78\x9A\xBC"


/****************************/
/*** PROTOCOLS TO INCLUDE ***/
/****************************/
//In this section select the protocols you want to be accessible when using the module.
//All the protocols will not fit in the Atmega328p module so you need to pick and choose.
//Comment the protocols you are not using with "//" to save Flash space.

//The protocols below need a CC2500 to be installed
#define	FRSKYD_CC2500_INO
//#define	FRSKYX_CC2500_INO
/***************************/
/*** PROTOCOLS SETTINGS  ***/
/***************************/

//DSM specific settings
//---------------------
//The DSM protocol is using by default the Spektrum throw of 1100..1900us @100% and 1000..2000us @125%.
// For more throw, 1024..1976us @100% and 904..2096us @125%, remove the "//" on the line below. Be aware that too much throw can damage some UMX servos. To achieve standard throw in this mode use a channel weight of 84%.
//#define DSM_MAX_THROW
//Some models (X-Vert, Blade 230S...) require a special value to instant stop the motor(s).
// You can disable this feature by adding "//" on the line below. You have to specify which channel (14 by default) will be used to kill the throttle channel.
// If the channel 14 is above -50% the throttle is untouched but if it is between -50% and -100%, the throttle output will be forced between -100% and -150%.
// For example, a value of -80% applied on channel 14 will instantly kill the motors on the X-Vert.
//#define DSM_THROTTLE_KILL_CH 14 

//Enable DSM Forward Programming
//#define DSM_FWD_PGM

//AFHDS2A specific settings
//-------------------------
//When enabled (remove the "//"), the below setting makes LQI (Link Quality Indicator) available on one of the RX ouput channel (5-14).
//#define AFHDS2A_LQI_CH 14

/**************************/
/*** FAILSAFE SETTINGS  ***/
/**************************/
//The following protocols are supporting failsafe: FrSkyX, Devo, WK2x01, Futaba/SFHSS, HISKY/HK310 and AFHDS2A
//In Serial mode failsafe is configured on the radio itself.
//In PPM mode and only after the module is up and fully operational, press the bind button for at least 5sec to send the current stick positions as failsafe to the RX.
//If you want to disable failsafe globally comment the line below using "//".
//#define FAILSAFE_ENABLE

/**************************/
/*** TELEMETRY SETTINGS ***/
/**************************/
//In this section you can configure the telemetry.

//If you do not plan using the telemetry comment this global setting using "//" and skip to the next section.
//#define TELEMETRY

//Comment to invert the polarity of the output telemetry serial signal.
//This function takes quite some flash space and processor power on an atmega.
//For a Taranis/T16 with an external module it must be uncommented. For a T16 internal module it must be commented.
//A 9XR_PRO running erskyTX will work with both commented and uncommented depending on the radio setting Invert COM1 under the Telemetry menu.
//On other addon/replacement boards like the 9xtreme board or the Ar9x board running erskyTX, you need to uncomment the line below.
//For er9x it depends if you have an inveter mod or not on the telemetry pin. If you don't have an inverter comment this line.
#define INVERT_TELEMETRY
//For STM32 and OrangeRX modules, comment to prevent the TX from forcing the serial telemetry polarity normal/invert.
#define INVERT_TELEMETRY_TX

//Uncomment if you want to send Multi status telemetry frames (Protocol available, Bind in progress, version...)
//Use with er9x/erskyTX, for OpenTX you must select MULTI_TELEMETRY below
//#define MULTI_STATUS

//Sends Multi status and allow OpenTX to autodetect the telemetry format. Comment to disable.
//Supported by OpenTX version 2.2 RC9 and newer. NOT supported by er9x/erskyTX use MULTI_STATUS instead.
#define MULTI_TELEMETRY
//Work in progress: Sync OpenTX frames with the current protocol timing. This feature is only available on the STM32 module. Uncomment to enable.
//#define MULTI_SYNC

//Comment a line to disable a specific protocol telemetry
#define DSM_TELEMETRY				// Forward received telemetry packet directly to TX to be decoded by er9x, erskyTX and OpenTX
#define SPORT_TELEMETRY				// Use FrSkyX format to send/receive telemetry
#define AFHDS2A_FW_TELEMETRY		// Forward received telemetry packet directly to TX to be decoded by erskyTX and OpenTX
#define AFHDS2A_HUB_TELEMETRY		// Use FrSkyD Hub format to send basic telemetry to TX like er9x
#define HUB_TELEMETRY				// Use FrSkyD Hub format to send telemetry to TX
#define BAYANG_HUB_TELEMETRY		// Use FrSkyD Hub format to send telemetry to TX
#define BUGS_HUB_TELEMETRY			// Use FrSkyD Hub format to send telemetry to TX
#define DEVO_HUB_TELEMETRY			// Use FrSkyD Hub format to send telemetry to TX
#define HUBSAN_HUB_TELEMETRY		// Use FrSkyD Hub format to send telemetry to TX
#define NCC1701_HUB_TELEMETRY		// Use FrSkyD Hub format to send telemetry to TX
#define OMP_HUB_TELEMETRY			// Use FrSkyD Hub format to send telemetry to TX
#define PROPEL_HUB_TELEMETRY		// Use FrSkyD Hub format to send telemetry to TX
#define CABELL_HUB_TELEMETRY		// Use FrSkyD Hub format to send telemetry to TX
#define RLINK_HUB_TELEMETRY			// Use FrSkyD Hub format to send telemetry to TX
#define HITEC_HUB_TELEMETRY			// Use FrSkyD Hub format to send basic telemetry to the radios which can decode it like er9x, erskyTX and OpenTX
#define HITEC_FW_TELEMETRY			// Forward received telemetry packets to be decoded by erskyTX and OpenTX
#define SCANNER_TELEMETRY			// Forward spectrum scanner data to TX
#define FRSKY_RX_TELEMETRY			// Forward channels data to TX
#define AFHDS2A_RX_TELEMETRY		// Forward channels data to TX
#define HOTT_FW_TELEMETRY			// Forward received telemetry packets to be decoded by erskyTX and OpenTX
#define BAYANG_RX_TELEMETRY			// Forward channels data to TX

/****************************/
/*** SERIAL MODE SETTINGS ***/
/****************************/
//In this section you can configure the serial mode.
//The serial mode enables full editing of all the parameters in the GUI of the radio. It is enabled by placing the rotary switch on position 0.
//This is available natively for ER9X, ERSKY9X and OpenTX.

//If you do not plan to use the Serial mode comment this line using "//" to save Flash space
//#define ENABLE_SERIAL


/*************************/
/*** PPM MODE SETTINGS ***/
/*************************/
//In this section you can configure all details about PPM.
//If you do not plan to use the PPM mode comment this line using "//" to save Flash space, you don't need to configure anything below in this case
#define ENABLE_PPM

/** TX END POINTS **/
#define TX_CUSTOM		//Custom

#if defined(TX_CUSTOM)
	#define PPM_MAX_100	1900	//	100%
	#define PPM_MIN_100	1100	//	100%
#endif

/** Number of PPM Channels **/
// The line below is used to set the minimum number of channels which the module should receive to consider a PPM frame valid.
// The default value is 4 to receive at least AETR for flying models but you could also connect the PPM from a car radio which has only 3 channels by changing this number to 3.
#define MIN_PPM_CHANNELS 4
// The line below is used to set the maximum number of channels which the module should work with. Any channels received above this number are discarded.
// The default value is 16 to receive all possible channels but you might want to filter some "bad" channels from the PPM frame like the ones above 6 on the Walkera PL0811.
#define MAX_PPM_CHANNELS 8

/** Telemetry **/
//Send simple FrSkyX telemetry using the FrSkyD telemetry format
#define TELEMETRY_FRSKYX_TO_FRSKYD

/** Rotary Switch Protocol Selector Settings **/
//The table below indicates which protocol to run when a specific position on the rotary switch has been selected.
//All fields and values are explained below. Everything is configurable from here like in the Serial mode.
//Tip: You can associate multiple times the same protocol to different rotary switch positions to take advantage of the model match based on RX_Num

//A system of banks enable the access to more protocols than positions on the rotary switch. Banks can be selected by placing the rotary switch on position 15, power up the module and
// short press the bind button multiple times until you reach the desired one. The bank number currently selected is indicated by the number of LED flash.
// Full procedure is located here: https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Protocols_Details.md#protocol-selection-in-ppm-mode

//The parameter below indicates the number of desired banks between 1 and 5. Default is 1.
#define NBR_BANKS 1

const PPM_Parameters PPM_prot[14*NBR_BANKS]=	{
#if NBR_BANKS > 0
//******************************       BANK 1       ******************************
//	Switch	Protocol 		Sub protocol	RX_Num	Power		Auto Bind		Option	Chan Order
/*	1	*/	{PROTO_FLYSKY,	Flysky		,	0	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },
/*	2	*/	{PROTO_AFHDS2A,	PWM_IBUS	,	0	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },	// RX number 0
/*	3	*/	{PROTO_AFHDS2A,	PWM_IBUS	,	1	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },	// RX number 1
/*	4	*/	{PROTO_AFHDS2A,	PWM_IBUS	,	2	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },	// RX number 2
/*	5	*/	{PROTO_AFHDS2A,	PWM_IBUS	,	3	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },	// RX number 3
/*	6	*/	{PROTO_AFHDS2A,	PWM_IBUS	,	2	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },	// RX number 4
/*	7	*/	{PROTO_AFHDS2A,	PWM_IBUS	,	3	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },	// RX number 5
/*	8	*/	{PROTO_FUTABA,	NONE		,	0	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },	// option=fine freq tuning
/*	9	*/	{PROTO_FRSKYV,	NONE		,	0	,	P_HIGH	,	NO_AUTOBIND	,	40	,	0x00000000 },	// option=fine freq tuning
/*	10	*/	{PROTO_FRSKYD,	NONE		,	0	,	P_HIGH	,	NO_AUTOBIND	,	40	,	0x00000000 },	// option=fine freq tuning
/*	11	*/	{PROTO_FRSKYX,	CH_16		,	0	,	P_HIGH	,	NO_AUTOBIND	,	40	,	0x00000000 },	// option=fine freq tuning
/*	12	*/	{PROTO_FRSKYX,	EU_16		,	0	,	P_HIGH	,	NO_AUTOBIND	,	40	,	0x00000000 },	// option=fine freq tuning
/*	13	*/	{PROTO_DEVO	,	NONE		,	0	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },
/*	14	*/	{PROTO_WK2x01,	WK2801		,	0	,	P_HIGH	,	NO_AUTOBIND	,	0	,	0x00000000 },
#endif
};
// RX_Num is used for TX & RX match. Using different RX_Num values for each receiver will prevent starting a model with the false config loaded...
// RX_Num value is between 0 and 15.

// Power P_HIGH or P_LOW: High or low power setting for the transmission.
// For indoor P_LOW is more than enough.

// Auto Bind	AUTOBIND or NO_AUTOBIND
// For protocols which does not require binding at each power up (like Flysky, FrSky...), you might still want a bind to be initiated each time you power up the TX.
// As an example, it's usefull for the WLTOYS F929/F939/F949/F959 (all using the Flysky protocol) which requires a bind at each power up.
// It also enables the Bind from channel feature, allowing to execute a bind by toggling a designated channel.

// Option: the value is between -128 and +127.
// The option value is only valid for some protocols, read this page for more information: https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Protocols_Details.md

// Chan order: if the value is different from 0, this setting will remap the first 8 channels in any given order before giving them to the protocol.
// It does not disable the automatic channel remapping of the protocol itself but changes the input of it.
// Even if your TX is sending less than 8 channels you have to respect the format like if it was.
// Examples:
//  - 0x12345678 will give to the protocol the channels in the order 1,2,3,4,5,6,7,8 which is equivalent to 0x00000000.
//  - 0x42315678 will give to the protocol the channels in the order 4,2,3,1,5,6,7,8 swapping channel 1 and 4.
//  - 0x40010000 will give to the protocol the channels in the order 4,2,3,1,5,6,7,8 swapping channel 1 and 4. Note: 0 means leave the channel where it is.
//  - 0x0000ABCD will give to the protocol the channels in the order 1,2,3,4,10,11,12,13 which potentially enables acces to channels not available on your TX. Note A=10,B=11,C=12,D=13,E=14,F=15.

/* Available protocols and associated sub protocols to pick and choose from (Listed in alphabetical order)
	PROTO_AFHDS2A
		PWM_IBUS
		PPM_IBUS
		PWM_SBUS
		PPM_SBUS
		PWM_IB16
		PPM_IB16
	PROTO_AFHDS2A_RX
		NONE
	PROTO_ASSAN
		NONE
	PROTO_BAYANG
		BAYANG
		H8S3D
		X16_AH
		IRDRONE
		DHD_D4
		QX100
	PROTO_BAYANG_RX
		NONE
	PROTO_BUGS
		NONE
	PROTO_BUGSMINI
		BUGSMINI
		BUGS3H
	PROTO_CABELL
		CABELL_V3
		CABELL_V3_TELEMETRY
		CABELL_SET_FAIL_SAFE
		CABELL_UNBIND
	PROTO_CFLIE
		NONE
	PROTO_CG023
		CG023
		YD829
	PROTO_CORONA
		COR_V1
		COR_V2
		FD_V3
	PROTO_CX10
		CX10_GREEN
		CX10_BLUE
		DM007
		JC3015_1
		JC3015_2
		MK33041
	PROTO_DEVO
		NONE
	PROTO_DM002
		NONE
	PROTO_DSM
		DSM2_22
		DSM2_11
		DSMX_22
		DSMX_11
	PROTO_DSM_RX
		NONE
	PROTO_E01X
		E012
		E015
		E016H
	PROTO_ESKY
		ESKY_STD
		ESKY_ET4
	PROTO_ESKY150
		ESKY150_4CH
		ESKY150_7CH
	PROTO_ESKY150V2
		NONE
	PROTO_FLYSKY
		Flysky
		V9X9
		V6X6
		V912
		CX20
	PROTO_FQ777
		NONE
	PROTO_FRSKY_RX
		FRSKY_RX
		FRSKY_CLONE
	PROTO_FRSKYD
		FRSKYD
		DCLONE
	PROTO_FRSKYL
		LR12
		LR12_6CH
	PROTO_FRSKYR9
		R9_915
		R9_868
		R9_915_8CH
		R9_868_8CH
		R9_FCC
		R9_EU
		R9_FCC_8CH
		R9_EU_8CH
	PROTO_FRSKYV
		NONE
	PROTO_FRSKYX
		CH_16
		CH_8
		EU_16
		EU_8
		XCLONE_16
		XCLONE_8
	PROTO_FRSKYX2
		CH_16
		CH_8
		EU_16
		EU_8
		XCLONE
	PROTO_FRSKY_RX
		FRSKY_RX
		FRSKY_CLONE
	PROTO_FX816
		NONE
	PROTO_FY326
		FY326
		FY319
	PROTO_GD00X
		GD_V1
		GD_V2
	PROTO_GW008
		NONE
	PROTO_H8_3D
		H8_3D
		H20H
		H20MINI
		H30MINI
	PROTO_HEIGHT
		HEIGHT_5CH
		HEIGHT_8CH
	PROTO_HISKY
		Hisky
		HK310
	PROTO_HITEC
		OPT_FW
		OPT_HUB
		MINIMA
	PROTO_HONTAI
		HONTAI
		JJRCX1
		X5C1
		FQ777_951
	PROTO_HOTT
		HOTT_SYNC
		HOTT_NO_SYNC
	PROTO_HUBSAN
		H107
		H301
		H501
	PROTO_J6PRO
		NONE
	PROTO_JJRC345
		JJRC345
		SKYTMBLR
	PROTO_KF606
		NONE
	PROTO_KN
		WLTOYS
		FEILUN
	PROTO_KYOSHO
		KYOSHO_FHSS
		KYOSHO_HYPE
	PROTO_MJXQ
		WLH08
		X600
		X800
		H26D
		E010
		H26WH
		PHOENIX
	PROTO_MT99XX
		MT99
		H7
		YZ
		LS
		FY805
	PROTO_NCC1701
		NONE
	PROTO_OMP
		NONE
	PROTO_PELIKAN
		PELIKAN_PRO
		PELIKAN_LITE
	PROTO_POTENSIC
		NONE
	PROTO_PROPEL
		NONE
	PROTO_Q2X2
		Q222
		Q242
		Q282
	PROTO_Q303
		Q303
		CX35
		CX10D
		CX10WD
	PROTO_Q90C
		NONE
	PROTO_REALACC
		NONE
	PROTO_REDPINE
		RED_FAST
		RED_SLOW
	PROTO_RLINK
		NONE
	PROTO_SCANNER
		NONE
	PROTO_FUTABA
		NONE
	PROTO_SHENQI
		NONE
	PROTO_SKYARTEC
		NONE
	PROTO_SLT
		SLT_V1
		SLT_V2
		Q100
		Q200
		MR100
	PROTO_SYMAX
		SYMAX
		SYMAX5C
	PROTO_TIGER
		NONE
	PROTO_TRAXXAS
		RX6519
	PROTO_V2X2
		V2X2
		JXD506
		V2X2_MR101
	PROTO_V761
		V761_3CH
		V761_4CH
	PROTO_V911S
		V911S_STD
		V911S_E119
	PROTO_WFLY
		NONE
	PROTO_WK2x01
		WK2801
		WK2401
		W6_5_1
		W6_6_1
		W6_HEL
		W6_HEL_I
	PROTO_XK
		X450
		X420
	PROTO_YD717
		YD717
		SKYWLKR
		SYMAX4
		XINXUN
		NIHUI
	PROTO_ZSX
		NONE
*/

/**********************************/
/*** DIRECT INPUTS SETTINGS ***/
/**********************************/
//In this section you can configure the direct inputs.
//It enables switches wired directly to the board
//Direct inputs works only in ppm mode and only for stm_32 boards
//Uncomment following lines to enable derect inputs or define your own configuration in _MyConfig.h
/*
#define ENABLE_DIRECT_INPUTS
		
#define DI1_PIN				PC13	
#define IS_DI1_on			(digitalRead(DI1_PIN)==LOW)

#define DI2_PIN				PC14	
#define IS_DI2_on			(digitalRead(DI2_PIN)==LOW)

#define DI3_PIN				PC15	
#define IS_DI3_on			(digitalRead(DI3_PIN)==LOW)

//Define up to 4 direct input channels
//CHANNEL1 - 2pos switch
#define DI_CH1_read			IS_DI1_on ? PPM_MAX_100*2 : PPM_MIN_100*2
//CHANNEL2 - 3pos switch
#define DI_CH2_read			IS_DI2_on ? PPM_MAX_100*2 : (IS_DI2_on ? PPM_MAX_100 + PPM_MIN_100 : PPM_MIN_100*2)
*/
