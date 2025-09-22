/////////////////////////////////////////////////////////////////////////////
//
//  PROJECT PAGE
//  https://github.com/EngineerGuy314/logger
//
//  Much of the code forked from work by
//  Roman Piksaykin [piksaykin@gmail.com], R2BDY
//  https://github.com/RPiks/pico-WSPR-tx
///////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "defines.h"
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "pico/sleep.h"      
#include "hardware/rtc.h" 


WSPRbeaconContext *pWSPR;

char _callsign[7];        //these get set via terminal, and then from NVRAM on boot
char _id13[3];
char _start_minute[2];
char _lane[2];
char _suffix[2];
char _verbosity[2];
char _Optional_Debug[4];
char _custom_PCB[2];   
char _DEXT_config[5];     
char _battery_mode[2];
char _Klock_speed[4];         
char _Datalog_mode[2]; 
char _U4B_chan[4];
char _band_hop[2];
char _band[2];


static uint32_t telen_values[4];  //consolodate in an array to make coding easier
static absolute_time_t LED_sequence_start_time;
static int GPS_PPS_PIN;     //these get set based on values in defines.h, and also if custom PCB selected in user menu
int RFOUT_PIN;            //will be fixed at 21 to use Kazu's fraction-pll
int Main_System_Clock_Speed;

int force_transmit = 0;
uint32_t fader; //for creating "breathing" effect on LED to indicate corruption of NVRAM
uint32_t fade_counter;
int maxdevs = 10;
uint64_t OW_romcodes[10];



RfGenStruct RfGen = {0};
static float volts=0;
static float tempC=0;
uint32_t XMIT_FREQUENCY;
uint32_t XMIT_FREQUENCY_10_METER;  //deprecated
const uint32_t freqs[14] =   							//A:LF,B:MF,C:160,D:80,E:60,F:40,G:30,H:20,I:17,J:15,K:12,L:10,M:6,N:2 
    {137500,475700,1838100,3570100,5288700,7040100,10140200,14097100,18106100,21096100,24926100,28126100,50294500,144490500}; 


int main()
{
	
	StampPrintf("\n");DoLogPrint(); // needed asap to wake up the USB stdio port (because StampPrintf includes stdio_init_all();). why though?
	for (int i=0;i < 20;i++) {printf("*");sleep_ms(100);}			
 
	gpio_init(LED_PIN);	gpio_set_dir(LED_PIN, GPIO_OUT); //initialize LED output
	for (int i=0;i < 20;i++)     //do some blinky on startup, allows time for power supply to stabilize before GPS unit enabled
		{gpio_put(LED_PIN, 1); sleep_ms(100);gpio_put(LED_PIN, 0);sleep_ms(100);}
	read_NVRAM();				//reads values of _callsign,  _verbosity etc from NVRAM. MUST READ THESE *BEFORE* InitPicoPins
	if (check_data_validity()==-1)  //if data was bad, breathe LED for 10 seconds and reboot. or if user presses a key enter setup
		{
			printf("\nBAD values in NVRAM detected! will reboot in 10 seconds... press any key to enter user-setup menu..\n");
			fader=0;fade_counter=0;
					while (getchar_timeout_us(0)==PICO_ERROR_TIMEOUT) //looks for input on USB serial port only @#$%^&!! they changed this function in SDK 2.0!. used to use -1 for no input, now its -2 PICO_ERROR_TIMEOUT
						{
							 fader+=1;
							 if ((fader%5000)>(fader/100))
								 gpio_put(LED_PIN, 1); 
									else
								 gpio_put(LED_PIN, 0);	
							 if (fader>500000) {fader=0;fade_counter+=1;if (fade_counter>10) {watchdog_enable(100, 1);for(;;)	{} }}  //after ~10 secs force a reboot														
						}	
				RfGen._pGPStime->user_setup_menu_active=1;	//if we get here, they pressed a button (to interrupt the "breathing" that indicates bad nvram)
				user_interface();  
		}
	process_chan_num(); //sets minute/lane/id from chan number. usually redundant at this point, but can't hurt
	
	if (getchar_timeout_us(0)>0)   //looks for input on USB serial port only. Note: getchar_timeout_us(0) returns a -2 (as of sdk 2) if no keypress. Must do this check BEFORE setting Clock Speed in Case you bricked it
		{
		RfGen._pGPStime->user_setup_menu_active=1;	
		user_interface();   
		}
		
	set_sys_clock_khz( Main_System_Clock_Speed* 1000, true);
	
	InitPicoPins();			// Sets GPIO pins roles and directions and also ADC for voltage and temperature measurements (NVRAM must be read BEFORE this, otherwise dont know how to map IO)
    printf("\nThe logger version: %s %s\nWSPR beacon init...",__DATE__ ,__TIME__);	//messages are sent to USB serial port, 115200 baud
	int band_as_int=_band[0]-'A';   
	XMIT_FREQUENCY=freqs[band_as_int];
	switch(_lane[0])                             //following lines set lane frequencies for u4b operation. The center freuency for Zactkep (wspr 3) xmitions is hard set in WSPRBeacon.c to 14097100UL
		{
			case '1':XMIT_FREQUENCY-=80;break;
			case '2':XMIT_FREQUENCY-=40;break;
			case '3':XMIT_FREQUENCY+=40;break;
			case '4':XMIT_FREQUENCY+=80;break;
			 
		}	
   
	 WSPRbeaconContext *pWB = WSPRbeaconInit(
        _callsign,/** the Callsign. */
        CONFIG_LOCATOR4,/**< the default QTH locator if GPS isn't used. */
        10,             /**< Tx power, dbm. */
        XMIT_FREQUENCY,
        0,           /**< the carrier freq. shift relative to dial freq. */ //not used
        RFOUT_PIN,       /**< RF output GPIO pin. */
		(uint8_t)_start_minute[0]-'0',   /**< convert ASCI digits to ints  */
		(uint8_t)_id13[0]-'0',   
		(uint8_t)_suffix[0]-'0',
		_DEXT_config,
		&RfGen
        );
    pWSPR = pWB;  //this lets things outside this routine access the WB context
    pWB->_txSched.force_xmit_for_testing = force_transmit;
	pWB->_txSched.led_mode = 0;  //0 means no serial comms from  GPS (critical fault if it remains that way)
	pWB->_txSched.verbosity=(uint8_t)_verbosity[0]-'0';       /**< convert ASCI digit to int  */
	pWB->_txSched.suffix=(uint8_t)_suffix[0]-'0';    /**< convert ASCI digit to int (value 253 if dash was entered) */
	pWB->_txSched.Optional_Debug=(uint8_t)atoi(_Optional_Debug);
	RfGen._pGPStime = GPStimeInit(9600); 
	RfGen._pGPStime->Optional_Debug=(uint8_t)atoi(_Optional_Debug);
	pWB->_txSched.low_power_mode=(uint8_t)_battery_mode[0]-'0';
	strcpy(pWB->_txSched.id13,_id13);
	RfGen._pGPStime->user_setup_menu_active=0;
	RfGen._pGPStime->verbosity=(uint8_t)_verbosity[0]-'0';   
    int tick = 0;int tick2 = 0;  //used for timing various messages
	LED_sequence_start_time = get_absolute_time();
	if (_Datalog_mode[0]=='1') datalog_loop();
	
    for(;;)   //loop every ~ half second
    {		

		if(WSPRbeaconIsGPSsolutionActive(pWB))
			{
				char *pgps_qth = WSPRbeaconGetLastQTHLocator(pWB);  //GET MAIDENHEAD       - this code in original fork wasnt working due to error in WSPRbeacon.c
				if(pgps_qth)
					strncpy(pWB->_pu8_locator, pgps_qth, 6);     //does full 6 char maidenhead 									 
			if (pWSPR->_pTX->_p_oscillator->_pGPStime->_time_data.sat_count > pWSPR->_txSched.max_sats_seen_today) pWSPR->_txSched.max_sats_seen_today=pWSPR->_pTX->_p_oscillator->_pGPStime->_time_data.sat_count;
			}        
        WSPRbeaconTxScheduler(pWB, YES);   
                
		if (pWB->_txSched.verbosity>=5)
		{
				if(0 == ++tick % 20)      //every ~20 secs dumps context.  
				 WSPRbeaconDumpContext(pWB);
		}	

		if (getchar_timeout_us(0)>0)   //looks for input on USB serial port only. Note: getchar_timeout_us(0) returns a -2 (as of sdk 2) if no keypress. But if you force it into a Char type, becomes something else
			{
			RfGen._pGPStime->user_setup_menu_active=1;	
			user_interface();   
			}

		const float conversionFactor = 3.3f / (1 << 12);          //read temperature
		adc_select_input(4);	
		float adc = (float)adc_read() * conversionFactor;
		float tempC_raw = 27.0f - (adc - 0.706f) / 0.001721f;		
		if (tempC==0) tempC=tempC_raw;  //if tempC still uninitialized, preload its value
		tempC= (0.99*tempC) + (0.01*tempC_raw);  // implements a 1st order IIR lowpass filter (aka "one-line DSP")
		pWB->_txSched.temp_in_Celsius=tempC;           
		RfGen._pGPStime->temp_in_Celsius=tempC;
		
		adc_select_input(3);  //if setup correctly, ADC3 reads Vsys   // read voltage
		float volts_raw = 3*(float)adc_read() * conversionFactor;         //times 3 because of onboard voltage divider
		if (volts==0) volts=volts_raw; //if volts still uninitialized, preload its value
		volts=	(0.99*volts)+(0.01*volts_raw);	// implements a 1st order IIR lowpass filter (aka "one-line DSP")
		pWB->_txSched.voltage=volts;

 		process_TELEN_data();                          //if needed, this puts data into DEXT variables. You can remove this and set the data yourself as shown in the next few lines
			/*pctx->telem_vals_and_ranges[2][0]=(v_and_r){2,8};  //[slot], specified range (inclusive of zero) and value for each 
			  pctx->telem_vals_and_ranges[2][1]=(v_and_r){2,3};
			  pctx->telem_vals_and_ranges[2][2]=(v_and_r){3,2};
			   .......   */
				if(0 == ++tick2 % 10)      //every ~5 sec
				{
				if (pWB->_txSched.verbosity>=1) StampPrintf("Temp: %0.1f  Volts: %0.1f  Altitude: %0.0f  Satellite count: %d\n", tempU,volts,RfGen._pGPStime->_altitude ,RfGen._pGPStime->_time_data.sat_count);		
				//if (pWB->_txSched.verbosity>=3) printf("TELEN Vals 1 through 4:  %d %d %d %d\n",telen_values[0],telen_values[1],telen_values[2],telen_values[3]);
				}
		
		for (int i=0;i < 10;i++) //Implements a pause of total 500ms, and spends it handling LED output
			{
				handle_LED(pWB->_txSched.led_mode); 
				sleep_ms(50); 
			}
		DoLogPrint(); 	
	}
}
///////////////////////////////////
static void sleep_callback(void) {
    printf("RTC woke us up\n");
}

/*****************************************************************************************************************/
/*****************************************************************************************************************/
/*****************************************************************************************************************/
/*****************************************************************************************************************/

void process_TELEN_data(void)
{
		const float conversionFactor = 33.0f / (1 << 12);   //. the 3.3 is from vref, the 10 is to convert to volt tenths the 12 bit shift is because thats resolution of ADC


		for (int i=2;i < 5;i++) //i is slot # (2,3,4)
		{			
		   switch(_DEXT_config[i-2])   //see end for traquito site scaling  
			{
				case '-':  break; //do nothing, telen chan is disabled
				case '0': 			//Minutes Since Boot, Minutes since GPS fix, GPS Valid, Sat Count (max: 1000,1000,1,60)
							pWSPR->telem_vals_and_ranges[i][0]=(v_and_r){pWSPR->_txSched.minutes_since_boot,1001}; 
							pWSPR->telem_vals_and_ranges[i][1]=(v_and_r){pWSPR->_txSched.seconds_for_lock,1001}; 
							pWSPR->telem_vals_and_ranges[i][2]=(v_and_r){pWSPR->_pTX->_p_oscillator->_pGPStime->_time_data._u8_is_solution_active,2}; 
							pWSPR->telem_vals_and_ranges[i][3]=(v_and_r){pWSPR->_pTX->_p_oscillator->_pGPStime->_time_data.sat_count,61}; 
							break;
	
				case '1': 			//ADC 0, 1, 2 (in tenths) (max: 350, 350, 350)
							adc_select_input(0); pWSPR->telem_vals_and_ranges[i][0]=(v_and_r){round((float)adc_read() * conversionFactor),351};
							adc_select_input(1); pWSPR->telem_vals_and_ranges[i][1]=(v_and_r){round((float)adc_read() * conversionFactor),351};
							adc_select_input(2); pWSPR->telem_vals_and_ranges[i][2]=(v_and_r){round((float)adc_read() * conversionFactor),351};
							break;

				case '2': 			//bus volts ADC3 (in hundreth, scaled), Dallas 1 (and sign), sat count (max: 900,120,1,60)
							break;
							
				case '5': 			//(must be used in first DEXT slot) sends additional 4 chars of Maidenhead 
		
							//printf("chars 7-10 %d %d %d %d and as chars %c%c%c%c   \n",pWSPR->grid7,pWSPR->grid8,pWSPR->grid9,pWSPR->grid10,pWSPR->grid7,pWSPR->grid8,pWSPR->grid9,pWSPR->grid10);
							pWSPR->telem_vals_and_ranges[i][0 ]=(v_and_r){pWSPR->grid7-'0' ,10}; 
							pWSPR->telem_vals_and_ranges[i][1 ]=(v_and_r){pWSPR->grid8 -'0',10};
							pWSPR->telem_vals_and_ranges[i][2 ]=(v_and_r){pWSPR->grid9 -'A',24}; 
							pWSPR->telem_vals_and_ranges[i][3 ]=(v_and_r){pWSPR->grid10-'A',24}; 
							pWSPR->telem_vals_and_ranges[i][4]=(v_and_r){(int)round(pWSPR->_txSched.minutes_since_boot/10.0),101}; 
							pWSPR->telem_vals_and_ranges[i][5]=(v_and_r){(int)round(pWSPR->_txSched.seconds_for_lock/2.0),101}; 			
							break;
							
				case '6': 			//idle and xmit volts, some low res times            
		
							pWSPR->telem_vals_and_ranges[i][0 ]=(v_and_r){ (int)round(100*pWSPR->_txSched.voltage_at_idle) ,501}; 
							pWSPR->telem_vals_and_ranges[i][1 ]=(v_and_r){ (int)round(100*pWSPR->_txSched.voltage_at_xmit) ,501}; 
							pWSPR->telem_vals_and_ranges[i][2]=(v_and_r){(int)round(pWSPR->_txSched.minutes_since_boot/10.0),61}; 
							pWSPR->telem_vals_and_ranges[i][3]=(v_and_r){(int)round(pWSPR->_txSched.seconds_for_lock/40.0),61}; 			
							
							break;

				case '7': 			//high res GPS lock/loss times

							/*  not applicable for logger
							pWSPR->telem_vals_and_ranges[i][0]=(v_and_r){pWSPR->_txSched.seconds_since_GPS_aquisition,3601}; 
							pWSPR->telem_vals_and_ranges[i][1]=(v_and_r){pWSPR->_txSched.seconds_since_GPS_loss,3601}; */
							pWSPR->telem_vals_and_ranges[i][2]=(v_and_r){(int)round(pWSPR->_txSched.minutes_since_boot/10.0),37}; 									
							
							break;

				case '8': 			//some things   (58-)
							pWSPR->telem_vals_and_ranges[i][0]=(v_and_r){pWSPR->_txSched.max_sats_seen_today,61}; 
							pWSPR->telem_vals_and_ranges[i][1]=(v_and_r){(uint32_t)pWSPR->_txSched.seconds_for_lock,1801}; 
							pWSPR->telem_vals_and_ranges[i][2]=(v_and_r){round((float)adc_read() * conversionFactor * 3.0f * 10),901}; 									
			}	
		}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void handle_LED(int led_state)
/**
 * @brief Handles setting LED to display mode.
 * 
 * @param led_state 1,2 or 3to indicate the number of LED pulses. 0 is a special case indicating serial comm failure to GPS
 */
 			//////////////////////// LED HANDLING /////////////////////////////////////////////////////////
			
			/*
			LED MODE:
				0 - no serial comms to GPS module
				1 - No valid GPS, not transmitting
				2 - Valid GPS, waiting for time to transmitt
				3 - Transmitting (GPS disabled)

				x brief pulses to indicate mode, followed by pause. 0 is special case, continous rapid blink
				there is also "breathing" to indicate corrupted NVRAM
			*/

{
 static int tik;
 uint64_t t = absolute_time_diff_us(LED_sequence_start_time, get_absolute_time());
 int i = t / 400000ULL;     //400mS total period of a LED flash

  if (led_state==0) 						//special case indicating serial comm failure from GPS. blink as rapidly as possible 
		  {
			if(0 == ++tik % 2) gpio_put(LED_PIN, 1); else gpio_put(LED_PIN, 0);     //very rapid
		  }
  else
  {
		  if (i<(led_state+1))
				{
				 if(t -(i*400000ULL) < 50000ULL)           //400mS total period of a LED flash, 50mS on pulse duration
							gpio_put(LED_PIN, 1);
				 else 
							gpio_put(LED_PIN, 0);
				}
		  if (t > 2500000ULL) 	LED_sequence_start_time = get_absolute_time();     //resets every 2.5 secs (total repeat length of led sequence).
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Prints out hex listing of the settings NVRAM to stdio
 * 
 * @param buf Address of NVRAM to list
 * @param len Length of storage to list
 */
void print_buf(const uint8_t *buf, size_t len) {	

	printf(CLEAR_SCREEN);printf(BRIGHT);
	printf(BOLD_ON);printf(UNDERLINE_ON);
	printf("\nNVRAM dump: \n");printf(BOLD_OFF); printf(UNDERLINE_OFF);
 for (size_t i = 0; i < len; ++i) {
        printf("%02x", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");
    }
	printf(NORMAL);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void display_intro(void)
{
printf(CLEAR_SCREEN);
printf(CURSOR_HOME);
printf(BRIGHT);
printf("\n\n\n\n\n\n\n\n\n\n\n\n");
printf("================================================================================\n\n");printf(UNDERLINE_ON);
printf("logger (Just Another Wspr Beacon Of Noisy Electronics) by KC3LBR,  version: %s %s\n\n",__DATE__ ,__TIME__);printf(UNDERLINE_OFF);printf(NORMAL); 
printf("Instructions and source: https://github.com/EngineerGuy314/logger\n");
printf("Originally forked from : https://github.com/RPiks/pico-WSPR-tx\n");
printf("Additional functions, fixes and documention by https://github.com/serych\n");
printf("Consult https://traquito.github.io/channelmap/ to find and reserve an open channel \n\n");printf(BRIGHT);printf(UNDERLINE_ON);
printf("BAND ENUMERATION:\n");printf(UNDERLINE_OFF);
printf("(default is 'H' 20 Meter) F:40,G:30,H:20,I:17,J:15,K:12,L:10,M:6 \n");
printf("---WARNING!--- For bands other than H (20M)  you may need to use a different U4B channel to get the starting minute you want!!\n\n");



printf("System clock: %.2f MHz (defaults to 125Mhz if interrrupted boot process) \n", clock_get_hz(clk_sys)/1000000.0);
printf("USB clock: %.2f MHz\n", clock_get_hz(clk_usb)/1000000.0);
printf("ADC clock: %.2f MHz\n", clock_get_hz(clk_adc)/1000000.0);
printf("rtc clock: %.4f MHz\n", clock_get_hz(clk_rtc)/1000000.0);
printf("ref clock: %.2f MHz\n", clock_get_hz(clk_ref)/1000000.0);
printf("Peri clock: %.2f MHz (UART,I2C, etc) GETS FORCED by the SDK TO 48MHZ whenever main sys is changed, because of reasons Grok cant explain\n", clock_get_hz(clk_peri)/1000000.0);

printf("\n================================================================================\n");

printf(RED);printf("press anykey to continue");printf(NORMAL); 
char c=getchar_timeout_us(60000000);	//wait 
printf(CLEAR_SCREEN);

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_TELEN_msg()
{
printf(BRIGHT);
printf("\n\n\n\n");printf(UNDERLINE_ON);
printf("DEXT (Doug's EXtended Telemetry) CONFIG INSTRUCTIONS:\n\n");printf(UNDERLINE_OFF);
printf(NORMAL); 
printf("\n (DEXT is also known as community driven Extended Telemetry )\n\n");
printf("* There are 3 possible DEXT values, corresponding to DEXT slots 2,3 and 4,\n");
printf("  (Slots 0 and 1 are used by WSPR Type 1 and U4B Basic Telemetry)\n");
printf("  DEXT slot 2 type, DEXT slot 3 type and DEXT slot 4 type.\n");
printf("* Enter 3 characters in DEXT_config. use a '-' (minus) to disable one \n");
printf("  or more values.\n* example: '---' disables all DEXT \n");
printf("* example: '01-' sets DEXT 2  to type 0, \n  DEXT 3 to type 1,  disables DEXT slot 4 \n"); printf(BRIGHT);printf(UNDERLINE_ON);
printf("\nDEXT Types:\n\n");printf(UNDERLINE_OFF);printf(NORMAL); 
printf("-: disabled, 0: minutes since boot, minutes since GPS fix aquired, GPS valid bit and Sat count \n");
printf("... many more !... \n");
printf("See the Wiki for full list and range info.\n\n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Function that implements simple user interface via UART
 * 
 * For every new config variable to be added to the interface:
	1: create a global character array at top of main.c 
	2: add entry in read_NVRAM()
	3: add entry in write_NVRAM()
	4: add limit checking in check_data_validity()
	5: add limit checking in check_data_validity_and_set_defaults()
	6: add TWO entries in show_values() (to display name and value, and also to display which key is used to change it)
	7: add CASE statement entry in user_interface()
	8: Either do something with the variable locally in Main.c, or if needed elsewhere:
		-- add a member to the GPStimeContext or WSPRbeaconContext structure
		-- add code in main.c to move the data from the local _tag to the context structure
		-- do something with the data elsewhere in the program
 */
void user_interface(void)                                //called if keystroke from terminal on USB detected during operation.
{
int c;
char str[10];
gpio_put(GPS_ENABLE_PIN,1);gpio_put(VFO_ENABLE_PIN,1);
sleep_ms(100);
gpio_put(LED_PIN, 1); //LED on.	

display_intro();
printf(CLEAR_SCREEN);
show_values();          /* shows current VALUES  AND list of Valid Commands */

    for(;;)
	{	
																 printf(UNDERLINE_ON);printf(BRIGHT);
		printf("\nEnter the command (X,C,S,U,B,V,P,T,B,F,O):");printf(UNDERLINE_OFF);printf(NORMAL);	
		c=getchar_timeout_us(60000000);		   //just in case user setup menu was enterred during flight, this will reboot after 60 secs
		printf("%c\n", c);
		if (c==PICO_ERROR_TIMEOUT) {printf(CLEAR_SCREEN);printf("\n\n TIMEOUT WAITING FOR INPUT, REBOOTING FOR YOUR OWN GOOD!\n");sleep_ms(100);watchdog_enable(100, 1);for(;;)	{}}
		if (c>90) c-=32; //make it capital either way
		switch(c)
		{
			case 'X':printf(CLEAR_SCREEN);printf("\n\nGOODBYE");watchdog_enable(100, 1);for(;;)	{}
			//case 'R':printf(CLEAR_SCREEN);printf("\n\nCorrupting data..");strncpy(_callsign,"!^&*(",6);write_NVRAM();watchdog_enable(100, 1);for(;;)	{}  //used for testing NVRAM check on boot feature
			case 'C':get_user_input("Enter callsign: ",_callsign,sizeof(_callsign)); convertToUpperCase(_callsign); write_NVRAM(); break;
			case 'S':get_user_input("Enter single digit numeric suffix: ", _suffix, sizeof(_suffix)); convertToUpperCase(_suffix); write_NVRAM(); break;
			case 'U':get_user_input("Enter U4B channel: ", _U4B_chan, sizeof(_U4B_chan)); process_chan_num(); write_NVRAM(); break;
			case 'B':get_user_input("Band (F-M): ", _band, sizeof(_band));   convertToUpperCase(_band); write_NVRAM(); break;
/*			case 'I':get_user_input("Enter id13: ", _id13,sizeof(_id13)); convertToUpperCase(_id13); write_NVRAM(); break; //still possible but not listed or recommended
			case 'M':get_user_input("Enter starting Minute: ", _start_minute, sizeof(_start_minute)); write_NVRAM(); break; //still possible but not listed or recommended. i suppose needed for when to start standalone beacon or Zachtek
			case 'L':get_user_input("Enter Lane (1,2,3,4): ", _lane, sizeof(_lane)); write_NVRAM(); break; //still possible but not listed or recommended 
*/
			case 'V':get_user_input("Verbosity level (0-9): ", _verbosity, sizeof(_verbosity)); write_NVRAM(); break;
			case 'O':get_user_input("Optional debug (0-255 bitmapped): ", _Optional_Debug, sizeof(_Optional_Debug)); write_NVRAM(); break;
//			case 'P':get_user_input("custom Pcb mode (0,1): ", _custom_PCB, sizeof(_custom_PCB)); write_NVRAM(); break;
			//case 'H':get_user_input("band Hop mode (0,1): ", _band_hop, sizeof(_band_hop)); write_NVRAM(); break;
			case 'T':show_TELEN_msg();get_user_input("Telemetry (dexT) config: ", _DEXT_config, sizeof(_DEXT_config)-1); convertToUpperCase(_DEXT_config); write_NVRAM(); break;
			//case 'B':get_user_input("Battery mode (0,1): ", _battery_mode, sizeof(_battery_mode)); write_NVRAM(); break;
			/*case 'D':get_user_input("Data-log mode (0,1,Wipe,Dump): ", _Datalog_mode, sizeof(_Datalog_mode));
						convertToUpperCase(_Datalog_mode);
						if ((_Datalog_mode[0]=='D') || (_Datalog_mode[0]=='W') ) 
								{
									datalog_special_functions();
									_Datalog_mode[0]='0';
								}						 
							write_NVRAM(); 
						break;*/

			//case 'K':get_user_input("Klock speed: ", _Klock_speed, sizeof(_Klock_speed)); write_NVRAM(); break;
			
			case 'F':
				printf("Fixed Frequency output (antenna tuning mode). Enter frequency (for example 14.097) or 0 for exit.\n\t");
				char _tuning_freq[12];
				float frequency;
				while(1)
				{
					get_user_input("Frequency to generate (MHz):  ", _tuning_freq, sizeof(_tuning_freq));  //blocking until next input
					frequency = 1000000*atof(_tuning_freq);
					if (!frequency) {break;}
					InitPicoPins();			// Sets GPIO pins roles and directions and also ADC for voltage and temperature measurements (NVRAM must be read BEFORE this, otherwise dont know how to map IO)
					gpio_put(VFO_ENABLE_PIN,0);  //turns on VFO
					sleep_ms(2);
					printf("Generating %f Hz.   Press any key to stop. \n", frequency);
					si5351aSetFrequency((uint64_t)(frequency*(uint64_t)100));
					c=getchar();c=getchar();
					si5351_stop();				
					gpio_put(VFO_ENABLE_PIN,1);  //turns off VFO										
					break;
				}
				break;

			/*case '<': {printf("< was pressed");InitPicoPins();gpio_put(VFO_ENABLE_PIN,0);I2C_init();sleep_ms(2);si5351aSetFrequency(1400000000);} break;
			case '>': {printf("> was pressed");InitPicoPins();gpio_put(GPS_ENABLE_PIN,0);} break;
			case '?': {printf("? was pressed");InitPicoPins();gpio_put(VFO_ENABLE_PIN,1);gpio_put(GPS_ENABLE_PIN,1);} break;
			case 'M': {	InitPicoPins();		gpio_put(GPS_ENABLE_PIN,0);		gpio_put(VFO_ENABLE_PIN,0);I2C_init();sleep_ms(2);si5351aSetFrequency(1400000000);} break;*/
			case 13:  break;
			case 10:  break;
			default: printf(CLEAR_SCREEN); printf("\nYou pressed: %c - (0x%02x), INVALID choice!! ",c,c);sleep_ms(1000);break;		
		}
		check_data_validity_and_set_defaults();
		show_values();
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Reads part of the program memory where the user settings are saved
 * prints hexa listing of data and calls function which check data validity
 * 
 */
void read_NVRAM(void)
{
const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET); //a pointer to a safe place after the program memory

print_buf(flash_target_contents, FLASH_PAGE_SIZE); //256

strncpy(_callsign, flash_target_contents, 6);
strncpy(_id13, flash_target_contents+6, 2);
strncpy(_start_minute, flash_target_contents+8, 1);
strncpy(_lane, flash_target_contents+9, 1);
strncpy(_suffix, flash_target_contents+10, 1);
strncpy(_verbosity, flash_target_contents+11, 1);
//strncpy(_Optional_Debug, flash_target_contents+12, 1); MOVED TO END BNECAUSE IT SBIGGER NOW
strncpy(_custom_PCB, flash_target_contents+13, 1);
strncpy(_DEXT_config, flash_target_contents+14, 4); //only needs 3, kept at 4 for historical ease
strncpy(_battery_mode, flash_target_contents+18, 1);
strncpy(_Klock_speed, flash_target_contents+19, 3); _Klock_speed[3]=0; //null terminate cause later will use atoi
Main_System_Clock_Speed = atoi(_Klock_speed);  // was hardcoded for Kazu PLL method at 48
strncpy(_Datalog_mode, flash_target_contents+22, 1);
strncpy(_U4B_chan, flash_target_contents+23, 3); _U4B_chan[3]=0; //null terminate cause later will use atoi
strncpy(_band_hop, flash_target_contents+26, 1);
strncpy(_band, flash_target_contents+27, 1);
strncpy(_Optional_Debug, flash_target_contents+28, 3);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Writes the user entered data into NVRAM
 * 
 */
void write_NVRAM(void)
{
    uint8_t data_chunk[FLASH_PAGE_SIZE];  //256 bytes

	strncpy(data_chunk,_callsign, 6);
	strncpy(data_chunk+6,_id13,  2);
	strncpy(data_chunk+8,_start_minute, 1);
	strncpy(data_chunk+9,_lane, 1);
	strncpy(data_chunk+10,_suffix, 1);
	strncpy(data_chunk+11,_verbosity, 1);
	//strncpy(data_chunk+12,_Optional_Debug, 1);  MOVED TO END BECAUSE ITS BIGGER NOW
	strncpy(data_chunk+13,_custom_PCB, 1);
	strncpy(data_chunk+14,_DEXT_config, 4);  //only needs 3, kept at 4 for historical ease
	strncpy(data_chunk+18,_battery_mode, 1);
	strncpy(data_chunk+19,_Klock_speed, 3);
	strncpy(data_chunk+22,_Datalog_mode, 1);
	strncpy(data_chunk+23,_U4B_chan, 3);
	strncpy(data_chunk+26,_band_hop, 1);
	strncpy(data_chunk+27,_band, 1);
	strncpy(data_chunk+28,_Optional_Debug, 3);

	uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);  //a "Sector" is 4096 bytes             FLASH_TARGET_OFFSET,FLASH_SECTOR_SIZE,FLASH_PAGE_SIZE = 040000x, 4096, 256
	flash_range_program(FLASH_TARGET_OFFSET, data_chunk, FLASH_PAGE_SIZE);  //writes 256 bytes (one "page") (16 pages per sector)
	restore_interrupts (ints);												//you could theoretically write 16 pages at once (a whole sector)

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Checks validity of user settings and if something is wrong, it sets "factory defaults"
 * This is ONLY called when explicitly enterring the user-setup menu. It will NOT be called, and no values will be changed, on regular reboot
 * this is to give a bad EEPROM read in morning a chance to reboot and still work correctly
 */
void check_data_validity_and_set_defaults(void)
{
//do some basic plausibility checking on data, set reasonable defaults if memory was uninitialized							
	if ( ((_callsign[0]<'A') || (_callsign[0]>'Z')) && ((_callsign[0]<'0') || (_callsign[0]>'9'))    ) {   strncpy(_callsign,"AB1CDE",6);     ; write_NVRAM();} 
	if ( ((_suffix[0]<'0') || (_suffix[0]>'9')) && (_suffix[0]!='X') ) {_suffix[0]='-'; write_NVRAM();} //by default, disable zachtek suffix
	if ( (_id13[0]!='0') && (_id13[0]!='1') && (_id13[0]!='Q')&& (_id13[0]!='-')) {strncpy(_id13,"Q0",2); write_NVRAM();}
	if ( (_start_minute[0]!='0') && (_start_minute[0]!='2') && (_start_minute[0]!='4')&& (_start_minute[0]!='6')&& (_start_minute[0]!='8')) {_start_minute[0]='0'; write_NVRAM();}
	if ( (_lane[0]!='1') && (_lane[0]!='2') && (_lane[0]!='3')&& (_lane[0]!='4')) {_lane[0]='2'; write_NVRAM();}
	if ( (_verbosity[0]<'0') || (_verbosity[0]>'9')) {_verbosity[0]='1'; write_NVRAM();} //set default verbosity to 1
	if ( (atoi(_Optional_Debug)<0) || (atoi(_Optional_Debug)>255)) {strcpy(_Optional_Debug,"0"); _Optional_Debug[1]=0;write_NVRAM();} 
	if (atoi(_Optional_Debug)==0) {strcpy(_Optional_Debug,"0"); _Optional_Debug[1]=0;}  //this is an anti-stupid in case _Optional_Debug has alpha (non numeric) content. atoi still evalues any alpha as zero, this makes damn sure that if its zero, its really a zero character in the variable. Doesnt do write_NVRAM, because somethig else will prolly do it anyway.
	if ( (_custom_PCB[0]<'0') || (_custom_PCB[0]>'1')) {_custom_PCB[0]='0'; write_NVRAM();} //set default IO mapping to original Pi Pico configuration
	if ( (_DEXT_config[0]<'0') || (_DEXT_config[0]>'F')) {strncpy(_DEXT_config,"---",3); write_NVRAM();}
	if ( (_battery_mode[0]<'0') || (_battery_mode[0]>'1')) {_battery_mode[0]='0'; write_NVRAM();} //
	if ( (atoi(_Klock_speed)<5) || (atoi(_Klock_speed)>300)) {strcpy(_Klock_speed,"18"); write_NVRAM();} 
	if ( (atoi(_U4B_chan)<0) || (atoi(_U4B_chan)>599)) {strcpy(_U4B_chan,"599"); write_NVRAM();} 
	if ( (_Datalog_mode[0]!='0') && (_Datalog_mode[0]!='1') && (_Datalog_mode[0]!='D') && (_Datalog_mode[0]!='W')) {_Datalog_mode[0]='0'; write_NVRAM();}
	if ( (_band_hop[0]<'0') || (_band_hop[0]>'1')) {_band_hop[0]='0'; write_NVRAM();} //
	if ( (_band[0]<'F') || (_band[0]>'M')) {_band[0]='H'; write_NVRAM();} //

//certain modes have been hidden. following lines make sure they are not accidentally enabled from data corruption
strcpy(_Klock_speed,"18");
_battery_mode[0]='0';
_Datalog_mode[0]='0';
_band_hop[0]='0'; 

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Checks validity of user settings and returns -1 if something wrong. Does NOT set defaults or alter NVRAM.
 * 
 */
int check_data_validity(void)
{
int result=1;	
//do some basic plausibility checking on data				
	if ( ((_callsign[0]<'A') || (_callsign[0]>'Z')) && ((_callsign[0]<'0') || (_callsign[0]>'9'))    ) {result=-1;} 
	if ( ((_suffix[0]<'0') || (_suffix[0]>'9')) && (_suffix[0]!='-') && (_suffix[0]!='X') ) {result=-1;} 
	if ( (_id13[0]!='0') && (_id13[0]!='1') && (_id13[0]!='Q')&& (_id13[0]!='-')) {result=-1;}
	if ( (_start_minute[0]!='0') && (_start_minute[0]!='2') && (_start_minute[0]!='4')&& (_start_minute[0]!='6')&& (_start_minute[0]!='8')) {result=-1;}
	if ( (_lane[0]!='1') && (_lane[0]!='2') && (_lane[0]!='3')&& (_lane[0]!='4')) {result=-1;}
	if ( (_verbosity[0]<'0') || (_verbosity[0]>'9')) {result=-1;} 
	if ( (atoi(_Optional_Debug)<0) || (atoi(_Optional_Debug)>255)) {result=-1;} 
	if ( (_custom_PCB[0]<'0') || (_custom_PCB[0]>'1')) {result=-1;} 
	if ( ((_DEXT_config[0]<'0') || (_DEXT_config[0]>'F'))&& (_DEXT_config[0]!='-')) {result=-1;}
	if ( (_battery_mode[0]<'0') || (_battery_mode[0]>'1')) {result=-1;} 	
	if ( (atoi(_Klock_speed)<5) || (atoi(_Klock_speed)>300)) {result=-1;} 	
	if ( (_Datalog_mode[0]!='0') && (_Datalog_mode[0]!='1')) {result=-1;}
	if ( (atoi(_U4B_chan)<0) || (atoi(_U4B_chan)>599)) {result=-1;} 
	if ( (_band_hop[0]<'0') || (_band_hop[0]>'1')) {result=-1;} 
	if ( (_band[0]<'F') || (_band[0]>'M')) {result=-1;} 


return result;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Function that writes out the current set values of parameters
 * 
 */
void show_values(void) /* shows current VALUES  AND list of Valid Commands */
{
check_data_validity_and_set_defaults(); //added may 2025, will this cause problems? with fresh out of box pico?

int band_as_int=_band[0]-'A';       
printf(CLEAR_SCREEN);
printf("logger (Just Another Wspr Beacon Of Noisy Electronics) by KC3LBR,  version: %s %s\n\n",__DATE__ ,__TIME__);
printf(UNDERLINE_ON);printf(BRIGHT);
printf("\n\nCurrent values:\n");printf(UNDERLINE_OFF);printf(NORMAL);

printf("\n\tCallsign:%s\n\t",_callsign);
printf("Suffix (zachtek):%s   (please set to '-' if unused)\n\t",_suffix);
printf("U4b channel:%s",_U4B_chan);
printf(" (Id13:%s",_id13);
printf(" Start Minute:%s",_start_minute);
printf(" Lane:%s)\n\t",_lane);
printf("Band:%s (%d Hz)\n\t",_band,freqs[band_as_int]);
printf("Verbosity:%s\n\t",_verbosity);
printf("Optional debug:%s\n\t",_Optional_Debug);
//printf("custom Pcb IO mappings:%s\n\t",_custom_PCB);
printf("Telemetry config:%s   (please set to '---' if unused)\n\t",_DEXT_config);
//printf("Klock speed (temp) :%sMhz  \n",_Klock_speed);
/*printf("Datalog mode:%s\n\t",_Datalog_mode);
printf("Battery (low power) mode:%s\n\t",_battery_mode);
printf("secret band Hopping mode:%s\n\n",_band_hop);*/

							printf(UNDERLINE_ON);printf(BRIGHT);
printf("VALID commands: ");printf(UNDERLINE_OFF);printf(NORMAL);

printf("\n\n\tX: eXit configuraiton and reboot\n\tC: change Callsign (6 char max)\n\t");
printf("S: change Suffix ( for WSPR3/Zachtek) use '-' to disable WSPR3\n\t");
printf("U: change U4b channel # (0-599)\n\t");
printf("B: change Band (F-M) default 20M is H\n\t");
/*printf("I: change Id13 (two alpha numeric chars, ie Q8) use '--' to disable U4B\n\t");
printf("M: change starting Minute (0,2,4,6,8)\n\tL: Lane (1,2,3,4) corresponding to 4 frequencies in 20M band\n\t");*/ //it is still possible to directly change these, but its not shown
printf("V: Verbosity level (0 for no messages, 9 for too many) \n\t");
printf("O: Optional debug functions (bitmapped 0 - 255) \n\t");
//printf("P: custom Pcb mode IO mappings (0,1)\n\t");
printf("T: Telemetry (dexT) config\n\t");
//printf("K: Klock speed  \n\t");
//printf("D: Datalog mode (0,1,(W)ipe memory, (D)ump memory) see wiki\n\t");
//printf("B: Battery (low power) mode \n\t");
printf("F: Frequency output (antenna tuning mode)\n\t");
//printf("H: secret band Hopping mode \n\n");

}
/**
 * @brief Converts string to upper case
 * 
 * @param str string to convert
 * @return No return value, string is converted directly in the parameter *str  
 */
void convertToUpperCase(char *str) {
    while (*str) {
        *str = toupper((unsigned char)*str);
        str++;
    }
}
/**
 * @brief Initializes Pico pins
 * 
 */
void InitPicoPins(void)
{
/*  gpio_init(18); 
	gpio_set_dir(18, GPIO_OUT); //GPIO 18 used for fan control when testing TCXO stability */

			gpio_init(GPS_ENABLE_PIN); gpio_set_dir(GPS_ENABLE_PIN, GPIO_OUT); //initialize GPS enable output (INVERSE LOGIC on custom PCB, so just initialize it, leave it at zero state)	
			gpio_init(VFO_ENABLE_PIN); gpio_set_dir(VFO_ENABLE_PIN, GPIO_OUT); //initialize VFO synth enable output (INVERSE LOGIC on custom PCB)
			gpio_put(GPS_ENABLE_PIN,1);
			gpio_put(VFO_ENABLE_PIN,1);
			//this turn them BOTH off, must turn on later as needed
	
	
	for (int i=0;i < 3;i++)   //init ADC(s) as needed for TELEN
		{			
/* 		removed (temporarily) Jane 2025 because 0 1 and 2 are used for custome DEXT messages, NOT actual ADC channel reads. ADC chan initiation needs to be done if/when you use a DEXt that includes analog. I dont think thats implemented yet

		 switch(_DEXT_config[i])
			{
				case '-':  break; //do nothing, telen chan is disabled
				case '0': gpio_init(26);gpio_set_dir(26, GPIO_IN);gpio_set_pulls(26,0,0);break;
				case '1': gpio_init(27);gpio_set_dir(27, GPIO_IN);gpio_set_pulls(27,0,0);break;
				case '2': gpio_init(28);gpio_set_dir(28, GPIO_IN);gpio_set_pulls(28,0,0);break; 
			}
			*/
		}
	
	gpio_init(PICO_VSYS_PIN);  		//Prepare ADC 3 to read Vsys
	gpio_set_dir(PICO_VSYS_PIN, GPIO_IN);
	gpio_set_pulls(PICO_VSYS_PIN,0,0);
    adc_init();
    adc_set_temp_sensor_enabled(true); 	//Enable the onboard temperature sensor

}


void datalog_special_functions()   //this called only from user-setup menu
{
/*		FLASH_TARGET_OFFSET(0x4 0000): a pointer to a safe place after the program memory  (
		xip_base offset (0x1000 0000) only needed when READING, not writing)
	    FLASH_TARGET_OFFSET = 040000x
		FLASH_SECTOR_SIZE,   4096
		FLASH_PAGE_SIZE       256 */		

uint8_t *pointer_to_byte;
char c;
uint32_t byte_counter;
uint32_t sector_count; //65- 321  //add xip_base offset ONLY when reading, each sector is 4096 bytes. this is 1MB of data in a safe place (could go close to 2MB theoreticallY). sector 64 is where NBRAM (user settings) are
	
if (_Datalog_mode[0]=='D') //Dumps memory to usb serial port
{
			printf("About to dump...\n");

			for (sector_count=65;sector_count<(321-1);sector_count+=1)   //sector 64 is  where user settings are, so start at 65			
			{
				for (byte_counter=0;byte_counter<(FLASH_SECTOR_SIZE-1);byte_counter+=1)   
				{
					pointer_to_byte=(char *)(XIP_BASE+byte_counter+(sector_count*FLASH_SECTOR_SIZE));
					c = *pointer_to_byte;
					if (c==255) break;    //255 is uninitialized or blank					
					printf("%c",c);
				//sleep_ms(5);                     //may or may not be needed for very large transfers?
				}  
				if (c==255) break;
			}
			printf("\nDone dumping memory, zero reached at %d bytes in sector %d\n",byte_counter,sector_count);
}

if(_Datalog_mode[0]=='W')   
{
	printf("WIPING EVERYTHING in 5 seconds! press a key to abort....\n");
	int cc=getchar_timeout_us(6000000);		

  if (cc==PICO_ERROR_TIMEOUT)
  {
	printf("wiping in process, please wait...\n");
	uint32_t ints = save_and_disable_interrupts();	
	flash_range_erase(FLASH_SECTOR_SIZE*65L,FLASH_SECTOR_SIZE*256L );  
	restore_interrupts (ints);
	printf("* * * Done Wiping! * * * \n");
  }
  else	printf("Wipe aborted. Phew!\n");  
}

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void write_to_next_avail_flash(char *text)   //text can be a literal, a pointer to char arracy, or a char array
{
uint32_t byte_counter;
uint32_t sector_count;	
uint32_t found_byte_location;
uint32_t found_sector;	
uint8_t current_sector_data[4096];		
uint8_t next_sector_data[4096];		
uint8_t *pointer_to_byte;
char c;
size_t length_of_input = strlen(text);

		for (sector_count=65;sector_count<(321-1);sector_count+=1)     //find next open spot	
			{
				for (byte_counter=0;byte_counter<(FLASH_SECTOR_SIZE-1);byte_counter+=1)   
				{
					pointer_to_byte=(char *)(XIP_BASE+byte_counter+(sector_count*FLASH_SECTOR_SIZE));
					c = *pointer_to_byte;
					if (c==255) break;    //255 is uninitialized or blank					
				}  
				if (c==255) break;
			}
	printf("found opening at byte # %d bytes in sector # 	%d\n",byte_counter,sector_count);
	found_sector=sector_count;
	found_byte_location=byte_counter;

//read the whole sector
	for (byte_counter=0;byte_counter<(FLASH_SECTOR_SIZE-1);byte_counter+=1)   
				{
					pointer_to_byte=(char *)(XIP_BASE+byte_counter+(found_sector*FLASH_SECTOR_SIZE));
					c = *pointer_to_byte;
					current_sector_data[byte_counter]=c;					
				}  

//read the entire NEXT sector (just in case wrapping is needed)
	for (byte_counter=0;byte_counter<(FLASH_SECTOR_SIZE-1);byte_counter+=1)   
				{
					pointer_to_byte=(char *)(XIP_BASE+byte_counter+((found_sector+1)*FLASH_SECTOR_SIZE));
					c = *pointer_to_byte;
					next_sector_data[byte_counter]=c;					
				}  
	
if ( (length_of_input + found_byte_location)>FLASH_SECTOR_SIZE)  //then need to wrap
				//need to span 2 sectors
			{
				for (byte_counter=found_byte_location;byte_counter<FLASH_SECTOR_SIZE;byte_counter+=1)   //first part
				{
					current_sector_data[byte_counter]= *((char *)(text+byte_counter-found_byte_location));					
				}  
				for (byte_counter=0;byte_counter<(length_of_input-(FLASH_SECTOR_SIZE-found_byte_location));byte_counter+=1)   //2nd part part
				{
					next_sector_data[byte_counter]= *((char *)(text+byte_counter + (FLASH_SECTOR_SIZE-found_byte_location)      ));					
				}
				
			}
			else
				//can fit new data in current sector
			{
				for (byte_counter=found_byte_location;byte_counter<(found_byte_location+length_of_input);byte_counter+=1)   
				{
					current_sector_data[byte_counter]= *((char *)(text+byte_counter-found_byte_location));					
				}
			}
	
	uint32_t ints = save_and_disable_interrupts();	
	flash_range_erase(FLASH_SECTOR_SIZE*found_sector,FLASH_SECTOR_SIZE);  	
	flash_range_program(FLASH_SECTOR_SIZE*found_sector, current_sector_data, FLASH_SECTOR_SIZE);  
	flash_range_erase(FLASH_SECTOR_SIZE*(1+found_sector),FLASH_SECTOR_SIZE);  
	flash_range_program(FLASH_SECTOR_SIZE*(1+found_sector), next_sector_data, FLASH_SECTOR_SIZE); 
	restore_interrupts (ints);
	
	printf("size of input string %s is: %d wrote it to byte %d in sector %d\n",text,length_of_input,found_byte_location,found_sector);

}
//////////////////////////
void datalog_loop()          //datalogging is very out of date
{
	char string_to_log[400];
	absolute_time_t GPS_wait_start_time;
	uint64_t t;
	int elapsed_seconds;

				printf("Enterring DATA LOG LOOP. waiting for sat lock or 65 sec max\n");
				const float conversionFactor = 3.3f / (1 << 12);          //read temperature
				adc_select_input(4);	
				float adc = (float)adc_read() * conversionFactor;
				float tempf =32+(( 27.0f - (adc - 0.706f) / 0.001721f)*(9.0f/5.0f));						
				adc_select_input(3);  //if setup correctly, ADC3 reads Vsys   // read voltage
				volts = 3*(float)adc_read() * conversionFactor;  

				GPS_wait_start_time = get_absolute_time();
	 
				do
					{
						t = absolute_time_diff_us(GPS_wait_start_time, get_absolute_time());	
										if (getchar_timeout_us(0)>0)   //looks for input on USB serial port only. Note: getchar_timeout_us(0) returns a -2 (as of sdk 2) if no keypress. But if you force it into a Char type, becomes something else
										{
											RfGen._pGPStime->user_setup_menu_active=1;	
											user_interface();   
										}
					} 
				while (( t<450000000ULL )&&(RfGen._pGPStime->_time_data.sat_count<4));               //wait for RfGen._pGPStime->_time_data.sat_coun>4 with 65 second maximum time
					//set to 450 seconds !!!!!
				elapsed_seconds= t  / 1000000ULL;

				if (RfGen._pGPStime->_time_data.sat_count>=4)
				{
				sleep_ms(3000); //even though sat count seen, wait a bit longer
				sprintf(string_to_log,"latitutde:,%lli,longitude:,%lli,altitude:,%f,sat count:,%d,time:,%s,temp:,%f,bat voltage:,%f,seconds to aquisition:,%d\n",RfGen._pGPStime->_time_data._i64_lon_100k,RfGen._pGPStime->_time_data._i64_lat_100k,RfGen._pGPStime->_altitude,RfGen._pGPStime->_time_data.sat_count,RfGen._pGPStime->_time_data._full_time_string,tempf,volts,elapsed_seconds);
				write_to_next_avail_flash(string_to_log);
				printf("GPS data has been logged.\n");
				}
					else
				{
				sprintf(string_to_log,"no reading, time might be:,%s,temp:,%f,bat voltage:,%f\n",RfGen._pGPStime->_time_data._full_time_string,tempf,volts);
				write_to_next_avail_flash(string_to_log);
				printf("NO GPS seen :-(\n");
				}

				printf("About to sleep!\n");
				gpio_set_dir(GPS_ENABLE_PIN, GPIO_IN);  //let the mosfet drive float


				go_to_sleep();

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void reboot_now()
{
printf("\n\nrebooting...");watchdog_enable(100, 1);for(;;)	{}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void go_to_sleep()
{
			/*
			removing for now because 1) sleep doesnt work with new PLL setup and 2) updated pico-extras caused issues
			datetime_t t = {.year  = 2020,.month = 01,.day= 01, .dotw= 1,.hour=1,.min= 1,.sec = 00};			
			rtc_init(); // Start the RTC
			rtc_set_datetime(&t);
			uart_default_tx_wait_blocking();
			datetime_t alarm_time = t;

			alarm_time.min += 20;	//sleep for 20 minutes.
			//alarm_time.sec += 15;

			gpio_set_irq_enabled(GPS_PPS_PIN, GPIO_IRQ_EDGE_RISE, false); //this is needed to disable IRQ callback on PPS
			pico_fractional_pll_deinit();  //this is (was?) needed, otherwise causes instant reboot
			sleep_run_from_dormant_source(DORMANT_SOURCE_ROSC);  //this reduces sleep draw to 2mA! (without this will still sleep, but only at 8mA)
			sleep_goto_sleep_until(&alarm_time, &sleep_callback);	//blocks here during sleep perfiod
			{watchdog_enable(100, 1);for(;;)	{} }  //recovering from sleep is messy, so this makes it reboot to get a fresh start
			*/
}
////////////////////////////////////
void process_chan_num()   //need to update for bands other than 20M and 10M
{
	if ( (atoi(_U4B_chan)>=0) && (atoi(_U4B_chan)<600)) 
	{
		
		_id13[0]='1';
		if  (atoi(_U4B_chan)<200) _id13[0]='0';
		if  (atoi(_U4B_chan)>399) _id13[0]='Q';

		int id3 = (atoi(_U4B_chan) % 200) / 20;
		_id13[1]=id3+'0';
		
		int lane = (atoi(_U4B_chan) % 20) / 5;
		_lane[0]=lane+'1';

		int txSlot = atoi(_U4B_chan) % 5;
		

		_start_minute[0] = '0' + (2*((txSlot+14)%5)); //default starting minute for 20M

		if (_band[0]=='L')  //10 Meter
		_start_minute[0] = '0' + (2*((txSlot+12)%5)); 

	}
}
/**
* @note:
* Verbosity notes:
* 0: none
* 1: temp/volts every second, message if no gps
* 2: GPS status every second
* 3:          messages when a xmition started
* 4: x-tended messages when a xmition started 
* 5: dump context every 20 secs
* 6: show PPB every second
* 7: Display GxRMC and GxGGA messages
* 8: display ALL serial input from GPS module
*/

//DEXT definitions:
/*
0:
{ "name": "MinSinceBoot",      "unit": "Count",  "lowValue":   0, "highValue": 1000,    "stepSize": 1 },
{ "name": "MinSinceGPSValid",    "unit": "Count",  "lowValue":   0, "highValue": 1000,    "stepSize": 1 },
{ "name": "GPS_Valid",   "unit": "Count",  "lowValue":   0, "highValue": 1,    "stepSize": 1 },
{ "name": "SatCount",       "unit": "Count",  "lowValue":   0, "highValue": 60,    "stepSize": 1 },

1:
{ "name": "ADC0",      "unit": "Count",  "lowValue":   0, "highValue": 350,    "stepSize": 1 },
{ "name": "ADC1",      "unit": "Count",  "lowValue":   0, "highValue": 350,    "stepSize": 1 },
{ "name": "ADC2",      "unit": "Count",  "lowValue":   0, "highValue": 350,    "stepSize": 1 },

2:
{ "name": "BusVolts",      "unit": "Count",  "lowValue":   0, "highValue": 900,    "stepSize": 1 },
{ "name": "DALLAS1",      "unit": "Count",  "lowValue":   0, "highValue": 120,    "stepSize": 1 },
{ "name": "Dallas1_sign",      "unit": "Count",  "lowValue":   0, "highValue": 1,    "stepSize": 1 },
{ "name": "SatCount",       "unit": "Count",  "lowValue":   0, "highValue": 60,    "stepSize": 1 },


3:
{ "name": "DALLAS1",      "unit": "Count",  "lowValue":   0, "highValue": 120,    "stepSize": 1 },
{ "name": "Dallas1_sign",      "unit": "Count",  "lowValue":   0, "highValue": 1,    "stepSize": 1 },
{ "name": "DALLAS2",      "unit": "Count",  "lowValue":   0, "highValue": 120,    "stepSize": 1 },
{ "name": "Dallas2_sign",      "unit": "Count",  "lowValue":   0, "highValue": 1,    "stepSize": 1 },

3:
{ "name": "DALLAS3",      "unit": "Count",  "lowValue":   0, "highValue": 120,    "stepSize": 1 },
{ "name": "Dallas3_sign",      "unit": "Count",  "lowValue":   0, "highValue": 1,    "stepSize": 1 },
{ "name": "DALLAS4",      "unit": "Count",  "lowValue":   0, "highValue": 120,    "stepSize": 1 },
{ "name": "Dallas4_sign",      "unit": "Count",  "lowValue":   0, "highValue": 1,    "stepSize": 1 },

5:
{ "name": "grid_char7",   "unit": "digit",   "lowValue":   0,    "highValue": 9,    "stepSize": 1   },
{ "name": "grid_char8",   "unit": "digit",    "lowValue":   0,    "highValue":    9,    "stepSize":  1 },
{ "name": "grid_char9",   "unit": "alpha",      "lowValue":   0,    "highValue":   23,    "stepSize":  1 },
{ "name": "grid_char10",  "unit": "alpha",      "lowValue":   0,  "highValue":     23,  "stepSize":  1 },
{ "name": "since_boot",   "unit": "minutes10",    "lowValue":   0,  "highValue":     100,  "stepSize":  1 },
{ "name": "time_for_lock_2s",  "unit": "double_secs",  "lowValue":   0,  "highValue":     100,  "stepSize":  1 },


7:
{ "name": "since_lock",     "unit": "secs",   "lowValue":   0,    "highValue": 3600,    "stepSize": 1   },
{ "name": "since_loss",      "unit": "secs",    "lowValue":   0,    "highValue":    3600,    "stepSize":  1   },
{ "name": "since_boot_tens",      "unit": "mins",    "lowValue":   0,    "highValue":    36,    "stepSize":  1   },

8:
{ "name": "most_sats_seen_today",      "unit": "Count",  "lowValue":   0, "highValue": 60,    "stepSize": 1 },
{ "name": "seconds_for_1st_aquisition","unit": "Count","lowValue":   0, "highValue": 1800,    "stepSize": 1 },
{ "name": "Vbus",   "unit": "mV",  "lowValue":   0, "highValue": 900,    "stepSize": 1 },

 */
