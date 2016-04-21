#include "MCP3911_EVB.h"
#include  "math.h"
#include "wdt.h"
#include "PwrMgnt.h"

// <editor-fold defaultstate="collapsed" desc="Detailed description">
/*
 * CNK is PIC24FJ256GA110 MCU based design. MCP3911 AFE is a Two-Channel Analog Front End containing two synchronous sampling
 * Delta-Sigma Analog-to-Digital Converters (ADC), two PGAs, phase delay compensation block, low-drift
 * internal voltage reference, modulator output block, digital offset and gain errors calibration registers, and
 * high-speed 20 MHz SPI compatible serial interface.
 * AFE pinout and connection to the MCU:
 *    AFE     |      MCU
 * ---------------------------
 * SCLK       |     RF6/RPI45
 * ---------------------------
 * SDO        |     RF7/RPI44
 * ---------------------------
 * SDI        |     RF8/RP15
 * ---------------------------
 * Data Ready |     RE9/RPI34
 * ---------------------------
 * CS         |     RA4
 * ---------------------------
 * CLKIN      |     RD0/RP11
 * ---------------------------
 * CLKIN      |     RD0/RP11
 * ---------------------------
 * CLKIN      |     RD0/RP11
 * ---------------------------
 * MAT0       |     RD10
 * ---------------------------
 * MAT1       |     RD11
 * ---------------------------
 * RESET      |     RA5
 * ---------------------------
 *
 * MCP3911 AFE receives analog values from the Analog Section. Input analog signal is scaled down based
 * on the amplitude value. Input signal is measured and compared to a buit in reference voltage(Vref). If the input
 * amplitude is higher then the Vref input voltage dividor is switched to a higher dividor value and the process
 * is repeated.
 * Push buttons
 * #define menu_pb     _RA14 #define menu_dwn    _RA15 #define menu_up     _RC2
 * ------------------------
 * MENU       |     RA14
 * ------------------------
 * MENU_DWN   |     RA15
 * ------------------------
 * MENU_UP    |     RC2
 * ------------------------

 */// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC24FJ256GA110 Configuration settings">
/*
 * #define FWDTEN_OFF           0x7F7F
 * #define FWDTEN_ON            0x7FFF
 * If the FWDTEN Configuration bit is set, the WDT is always enabled (FWDTEN_ON). However, the WDT can be
 * optionally controlled in the user software when the FWDTEN Configuration bit has been
 * programmed to '0', FWDTEN_OFF.
 * The WDT is enabled in software by setting the SWDTEN control bit (RCON<5>). The SWDTEN
 * control bit is cleared on any device Reset. The software WDT option allows the user to enable
 * the WDT for critical code segments and disable the WDT during non-critical segments for
 * maximum power savings. So no changes to the configuration register: FWDTEN_OFF & WINDIS_OFF do not change
 * FWPSA_PR128 = pre-scaler set to                  -> Pre-scaler ratio of 1:128
 * WDTPS_PS32768 = Watchdog Timer Post-scaler set to -> WDTPS_PS32768 - 1:32,768 
 * 31000 / (128 * 32768) = 0.007391 Hz = 135 sec (Has to be at least 5 sec)
 * LPRC INPUT --> Pre-scaler ratio --> WDT counter --> Post-scaler --> RESET 
 */
//_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_OFF & COE_OFF & ICS_PGx2 & FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768)
//31000 / (128 * 4096) = 0.05913 Hz => 1/0.05913 = 16.9125 sec
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & BKBUG_OFF & COE_OFF & ICS_PGx2 & FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS4096)  //WDt ~16 sec      
_CONFIG2(IESO_OFF & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & IOL1WAY_OFF & I2C2SEL_PRI & POSCMOD_XT)
_CONFIG3(WPCFG_WPCFGDIS & WPDIS_WPDIS)
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Main data buffer declaration">
//Main buffer declaration (8bits). The data gets collected in INT2 function
unsigned char voltage_msb[buffer_lenght], voltage_nsb[buffer_lenght], voltage_lsb[buffer_lenght]; //Allocated storage for CH0 data
unsigned char current_msb[buffer_lenght], current_nsb[buffer_lenght], current_lsb[buffer_lenght]; //Allocated storage for CH1 data
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="TR_cntr, counter_buffer, counter_tx">
unsigned int TR_cntr;           // Counts number of data samples being transmitted via UART
unsigned int counter_buffer;    // Used in INT2 interrupt to count number of INT2 interrupts
unsigned int counter_tx;        // Used in OC2 interrupt to count number of sent samples// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="divider, dividerCH0, dividerCH1, CH0_sel, CH1_sel, tx_buf[10], tx_bufc[3], tx_bufi[5], types, internal_registers[24], flags">
//unsigned char divider, dividerCH0, dividerCH1; //Variables to control input voltage dividors
unsigned char dividerCH0, dividerCH1; //Variables to control input voltage dividors
unsigned char CH0_sel, CH1_sel; //Input channel select variables. Used to enable/disable input channels.
unsigned char CH0_sel_u2, CH1_sel_u2;
//Setting CHx_sel=1 enables data collection from the channel and UART data transfer
//Additional flag TR_EN (transmit enable) has to be set to enable UART data transfer
//unsigned char cntr;                           //Unused 04102014
unsigned char tx_buf[10], tx_bufc[3], tx_bufi[5], types, internal_registers[24];
//unsigned char rxcounter;                      //Unused 04102014
//unsigned char char_to_int[3],i_valley;        //Unused 04102014
volatile unsigned char flags;
//volatile unsigned char LaFlag;
//flags = 5 is set in communication protocol by case 18 cap test
//flags = 1 indicates sleep mode// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="range_test_delay, urxdata, rxbuffer[96], start_cycle, rx_data, write_reg_count, TR_EN, frequency_cntrCH0, frequency_cntrCH1, no_sleep, sleep_counter, rng_srch">
unsigned char range_test_delay;
unsigned char urxdata, rxbuffer[96], start_cycle, rx_data, write_reg_count;
unsigned char TR_EN, frequency_cntrCH0, frequency_cntrCH1;
unsigned char no_sleep, sleep_counter;
unsigned char rng_srch;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="rx_function, PWM_dty, PWM_prd, selection">
volatile unsigned int rx_function;
volatile unsigned int rx_function_U2;
unsigned int PWM_dty, PWM_prd;
unsigned char selection; // </editor-fold>


unsigned long reading_nV, voltage_ave, current_ave, voltage_sum;                        //32 bit
unsigned long voltage_val, current_val, voltage_acc, current_acc;
//unsigned long vals[900];                                                              //<-------------------- 900 unsigned long
unsigned long vals[10];     //Used in median calculation, in Functions
//unsigned long VdiffCH0, VdiffCH1;
//The following variables are global because they get updated after INT2 is done ==============
unsigned long freq1CH0, freq2CH0, freq3CH0, freq4CH0, freqCH0, f_remCH0;                //Frequency variables
unsigned long freq1CH1, freq2CH1, freq3CH1, freq4CH1, freqCH1, f_remCH1;

//volatile unsigned long Vpk; //Peak Voltage calculated in voltage_display_CH0_Rev01x.c
//volatile unsigned long Ipk; //Peak Voltage calculated in voltage_display_CH1_Rev01x.c
unsigned long Vpk; //Peak Voltage calculated in voltage_display_CH0_Rev01x.c. Used to determine AC range on CH0
unsigned long Ipk; //Peak Voltage calculated in voltage_display_CH1_Rev01x.c. Used to determine AC range on CH1
//==============================================================================================
//Calibration constants
unsigned char cal;          // Used in voltage_display and current_display
unsigned int  ac_cal;       // Used in voltage_display and current_display
volatile unsigned long ints;// Used in logic analyzer to count number of received CIN interrupts (interrupts_Revxx)
//unsigned int buffer_size;
unsigned char tmp_glbl;
unsigned char debug_gbl;

//OC2_interrupt subroutine variables
volatile unsigned char buffer_full;//This flag is set in the OC2 routine
volatile unsigned char vflag, cflag, CIN_flg, CH0_disp_frz;                                                 //8 bit
//volatile unsigned long v_valley, crossing, crossing_CH1;
volatile unsigned long crossing; //Number of zero crossings on CH0 used in frequency calculations
volatile unsigned long crossing_CH1;//Number of zero crossings on CH1 used in frequency calculations
volatile unsigned int period, max_TR_Buff, v_min_counter, v_max_counter; //16 bit
volatile unsigned int sampl_cntr;
volatile WORD_VAL timer[8]; // LA timer
volatile WORD_VAL TimeBase; //Logic analyzer Base Time Value
volatile DWORD_VAL NmbrOfBits;
volatile unsigned int min_val;
volatile unsigned long v_min, i_min, lst_smpl;              //32 bits
volatile unsigned long v_min_n, i_min_n;                    //Used in OC2 to store magnitude of the minimum value
volatile unsigned long v_max_p, i_max_p;                    //v_max_p and i_max_p are computed in OC2 and used in voltage_dividor_Rev_11x.c
volatile unsigned long v_smple_sum, voltage_sum_p_sqr, c_smple_sum, current_sum_p_sqr;                                                                         //32 bits
volatile unsigned long long voltage_sum_p, voltage_sum_n, current_sum_p, current_sum_n;  //64 bit
volatile unsigned long CH0_n_cal, CH0_p_cal, CH1_n_cal, CH1_p_cal;
volatile unsigned char success;
//volatile unsigned char timer_dice;

// <editor-fold defaultstate="collapsed" desc="Voltage calibration shift values and sign CH0">
// Voltage calibration shift values and sign CH0
unsigned long CH0vshft, CH0510kVal, CH080kVal, CH010kVal;   // 'Vshift for every range 32 bit
unsigned char CH0vsftSgn, CH0510kSgn, CH080kSgn, CH010kSgn; // Sign for the shift// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Voltage calibration shift values and sign CH1">
// Voltage calibration shift values and sign CH1
unsigned long CH1vshft, CH1510kVal, CH180kVal, CH110kVal;   //V shift for every range 32 bit
unsigned char CH1vsftSgn, CH1510kSgn, CH180kSgn, CH110kSgn; //Sign for the shift// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Calibration gain values for CH0 and CH1">
//Calibration gain values for CH0 and CH1
unsigned int cal_corr_CH0, cal_corr_CH1, ac_cal_corr_CH0, ac_cal_corr_CH1; // </editor-fold>

unsigned char ADHS_m, ADHS_b;

//volatile unsigned char LAcntr, LAflg;                                   //Logic analyzer counter
volatile unsigned int LAcntr;                              // Logic analyzer counter
// unsigned char i;                                         // Used in power
volatile unsigned char mask_h;                              // Logic analyzer mask_h
volatile unsigned char mask_l;                              // Logic analyzer mask_l

unsigned char rtestcntr;
unsigned int VpkIndxNmbrCH0, VpkIndxNmbrCH1;
volatile unsigned int T1InrrptCntr;
volatile unsigned long T1InrrptLEDCntr;
unsigned char TimeIntrval;
unsigned int DataCntr;
//unsigned long median;

//unsigned char LCD_mrr;


// <editor-fold defaultstate="collapsed" desc="mydelay_ms">
//**************************************************************************************
// delay x number of millseconds
//**************************************************************************************

void mydelay_ms(unsigned int cycles) {
    unsigned int i;

    for (i = 1; i <= cycles; i++) {        
        __delay_ms(1);
    }
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="mydelay_sec">
//**************************************************************************************
// delay x number of seconds
//**************************************************************************************

void mydelay_sec(unsigned int cycles) {
    unsigned char i;

    for (i = 1; i <= cycles; i++) {
        //ClrWdt();             // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        mydelay_ms(1000);
    }
}// </editor-fold>


int main (void)
{
/* 
 * Initially in init() function interrupts are disabled.
 * Interrupts are enabled in the individual menu
 * selections by the execute_function()
 */
   unsigned long i;
   unsigned char a, freez_cntr, freez_En, slpmax;

   // <editor-fold defaultstate="collapsed" desc="All initialize">
    unlockIO();
    ioMap();
    lockIO();
    Init();                                     //Enables I/Os all and select interrupts
    //NFC_init();                               //Not used with the current hardware revision
    for (i = 0; i < 524288; i++) {};
    Cal_values();                               //Apply calibration values
    _RB13 = 1;                                  //Turn ON LCD
    SNSR5VEN = SNSR5VEN_STAT;                   //Enable 5V to K1
    //_LATC1 = 1;
    GS_EN = En_msr_V;                           //Enable voltage and current read in AFE in K1    
    mydelay_ms(100);              
    initLCD();    
    RESET = 1;                                  //AFE outof reset
    write_all_reg(); //Configure MCP3911
    Read_Internal_Registers();
    clrLCD();
    SRbits.IPL = 3; //CPU interrupt priority level is 3 (11). Enable user interrupts.
    ADCH8_init();   //Battery level
    // </editor-fold>
   
    ADCH0_init();                              //Enable AN0, AN4, AN8, AN12 inputs
    selection           = 1;                   //By default enable voltage measurement
    rtestcntr           = 0;                   //Counter of resistance tests
    freez_cntr          = 0;
    freez_En            = Disable;
    _RA0                = 0; 
    
    
    CNEN1bits.CN6IE     = Disable;              //AN_CH4
    CNPD1bits.CN6PDE    = Disable;              //Pull down enable (disabled)
    CNPU1bits.CN6PUE    = Disable;   
    IEC1bits.CNIE       = Enable;
    EnableWDT(WDT_ENABLE);                      // Enable Watchdog Timer before the while (1) loop    <><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //ClrWdt();
    //WDTPS_PS32768;        //1:32,768
    //FWPSA_PR128           //Pre-scaler ratio of 1:128
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    while(1){
        ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        rng_srch = Lng_buffr;       // Used to set buffer size in INT2 interrupt routine. If rng_srch ==0
                                    // the buffer size is 2048 samples,
                                    // if rng_srch ==1 the buffer size is 256 samples          
        // Enter the MAIN MENU       
        if ((!menu_pb && menu_dwn) && (selection != 18) && (flags != 4)) {             // S2 jump to the main menu if S2 is pressed            
            // <editor-fold defaultstate="collapsed" desc="Enter the main menu: If S2 is pressed enter the main menu. If statement for the main menu">
            //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            mydelay_ms(500);
            if (!menu_pb && !menu_dwn){ // If both buttons pressed at the same time
                DebugMenu ();
            }
            __asm__ volatile("disi #0x3FFF");   // disable interrupts
            // <editor-fold defaultstate="collapsed" desc="LCD print: To test current press SELECT">
            clrLCD();
            cmdLCD((char) 0x80);
            putsLCD("To test current ");
            cmdLCD((char) 0xc0);
            putsLCD("  press SELECT  "); 
            // </editor-fold>           
            while(!menu_pb){
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="Pulsate Menu LED while user holds Menu PB">
                MENU_LED = 1;
                mydelay_ms(200);
                MENU_LED = 0;
                mydelay_ms(200); 
                // </editor-fold>
            }                   // Wait until Menu PB is released            
            for (i = 0; i < 20; i++){                
                // <editor-fold defaultstate="collapsed" desc="Flashing screen: To test current press select">
                cmdLCD((char) 0x80);
                putsLCD("To test current ");
                cmdLCD((char) 0xc0);
                putsLCD("  press SELECT  ");
                MENU_LED = 1;   // D11 is ON
                if (!menu_pb || !menu_up || !menu_dwn)
                    i = 200; // exit for loop
                mydelay_sec(1);     // CLRWDT is in mydelay_sec(1);
                clrLCD();
                MENU_LED = 0;   // D11 is OFF
                if (!menu_pb || !menu_up || !menu_dwn)
                    i = 200; // exit for loop
                mydelay_ms(300); 
                // </editor-fold>
            }
            while (!menu_pb){   //MENU PB is pressed, current is selected
                //ClrWdt();                       // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="LCD print 'OK Current test...'">
                selection = 2; // Enable current test
                cmdLCD((char) 0x80);
                putsLCD("Current Test... ");
                cmdLCD((char) 0xc0);
                putsLCD("Use Left Input  ");
                mydelay_sec(1); 
                // </editor-fold>
            }
            if (selection != 2){
                clrLCD();
                //ClrWdt();               // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="Running splash screen - Main Menue ready">            
                cmdLCD((char) 0x80); //When _RA14 is pressed the prog will jump to the next while loop
                putsLCD("niaM unem ydaer "); //Main Menu ready
                mydelay_ms(50);
                cmdLCD((char) 0x80);
                putsLCD("niMa unem ydaer "); //Main Menu ready
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD((char) 0x80);
                putsLCD("nMia unem ydaer "); //Main Menu ready
                mydelay_ms(50);
                cmdLCD((char) 0x80);
                putsLCD("Mnia unem ydaer "); //Main Menu ready
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD((char) 0x80);
                putsLCD("Mina unem ydaer "); //Main Menu ready
                mydelay_ms(50);
                cmdLCD((char) 0x80);
                putsLCD("Main unem ydaer "); //Main Menu ready
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD((char) 0x80);
                putsLCD("Main mneu ydaer "); //Main Menu ready
                mydelay_ms(50);
                cmdLCD((char) 0x80);
                putsLCD("Main meNu ydaer "); //Main Menu ready
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD((char) 0x80);
                putsLCD("Main menu ydaer "); //Main Menu ready
                mydelay_ms(50);            
                cmdLCD((char) 0x80);
                putsLCD("Main menu radey "); //Main Menu ready
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD((char) 0x80);
                putsLCD("Main menu reday "); //Main Menu ready
                mydelay_ms(50);
                cmdLCD((char) 0x80);
                putsLCD("Main menu ready "); //Main Menu ready
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
            // </editor-fold>
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="Splash screen - continue...">            
                cmdLCD(0xc0);
                putsLCD("e               ");    //continue
                mydelay_ms(100);
                cmdLCD(0xc0);
                putsLCD("ue              ");
                mydelay_ms(95);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("nue             ");
                mydelay_ms(90);
                cmdLCD(0xc0);
                putsLCD("inue            ");
                mydelay_ms(85);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("tinue           ");
                mydelay_ms(80);
                cmdLCD(0xc0);
                putsLCD("ntinue          ");
                mydelay_ms(70);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("ontinue         ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue        ");
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("continue       .");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue      . ");
                mydelay_ms(50);
                if (!menu_pb)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("continue     .  ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue    .   ");
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("continue   .    ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue  .     ");
                cmdLCD(0xc0);
                putsLCD("continue.      .");
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("continue.     . ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue.    .  ");
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("continue.   .   ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue.  .    ");
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("continue. .     ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue..      ");
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                putsLCD("continue..     .");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue..    . ");
                mydelay_ms(50);
                if (!menu_pb || !menu_dwn || !menu_up)
                    goto skip_main;
                cmdLCD(0xc0);
                putsLCD("continue..   .  ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue..  .   ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue.. .    ");
                mydelay_ms(50);
                cmdLCD(0xc0);
                putsLCD("continue...     ");
                //mydelay_ms(100);
                mydelay_sec(1);
                // </editor-fold>
skip_main:
                selection = 0;                      //Reset selection
                menu();                             //Call menu function
                //mydelay_ms(100);
            }
            __asm__ volatile("disi #0x0000"); /*enable interrupts*/
            // </editor-fold>
        }               
        // Enter the COMPONENT MENU or Debug Menu
        if (((menu_pb && !menu_dwn) && (selection != 18) && (flags != 4)) || flags == 15) {//S2 jump to the main menu if S2 is pressed                       
            mydelay_ms(500);
            if (!menu_pb && !menu_dwn){ // If both buttons pressed at the same time                
                DebugMenu ();
            }
            // <editor-fold defaultstate="collapsed" desc="Enter the component test menu: If component menu button is pressed enter the component test menu. If statement for the component test menu">
            // Press menu_up for Amps Press menu_dwn for Component menu
            __asm__ volatile("disi #0x3FFF"); /* disable interrupts */
            //cmdLCD((char) 0x80);
            clrLCD();
            //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            cmdLCD((char) 0x80); //When _RA14 is pressed the prog will jump to the next while loop
            putsLCD("tnmopnenC uenm  "); //Component menu
            mydelay_sec(1);
            cmdLCD((char) 0x80);
            putsLCD("Cnmopnent uenm  "); //Component menu
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD((char) 0x80);
            putsLCD("Comopnent uenm  "); //Component menu
            mydelay_ms(50);
            cmdLCD((char) 0x80);
            putsLCD("Comopnent uenm  "); //Component menu
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD((char) 0x80);
            putsLCD("Component uenm  "); //Component menu
            mydelay_ms(50);

            cmdLCD((char) 0x80);
            putsLCD("Component menm  "); //Component menu
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            //if (selection != 2 ){
            cmdLCD((char) 0x80);
            putsLCD("Component menu  "); //Component menu
            mydelay_ms(50);
            
            cmdLCD(0xc0);
            putsLCD("e               ");    //continue
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("ue              ");
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD(0xc0);
            putsLCD("nue             ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("inue            ");
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD(0xc0);
            putsLCD("tinue           ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("ntinue          ");
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD(0xc0);
            putsLCD("ontinue         ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue        ");
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD(0xc0);
            putsLCD("continue       .");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue      . ");
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD(0xc0);
            putsLCD("continue     .  ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue    .   ");
            mydelay_ms(50);
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD(0xc0);
            putsLCD("continue   .    ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue  .     ");
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            cmdLCD(0xc0);
            putsLCD("continue.      .");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue.     . ");
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue.    .  ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue.   .   ");
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue.  .    ");
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue. .     ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue..      ");
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            putsLCD("continue..     .");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue..    . ");
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue..   .  ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue..  .   ");
            if (!menu_pb || !menu_dwn || !menu_up)
                goto skip;
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue.. .    ");
            mydelay_ms(50);
            cmdLCD(0xc0);
            putsLCD("continue...     ");
            //mydelay_ms(100);
            mydelay_sec(1);
skip:               
            selection = 0; //Reset selection
            comp_test_menu();                   //Call menu function                
            __asm__ volatile("disi #0x0000"); /* enable interrupts */
            // </editor-fold>
        }
        //Enter Debug Menu
        if (!menu_pb && !menu_dwn && (selection != 18) && (flags != 4)){            
            DebugMenu ();
        }       
        if (flags == 19){            
            Cal_values();   // Apply calibration values               
            flags = 0;
        }
        
        ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        switch( selection )
        {
            case 1:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                //<editor-fold defaultstate="collapsed" desc="selection == 1 || selection == 2 Take voltage and or current measurement ">
                /*
                 * By default "selection" is set to 1. Selection is set to 1 every time a function is done.
                 * Both inputs measure voltage or CH0 voltage and CH1 current
                 * If selection is 2 CH1 is set to measure current. Selection is used as a flag
                 * in display_CH0CH1.c
                 */                                
                if (SNSR5VEN != SNSR5VEN_STAT)
                    SNSR5VEN = SNSR5VEN_STAT;               // Enable 5V to the board. Sensor power. Why?
                if (GS_EN != En_msr_V)
                    GS_EN = En_msr_V;                       // Enable voltage and current read in AFE in K1
                //K1 could get engaged and disengaged in INT2 interrupt routine
                if (rng_srch != Lng_buffr)
                    rng_srch = Lng_buffr;                   // Long buffer 2048 global variable. Value ised in INT2 to set the buffer size
                if (range_test_delay != 3)
                    range_test_delay = 3;                    // Long delay global variable used in the delay "for loop".
                
                // <editor-fold defaultstate="collapsed" desc="range_test()- Runs when both dividers are not 0. Enacts voltage measurements. Resets dividers. Updates accumulators. No display.">
                /*
                 * Below:
                 * range_test() function will run only if both dividers are not zero.
                 * It continuously reads applied voltage with a previously selected voltage divider.
                 * Calls data_comp_CH0 () - loc. voltage_compute_CH0_Rev04x
                 * and data_comp_CH1 () - loc. current_compute_CH1_Rev03x functions
                 * Verifies selected voltage ranges and resets if dividorCH0 and/or dividorCH1 if one of them
                 * is out of range. Refreshes voltage_val and current_val
                 * This function is located in the voltage_divider.c file.
                 * range_test() does NOT have display
                 */
                 range_test();  //File: Range Test Rev04x.c
                
                // <editor-fold defaultstate="collapsed" desc="TstAndResetVoltDiv(), execute_function(): take voltage readings on both channels with previously selected dividers">
                //rx_function = 13;           //Collect data without transmitting to PC. This function enavbles INT2 and OC2
                //execute_function();         //Start cycle. Collect data samples
                //while (!buffer_full) {}     //Wait until all data is collected                
                //TstAndResetVoltDiv(cflag,rng_srch,c_smple_sum,current_sum_n,current_sum_p,cal,
                //        CH1vsftSgn,CH1vshft,dividerCH1,vflag,v_smple_sum,voltage_sum_n,voltage_sum_p,
                //        CH0vsftSgn,CH0vshft,dividerCH0,buffer_full);                               
                // </editor-fold>
                                           
                // </editor-fold>

                //if ((cflag == 0x07) && (frequency_cntrCH1 == 6)) {       
                if (cflag == 0x07){                             // If AC voltage detected                    
                    display_freq_CH1();                         // Display frequency CH1                   
                }

                //if ((vflag == 0x07) && (frequency_cntrCH0 == 6)) {       
                if ((vflag == 0x07) && (CH0_disp_frz == Disable)){   // If AC voltage detected and channel is not frozen
                    display_freq_CH0();                         // Display frequency CH0                    
                }
                
                while ((!menu_up && freez_En == Disable)&&(selection != 18)) {
                    // <editor-fold defaultstate="collapsed" desc="'Menu up' push button controlled 'save current reading' count down timer init.">
                    /*
                     * When user presses menu_up PB get ready to save the current reading on CH0.
                     *
                     */
                    //_RC2 - S4
                    for (i = 0; i < 124288; i++) {} //menu_up Press and Hold delay
                    freez_En    = Enable;           //Enable Freez counter
                    Buzzer      = Enable;           //Turn on buzzer
                    freez_cntr  = 0;                //Reset freez counter
                    for (i = 0; i < 124288; i++) {} //Buzzer ON time delay
                    Buzzer = Disable;
                    mydelay_sec(1);
                    cmdLCD(0x80); //writeLCD(0x00, 0xc0) 0000 0000, 1100 0000
                    putsLCD("Ready to store  "); //CH0_disp_frz. putLCD( *s++);writeLCD( LCDDATA, (d)); LCDDATA = 0x01
                    cmdLCD(0xc0); //writeLCD(0x00, 0xc0) 0000 0000, 1100 0000
                    putsLCD("reading on INPT1"); //CH0_disp_frz. putLCD( *s++);writeLCD( LCDDATA, (d)); LCDDATA = 0x01
                    mydelay_sec(3);
                    // </editor-fold>
                }               
                
                // Unfreeze the display on Input1
                while ((!menu_up && freez_En == Enable)&&(selection != 18)) {
                    // <editor-fold defaultstate="collapsed" desc="'Menu up' unfreeze display. Push button controlled. Dependant on: (!menu_up) && (CH0_disp_frz == 1)">
                    for (i = 0; i < 124288; i++) {
                    } //menu_up Press and Hold delay
                    CH0_disp_frz = 0; //UNFREEZE
                    freez_En    = Disable;          //Disable Freez counter
                    Buzzer      = Enable;           //Turn on buzzer
                    freez_cntr  = 0;                //Reset freez counter
                    for (i = 0; i < 124288; i++) {
                    }//Buzzer ON time delay
                    Buzzer = Disable;
                    sleep_counter = 0;//Sleep counter is reset
                    cmdLCD(0x80); //Print the following on the second line
                    putsLCD("Auto Running... "); //CH0_disp_frz
                    cmdLCD(0xc0); //Print the following on the second line
                    putsLCD("Auto Running... "); //CH0_disp_frz
                    for (a = 0; a < 20; a++) {
                        for (i = 0; i < 124288; i++) {} //delay
                    }
                    // </editor-fold>
                }
                
                //If at least one of the dividers has been reset by the range_test() call
                //voltage_divider () function. With buffer size 256 samples to determine new divider
                if ((dividerCH0 == 0) || (dividerCH1 == 0)) {
                    rng_srch            = Shrt_buffr;               //Short buffer 256 samples: rng_srch = 1
                    range_test_delay    = 1;                        //Short delay time 3 is maximum
                    
                    while ((!menu_up) && (freez_En == Disable)) {   //_RC2 - S4
                        // <editor-fold defaultstate="collapsed" desc="'Menu up' freeze display. Push button controlled. Dependant on: (!menu_up) && (CH0_disp_frz == 0)">
                        /*
                         * When user presses menu_up PB get ready to save the current reading on CH0.
                         *
                         */
                        for (i = 0; i < 124288; i++) {}             //menu_up Press and Hold delay
                        freez_En    = Enable;                       //Enable Freez counter
                        Buzzer      = Enable;                       //Turn on buzzer
                        freez_cntr  = 0;                            //Reset freez counter
                        for (i = 0; i < 124288; i++) {}             //Buzzer ON time delay
                        Buzzer = Disable;                        
                        mydelay_sec(1);
                        cmdLCD(0x80); //writeLCD(0x00, 0xc0) 0000 0000, 1100 0000
                        putsLCD("Ready to store  "); //CH0_disp_frz. putLCD( *s++);writeLCD( LCDDATA, (d)); LCDDATA = 0x01
                        cmdLCD(0xc0); //writeLCD(0x00, 0xc0) 0000 0000, 1100 0000
                        putsLCD("reading on INPT1"); //CH0_disp_frz. putLCD( *s++);writeLCD( LCDDATA, (d)); LCDDATA = 0x01
                        mydelay_sec(3);
                        // </editor-fold>
                    }                    
                   
                    while ((!menu_up) && (freez_En == Enable)) {
                        // <editor-fold defaultstate="collapsed" desc="'Menu up' unfreeze display. Push button controlled. Dependant on: (!menu_up) && (CH0_disp_frz == 1)">
                        //Unfreeze display
                        for (i = 0; i < 124288; i++) {
                        } //menu_up Press and Hold delay
                        CH0_disp_frz = 0; //UNFREEZE
                        freez_En    = Disable;          //Disable Freez counter
                        Buzzer      = Enable;           //Turn on buzzer
                        freez_cntr  = 0;                //Reset freez counter
                        for (i = 0; i < 124288; i++) {
                        }//Buzzer ON time delay
                        Buzzer = Disable;
                        sleep_counter = 0;//Sleep couneter is reset
                        cmdLCD(0x80); //Print the following on the second line
                        putsLCD("Auto Running... "); //CH0_disp_frz
                        cmdLCD(0xc0); //Print the following on the second line
                        putsLCD("Auto Running... "); //CH0_disp_frz
                        for (a = 0; a < 20; a++) {
                            for (i = 0; i < 124288; i++) {
                            } //delay
                        }
                        // </editor-fold>
                    }
                    /*
                     * The following function will find an appropriate range with short buffer enabled and short delay time
                     * voltage_divider () will go through all the ranges and enable the correct one.
                     * voltage_divider () has its own display function
                     */
                    voltage_divider();  //If at least one of the ranges was found to be wrong or is not set yet find a new correct one.
                    sleep_counter = 0;  //Since one of the voltage dividers has changed the instrument is being used and
                                        //sleep_counter needs to be reset to allow the instrument to operate                    
                }
                else {
                    //If both voltage dividers are the same as before that means that the voltage on both channels did
                    //not change.
                    displayCH0CH1();                                //Display measured values with previously selected dividers
                    /*
                     * Collect statistics and compute median
                     */
                    //median = median_CH0 ();           // Returns unsigned long median value of 10 voltage_val
                } // </editor-fold>
                break;
            case 2:                   
                // <editor-fold defaultstate="collapsed" desc="Measuting Current">
                if (SNSR5VEN != SNSR5VEN_STAT)
                    SNSR5VEN = SNSR5VEN_STAT; // Enable 5V to the board. Sensor power. Why?
                if (GS_EN != En_msr_V)
                    GS_EN = En_msr_V; // Enable voltage and current read in AFE in K1
                //K1 could get engaged and disengaged in INT2 interrupt routine
                if (rng_srch != Lng_buffr)
                    rng_srch = Lng_buffr;   // Long buffer 2048 global variable. Value ised in INT2 to set the buffer size
                if (range_test_delay != 3)
                    range_test_delay = 3;   // Long delay global variable used in the delay "for loop".
                
                if (flags != 22){                                    
                    Isetup();
                    _LATA9 = 1;             // Enable current read on CH0                                  
                }
                overcurrent(); // Continuously test for overcurrent  
                Imeasure(); 
                // </editor-fold>                                              
                break;
            case 3:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 3 Display Frequency">
                //Display Frequency
                SNSR5VEN = SNSR5VEN_STAT;   //Enable 5V to K1
                GS_EN = En_msr_V;           //Enable voltage and current read in AFE in K1
                range_test();               //Verify selected voltage ranges. range_test() will reset to zero (dividorCH0==0)||(dividorCH1==0) if one of the input
                //voltages is out of range.
                if ((dividerCH0 == 0) || (dividerCH1 == 0)) {//If any of the voltage ranges get reset, run voltage dividor function again
                    voltage_divider();      //If one of the ranges was found to be wrong find a new one
                    display_freq_CH0CH1();
                } else {
                    display_freq_CH0CH1();
                }
                // </editor-fold>
                break;
            case 5:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 5 Capacitance test">
                //5.Capacitance test   ==================================================================
                //This function is called from PC as well
                /*
                 * Capacitance test enables charging of an external capacitor and
                 * applies output charging waveform to the AFE CH0 as well as to AN4 MCU, through K2 relay.
                 * K2 has to be powered or GS_EN has to be set to 1. And SNSR5VEN has to be set to 1
                 * as well.
                 */
                // <editor-fold defaultstate="collapsed" desc="Disable interrupts: IEC1bits.INT2IE = 0, IEC0bits.OC2IE = 0">
                IEC1bits.INT2IE = 0; //INT2IE: External Interrupt 2 Enable bit. RPINR1 INT2R<5:0>
                IEC0bits.OC2IE  = 0; //OC2 is used to send data to PC thru UART TX
                // </editor-fold>
                if (flags == 2) {           // This flag is set to 2 in the component menu. The call came from user menu, not PC
                    a = 0;
                    while (menu_up && (a < 10) && menu_pb && menu_dwn) {
                        /*
                         * Pressing any of the buttons will make escape from the loop.  main menu to get back
                         * to the main menu. Pressing and holding menu_dwn will take user back to the component menu.
                         * Pressing and holding menu_pb will take the
                         * The program will stay in this while loop until one of the PBs is pressed
                         * !menu_up or !menu_pb, or when counter a is greater then 10
                         *
                         */
                        //clrLCD();
                        capacitance_display();
                        mydelay_sec(1);
                        selection   = 5;
                        flags       = 0;
                        a++;
                    }
                    cmdLCD((char) 0x80); // Place cursor 1st line loc 0
                    putsLCD(" Now Exiting... "); //First line print
                    cmdLCD(0xc0); // Place cursor 2nd line loc 0
                    putsLCD(" Now Exiting... ");   
                    mydelay_sec(2);
                    selection       = 1;
                    flags           = 0;        //Reset the flag
                }
            if (flags == 5) { //Flags is equal to 5 when call came from PC
                //Set program ready for the next call from PC. Once it is out of this if-statement
                //the program will continue with voltage measurements
                selection       = 1;
                flags           = 0;      //Reset flags
            }
            if (flags == 3) {
                capacitance_display();
                flags = 0;
                //            send_UART(&tx_buf);
            }
            // <editor-fold defaultstate="collapsed" desc="Reinable interrupts: IEC1bits.INT2IE = 1, IEC0bits.OC2IE = 1">
            IEC1bits.INT2IE = 1; //INT2IE: External Interrupt 2 Enable bit. RPINR1 INT2R<5:0>
            IEC0bits.OC2IE  = 1; //OC2 is used to send data to PC thru UART TX
        // </editor-fold>
        // </editor-fold>
                break;
            case 6:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 6 Vtest on CH0">
                //6. Vtest on CH0 =======================================================================
                //SNSR5VEN = SNSR5VEN_STAT;       //Enable 5V to K1
                GS_EN = 0; //AFE K1 connect AN_CH1 to AFE_CH0_sel. AN_CH1 is the voltage sense waveform output
                //K1 could get engaged and disengaged in INT2 interrupt routine
                //              if(autorange==1){//If autorange is enabled
                range_test(); //Verify selected voltage ranges
                if ((dividerCH0 == 0) || (dividerCH1 == 0)) {
                    voltage_divider(); //If one of the ranges was found to be wrong find a new one
                } else {
                    displayCH0CH1();
                }
                // </editor-fold>
                break;
            case 7:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 7 Display test values">
                //7. Display test values============================================================================

                SNSR5VEN = SNSR5VEN_STAT; //Enable 5V to K1
                GS_EN = En_msr_V; //AFE K1 connect AN_CH1 to AFE_CH0_sel. AN_CH1 is the voltage sence waveform output
                range_test(); //This function will load values into the buffer
                if ((dividerCH0 == 0) || (dividerCH1 == 0)) {
                    voltage_divider(); //If one of the ranges was found to be wrong find a new one
                } else {//If both voltage dividers are selected call display_hex_val.
                    //v_min_counter and v_max_counter values are computed in the OC2 Interrupt
                    //                        if(buffer_full==1) v_min_counter, v_max_counter;
                    //                            display_hex_val ((unsigned long) v_min_counter, (unsigned long) v_max_counter);
                    hexdec_long((unsigned long) v_min_counter);
                    writeLCD(0x00, 2);
                    //                           for (i=0;(tx_buf[i]-0x30)==0;i++);  //Removes front zeros from the remainder result
                    //                            putsLCD((char *)(&tx_buf[i]));      //Will stop printing when empty (non-printable) char is encountered
                    putsLCD((char *) (&tx_buf[0]));
                    putsLCD(" NEGp ");

                    hexdec_long((unsigned long) v_max_counter); //Convert value into a string
                    cmdLCD(0xc0);
                    //                            for (i=0;(tx_buf[i]-0x30)==0;i++);  //Removes front zeros from the remainder result
                    //                            putsLCD((char *)(&tx_buf[i]));      //Will stop printing when empty (non-printable) char is encountered
                    putsLCD((char *) (&tx_buf[0]));
                    putsLCD(" POSp ");
                }
        // </editor-fold>
                break;
            case 8:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 8 Manual range select 10k">
                //if (((sleep_counter%10) == 0) || ((sleep_counter%11) == 0) || ((sleep_counter%12) == 0)){
                    //8. Manual range select 10k
                    SNSR5VEN = SNSR5VEN_STAT;   //Enable 5V to K1
                    GS_EN = En_msr_V;           //AFE K1 connect AN_CH1 to AFE_CH0_sel. AN_CH1 is the voltage sence waveform output
                    CH0_450V_rng();
                    CH1_450V_rng();
                    rx_function = 13;           //Collect data without transmitting to PC. This function enavbles INT2 and OC2
                    execute_function();         //Start cycle. Collect 2048 data samples
                    while (!buffer_full) {}
                    // <editor-fold defaultstate="collapsed" desc="Disable interrupts: INT2IE = 0, OC2IE = 0 ">
                    IEC1bits.INT2IE = 0;    //INT2IE: External Interrupt 2 Enable bit. RPINR1 INT2R<5:0>
                    IEC0bits.OC2IE  = 0;    //OC2 is used to send data to PC thru UART TX
                    // </editor-fold>
                    voltage_val = data_comp_CH0(vflag, rng_srch, v_smple_sum, voltage_sum_n, voltage_sum_p, cal, CH0vsftSgn, CH0vshft, dividerCH0);
                    current_val = data_comp_CH1(cflag, rng_srch, c_smple_sum, current_sum_n, current_sum_p, cal, CH1vsftSgn, CH1vshft, dividerCH1);
                    voltage_acc = 0;
                    current_acc = 0;                    
                    dividerCH0 = 10;
                    dividerCH1 = 10;
                    displayCH0CH1();
                //}
//                else{
//                    hexdec_long((unsigned long)CH010kVal);
//                    cmdLCD(0x80);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH0:  ");
//                    putsLCD((char *) (tx_buf));
//                    
//                    hexdec_long((unsigned long)CH110kVal);
//                    cmdLCD(0xc0);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH1:  ");
//                    putsLCD((char *) (tx_buf));
//                    mydelay_sec(3);
//                }
                // </editor-fold>
                break;
            case 9:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 9 Manual range select 80k">
                //if (((sleep_counter%10) == 0) || ((sleep_counter%11) == 0) || ((sleep_counter%12) == 0)){
                    //Manual range select 80k
                    SNSR5VEN = SNSR5VEN_STAT; //Enable 5V to K1
                    GS_EN = En_msr_V; //AFE K1 connect AN_CH1 to AFE_CH0_sel. AN_CH1 is the voltage sence waveform output
                    CH0_110V_rng ();
                    CH1_110V_rng ();
                    rx_function = 13; //Collect data without transmitting to PC. This function enavbles INT2 and OC2
                    execute_function(); //Start cycle. Collect 2048 data samples
                    //                for(a=0;a<b;a++){                   //Originally a= 4
                    //                    for(i=0;i<524288;i++){}         //delay
                    //                }
                    while (!buffer_full) {}
                    // <editor-fold defaultstate="collapsed" desc="Disable interrupts: INT2IE = 0, OC2IE = 0 ">
                    IEC1bits.INT2IE = 0;    //INT2IE: External Interrupt 2 Enable bit. RPINR1 INT2R<5:0>
                    IEC0bits.OC2IE  = 0;    //OC2 is used to send data to PC thru UART TX
                    // </editor-fold>
                    voltage_val = data_comp_CH0(vflag, rng_srch, v_smple_sum, voltage_sum_n, voltage_sum_p, cal, CH0vsftSgn, CH0vshft, dividerCH0);
                    current_val = data_comp_CH1(cflag, rng_srch, c_smple_sum, current_sum_n, current_sum_p, cal, CH1vsftSgn, CH1vshft, dividerCH1);
                    voltage_acc = 0;
                    current_acc = 0;                    
                    dividerCH0 = 80;
                    dividerCH1 = 80;
                    displayCH0CH1();
                //}
//                else{
//                    hexdec_long((unsigned long)CH080kVal);
//                    cmdLCD(0x80);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH0:  ");
//                    putsLCD((char *) (tx_buf));
//                    
//                    hexdec_long((unsigned long)CH180kVal);
//                    cmdLCD(0xc0);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH1:  ");
//                    putsLCD((char *) (tx_buf));
//                    mydelay_sec(3);
//                }
                // </editor-fold>
                break;
            case 10:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 10 Manual range select 510k">
                //if (((sleep_counter%10) == 0) || ((sleep_counter%11) == 0) || ((sleep_counter%12) == 0)){
                    //Manual range select INF
                    SNSR5VEN = SNSR5VEN_STAT; //Enable 5V to K1
                    GS_EN = En_msr_V; //AFE K1 connect AN_CH1 to AFE_CH0_sel. AN_CH1 is the voltage sence waveform output
                    CH0_18V_rng ();
                    CH1_18V_rng ();
                    rx_function = 13; //Collect data without transmitting to PC. This function enavbles INT2 and OC2
                    execute_function(); //Start cycle. Collect 2048 data samples
                    //                for(a=0;a<b;a++){                   //Originally a= 4
                    //                    for(i=0;i<524288;i++){}         //delay
                    //                }
                    while (!buffer_full) {}
                    // <editor-fold defaultstate="collapsed" desc="Disable interrupts: INT2IE = 0, OC2IE = 0 ">
                    IEC1bits.INT2IE = 0;    //INT2IE: External Interrupt 2 Enable bit. RPINR1 INT2R<5:0>
                    IEC0bits.OC2IE  = 0;    //OC2 is used to send data to PC thru UART TX
                    // </editor-fold>
                    voltage_val = data_comp_CH0(vflag, rng_srch, v_smple_sum, voltage_sum_n, voltage_sum_p, cal, CH0vsftSgn, CH0vshft, dividerCH0);
                    current_val = data_comp_CH1(cflag, rng_srch, c_smple_sum, current_sum_n, current_sum_p, cal, CH1vsftSgn, CH1vshft, dividerCH1);
                    voltage_acc = 0;
                    current_acc = 0;
                    dividerCH0 = 51;
                    dividerCH1 = 51;                    
                    displayCH0CH1();
                //}
//                else{
//                    hexdec_long((unsigned long)CH0510kVal);
//                    cmdLCD(0x80);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH0:  ");
//                    putsLCD((char *) (tx_buf));
//                    
//                    hexdec_long((unsigned long)CH1510kVal);
//                    cmdLCD(0xc0);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH1:  ");
//                    putsLCD((char *) (tx_buf));
//                    mydelay_sec(3);
//                }
                                    
                // </editor-fold>
                break;
           case 11:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 11 Send Long Buffer to PC (2048 bytes) and if flags ==7 send char string">
                //Send Long Buffer to PC (2048 bytes)
                //This case is enable from a PC.
                SNSR5VEN    = SNSR5VEN_STAT;    // Enable power to the board
                GS_EN       = En_msr_V;         // Enable voltage and current read in AFE in K1
                max_TR_Buff = buffer_lenght;    // Number of data-points to transmit
                //while(selection==11){

                rng_srch    = Lng_buffr;        // Long buffer 2048 samples
                //range_test_delay = 3;
                rx_function = 13; //Collect data without transmitting to PC. This function enables INT2 and OC2
                //Collected data is transferred to PC when TR_EN = 1;
                //OC2 interrupt transmits the data
                execute_function(); //Enable interrupts. Start cycle. Collect 2048 data samples. After completion of an interrupt the program will get into
//              for(a=0;a<range_test_delay;a++){ //the delay loop. range_test_delay is set according to the buffer size.
//                  for(i=0;i<262144;i++){}         //For 2048 samples it is 3, for 256 it is 1.
//              }
                while (!buffer_full) {}
                //}
                flags = 0;
                TR_EN = 0; //Disables data transfer to a PC. At this point data has been transmitted to PC
        // </editor-fold>
                break;
           case 12:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 12 Resistance test">              
                /*
                 * Menu function and communication protocol function set selection equal to 12 and make no
                 * function calls
                 * 
                 */                             
                resistance_display();        //Has its own LCD display function
                if (flags == 6) {
                    // <editor-fold defaultstate="collapsed" desc="If flags == 6, Display LCD Resistance test LabView UI call">
                    writeLCD(0x00, 2);
                    putsLCD("Resistance test ");
                    cmdLCD(0xc0); //Print the following on the second line
                    putsLCD("LabView UI call ");
                    mydelay_sec(2);  
                    flags = 0;
                    // </editor-fold>
                }
                else{//When flags == 6 the request came from PC so skip the following
                    mydelay_sec(2);                   
                    //Make a trap for the program at this point              
                    if (rtestcntr < 10) {
//                        if ((rtestcntr == 3) || (rtestcntr == 6)){
//                            cmdLCD((char) 0x80);
//                            //putsLCD(" Now Exiting....");
//                            putsLCD("To exit press and");
//                            cmdLCD((char) 0xc0);
//                            putsLCD("hold menu DOWN   ");
//                            mydelay_sec(2);
//                            clrLCD();
//                        }

                        // <editor-fold defaultstate="collapsed" desc="selection = 12; rtestcntr++;">
                        selection = 12;
                        rtestcntr++;
                        // </editor-fold>
                    }
                    else {//If 'rtestcntr >= 10 stop taking resistance readings (selection = 1) and reset counter
                        // <editor-fold defaultstate="collapsed" desc="selection = 1; rtestcntr = 0;">
                        selection = 1; // Go back to voltage measurements
                        rtestcntr = 0; //Reset function call counter
                        Timer1_stop();
                        _LATA0 = 0;
                        // </editor-fold>
                    }
                }                               
                if (!menu_dwn){
                    rtestcntr = 12;
                    cmdLCD((char) 0x80);
                    putsLCD(" Now Exiting....");
                    cmdLCD((char) 0xc0);
                    putsLCD(" Now Exiting....");
                    mydelay_sec(3);
                    selection = 1;
                    Timer1_stop();
                    _LATA0 = 0;
                }
                flags = 12;
                // </editor-fold>
                break;
           case 13:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 13 Send Short Buffer to PC (256 bytes) and if flags ==7 send char string">
                //Send Long Buffer to PC (2048 bytes)
                //This case is enable from a PC.
                SNSR5VEN    = SNSR5VEN_STAT;    // Enable power to the board
                GS_EN       = En_msr_V;         // Enable voltage and current read in AFE in K1
                max_TR_Buff = buffer_256;       // Number of data-points to transmit
                //while(selection==11){

                rng_srch    = Shrt_buffr;        // Long buffer 2048 samples
                //range_test_delay = 3;
                rx_function = 13; //Collect data without transmitting to PC. This function enables INT2 and OC2
                //Collected data is transferred to PC when TR_EN = 1;
                //OC2 interrupt transmits the data
                execute_function(); //Enable interrupts. Start cycle. Collect 2048 data samples. After completion of an interrupt the program will get into
//              for(a=0;a<range_test_delay;a++){ //the delay loop. range_test_delay is set according to the buffer size.
//                  for(i=0;i<262144;i++){}         //For 2048 samples it is 3, for 256 it is 1.
//              }
                while (!buffer_full) {}
                //}
                //if(buffer_full==1){
                flags = 0;
                TR_EN = 0; //Disables data transfer to a PC. At this point data has been transmitted to PC

        // </editor-fold>
                break;
           case 14:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 14 Diode test">
                //Diode test -------------------------------------------------------------------------------------------------------
                diode_display();
                for (a = 0; a < 30; a++) {
                    for (i = 0; i < 124288; i++) {
                    } //delay
                }
                //Make a trap for the program at this point
                writeLCD(0x00, 2);
                putsLCD("To test press UP");
                while ((menu_up) && (menu_pb)); //Wait untill the user will press menu button or UP button
                for (i = 0; i < 424288; i++) {
                } //delay
                // </editor-fold>
                break;
           case 15:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 15 Datalogger -----> ">
                /* Data-logger
                 * The first sector is used by the calibration values.
                 * The first sector extends from 0x000000 to 0x000FFF
                 * The second sector starts at 0x000FFF + 1 = 0x001000 to 0x001000+4095 = 0x001000 + 0x000FFF = 1FFF
                 * For each sector the starting address will be the sum of last address from the previous sector plus one.
                 * Each sector is 4096 long

                 */
                //Data recording. The first sector (4096 addresses) is occupied by the calibration values
                datalogger ();
                // </editor-fold>
                break;
           case 16:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 16 Send Chart data">
                //Send Chart data
                //This case is enable from a PC.
                IEC1bits.INT2IE = 0; //The program will be here between the computer calls. Disable INT2 since no data should
                //be collected untill there is a function call from PC.
                IEC0bits.OC2IE = 0;
                SNSR5VEN = SNSR5VEN_STAT; //Enable 5V to K1
                GS_EN = En_msr_V; //Enable voltage and current read in AFE in K1
                //max_TR_Buff=buffer_lenght-1;    //Number of datapoints to transmit
                //K1 could get engaged and disengaged in INT2 interrupt routine
                writeLCD(0x00, 2);
                putsLCD("PC Chart data   ");
                cmdLCD(0xc0);
                putsLCD("transfer........");

                //rng_srch = 0;   //Long buffer 2048 samples
                //range_test_delay = 3;
                //rx_function = 13;                   //Collect data without transmitting to PC. This function enavbles INT2 and OC2
                //Collected data is transferred to PC when TR_EN = 1;
                //OC2 interrupt transmitts the data
                //execute_function();                 //Enable interrupts. Start cycle. Collect 256 data samples. After completion of an interrupt the program will get into
                //for(a=0;a<range_test_delay;a++){    //the delay loop. range_test_delay is set according to the buffer size.
                //    for(i=0;i<262144;i++){}         //For 2048 samples it is 3, for 256 it is 1.
                //}
                //if(buffer_full==1){
                //    TR_EN = 0;      //Disables data transfer to a PC
                //}
                // </editor-fold>
                break;
           case 17:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 17 NFC this function will display setting sent to NFC">
                //NFC this function will display setting sent to NFC
                Trf797xInitialSettings(); //will display data on LCD
                // </editor-fold>
                break;
           case 18:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 18 Pulse generator">
                //Pulse generator                
                if (!menu_pb && (selection == 18)){
                    OC4CON1 = 0;        // Disable OC4
                    OC4CON2 = 0; 
                    writeLCD(0x00, 2);
                    putsLCD(" Exiting pulse  ");
                    cmdLCD(0xc0);                       //Place cursor 2nd line loc 0
                    putsLCD("generator...    "); 
                    mydelay_sec(2);
                    selection = 1;
                }
                else{
                    selection = 18;      // Nop()             
                }
                if (!menu_up && (selection == 18)){
                    PWM_dty = PWM_dty + 25;
                    PWM_prd = PWM_prd + 25;
                }
                if (!menu_dwn && (selection == 28)){
                    PWM_dty = PWM_dty - 25;
                    PWM_prd = PWM_prd - 25;
                }
                function_gen((unsigned int) PWM_dty, (unsigned int) PWM_prd);                                                
                selection = 18;                      
            // </editor-fold>
                break;
           case 19:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 19 Do nothing. Selection 19 is set in the communication protocol by case 25">
                /*
                 * Do nothing. Selection 19 is set in the communication protocol by case 25 to prevent
                 * changing text on the LCD display by the main() function
                 */                   
                Nop();                
                // </editor-fold>
                break;
           case 20:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                power ();
                if (!menu_dwn){
                    rtestcntr = 12;
                    cmdLCD((char) 0x80);
                    putsLCD(" Now Exiting....");
                    cmdLCD((char) 0xc0);
                    putsLCD(" Now Exiting....");
                    for (a = 0; a < 20; a++) {
                        for (i = 0; i < 124288; i++) {} //delay                            
                    }                    
                }
                break;
           case 21:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                cmdLCD((char)0x80);
                putLCD(U1);
                cmdLCD((char)0xC0);
                putLCD(L1);
                break;
            case 22:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="Battery charging status">
                if (!_RB9 && !_RB10) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Battery Status: ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("  No Battery    ");                   
                }
                else if (_RB9 && !_RB10) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Battery Status: ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("  Charging...   ");                    
                }
                else if (!_RB9 && _RB10) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Battery Status: ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Charge Complete ");                    
                }
                else if (_RB9 && _RB10) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Battery Status: ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("   Error...     ");                    
                }
                else if (!_RB11) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD(" The Charger is ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD(" powered down   ");                   
                }
                else{
                    
                }
                mydelay_sec(3);
                while (!menu_pb) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD(" Exiting....    ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD(" Exiting....    ");
                }
                selection = 1; // </editor-fold>
                break;  
            case 23:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="'a' - Resistance test 270k">  
                SNSR5VEN                = SNSR5VEN_STAT;    // Enables 5V pulse voltage.
                GS_EN                   = En_elmnt_rd;      // AFE: K1 connect MQ5_A to MCP3911 CH1.
                Cap_Ind_5V_En           = Enable;           // Enables 5V to the VSENCE page "Cap_Ind_5V_En" the switch is located on Power1 page
                R27k                    = Disable;
                R270                    = Disable;          //_RD2 controls 270 Ohm resistor on both old and the new hardware
                R2_7                    = Disable;          //_RC14 controls 2.7 Ohm resistor on both old and the new hardware
                R2_7k                   = Disable;          //_RC13 controls 2.7K
                SHUNT                   = Disable;          // Disable shunt MOSFET (Q41 SDIT)
                rng_srch                = Lng_buffr; 
                TR_EN                   = 0;                // Disable data transfer in OC2              
                SHUNT                   = Disable;                          // Turn OFF the shunt MOSFET.  
                CH1_sel                 = 1;                // Enables data load into the data buffer in INT routine and transmits data-point in OC2
                CH0_sel                 = 1; 
                rx_function             = 13;               // Collect data without transmitting to PC. This function enables INT2 and OC2

                cmdLCD((char)0x80);
                putsLCD("Resistance test ");
                cmdLCD((char)0xc0);
                putsLCD("270k shunt, PCUI");
                execute_function();                         // Start cycle. Collect 2048 data samples. OC2 will compute and assign a value to 'lst_'smpl.       
                while (!buffer_full) {}    
                // <editor-fold defaultstate="collapsed" desc="Reset: counter_buffer = 0, IEC1bits.INT2IE = 0, IEC0bits.OC2IE = 0">   
                IEC1bits.INT2IE = 0;                //Disabling INT2 interrupt will allow writing the following 0x00
                IEC0bits.OC2IE  = 0;                //into OC2 buffer. Since OC2 is still running
                counter_buffer  = 0;
                // </editor-fold>
                send_UART("Resistance Range is 1.7M to 17M ");
                // <editor-fold defaultstate="collapsed" desc="Send previously collected data to PC. Resistance on CH1. Passed">
                    for (i = 0; i < 2048; i++) {
                        while (!U1STAbits.TRMT);//Resistance is measured on CH1 only
                        U1TXREG = voltage_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_lsb[i];

                        while (!U1STAbits.TRMT);
                        U1TXREG = current_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_lsb[i];
                    }
                // </editor-fold>
                //rx_function = 0;
                flags = 0;
                // </editor-fold>
                break;
            case 24: 
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="'b' - Resistance test 27k">  
                SNSR5VEN                = SNSR5VEN_STAT;    // Enables 5V pulse voltage.
                GS_EN                   = En_elmnt_rd;      // AFE: K1 connect MQ5_A to MCP3911 CH1.
                Cap_Ind_5V_En           = Enable;           // Enables 5V to the VSENCE page "Cap_Ind_5V_En" the switch is located on Power1 page
                R27k                    = Enable;
                R270                    = Disable;          //_RD2 controls 270 Ohm resistor on both old and the new hardware
                R2_7                    = Disable;          //_RC14 controls 2.7 Ohm resistor on both old and the new hardware
                R2_7k                   = Disable;          //_RC13 controls 2.7K
                SHUNT                   = Disable;          // Disable shunt MOSFET (Q41 SDIT)
                rng_srch                = Lng_buffr; 
                TR_EN                   = 0;                // Disable data transfer in OC2              
                //SHUNT                   = Disable;                          // Turn OFF the shunt MOSFET.  
                CH1_sel                 = 1;                // Enables data load into the data buffer in INT routine and transmits data-point in OC2
                CH0_sel                 = 1; 
                rx_function             = 13;               // Collect data without transmitting to PC. This function enables INT2 and OC2

                cmdLCD((char)0x80);
                putsLCD("Resistance test ");
                cmdLCD((char)0xc0);
                putsLCD("27k shunt, PCUI ");
                execute_function();                         // Start cycle. Collect 2048 data samples. OC2 will compute and assign a value to 'lst_'smpl.       
                while (!buffer_full) {}    
                // <editor-fold defaultstate="collapsed" desc="Reset: counter_buffer = 0, IEC1bits.INT2IE = 0, IEC0bits.OC2IE = 0">   
                IEC1bits.INT2IE = 0;                //Disabling INT2 interrupt will allow writing the following 0x00
                IEC0bits.OC2IE  = 0;                //into OC2 buffer. Since OC2 is still running
                counter_buffer  = 0;
                // </editor-fold>
                send_UART("Resistance Range is 170k to 1.7M");
                // <editor-fold defaultstate="collapsed" desc="Send previously collected data to PC. Resistance on CH1. Passed">
                    for (i = 0; i < 2048; i++) {
                        while (!U1STAbits.TRMT);//Resistance is measured on CH1 only
                        U1TXREG = voltage_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_lsb[i];

                        while (!U1STAbits.TRMT);
                        U1TXREG = current_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_lsb[i];
                    }
                // </editor-fold>
                //rx_function = 0;
                flags = 0;
            // </editor-fold>
                break;
            case 25:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="'c' - Resistance test 2.7k">  
                SNSR5VEN                = SNSR5VEN_STAT;    // Enables 5V pulse voltage.
                GS_EN                   = En_elmnt_rd;      // AFE: K1 connect MQ5_A to MCP3911 CH1.
                Cap_Ind_5V_En           = Enable;           // Enables 5V to the VSENCE page "Cap_Ind_5V_En" the switch is located on Power1 page
                R27k                    = Disable;
                R270                    = Disable;          //_RD2 controls 270 Ohm resistor on both old and the new hardware
                R2_7                    = Disable;          //_RC14 controls 2.7 Ohm resistor on both old and the new hardware
                R2_7k                   = Enable;          //_RC13 controls 2.7K
                SHUNT                   = Disable;          // Disable shunt MOSFET (Q41 SDIT)
                rng_srch                = Lng_buffr; 
                TR_EN                   = 0;                // Disable data transfer in OC2              
                //SHUNT                   = Disable;                          // Turn OFF the shunt MOSFET.  
                CH1_sel                 = 1;                // Enables data load into the data buffer in INT routine and transmits data-point in OC2
                CH0_sel                 = 1; 
                rx_function             = 13;               // Collect data without transmitting to PC. This function enables INT2 and OC2

                cmdLCD((char)0x80);
                putsLCD("Resistance test ");
                cmdLCD((char)0xc0);
                putsLCD("2.7k shunt, PCUI");
                execute_function();                         // Start cycle. Collect 2048 data samples. OC2 will compute and assign a value to 'lst_'smpl.       
                while (!buffer_full) {}    
                // <editor-fold defaultstate="collapsed" desc="Reset: counter_buffer = 0, IEC1bits.INT2IE = 0, IEC0bits.OC2IE = 0">   
                IEC1bits.INT2IE = 0;                //Disabling INT2 interrupt will allow writing the following 0x00
                IEC0bits.OC2IE  = 0;                //into OC2 buffer. Since OC2 is still running
                counter_buffer  = 0;
                // </editor-fold>
                send_UART("Resistance Range is 17k to 170k ");
                // <editor-fold defaultstate="collapsed" desc="Send previously collected data to PC. Resistance on CH1. Passed">
                    for (i = 0; i < 2048; i++) {
                        while (!U1STAbits.TRMT);//Resistance is measured on CH1 only
                        U1TXREG = voltage_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_lsb[i];

                        while (!U1STAbits.TRMT);
                        U1TXREG = current_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_lsb[i];
                    }
                // </editor-fold>
                //rx_function = 0;
                flags = 0;
            // </editor-fold>                                 
                break;
            case 26: 
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="'d' - Resistance test 270">  
                SNSR5VEN                = SNSR5VEN_STAT;    // Enables 5V pulse voltage.
                GS_EN                   = En_elmnt_rd;      // AFE: K1 connect MQ5_A to MCP3911 CH1.
                Cap_Ind_5V_En           = Enable;           // Enables 5V to the VSENCE page "Cap_Ind_5V_En" the switch is located on Power1 page
                R27k                    = Disable;
                R270                    = Enable;          //_RD2 controls 270 Ohm resistor on both old and the new hardware
                R2_7                    = Disable;          //_RC14 controls 2.7 Ohm resistor on both old and the new hardware
                R2_7k                   = Disable;          //_RC13 controls 2.7K
                SHUNT                   = Disable;          // Disable shunt MOSFET (Q41 SDIT)
                rng_srch                = Lng_buffr; 
                TR_EN                   = 0;                // Disable data transfer in OC2              
                //SHUNT                   = Disable;                          // Turn OFF the shunt MOSFET.  
                CH1_sel                 = 1;                // Enables data load into the data buffer in INT routine and transmits data-point in OC2
                CH0_sel                 = 1; 
                rx_function             = 13;               // Collect data without transmitting to PC. This function enables INT2 and OC2
                cmdLCD((char)0x80);
                putsLCD("Resistance test ");
                cmdLCD((char)0xc0);
                putsLCD("270k shunt, PCUI");
                execute_function();                         // Start cycle. Collect 2048 data samples. OC2 will compute and assign a value to 'lst_'smpl.       
                while (!buffer_full) {}    
                // <editor-fold defaultstate="collapsed" desc="Reset: counter_buffer = 0, IEC1bits.INT2IE = 0, IEC0bits.OC2IE = 0">   
                IEC1bits.INT2IE = 0;                //Disabling INT2 interrupt will allow writing the following 0x00
                IEC0bits.OC2IE  = 0;                //into OC2 buffer. Since OC2 is still running
                counter_buffer  = 0;
                // </editor-fold>
                send_UART("Resistance Range is 1.7k to 17k ");
                // <editor-fold defaultstate="collapsed" desc="Send previously collected data to PC. Resistance on CH1. Passed">
                    for (i = 0; i < 2048; i++) {
                        while (!U1STAbits.TRMT);//Resistance is measured on CH1 only
                        U1TXREG = voltage_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = voltage_lsb[i];

                        while (!U1STAbits.TRMT);
                        U1TXREG = current_msb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_nsb[i];
                        while (!U1STAbits.TRMT);
                        U1TXREG = current_lsb[i];
                    }
                // </editor-fold>
                //rx_function = 0;
                flags = 0;
            // </editor-fold>
                break;
            case 27:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="'e' - Resistance test 27.0">  
                //_TRISC14 = 0;
                TRISCbits.TRISC14 = 1;
                SNSR5VEN                = SNSR5VEN_STAT;    // Enables 5V pulse voltage.
                GS_EN                   = En_elmnt_rd;      // AFE: K1 connect MQ5_A to MCP3911 CH1.
                Cap_Ind_5V_En           = Enable;           // Enables 5V to the VSENCE page "Cap_Ind_5V_En" the switch is located on Power1 page
                R27k                    = Disable;
                R270                    = Disable;          //_RD2 controls 270 Ohm resistor on both old and the new hardware
                //PORTCbits.RC14 = 1;          //_RC14 controls 2.7 Ohm resistor on both old and the new hardware
                CNPU1bits.CN0PUE = 1;
                R2_7k                   = Disable;          //_RC13 controls 2.7K
                SHUNT                   = Disable;          // Disable shunt MOSFET (Q41 SDIT)
                rng_srch                = Lng_buffr; 
                TR_EN                   = 0;                // Disable data transfer in OC2              
                //SHUNT                   = Disable;                          // Turn OFF the shunt MOSFET.  
                CH1_sel                 = 1;                // Enables data load into the data buffer in INT routine and transmits data-point in OC2
                CH0_sel                 = 1; 
                rx_function             = 13;               // Collect data without transmitting to PC. This function enables INT2 and OC2
                cmdLCD((char)0x80);
                putsLCD("Resistance test ");
                cmdLCD((char)0xc0);
                putsLCD("27.0 shunt, PCUI");
                execute_function();                         // Start cycle. Collect 2048 data samples. OC2 will compute and assign a value to 'lst_'smpl.       
                while (!buffer_full) {}    
                // <editor-fold defaultstate="collapsed" desc="Reset: counter_buffer = 0, IEC1bits.INT2IE = 0, IEC0bits.OC2IE = 0">   
                IEC1bits.INT2IE = 0;                //Disabling INT2 interrupt will allow writing the following 0x00
                IEC0bits.OC2IE  = 0;                //into OC2 buffer. Since OC2 is still running
                counter_buffer  = 0;
                // </editor-fold>
                send_UART("Resistance Range is 17 to 170   ");
                // <editor-fold defaultstate="collapsed" desc="Send previously collected data to PC. Resistance on CH1. Passed">
                for (i = 0; i < 2048; i++) {
                    while (!U1STAbits.TRMT);//Resistance is measured on CH1 only
                    U1TXREG = voltage_msb[i];
                    while (!U1STAbits.TRMT);
                    U1TXREG = voltage_nsb[i];
                    while (!U1STAbits.TRMT);
                    U1TXREG = voltage_lsb[i];

                    while (!U1STAbits.TRMT);
                    U1TXREG = current_msb[i];
                    while (!U1STAbits.TRMT);
                    U1TXREG = current_nsb[i];
                    while (!U1STAbits.TRMT);
                    U1TXREG = current_lsb[i];
                }
                // </editor-fold>
                //rx_function = 0;
                CNPU1bits.CN0PUE = 0;
                TRISCbits.TRISC14 = 0;
                flags = 0;
            // </editor-fold>  
                break; 
            case 28:                                            //  
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                if (_RB9 && !_RB10) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Battery Status: ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("  Charging...   ");                    
                }
                if (!_RB9 && _RB10) {
                    cmdLCD((char) 0x80); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Battery Status: ");
                    cmdLCD((char) 0xc0); //cmdLCD((char)0x80)- Line 1 loc 0; cmdLCD(0xc0) - line 2 loc 0
                    putsLCD("Charge Complete ");                   
                }
                mydelay_sec(3);
                selection = 1;
                break;
            case 29:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="Zeroing Out">    
                rx_function = 8; //'v' erase calibration values
                execute_function();        
                mydelay_ms(500);
                cal_zero_out(); 
                Cal_values();   // Apply calibration values
                // </editor-fold>
                break;
            case 30:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="selection == 30 Manual range select 1">
                //Manual range select INF
                //if (((sleep_counter%10) == 0) || ((sleep_counter%11) == 0) || ((sleep_counter%12) == 0)){
                    SNSR5VEN = SNSR5VEN_STAT; //Enable 5V to K1
                    GS_EN = En_msr_V; //AFE K1 connect AN_CH1 to AFE_CH0_sel. AN_CH1 is the voltage sence waveform output
                    reset_dividers_CH0();
                    reset_dividers_CH1();
                    rx_function = 13; //Collect data without transmitting to PC. This function enavbles INT2 and OC2
                    execute_function(); //Start cycle. Collect 2048 data samples                
                    while (!buffer_full) {}
                    voltage_val = data_comp_CH0(vflag, rng_srch, v_smple_sum, voltage_sum_n, voltage_sum_p, cal, CH0vsftSgn, CH0vshft, dividerCH0);
                    current_val = data_comp_CH1(cflag, rng_srch, c_smple_sum, current_sum_n, current_sum_p, cal, CH1vsftSgn, CH1vshft, dividerCH1);
                    dividerCH0 = 1;
                    dividerCH1 = 1;                    
                    displayCH0CH1();
                //}
//                else{
//                    hexdec_long((unsigned long)CH0vshft);
//                    cmdLCD(0x80);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH0:  ");
//                    putsLCD((char *) (tx_buf));
//                    
//                    hexdec_long((unsigned long)CH1vshft);
//                    cmdLCD(0xc0);                       //void writeLCD( int addr, char c)
//                    putsLCD("CH1:  ");
//                    putsLCD((char *) (tx_buf));
//                    mydelay_sec(3);
//                }
                // </editor-fold>
                break;
            case 31:    // Digital Sniffer Status Print                
                // <editor-fold defaultstate="collapsed" desc="Print LCD number of CIN interrupts and number of bits">
                T1InrrptCntr = T1InrrptCntr + 1;                    // To avoid zero                
                hexdec_long((unsigned long)LAcntr);             // Convert number of CN interrupts
                cmdLCD(0x80);                                   // Will display data when no CN Interrupts occur. Timer interrupt happens every 68 u-sec. UART interrupt 
                putsLCD((char *) tx_buf);                       // due to timer expiration happens every 1.356 sec.
                putsLCD(" bytes");
                hexdec_long((unsigned long) NmbrOfBits.Val);
                cmdLCD(0xc0);
                putsLCD((char *) tx_buf);
                putsLCD(" bits ");  
                //MENU_DWN_LED = ~MENU_DWN_LED;                                 
                // 
                // </editor-fold>                   
                break;
            case 32:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                // <editor-fold defaultstate="collapsed" desc="execute_function(): take voltage readings on both channels with previously selected dividers">
                rx_function = 13;           //Collect data without transmitting to PC. This function enavbles INT2 and OC2
                execute_function();         //Start cycle. Collect data samples
                while (!buffer_full) {}     //Wait until all data is collected
                // </editor-fold>
                voltage_val = data_comp_CH0(vflag, rng_srch, v_smple_sum, voltage_sum_n, voltage_sum_p, cal, CH0vsftSgn, CH0vshft, dividerCH0); //(nV) //Returns average voltage value in nV
                current_val = data_comp_CH1(cflag, rng_srch, c_smple_sum, current_sum_n, current_sum_p, cal, CH1vsftSgn, CH1vshft, dividerCH1); //(nV) //Returns average voltage value in nV
//              //displayCH0CH1();    //Display measured values with dividers set in DataloggerConfig
                //unsigned long CH0vshft, CH0510kVal, CH080kVal, CH010kVal;   // 'Vshift for every range 32 bit
                //unsigned char CH0vsftSgn, CH0510kSgn, CH080kSgn, CH010kSgn; 
                if (dividerCH0 == 1){                    
                    display_nV (voltage_val, 0x80, vflag, voltage_sum_p, voltage_sum_n, CH0vshft, CH0vsftSgn);
                }
                else if(dividerCH0 == 51) {
                    display_V (voltage_val, 0x80, vflag, voltage_sum_p, voltage_sum_n, CH0510kVal, CH0510kSgn, dividerCH0);                    
                }  
                else if(dividerCH0 == 80) {
                    display_V (voltage_val, 0x80, vflag, voltage_sum_p, voltage_sum_n, CH080kVal, CH080kSgn, dividerCH0);
                    //display_V (current_val, 0xc0, cflag, current_sum_p, current_sum_n, CH180kVal, CH080kSgn, dividerCH1);
                }
                else if(dividerCH0 == 10) {
                    display_V (voltage_val, 0x80, vflag, voltage_sum_p, voltage_sum_n, CH010kVal, CH010kSgn, dividerCH0);
                    //display_V (current_val, 0xc0, cflag, current_sum_p, current_sum_n, CH180kVal, CH080kSgn, dividerCH1);
                }
                else{
                    
                }
                if (dividerCH1 == 1){                    
                    display_nV (current_val, 0xc0, cflag, current_sum_p, current_sum_n, CH1vshft, CH1vsftSgn);
                }
                else if(dividerCH1 == 51) {
                    //display_V (current_val, 0x80, vflag, voltage_sum_p, voltage_sum_n, CH0510kVal, CH0510kSgn, dividerCH0);
                    display_V (current_val, 0xc0, cflag, current_sum_p, current_sum_n, CH1510kVal, CH0510kSgn, dividerCH1);
                }  
                else if(dividerCH1 == 80) {
                    //display_V (current_val, 0x80, vflag, voltage_sum_p, voltage_sum_n, CH080kVal, CH080kSgn, dividerCH0);
                    display_V (current_val, 0xc0, cflag, current_sum_p, current_sum_n, CH180kVal, CH080kSgn, dividerCH1);
                }
                else if(dividerCH1 == 10) {
                    //display_V (current_val, 0x80, vflag, voltage_sum_p, voltage_sum_n, CH010kVal, CH010kSgn, dividerCH0);
                    display_V (current_val, 0xc0, cflag, current_sum_p, current_sum_n, CH180kVal, CH080kSgn, dividerCH1);
                }
                else{
                    
                }
                
                if (!menu_pb)
                    selection = 1;               
                break;
                
                case 33:
                    EnableWDT(WDT_ENABLE);          // Enable Watchdog Timer
                    //ClrWdt();                       // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                    break;
                
          default:
                //ClrWdt();                   // To prevent a WDT Time-out Reset <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                Nop();
                break;
        }
        //The following code has no condition and will be executed every time program is in the main loop
        freez_cntr ++;
        if ((freez_cntr>10) && (freez_En == Enable)){
            CH0_disp_frz    = 1;                //FREEZE display on INPUT1 (R), Top Line
            Buzzer          = Enable;           //Turn on buzzer
            mydelay_ms(100);
            Buzzer          = Disable;
            mydelay_ms(200);
            Buzzer          = Enable;           //Turn on buzzer
            mydelay_ms(100);
            Buzzer          = Disable;
            freez_cntr = 0;                     //Reset freeze counter
            //freez_En = Disable;               //<--------------------------------------------------------12/09/2015
        }
        if (selection == 2)
            slpmax = 240;
        else
            slpmax = 80;
        // <editor-fold defaultstate="collapsed" desc="Sleep Function">
        if (!no_sleep) { // if(no_sleep == 0) // sleep is enabled
            if (sleep_counter < slpmax) {
                sleep_counter++;
            } 
            else {
                BattLevelCH8();
                mydelay_sec(3);
                sleep_counter = 0;
                sleep_();
            }
        }
        // </editor-fold>

    }//End of infinite loop
        return 0;                   //03/19/2014
}

