#include "MCP3911_EVB.h"

/*
 * CNInterrupt gets activated from LA function with flag = 4.
 * The interrupt is activated when there is a change on the pin.
 * On every interrupt test current state of the pin. Save the data.
 * 
 */
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void){
    
    //DWORD_VAL NmbrOfBits;    
    CIN_flg = 1;                            // Used in sleep() and logic analyzer function function
    CH0_disp_frz = Disable;                 // Re-enable CH0 frozen display
    IFS1bits.CNIF = 0;                      // Clear CIn interrupt flag  
    DWORD TimeGap = 0;
    //DWORD min_val;
    //unsigned char timer_dice;
        
    if (flags == 13 || flags == 4) { 
        // <editor-fold defaultstate="collapsed" desc="Digital Sniffer">           
        /* 
         * Timer is started in the communication protocol case 46
         * Data is transmitted from timer interrupt in Interrupts_Revxx
         * Record time on every interrupt. Data rate has to be given. Divide recorded time by the time of a single pulse
         * Input: Data-rate in us
         * Measure: Time between interrupts in ns
         * (measured time between the events)/data rate = number of bits [sec/(bits/sec)] = bits
         * TMR1 is 16 bit timer counter register 2^16 = 65536
         * timer = TMR1 * 67.8e-9;   // 67.8e-9 n-sec = 0.0678 u-sec timer per tick.
         * Timer Tick time = 67.8e-9 sec
         * timer = TMR1 * 67.8e-9 n-sec/(per tick)
         * 
         * Digital Sniffer: Mask is initialized in the 'init() function to mask_h = 0xFE and mask_l = 0x01;
         * CNI interrupt is active every time. The timer should have to have time from one edge to another
         * Variables:
         * ints - increments on every CN interrupt. Gets reset with every byte received.
         * TMR1
         * 
         * INTERRUPT:
         * The interrupt process takes four instruction cycles.
         * TCY = 1 / FCY = 67.8 ns
         */                               
        //Nothing happens on the first interrupt, just the TMR1 gets reset
        if (ints == 0)  // If this is the first CIN interrupt clear TMR1 timer register.
            TMR1 = 0;   // If this is the first interrupt record time zero. Nothing happens on the first interrupt.                                                        
        /*         
         * The total time between two CIN interrupts
         * dCIN(t) = TMR1 * 68 n-sec/per tick
         * 
         * Bit rate:
         * The speed of the data is expressed in bits per second (bits/s or bps). 
         * The data rate R is a function of the duration of the bit or bit time (TB) (Fig. 1, again):
         * R = 1/TB
         * Rate is also called channel capacity C. 
         * 
         * R = 1/68 n-sec = 0.01471 * 10^9 bits/sec = 14710 k-bits/sec
         * Looks like 10 us is the minimum TimeBase
         */
        
        // <editor-fold defaultstate="collapsed" desc="Compute number of bits received based on Time Base and current timer">                      
        /*
         * Timer1 is a 16 bit timer. 2^16 * 68 = 4456448 count 
         */       
        TimeGap = (DWORD)(TMR1 * 68); //n-sec. Time interval (n-sec) = Number of counts * Time of one count
        
        NmbrOfBits.Val = (DWORD)(TimeGap / TimeBase.Val); //
                
        // </editor-fold>
                                        
        /* 
         * The maximum number of bits, NmbrOfBits.Val, can not be greater than 2048 * 8 = 16384
         * Byte count = NmbrOfBits.Val / 8;
         * LAcntr = LAcntr + NmbrOfBits.Val / 8;
         * 
         * if timer[ints].Val == TimeBase.Val       ->    NmbrOfBits.Val = 1
         * if timer[ints].Val == 2 * TimeBase.Val   ->    NmbrOfBits.Val = 2
         * TimeBase is entered in communication protocol in us. NmbrOfBits now has number of bits. 
         * NmbrOfBits.Val will be 8 or more when: timer[ints].Val > 8 * TimeBase.Val.
         * 
         */
                
        TMR1 = 0;                      // Current time is recorded above, reset timer 1 register
/*
 *  ____      ____
 * |    |    |    |
 * |    |    |    |
 * |    |____|    |
 * 0    1    2    3 
 */        
start2047: 
    if (ints != 0){         // If this is not the first interrupt. This guarantees that time is not zero        
        //This flag is set in the Logic Analyzer function once it is called
        if (LAcntr < 2048){                                 // If byte count is less then 2048 
            // <editor-fold defaultstate="collapsed" desc="Load data into the voltage_msb buffer">  
            if (NmbrOfBits.Val == 0){                       // 8 bits left
                //If time base is large compared to actual bit time or no data present (how? interrupt can happen only when there is a change)
                // <editor-fold defaultstate="collapsed" desc="0 bits">                
//                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG15 is low set current bit to zero with mask_h = 0xFE
//                    voltage_msb[LAcntr] = 0x00; // mask_h  is initialized to 0b1111 1110
//                } else { // If RG15 is high set current bit to one with mask_h = 0x01
//                    voltage_msb[LAcntr] = 0xFF; // ~mask_h = 0b0000 0001
//                }
                //mask_h = mask_h << 8; // 0000 0001 << 8 = 0000 0000 will result in LAcntr++ and reset both of masks
                goto skip;          // Skip all
                // </editor-fold>     
            }
            else if (NmbrOfBits.Val == 1){                       // 7 bits left
                // <editor-fold defaultstate="collapsed" desc="1 bit">                
                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG15 is low set current bit to zero with mask_h = 0xFE
                    voltage_msb[LAcntr] = mask_h;  // mask_h  is initialized to 0b1111 1110
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 0001
                }                 
                // </editor-fold>    
            }
            else if (NmbrOfBits.Val == 2){
                // <editor-fold defaultstate="collapsed" desc="2 bit">   
                mask_h = mask_h << 1; // 1111 1110 << 1 = 1111 1100
                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] = mask_h;  // mask_h << 1  = 0b1111 1100
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; // ~mask_h << 1 = 0b0000 0011
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 3){
                // <editor-fold defaultstate="collapsed" desc="3 bit">     
                mask_h = mask_h << 2; // 1111 1110 << 2 = 1111 1000
                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG14 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] = mask_h;  // mask_h  << 2 = 0b1111 1000
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; // ~mask_h << 2 = 0b0000 0111
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 4){                  // 0000 0001 << 4 = 0001 0000
                // <editor-fold defaultstate="collapsed" desc="4 bit"> 
                mask_h = mask_h << 3; // 1111 1110 << 3 = 1111 0000
                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG14 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] = mask_h;  // mask_h << 3  = 0b1111 0000
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; // ~mask_h << 3 = 0b0000 1111
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 5){                  // 0000 0001 << 5 = 0010 0000
                // <editor-fold defaultstate="collapsed" desc="5 bit">  
                mask_h = mask_h << 4; // 1111 1110 << 4 = 1110 0000
                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG14 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG14 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; // Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 6){                  // 0000 0001 << 6 = 0100 0000
                // <editor-fold defaultstate="collapsed" desc="6 bit">   
                mask_h = mask_h << 5; // 1111 1110 << 5 = 1100 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; // Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }
                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 7){                  // 0000 0001 << 7 = 1000 0000
                // <editor-fold defaultstate="collapsed" desc="7 bit">
                mask_h = mask_h << 6; // 1111 1110 << 6 = 1000 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; // Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }
                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 8){                  // 0000 0001 << 7 = 0000 0000
                // <editor-fold defaultstate="collapsed" desc="8 bit">
                mask_h = mask_h << 7; // 1111 1110 << 7 = 0000 0000
                //mask_l = mask_l << 8;
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }
                
                // </editor-fold>               
            }          
            else { // If time lapse is more then 8 bit
                //mask_l = mask_l << 8;                     // Indicate full byte has been received
                mask_h = mask_h << 7;                       // This will indicate that a full byte has been received
                if (_RG14) {                                // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_msb[LAcntr] =  mask_h;     // LAcntr is incremented when byte of data has been shifted
                } else {                                    // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_msb[LAcntr] = ~mask_h;     // Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }
                LAcntr++;               // Increment byte count 
                mask_h  = 0xFE;         // Reset mask for the new byte
                //mask_l  = 0x01;         // Reset mask for the new byte                 
                if(NmbrOfBits.Val > 8){
                    NmbrOfBits.Val = NmbrOfBits.Val - 8;
                    goto start2047;
                }               
                else
                    mask_h == 0x00;     // Exit to if (mask_l == 0x00)                         
            }
            // </editor-fold>
        }    
        else if ((2048 <= LAcntr) && (LAcntr < 4096)){
            // <editor-fold defaultstate="collapsed" desc="Load data into the voltage_nsb buffer">
            if (NmbrOfBits.Val == 0){                       // 7 bits left
                // <editor-fold defaultstate="collapsed" desc="0 bits">
                //If time base is large compared to actual bit time.
//                mask_l = mask_l << 8; // 0000 0001 << 8 = 0000 0000 will result in LAcntr++ and reset both of masks
//                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG15 is low set current bit to zero with mask_h = 0xFE
//                    voltage_msb[LAcntr] = 0x00; // mask_h  = 0b1111 1110
//                } else { // If RG15 is high set current bit to one with mask_h = 0x01
//                    voltage_msb[LAcntr] = 0xFF; // ~mask_h = 0b0000 0001
//                }
                goto skip;
                // </editor-fold> 
            }
            else if (NmbrOfBits.Val == 1){                         // 7 bits left
                // <editor-fold defaultstate="collapsed" desc="1 bit">
                
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0xFE
                    voltage_nsb[LAcntr] = mask_h; // mask_h = 0b1111 1110
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 0001                   
                }                
                // </editor-fold>     
            }
            else if (NmbrOfBits.Val == 2){
                // <editor-fold defaultstate="collapsed" desc="2 bit">   
                mask_h = mask_h << 1;// 1111 1110 << 1 = 1111 1100
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] = mask_h; // mask_h = 0b1111 1100
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 0011
                }               
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 3){
                // <editor-fold defaultstate="collapsed" desc="3 bit">
                mask_h = mask_h << 2;// 1111 1110 << 2 = 1111 1000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] = mask_h; // mask_h = 0b1111 1000
                } else {    // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 0111
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 4){
                // <editor-fold defaultstate="collapsed" desc="4 bit">
                mask_h = mask_h << 3;// 1111 1110 << 3 = 1111 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] = mask_h; // mask_h = 0b1111 0000
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 1111
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 5){
                // <editor-fold defaultstate="collapsed" desc="5 bit"> 
                mask_h = mask_h << 4;// 1111 1110 << 4 = 1110 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 6){
                // <editor-fold defaultstate="collapsed" desc="6 bit">      
                mask_h = mask_h << 5;// 1111 1110 << 5 = 1100 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 7){
                // <editor-fold defaultstate="collapsed" desc="7 bit">
                mask_h = mask_h << 6;// 1111 1110 << 6 = 1000 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }                                
                // </editor-fold>
            }
            else if (NmbrOfBits.Val == 8){
                // <editor-fold defaultstate="collapsed" desc="8 bit"> 
                mask_h = mask_h << 7;// 1111 1110 << 7 = 0000 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }                
                // </editor-fold>               
            }
            else {//If number of received bits is greater then 8  
                mask_h = mask_h << 8;
                if (_RG14) {                                // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_nsb[LAcntr] =  mask_h;     // LAcntr is incremented when byte of data has been shifted
                } else {                                     // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_nsb[LAcntr] = ~mask_h;     //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }
                LAcntr++;               // Byte count 
                mask_h  = 0xFE;         // Reset mask for the new byte
                //mask_l  = 0x01;         // Reset mask for the new byte
                if(NmbrOfBits.Val > 8){
                    NmbrOfBits.Val = NmbrOfBits.Val - 8;
                    goto start2047;
                }
                else
                    mask_h == 0x00; 
            }        
            // </editor-fold>
        }
        else if ((4096 <= LAcntr) && (LAcntr < 6144)){
            // <editor-fold defaultstate="collapsed" desc="Load data into the voltage_lsb buffer">
            if (NmbrOfBits.Val == 0){                       // 7 bits left
                // <editor-fold defaultstate="collapsed" desc="0 bits">
                //If time base is large compared to actual bit time.                
//                if (_RG14) { // Input is inverted !_RG14 is actually _RG14. If RG15 is low set current bit to zero with mask_h = 0xFE
//                    voltage_msb[LAcntr] = 0x00; // mask_h  = 0b1111 1110
//                } else { // If RG15 is high set current bit to one with mask_h = 0x01
//                    voltage_msb[LAcntr] = 0xFF; // ~mask_h = 0b0000 0001
//                }
//                mask_h = mask_h << 8; // 0000 0001 << 8 = 0000 0000 will result in LAcntr++ and reset both of masks
                goto skip;
                // </editor-fold> 
            }
            else if (NmbrOfBits.Val == 1){                         // 7 bits left
                // <editor-fold defaultstate="collapsed" desc="1 bit">                
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0xFE
                    voltage_lsb[LAcntr] = mask_h; // mask_h = 0b1111 1110
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 0001
                }                
                // </editor-fold>     
            }
            else if ((NmbrOfBits.Val == 2)){
                // <editor-fold defaultstate="collapsed" desc="2 bit">
                mask_h = mask_h << 1;// 1111 1110 << 1 = 1111 1100
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] = mask_h; // mask_h = 0b1111 1100
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 0011
                }// </editor-fold>
            }
            else if ((NmbrOfBits.Val == 3)){
                // <editor-fold defaultstate="collapsed" desc="3 bit">
                mask_h = mask_h << 2;// 1111 1110 << 2 = 1111 1000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] = mask_h; // mask_h = 0b1111 1000
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 0111
                }// </editor-fold>
            }
            else if ((NmbrOfBits.Val == 4)){
                // <editor-fold defaultstate="collapsed" desc="4 bit">
                mask_h = mask_h << 3;// 1111 1110 << 3 = 1111 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] = mask_h; // mask_h = 0b1111 0000
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; // ~mask_h = 0b0000 1111
                }// </editor-fold>
            }
            else if ((NmbrOfBits.Val == 5)){
                // <editor-fold defaultstate="collapsed" desc="5 bit">
                mask_h = mask_h << 4;// 1111 1110 << 4 = 1110 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }// </editor-fold>
            }
            else if ((NmbrOfBits.Val == 6)){
                // <editor-fold defaultstate="collapsed" desc="6 bit">
                mask_h = mask_h << 5;// 1111 1110 << 5 = 1100 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }// </editor-fold>
            }
            else if ((NmbrOfBits.Val == 7)){
                // <editor-fold defaultstate="collapsed" desc="7 bit">
                mask_h = mask_h << 6;// 1111 1110 << 6 = 1000 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }// </editor-fold>
            }
            else if ((NmbrOfBits.Val == 8)){
                // <editor-fold defaultstate="collapsed" desc="8 bit">
                mask_h = mask_h << 7;// 1111 1110 << 7 = 0000 0000
                if (_RG14) { // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] = mask_h; // LAcntr is incremented when byte of data has been shifted
                } else { // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }// </editor-fold>               
            }
            else {
                mask_h = mask_h << 7; 
                if (_RG14) {                                // If RG15 is low set current bit to zero with mask_h = 0x0b11111100
                    voltage_lsb[LAcntr] =  mask_h;     // LAcntr is incremented when byte of data has been shifted
                } else {                                    // If RG15 is high set current bit to one with mask_h = 0x01
                    voltage_lsb[LAcntr] = ~mask_h;     //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
                }
                LAcntr++;               // Byte count 
                mask_h  = 0xFE;         // Reset mask for the new byte
                mask_l  = 0x01;         // Reset mask for the new byte
                if(NmbrOfBits.Val > 8){
                    NmbrOfBits.Val = NmbrOfBits.Val - 8;
                    goto start2047;
                }
                else
                    mask_h == 0x00;
            }
            // </editor-fold>
        }
        else{                                               // All buffers are full <----------------------------------------------------------------***
            IPC0bits.T1IP   = 4;    // Setup Timer1 interrupt for desired priority level
            IPC4bits.CNIP   = 4;    // Interrupt priority 4
            //MENU_LED        = OFF; 
            ints            = 0;
        }        
    
        // <editor-fold defaultstate="collapsed" desc="commented out second channel">

        //        if (LAcntr < 2048){ // LAcntr is the byte count
        //            // <editor-fold defaultstate="collapsed" desc="Load data into the current_msb buffer">
        //            if (!_RG15) { // If RG15 is low set current bit to zero with mask_h = 0xFE
        //                current_msb[LAcntr] = mask_h&_RG15; //Turn off bit zero(mask_h = 0xFE). If the first bit is zero save zero in voltage_msb[0] bit 0
        //            } else { // If RG15 is high set current bit to one with mask_h = 0x01
        //                current_msb[LAcntr] = mask_l&_RG15; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
        //            }// </editor-fold>
        //        }
        //        else if ((2048 <= LAcntr) && (LAcntr < 4096)){
        //            // <editor-fold defaultstate="collapsed" desc="Load data into the current_nsb buffer">
        //            if (!_RG15) { // If RG15 is low set current bit to zero with mask_h = 0xFE
        //                current_nsb[LAcntr] = mask_h&_RG15; //Turn off bit zero(mask_h = 0xFE). If the first bit is zero save zero in voltage_msb[0] bit 0
        //            } else { // If RG15 is high set current bit to one with mask_h = 0x01
        //                current_nsb[LAcntr] = mask_l&_RG15; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
        //            }// </editor-fold>
        //        }
        //        else if ((4096 <= LAcntr) && (LAcntr < 6144)){
        //            // <editor-fold defaultstate="collapsed" desc="Load data into the current_lsb buffer">
        //            if (!_RG15) { // If RG15 is low set current bit to zero with mask_h = 0xFE
        //                current_lsb[LAcntr] = mask_h&_RG15; //Turn off bit zero(mask_h = 0xFE). If the first bit is zero save zero in voltage_msb[0] bit 0
        //            } else { // If RG15 is high set current bit to one with mask_h = 0x01
        //                current_lsb[LAcntr] = mask_l&_RG15; //Turn on bit zero (mask_l = 0x01). If the first bit is one save one in voltage_msb[0] bit 0
        //            }// </editor-fold>
        //        }
        //        else{
        //            
        //        }
        // </editor-fold>
    }   // End of if (ints != 0){}                            
        //On every interrupt Shift the mask to position for the next bit (mask_h is initialized to 0xFE)
        //mask_h = (mask_h<<1) | (~mask_h);       //(1111 1110 <<1) -> 1111 1100 | 0000 0001 = 1111 1101 turn off bit mask
        //mask_h = ( mask_h * 2 ) | (~mask_h);    //Equivalent to the above, shift left is x2
        //mask_h = mask_h + 1;                    //The LSB bit of mask_h has to be set to one
        //mask_l = (mask_l << 2);                 //(0000 0001 <<1) -> 0000 0010
        //mask_l = mask_l * 2;                    //(0000 0001 <<1) -> 0000 0010  

        /*
         * 0000 0001 - 1    init
         * 0000 0010 - 2    << 1
         * 0000 0100 - 4    << 2
         * 0000 1000 - 8    << 3
         * 0001 0000 - 16   << 4
         * 0010 0000 - 32   << 5
         * 0100 0000 - 64   << 6
         * 1000 0000 - 128  << 7
         */                                      
        //Below is: Move to the next byte: voltage_msb[2048]
        /*
         * 
         */
        if (mask_h == 0x00) {                       // If 8 data bits has been shifted in
            //MENU_DWN_LED = ~MENU_DWN_LED;         // MENU_UP_LED = ~MENU_UP_LED; is used in timer interrupt
            if (LAcntr < 6144){
                LAcntr++;                           // Increment byte count 
                //MENU_DWN_LED = OFF;               // MENU_UP_LED = ~MENU_UP_LED; is used in timer interrupt
            }
            mask_h  = 0xFE;                         // Reset mask for the new byte
            mask_l  = 0x01;                         // Reset mask for the new byte                                  
        }  
        if (LAcntr >= 6143){        // 2/3 of data is collected 4095
            IPC0bits.T1IP   = 4;    // Enable timer interrupt
            // Keep collecting data while transmitting?
            IPC4bits.CNIP   = 4;    // CPU interrupt priority level is 3 (11). Enable user interrupts.
            ints = 0;             
        } 
        else{
skip:            
            ints ++;                       // Increment the number of CIN interrupts
        }
    // </editor-fold>       
    }
    
    else if (flags == 9) {
        //Un-pause current operation
        flags = 0; //Resume current operation
    } 
    else if (flags == 16){
        T1InrrptCntr = 1024;
        flags = 0;
    }
    else if (flags == 1){ // sleep function
        //Disable interrupt on _RC2 and other PBs. This is the "Wake up PB"************************
        // <editor-fold defaultstate="collapsed" desc="Disable all CIN interrupts coming out of the sleep function">
        CNEN3bits.CN46IE = Disable; //S4 = RC2 = CN46. The "Wake Up button"
        CNPD3bits.CN46PDE = Disable; //Pull down enable (disabled)
        CNPU3bits.CN46PUE = Disable; //Pull up enable (disabled)
        CNEN3bits.CN44IE = Disable; //S4 = RC2 = CN46. The "Wake Up button"
        CNPD3bits.CN44PDE = Disable; //Pull down enable (disabled)
        CNPU3bits.CN44PUE = Disable; //Pull up enable (disabled)
        CNEN3bits.CN43IE = Disable; //S4 = RC2 = CN46. The "Wake Up button"
        CNPD3bits.CN43PDE = Disable; //Pull down enable (disabled)
        CNPU3bits.CN43PUE = Disable; //Pull up enable (disabled)
        CNEN2bits.CN29IE = Disable; //Power Good
        CNPD2bits.CN29PDE = Disable; //Pull down enable (disabled)
        CNPU2bits.CN29PUE = Disable; // </editor-fold>
        flags = 0;                          
    } 
    else if(flags == 18){
        success = TRUE;
        flags = 0;
        //for(i = 0;i < 497152; i++){}; //= 472 ms delay        
    }
    else{
        
    }
    
}
