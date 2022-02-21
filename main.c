
#include <stdio.h>
#include <stdlib.h>
#include <pic18.h>
#include "htc.h"
#include <math.h>
#include <stdint.h>
#include "fish.h"
#include "io.h"


/*Met A - Voltage Divider*/
#define RBIAS 1000          

/*Met B - Current Source*/
#define IBIAS 0.002505  //(Amps) 
#define AV 5

/* Thermistor 25ºC Data*/
#define R0 1000.0       //Ohms
#define T0 298.16       //(25ºC Kelvin)
/*----------*/

#define BETA 3493.0

#define ADC_RESOLUTION 1024

#define _XTAL_FREQ 20000000

#define STATE_STOPPED     1     // Machine state stopped
#define STATE_FILLING     2     // Machine state filling aquarium
#define STATE_DRAINNING   3     // Machine state drainning aquarium
#define STATE_CIRCULATION 4     // Machine state circulating and monitoring aquarium

#define COMMAND_START_MACHINE   0b00110001     // Starts machine command
#define COMMAND_START_DRAINNING 0b00110010   // Drain aquarium command
#define COMMAND_START_FILLING   0b00110011     // Fill aquarium command
#define COMMAND_STOP_MACHINE    0b00110100      // Stops machine
#define COMMAND_CHANGE_FISH_1   0b00110101     // Change fish command (Magikarp)
#define COMMAND_CHANGE_FISH_2   0b00110110     // Change fish command (Tubarao)
#define COMMAND_CHANGE_FISH_3   0b00110111     // Change fish command (Caracol)
#define COMMAND_CHANGE_FISH_4   0b00111000     // Change fish command (Plecostomo)
#define COMMAND_CHANGE_FISH_5   0b00111001     // Change fish command (Molly)
#define COMMAND_CHANGE_FISH_6   0b01000001     // Change fish command (Neon)

#define EV_RECIRCULACAO   LATB1
#define EV_ESGOTO         LATB2
#define EV_REDE           LATB3
#define BOMBA_AGUA        LATB4
#define RESISTENCIA_AQUEC LATB5
#define NVL_BAIXO   PORTAbits.RA2
#define NVL_ALTO    PORTAbits.RA1

//#pragma config CONFIG1H = 0x22
__CONFIG(1, FOSC_HSHP & PLLCFG_OFF & PRICLKEN_ON & FCMEN_OFF & IESO_OFF);
//#pragma config CONFIG2L = 0x1F
__CONFIG(2, PWRTEN_OFF & BOREN_SBORDIS & BORV_190);
//#pragma config CONFIG2H = 0x3C
__CONFIG(3, WDTEN_OFF & WDTPS_32768);
//#pragma config CONFIG3H = 0xBF
__CONFIG(4, CCP2MX_PORTC1 & PBADEN_ON & CCP3MX_PORTB5 & HFOFST_ON & T3CMX_PORTC0 & P2BMX_PORTB5 & MCLRE_EXTMCLR);
//#pragma config CONFIG4L = 0x81
__CONFIG(5, STVREN_ON & LVP_OFF & XINST_OFF);
//#pragma config CONFIG5L = 0xF
__CONFIG(6, CP0_OFF & CP1_OFF & CP2_OFF & CP3_OFF);
//#pragma config CONFIG5H = 0xC0
__CONFIG(7, CPB_OFF & CPD_OFF);
//#pragma config CONFIG6L = 0xF
__CONFIG(8, WRT0_OFF & WRT1_OFF & WRT2_OFF & WRT3_OFF);
//#pragma config CONFIG6H = 0xE0
__CONFIG(9, WRTC_OFF & WRTB_OFF & WRTD_OFF);
//#pragma config CONFIG7L = 0xF
__CONFIG(10, EBTR0_OFF & EBTR1_OFF & EBTR2_OFF & EBTR3_OFF);
//#pragma config CONFIG7H = 0x40
__CONFIG(11, EBTRB_OFF);

char RECEIVED_COMMAND = 0;


void interrupt ISR() {
    if (RC1IF && RC1IE) {
        RECEIVED_COMMAND = RCREG1;
    } 
}

void EEPROM_write(uint8_t adress, uint8_t value){
    
    EECON1bits.EEPGD = 0; //Data EEPROM
    EECON1bits.CFGS  = 0; //Acess data EEPROM
    EECON1bits.WREN  = 1; // Write Enable
    
    EEADR = adress;
    EEDATA = value;
    
    INTCONbits.GIE = 0;   // Disable Interrupts
    /*Write Sequence*/
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    /*End Write Sequence*/
    INTCONbits.GIE = 1;   // Enable Interrupts
    
    EECON1bits.WREN  =0; // Write Disable
}

uint8_t EEPROM_read(uint8_t adress){
    
    EEADR = adress;
    EECON1bits.EEPGD = 0; //Data EEPROM
    EECON1bits.CFGS  = 0; //Acess data EEPROM
    RD = 1;
  
    return EEDATA;
}

void PORT_init(){
    /* General IO*/
    TRISB = 0;
    LATB = 0;
    ANSB1  = 0;
    ANSB2  = 0;
    ANSB3  = 0;
    ANSB4  = 0;
    ANSB5  = 0;
    
    /*UART IO*/
    TRISC6 = 0;             // OUTPUT TX
    TRISC7 = 1;             // INPUT RX
    ANSC7  = 0;             // Set Input as DIGITAL
 
    /*ADC IO*/
    ANSELA = 0;
    TRISA0 = 1; 
    ANSA0  = 1;
    
    /*Entradas digitais*/
    TRISA1 = 1; 	// Input NVL_BAIXO
    TRISA2 = 1; 	//Input NVL_ALTO
    ANSA1  = 0; 	//Input DIGITAL
    ANSA2  = 0; 	//Input DIGITAL
    
    /*Interrupts*/
    PEIE = 1;
    GIE  = 1;
}

void UART_init(){
    /*Baud Rate*/
    SPBRGH1 = 0;
    SPBRG1  = 129; //9600
    
    /*Config Bits*/
    TXSTA1bits.SYNC    = 0;
    BAUDCON1bits.BRG16 = 0;
    TXSTA1bits.BRGH    = 1;
    RCSTA1bits.SPEN    = 1;
    TXSTA1bits.TXEN    = 1; 
    RCSTA1bits.CREN    = 1;
    
    RC1IE = 1; 	//Interrupt flag Enable
}

void ADC_init(){
    /*Config Bits*/
    ADCON0bits.CHS   = 0b00000;        //Analog channel AN1 
    ADCON1bits.PVCFG = 0b00;           //01 A/D VREF+ connected to external pin, VREF+
    ADCON1bits.NVCFG = 0b00;           //01 A/D VREF- connected to external pin, VREF-
    ADCON2bits.ADFM  = 1;              // 1 - Right Justified | 0 - LEFT Justified   
    ADCON2bits.ACQT  = 0b010;          // 20 Tad                   
    ADCON2bits.ADCS  = 0b010;          // Fosc/32                  
    ADCON0bits.ADON  = 1;              // ADC ON
}

uint16_t adc_capture(){    
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1);
    
    uint16_t code = 0x0000;         //16bit var reset
    code = ADRESH;                  //High byte -> 0b00000000 000000HH;
    code = code << 8;               //Shift left 8 times  -> 0b000000HH 00000000
    code = code | ADRESL;           // 0b000000HH LLLLLLLL 
  
    return code;
}

float Temp_calc(float Rterm){                                      //Calculate Temperature based on Thermistor resistor value;
    float t = (pow((1./(float)T0)+(1./(float)BETA)*log((float)Rterm/(float)R0),(-1.)))-273.16;
    return t;
}

float Rterm_calc(char method, float code){     
    float R;
    if (method == 'A'){
        R = RBIAS/((ADC_RESOLUTION/code)-1);
    }
    else if (method == 'B'){
        R = (code * AV)/(IBIAS * ADC_RESOLUTION);
    }
    return R;
}

float adc_average(int samples){
    float average = 0;
    uint8_t i;
    
    for (i = 0; i < samples; i++)
        average += adc_capture();

    average /= samples;
    
    return average;
}

void main() {
    PORT_init();
    UART_init();
    ADC_init();
    
    /* Fish Parameters Load */
    FishParams current_fish;
    if (EEPROM_read(0x00) != 255)
        set_current_fish(&current_fish, EEPROM_read(0x00));
    else
        set_current_fish(&current_fish, 1);
    
    float adc_capture_average;
    float Rterm;
    float Temp_measured;
    
    uint8_t last_fsm_state = 0;
    //uint8_t fsm_state    = 0;
    uint8_t fsm_state      = EEPROM_read(0x10);
    if (fsm_state != 1)
        fsm_state = 0;
    
    int timer_preload = 55770;
    int t1_counter    = 0;
    
    uint8_t last_NVL_ALTO = 0;
    /* TIMER CONFIG - p.154 */
    T0CONbits.TMR0ON = 1;
    T0CONbits.T08BIT = 0; // Usar timer 16 bits
    T0CONbits.T0CS   = 0; // Usa o mesmo clock que o microncontrolador
    T0CONbits.PSA    = 0; // Ativa Prescaler
    T0CONbits.T0PS   = 0b111; // Prescaler 128  
    TMR0H = (timer_preload >> 8 & 0xFF);
    TMR0L = (timer_preload & 0xFF);
    
    /* TIMER 1 CONFIG */
    T1CONbits.TMR1CS  = 0b00; 
    T1CONbits.T1CKPS  = 0b11; //1:8 pré (máx)
    T1CONbits.T1RD16  = 1;
    T1CONbits.TMR1ON  = 0;
    
    while (1){
        
        /* ---------------- Temperature Measurement ---------------- */
        adc_capture_average = adc_average(200);         //Captura e faz a média de 200 amostras do ADC;
        Rterm = Rterm_calc('A', adc_capture_average);   //Converte o codigo do adc em resistência
        Temp_measured = Temp_calc(Rterm);               //Converte o valor de resistência em Temperatura
        
        
        if (RECEIVED_COMMAND) {
            switch (RECEIVED_COMMAND) {
                /* Fish change commands */
                case COMMAND_CHANGE_FISH_1:
                    set_current_fish(&current_fish, 6);
                    EEPROM_write(0,'1');
                    break;
                case COMMAND_CHANGE_FISH_2:
                    set_current_fish(&current_fish, 2);
                    EEPROM_write(0,'2');
                    break;
                case COMMAND_CHANGE_FISH_3:
                    set_current_fish(&current_fish, 1);
                    EEPROM_write(0,'3');
                    break;
                case COMMAND_CHANGE_FISH_4:
                    set_current_fish(&current_fish, 5);
                    EEPROM_write(0,'4');
                    break;
                case COMMAND_CHANGE_FISH_5:
                    set_current_fish(&current_fish, 4);
                    EEPROM_write(0,'5');
                    break;
                case COMMAND_CHANGE_FISH_6:
                    set_current_fish(&current_fish, 3);
                    EEPROM_write(0,'6');
                    break;
            }                       
        }
        
        /* FSM main */
        switch(fsm_state){
            case 0:                      //Estado inicial / neutro
                EV_RECIRCULACAO   = 0;
                EV_ESGOTO         = 0;
                EV_REDE           = 0;
                BOMBA_AGUA        = 0;
                RESISTENCIA_AQUEC = 0;
                if (RECEIVED_COMMAND == COMMAND_START_MACHINE){
                    fsm_state = 1;
                }
                if (RECEIVED_COMMAND == COMMAND_START_DRAINNING){
                    fsm_state = 2;
                }
                if ((RECEIVED_COMMAND == COMMAND_START_FILLING) && !NVL_ALTO){
                    fsm_state = 3;
                }    
                break;
            
            case 1: //Recirculação
                EV_RECIRCULACAO   = 1;
                EV_ESGOTO         = 0;
                EV_REDE           = 0;
                BOMBA_AGUA        = 1;
                RESISTENCIA_AQUEC = 0;
                
                if ((Temp_measured <= (current_fish.average_temperature - 2)) && NVL_ALTO){                 
                        RESISTENCIA_AQUEC = 1;
                }
                else if(Temp_measured < current_fish.average_temperature){
                    asm("nop");
                }
                else{
                    RESISTENCIA_AQUEC = 0;
                }                
                
                if (RECEIVED_COMMAND == COMMAND_STOP_MACHINE || !NVL_BAIXO){
                    fsm_state = 0;
                } else if (RECEIVED_COMMAND == COMMAND_START_FILLING && !NVL_ALTO){
                    fsm_state = 3;
                } else if (RECEIVED_COMMAND == COMMAND_START_DRAINNING && NVL_BAIXO){
                    fsm_state = 2;
                }
                
                break;
            
            case 2: //Vazamento
                EV_RECIRCULACAO = 0;
                EV_ESGOTO = 1;
                EV_REDE = 0;
                BOMBA_AGUA = 1;
                RESISTENCIA_AQUEC = 0;
                
                if (RECEIVED_COMMAND == COMMAND_STOP_MACHINE || !NVL_BAIXO){
                    fsm_state = 0;
                } else if (RECEIVED_COMMAND == COMMAND_START_FILLING && !NVL_ALTO){
                    fsm_state = 3;
                } else if (RECEIVED_COMMAND == COMMAND_START_MACHINE && NVL_BAIXO){
                    fsm_state = 1;
                }
                break;
            
            case 3: //Enchimento
                EV_RECIRCULACAO   = 0;
                EV_ESGOTO         = 0;
                EV_REDE           = 1;
                BOMBA_AGUA        = 0;
                RESISTENCIA_AQUEC = 0;
                if (NVL_ALTO){
                    T1CONbits.TMR1ON  = 1;                
                }        
                
                // Temporização Overflow
                if (TMR1IF){                    
                    if (t1_counter > 300){
                        T1CONbits.TMR1ON  = 0;
                        t1_counter = 0;
                    fsm_state = 0;
                    } else {
                        t1_counter ++;
                    }
                    
                    TMR1H = (timer_preload >> 8 & 0xFF);
                    TMR1L = (timer_preload & 0xFF);
                    TMR1IF = 0;
                }
                                
                if (RECEIVED_COMMAND == COMMAND_STOP_MACHINE){
                    fsm_state = 0;
                    T1CONbits.TMR1ON  = 0;
                } else if (RECEIVED_COMMAND == COMMAND_START_DRAINNING && NVL_BAIXO){
                    fsm_state = 2;
                } else if (RECEIVED_COMMAND == COMMAND_START_MACHINE && NVL_BAIXO){
                    fsm_state = 1;
                }
                break;

        }
        
        /* Update EEPROM Value */     
        if (fsm_state != last_fsm_state){ 	//Update EEPROM when fsm_state changes
            EEPROM_write(0x10,fsm_state);        
        }
        last_fsm_state = fsm_state;
        
        /* Serial Send */
        if (TMR0IF) {
            send_sys_data(fsm_state, current_fish.identifier, Temp_measured);
            TMR0IF = 0;
            TMR0H = (timer_preload >> 8 & 0xFF);
            TMR0L = (timer_preload & 0xFF);
        }
        RECEIVED_COMMAND = 0;
    }
}

