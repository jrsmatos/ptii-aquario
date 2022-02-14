
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

//#define Reserva LATB0
#define EV_RECIRCULACAO   LATB1
#define EV_ESGOTO         LATB2
#define EV_REDE           LATB3
#define BOMBA_AGUA        LATB4
#define RESISTENCIA_AQUEC LATB5
//#define OCUPADO LATB6
//#define OCUPADO LATB7

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
        // Interrupcao serie
        RECEIVED_COMMAND = RCREG1;
    }
}
int adc_capture(){
    
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1);
    
    uint16_t code = 0x0000;         //16bit var reset
    code = ADRESH;                  //High byte -> 0b00000000 000000HH;
    code = code << 8;               //Shift left 8 times  -> 0b000000HH 00000000
    code = code | ADRESL;           // 0b000000HH LLLLLLLL 
  
    return code;
}
void PORT_init(){
    /* General IO*/
    TRISB = 0;
    LATB = 0;
    
    TRISA0 = 1; // Sensor de nivel (alto)
    TRISA1 = 1; // Sensor de nivel (baixo)
    ANSA0 = 0;
    ANSA1 = 0;
    
    /*UART IO*/
    TRISC6 = 0;             // OUTPUT TX
    TRISC7 = 1;             // INPUT RX
    ANSC7  = 0;             // Set Input as DIGITAL
 
    /*ADC IO*/
    ANSELA = 0;
    TRISA1 = 1; 
    ANSA1  = 1;
    
    /*Interrupts*/
    PEIE = 1;
    GIE = 1;
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
    
    RC1IE = 1; //Interrupt flag Enable

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
    
    // **** CURRENT FISH CONFIG ****
    FishParams current_fish;
    set_current_fish(&current_fish, 2);

    char machine_state = STATE_STOPPED;   // Define initial machine state
    //set_machine_state(machine_state);     // Sets initial machine state
    
    float adc_capture_average;
    float Rterm;
    float Temp_measured;
    
    while (1){
            
        /* ---------------- Temperature control ---------------- */
        adc_capture_average = adc_average(200);         //Captura e faz a média de 200 amostras do ADC;
        Rterm = Rterm_calc('B', adc_capture_average);   //Converte o codigo do adc em resistência
        Temp_measured = Temp_calc(Rterm);               //Converte o valor de resistência em Temperatura
                
        if (Temp_measured <= (current_fish.average_temperature - 2)){
            RESISTENCIA_AQUEC = 1;
        }
        else if(Temp_measured < current_fish.average_temperature){
            asm("nop");
        }
        else{
            RESISTENCIA_AQUEC = 0;
        }
        
        /* ---------------- Command processing ---------------- */
        if (RECEIVED_COMMAND) {
            switch (RECEIVED_COMMAND) {
                /* Machine State commands */
                case COMMAND_START_MACHINE:
                    machine_state = STATE_CIRCULATION;
                    break;
                case COMMAND_STOP_MACHINE:
                    machine_state = STATE_STOPPED;
                    break;
                case COMMAND_START_DRAINNING:
                    machine_state = STATE_DRAINNING;
                    break;
                case COMMAND_START_FILLING:                  
                    machine_state = STATE_FILLING;
                    break;
                
                /* Fish change commands */
                case COMMAND_CHANGE_FISH_1:
                    set_current_fish(&current_fish, 6);
                    break;
                case COMMAND_CHANGE_FISH_2:
                    set_current_fish(&current_fish, 2);
                    break;
                case COMMAND_CHANGE_FISH_3:
                    set_current_fish(&current_fish, 1);
                    break;
                case COMMAND_CHANGE_FISH_4:
                    set_current_fish(&current_fish, 5);
                    break;
                case COMMAND_CHANGE_FISH_5:
                    set_current_fish(&current_fish, 4);
                    break;
                case COMMAND_CHANGE_FISH_6:
                    set_current_fish(&current_fish, 3);
                    break;
            }
            RECEIVED_COMMAND = 0;
        }

        send_sys_data(machine_state, current_fish.identifier, Temp_measured);
            
        
        //__delay_ms(25);
    }
}

