#include <pic18.h>
#include "io.h"

void putch(unsigned char c) {
    while (!TXSTA1bits.TRMT);
    TXREG1 = c;
}

void send_sys_data(char machine_state, char fish, float temperature) {
    // **** Envia informacao sobre estado do sistema ****

    char sys_state = 0;
    sys_state = PORTB;
    
    char sys_state_2 = 0;
    sys_state_2 = PORTA;
    
    putch(machine_state);    // Estado da máquina
    putch(sys_state);        // Estado do sistema (controlos)
    putch(sys_state_2);      // Estado do sistema (sensores)
    putch(99);               // Valor pH
    putch((int)(temperature*10)/10);  // Valor temperatura (unidade)
    putch((int)(temperature*10)%10);  // Valor temperatura (decimal)
    putch(fish);             // Peixe selecionado
    putch('\n');             // Termino de comunicação
}
