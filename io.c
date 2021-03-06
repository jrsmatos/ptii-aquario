#include <pic18.h>
#include "io.h"

void putch(unsigned char c) {
    while (!TXSTA1bits.TRMT);
    TXREG1 = c;
}

void send_sys_data(char machine_state, char fish, float temperature) {
    // **** Envia informacao sobre estado do sistema ****    
    putch(machine_state);    // Estado da m?quina
    putch(PORTB);        // Estado do sistema (controlos)
    putch(PORTA);      // Estado do sistema (sensores)
    putch(99);               // Valor pH
    putch((int)(temperature*10)/10);  // Valor temperatura (unidade)
    putch((int)(temperature*10)%10);  // Valor temperatura (decimal)
    putch(fish);             // Peixe selecionado
    putch('\n');             // Termino de comunica??o
}
