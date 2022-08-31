#include "serial.h"

ISR (USART_RX_vect) {
    // Convert binary into integers that you can store
    // Receive @ then we start our reception

    buf[count] = UDR0;

    if (buf[count] == '@') {
        receiving = 1;
    }

    count++; // Will go up to 5.

    if ((buf[count - 1] == '+' || buf[count - 1] == '-') && buf[count - 2] != '@') {
        count = 0;
        receiving = 0;
    }
    else if (count > 2 && count < 5 && (buf[count - 1] < 48 || buf[count - 1] > 58)) {
        count = 0;
        receiving = 0;
    }
    else if (count == 5 && buf[count - 1] == '#') {
        received = 1;
        remoteTempTens = buf[2] - 48;
        remoteTempOnes = buf[3] - 48;
    }
}
