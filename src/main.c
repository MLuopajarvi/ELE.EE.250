#define F_CPU 16000000

#include "../include/functions.h"

volatile uint8_t switch_on = 0;
volatile uint16_t servo_angle = 90; // initial servo angle is set to 90 (0 for the simulation servo)
volatile float temp = 0;

ISR(USART_RX_vect){
	read_UART();
}

ISR(INT1_vect)
{
    if (switch_on == 0)
    {
        switch_on = 1;
    }
    else
    {
        switch_on = 0;
        enter_low_power_mode();
    }
}

ISR(TIMER0_COMPB_vect) {
    PORTB ^= (1 << PB1);   // toggle LED
}

int main(void)
{
    
    initialize_system();

    while (1)
    {
        if (switch_on == 1)
        {
            _delay_ms(1000);
            servo_angle = 0;
            OCR0B = ICR1 * 0.1; 
            update_led(servo_angle);
            // _delay_ms(100);
            // PORTB ^= (1 << PB1);   // toggle LED
            //send_UART(servo_pos, temp);
            _delay_ms(1000);
            servo_angle = 180;
           // OCR0B = 154 * 0.05;
            OCR0B = ICR1 * 0.05;
            update_led(servo_angle);
        }
    }
    
    return 0;
}