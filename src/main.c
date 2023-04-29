#define F_CPU 16000000

#include "../include/functions.h"

volatile uint8_t switch_on = 0;
volatile uint16_t servo_angle = 90; // initial servo angle is set to 90 (middle)
volatile float temp = 0;
volatile uint16_t pot_adc = 0;
volatile int terminal_increment = 0;


ISR(INT1_vect)
{
    if (switch_on == 0)
    {
        switch_on = 1;
    }
    else
    {
        switch_on = 0;
    }
}

ISR(TIMER0_COMPB_vect) {
    //For what ever reason, ISR for T0 needs to be added for the program to work at all
}

ISR(USART_RX_vect) {
    terminal_increment = read_UART();
}


int main(void)
{
    
    initialize_system();

    while (1)
    {
        if (switch_on == 1)
        {
            pot_adc = read_adc(POT_PIN);
            temp = read_temp();
            servo_angle = update_servo_position(pot_adc, terminal_increment, (int)temp);
            update_led(servo_angle);
            send_UART(servo_angle, temp);
            _delay_ms(1000);
        }
        else {
            enter_low_power_mode();
        }
    }
    
    return 0;
}