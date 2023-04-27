#define F_CPU 16000000ul

#include "../include/functions.h"

volatile uint8_t switch_on = 0;
volatile uint16_t servo_angle = 90; // initial servo angle is set to 90 (0 for the simulation servo)
volatile float temp = 0;
volatile int pot_adc = 0;


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

int main(void)
{
    
    initialize_system();

    while (1)
    {
        if (switch_on == 1)
        {
            pot_adc = 5000;
            servo_angle = pot_adc/28; // conversion from ad to angle, rough rounding from 27,77 to 28
            send_UART(servo_angle, temp);
            update_servo_position(servo_angle);
            update_led(servo_angle);
            _delay_ms(500);

            pot_adc = 2500;
            servo_angle = pot_adc/28; // conversion from ad to angle, rough rounding from 27,77 to 28
            send_UART(servo_angle, temp);
            update_servo_position(servo_angle);
            update_led(servo_angle);
            _delay_ms(500);
        }
    }
    
    return 0;
}