#include "../include/functions.h"

// Globals
volatile bool system_on = false;
volatile int servo_position = 0;
volatile uint16_t adc_value = 0;

// Interrupt service routine for the USART
ISR(USART_RX_vect)
{
    char input = UDR0;
    if (input == '+')
    {
        servo_position++;
        servo_tuned = true;
    }
    else if (input == '-')
    {
        servo_position--;
        servo_tuned = true;
    }
}

ISR(ADC_vect)
{
    adc_value = ADC;
    update_servo_position(adc_value);
}

// Main function
int main()
{
    initialize_system();
    system_on = true;
    //update_led_brightness();
    
    while (1)
    {
        if (system_on)
        {
            update_temperature();
            send_data_to_uart();
            _delay_ms(2000);
            
            if (servo_tuned)
            {
                //update_servo_position(temperature);
                //update_led_brightness();
                servo_tuned = false;
            }
        }
        else
        {
            enter_low_power_mode();
            wait_for_switch_on();
        }
    }
    
    return 0;
}