#include "../include/functions.h"

// Globals
volatile uint8_t system_on = 0;
volatile int servo_position = 0;
float temp = 0;

// ISR(INT1_vect) {
//     // Check switch state
//     if (PIND & (1 << PD3)) {
//         system_on = 1;
//     } else {
//         system_on = 0;
//         //servo_position = update_servo_position(1);
//         // Enter low power mode
//         //enter_low_power_mode();
//     }
// }

// // USART Receive interrupt
// ISR(USART_RX_vect)
// {
//     read_UART();
// }


// Main function
int main()
{
    //initialize_system();
    DDRB |= (1<<PB1);                                        // set PB1 as output (for LED)
    DDRD |= (1<<PD5);                                        // set PD5 as output (for servo signal)


    TCCR1A |= (1<<COM1A1 | 1<<COM1A0);                      // Inverted mode

    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);    // Fast PWM mode with top value of ICR1 & Prescaler of 8

    ICR1 = 39999;                                           // 16MHz / (8 prescaler * 50Hz) - 1 = 39999
    OCR0B = ICR1 - 4000;                                    // Set initial servo position to middle


    while (1) {
        // If switch is on, operate normally
        // if (system_on == 0) {
        //     servo_position = update_servo_position(0);
        //     update_led(servo_position);
        //     temp = read_temp();
        //     send_UART(servo_position, temp);
        // }
        // Turn on the LED
        PORTB |= (1 << PB1);

        // Wait for 500ms
        _delay_ms(100);

        OCR0B = ICR1 - 1600;
        _delay_ms(100);

        // Turn off the LED
        PORTB &= ~(1 << PB1);
        _delay_ms(100);

        OCR0B = ICR1 - 4400;
        _delay_ms(100);

        //send_UART(servo_position, temp);
        //servo_position = update_servo_position(0);
    }
    return 0;
    
}