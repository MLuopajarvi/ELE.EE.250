#include "../include/functions.h"

void initialize_system()
{
     // Set Timer 1 output pin as output
    DDRB |= (1 << PB1);
    
    // Set up clock
    CLKPR = (1<<CLKPCE); // enable change of clock prescaler
    CLKPR = 0; // set clock prescaler to 1

    // Set up IO pin
    DDRB |= (1<<PB1); // set PB1 as output (for LED)
    DDRD |= (1<<PD5); // set PD5 as output (for servo signal)
    PORTD &= ~(1<<PD5); // set PD5 low initially (for servo signal)  
    
    // Configure ADC for reading potentiometer
    ADMUX |= (1 << REFS0);                                   // Set reference voltage to AVCC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // Set ADC prescaler to 128
    ADCSRA |= (1 << ADIE);                                   // Enable ADC interrupt
    ADCSRA |= (1 << ADEN);                                   // Enable ADC
    
    // Set up PWM control of the servo
    TCCR1B |= (1 << CS11) | (1 << CS10); // Set Timer 1 prescaler to 64

    TCCR1A |= (1 << WGM11) | (1 << WGM10);  // Set Timer 1 to Fast PWM mode
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    TCCR1A |= (1 << COM1A1);    // Set Timer 1 to non-inverted output mode
    ICR1 = 4999;                // Set Timer 1 TOP value to 4999 for 20ms period
    OCR1A = 375;                // Set initial servo position to middle

    // Set up on/off switch   
    DDRD &= ~(1 << PD3);        // Set switch pin as input
    PORTD |= (1 << PD3);        // Enable pull-up resistor
    
    PCMSK2 |= (1 << PCINT18);   // Enable pin change interrupt on switch pin
    PCICR |= (1 << PCIE2);      // Enable pin change interrupt

    sei();
}


// Updates the position of the servo motor
int update_servo_position(int amount) {
    
    if (amount < -90) {
        amount = -90;
    }
    else if (amount > 90) {
        amount = 90;
    }
    
    int pulse_width = (amount + 90) * 10 + 1000;
    OCR1A = pulse_width / 4;

    return amount;
}

void update_led_brightness(uint8_t servo_pos) {
    OCR0A = brightness; // Update the PWM duty cycle for LED brightness control
}

void enter_low_power_mode() {
    sleep_flag = 1;
    TCCR0B = 0;  // stop Timer0  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode
}

void update_temperature() {

}

void send_data_to_uart() {

}

void wait_for_switch_on() {

}

void send_uart_string(char *str) {

}