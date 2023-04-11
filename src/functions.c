#include "../include/functions.h"

void initialize_system()
{
    // Configure pins for LED and servo
    //DDRB |= (1 << PB1) | (1 << PB3);
    //TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    //TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
    //OCR1A = 0;

    // Set up Timer/Counter 0 for Fast PWM
    TCCR0A |= (1 << WGM01) | (1 << WGM00);   // Fast PWM mode
    TCCR0A |= (1 << COM0A1);                 // Non-inverted PWM on OCR0A
    TCCR0B |= (1 << CS01);                   // Prescaler of 8
    
    // Set up PB0 (pin 8) for PWM output
    DDRB |= (1 << PB0);                      // Set PB0 as an output pin
    
    // Configure ADC for reading potentiometer
    ADMUX |= (1 << REFS1) | (1 << MUX0);
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);
    sei();
    
    // Configure USART for serial communication
    UBRR0 = F_CPU / (16UL * BAUDRATE) - 1;
    UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
    
    // Configure switch
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
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
    uint8_t brightness = map(servo_pos, 0, 180, 0, 255); // Map servo position to LED brightness range (0-255)
    OCR0A = brightness; // Update the PWM duty cycle for LED brightness control
}

void enter_low_power_mode() {
    sleep_flag = 1;
    TCCR0B = 0;  // stop Timer0  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode
    sleep_enable();  // enable sleep
    sleep_cpu();  // enter sleep  
}

void update_temperature() {

}

void send_data_to_uart() {

}

void wait_for_switch_on() {

}

void send_uart_string(char *str) {

}