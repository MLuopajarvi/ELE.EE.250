#include "../include/functions.h"

void initialize_system()
{
    //DDRB |= (1 << PB1);                                      // Set Timer 1 output pin as output

    // Set up clock
    //CLKPR = (1<<CLKPCE);                                     // enable change of clock prescaler
    //CLKPR = 0;                                               // set clock prescaler to 1

    // Set up IO pin
    DDRB |= (1<<PB1);                                        // set PB1 as output (for LED)
    //PORTD &= ~(1<<PD5);                                      // set PD5 low initially (for servo signal)  
    
    // Configure ADC
    //ADMUX |= (1 << REFS0);                                   // Set reference voltage to AVCC
    //ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // Set ADC prescaler to 128
    //ADCSRA |= (1 << ADIE);                                   // Enable ADC interrupt
    //ADCSRA |= (1 << ADEN);                                   // Enable ADC
    
    // Set up PWM control of the servo
    DDRD |= (1<<PD5);                                        // set PD5 as output (for servo signal)

    TCCR1B |= (1 << CS11);                                  // Set Timer 1 prescaler to 64
    TCCR1A |= (1 << WGM11);                                 // Set Timer 1 to Fast PWM mode
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    TCCR1A |= (1<<COM1A1 | 1<<COM1A0);                      // Inverted mode
    ICR1 = F_CPU/50;                                        // Set ICR1 for 50Hz
    OCR1A = ICR1 - 20000;                                   // Set initial servo position to middle (PW 20k cycles)

    // Set up PWM for the LED
    //TCCR0B |= (1 << CS01) | (1 << CS00);                    // Set Timer 0 prescaler to 64
    //TCCR0A |= (1 << WGM01) | (1 << WGM00);                  // Set Timer 0 to Fast PWM mode
    //TCCR0A |= (1 << COM0A1);                                // Set Timer 0 to non-inverted output mode


    // Set up on/off switch   
    //DDRD &= ~(1 << PD3);                                    // Set switch pin as input
    //PORTD |= (1 << PD3);                                    // Enable pull-up resistor
    
    //PCMSK2 |= (1 << PCINT19);                               // Enable pin change interrupt on switch pin
    //PCICR |= (1 << PCIE2);                                  // Enable pin change interrupt


    // Set up terminal UART
    //UBRR0 = 103;                                            // Set baud rate to 9600
    //UCSR0B |= (1 << TXEN0) | (1 << RXEN0);                  // Enable transmitter and receiver
    //UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);                // Set frame format: 8 data bits, no parity, 1 stop bit
    
    sei();
}



uint16_t read_adc(uint8_t adc_pin) {
    // Select ADC input pin
    ADMUX = (ADMUX & 0xF0) | (adc_pin & 0x0F);
    // Start ADC conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    // Return ADC result
    return ADC;
}


// Updates the position of the servo motor
int update_servo_position(int increment) {
    uint16_t adc_val = read_adc(POT_PIN);
    // Map potentiometer value to servo position
    int servo_pos = 375 + ((adc_val + increment) * 3) / 10;
    OCR1A = servo_pos;
    return servo_pos;
}

float read_temp() {
    // Read thermistor resistance
    uint16_t adc_val = read_adc(THERM_PIN);
    float thermistor_r = THERMISTOR_R0 * ((1023.0 / adc_val) - 1.0);
    // Calculate temperature in degrees Celsius
    float temp_c = (1.0 / ((log(thermistor_r / THERMISTOR_R0) / THERMISTOR_B) + (1.0 / 298.15))) - 273.15;
    return temp_c;
}

void enter_low_power_mode() {
    TCCR1B = 0;  // stop Timer0  
    // Enter sleep mode with ADC noise reduction
    set_sleep_mode(SLEEP_MODE_ADC);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

void update_led(uint16_t servo_pos) {
    // Map servo position to LED brightness (0-255)
    uint8_t brightness = (servo_pos * 255) / 1023;
    // Set Timer 0 compare match value to control LED brightness
    OCR0A = brightness;
}


void send_UART(uint16_t servo_pos, float temp) {
    // Convert servo position to ASCII string
    char buf[10];
    sprintf(buf, "Servo:%d  Temp:%f\r\n", servo_pos,temp);
    // Send string to UART
    for (uint8_t i = 0; i < strlen(buf); i++) {
        while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
        UDR0 = buf[i]; // Send next character
    }
}

void read_UART() {
    if (UCSR0A & (1 << RXC0)) {
        // Read input character from UART
        char input_char = UDR0;
        // Handle increment/decrement commands
        if (input_char == '+') {
            update_servo_position(1);
        } else if (input_char == '-') {
            update_servo_position(-1);
        }
    }
}