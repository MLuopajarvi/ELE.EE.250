#include "../include/functions.h"

uint8_t pwm_top = (F_CPU / PWM_PRESC) / (2 * PWM_FREQ) - 1; // calculate PWM TOP value: ((16Mhz/1024)/(2*50Hz))-1

void initialize_system()
{
    cli();

    CLKPR = (1 << CLKPCE);  // enable change of clock prescaler
    CLKPR = 0x00;           // set clock prescaler to 1 (default)

    DDRD &= ~(1 << SWITCH_PIN);    // set PD3 as input (for switch)
    PORTD |= (1 << SWITCH_PIN);    // enable pull-up resistor on PD3
    EICRA |= (1 << ISC01);  // set INT0 to trigger on falling edge
    EIMSK |= (1 << INT1);   // enable INT1

    // Set up servo PWM for timer 0
    DDRD |= (1 << SERVO_PIN);                                   // Enable Servo pin (PD5)
    OCR0A = pwm_top;                                            // Set TOP value for OCR0A
    TCNT0 = 0x0;                                                //Set count to 0
    TCCR0A = 0x23;                                              //COM0A 0; COM0B 2; WGM0 3;
    TCCR0B = 0xD;                                               //CS0 RUNNING_CLK_1024; FOC0A disabled; FOC0B disabled; Fast Mode PWM with OCR0A as TOP; 
    TIMSK0 = 0x04;                                              //OCIE0A disabled; OCIE0B disabled; TOIE0 enabled for debugging; 
    uint16_t servo_dc = pwm_top * 0.09 + (pwm_top * 0.056);      // Calibrated duty cycle: 9%: 0 degrees, 21%: 180 degrees, set initial position to 90 degrees
    OCR0B = servo_dc;                                           // Set OCR0B at 7.5% duty cycle for middle position


    // Set up LED PWM for timer 1
    DDRB |= (1 << LED_PIN);     // Enable led pin (PB1)
    ICR1 = pwm_top;             // Use same TOP as for timer 0, but using ICR1 register
    TCNT1 = 0x0;                //Set count to 0
    TCCR1A = 0x82;              //COM1A 2; COM1B 0; WGM1 3; 
    TCCR1B = 0x1D;              //CS1 RUNNING_CLK_1024; ICES1 disabled; ICNC1 disabled; WGM1 3; 
    TCCR1C = 0x0;               //FOC1A disabled; FOC1B disabled; 
    TIMSK1 = 0x0;               //ICIE1 disabled; OCIE1A disabled; OCIE1B disabled; TOIE1 disabled; 
    OCR1A = 0;                  // Set the LED brightness to 0

    // Set up ADC
    ADMUX = (1 << REFS0);       //external capacitor at Vref and ADC done from pin ADC0
	ADCSRA = (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); //enable and set prescaler
	ADCSRA |= (1 << ADEN); // Enable AD-converter
    //ADMUX |= (1 << ADLAR); // Setting to Left justified mode so only 8 bits can be read
    DIDR0 = (1 << ADC0D) | (1 << ADC1D); // disable input buffer for ADC pins.
    
    
	// USART Setup
    UBRR0 = 0x067;

    //DOR0 disabled; FE0 disabled; MPCM0 disabled; RXC0 disabled; TXC0 disabled; U2X0 disabled; UDRE0 disabled; UPE0 disabled; 
    UCSR0A = 0x0;

    //RXB80 disabled; RXCIE0 disabled; RXEN0 enabled; TXB80 disabled; TXCIE0 disabled; TXEN0 enabled; UCSZ02 disabled; UDRIE0 disabled; 
    UCSR0B = 0x18;

    //UCPOL0 disabled; UCSZ0 3; UMSEL0 Asynchronous Mode; UPM0 Disabled; USBS0 1-bit; 
    UCSR0C = 0x6;

    // Enable global interrups
    sei();
}



uint16_t read_adc(uint8_t adc_pin) {
    // Select ADC input pin
    ADMUX = (ADMUX & 0xF0) | (adc_pin & 0x0F);
    // Start ADC conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete
    while( !(ADCSRA & (1<<ADIF)) );
    // Return ADC result
    return ADC;
}


// Updates the position of the servo motor
void update_servo_position(int degrees) {
    // Convert degrees to pulse width
    int zero_point = pwm_top*0.09;

    int pulse_width = zero_point + (degrees * 0.1035);
    
    // Set duty cycle based on pulse width
    OCR0B = pulse_width;
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
    uint16_t brightness = servo_pos / 2;
    // Set Timer 0 compare match value to control LED brightness
    OCR1A = brightness;
}


void send_UART(uint16_t servo_pos, float temp) {
    // Convert servo position to ASCII string
    char buf[10];
    //char* string = "VIHHU SAATANA\r\n";
    //char dest[12];
    //strcpy( dest, string );
    //strcat(string, itoa(servo_pos, buf, 10));
    USART_Transmit_string(itoa(servo_pos, buf, 10));
}

void USART_Transmit_string( char *data )

{

	uint8_t i=0;

	while(data[i] != 0) {

		/* Wait for empty transmit buffer */

		while ( !( UCSR0A & (1<<UDRE0)) ) ;

        /* Put data into buffer, sends the data */

		UDR0 = data[i];

		i++;

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