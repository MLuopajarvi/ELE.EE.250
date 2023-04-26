#include "../include/functions.h"

void initialize_system()
{
    cli();

    CLKPR = (1 << CLKPCE);  // enable change of clock prescaler
    CLKPR = 0x00;           // set clock prescaler to 1 (default)

    DDRB |= (1 << LED_PIN);     // set PB1 as output
    DDRD &= ~(1 << SWITCH_PIN);    // set PD3 as input (for switch)
    PORTD |= (1 << SWITCH_PIN);    // enable pull-up resistor on PD3
    EICRA |= (1 << ISC01);  // set INT0 to trigger on falling edge
    EIMSK |= (1 << INT1);   // enable INT1
    

    // Enable Servo pin (PD5)
    DDRD |= (1 << SERVO_PIN);

    // SET TC0 TOP
    OCR0A = 0x9B;

    //Count
    TCNT0 = 0x0;

    //COM0A 0; COM0B 2; WGM0 3; 
    TCCR0A = 0x23;

    //CS0 RUNNING_CLK_1024; FOC0A disabled; FOC0B disabled; WGM02 1; 
    TCCR0B = 0xD;

    //OCIE0A disabled; OCIE0B disabled; TOIE0 disabled; 
    TIMSK0 = 0x04;

    OCR0B = 155 * 0.05 + (155 * 0.1);


    // Set up ADC
    ADMUX = (1 << REFS0); //external capacitor at Vref and ADC done from pin ADC0
	ADCSRA = (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); //enable and set prescaler
	ADCSRA |= (1 << ADEN); // Enable AD-converter
    ADMUX |= (1 << ADLAR); // Setting to Left justified mode so only 8 bits can be read
    DIDR0 = (1 << ADC0D) | (1 << ADC1D); // disable input buffer for ADC pins.
    
    
	// USART Setup
	// Set the USART0 module to the options selected in the user interface.
    //UBRR0 1; 
    UBRR0 = 0x1;
    //DOR0 disabled; FE0 disabled; MPCM0 disabled; RXC0 disabled; TXC0 disabled; U2X0 enabled; UDRE0 disabled; UPE0 disabled; 
    UCSR0A = 0x2;
    //RXB80 disabled; RXCIE0 disabled; RXEN0 enabled; TXB80 disabled; TXCIE0 disabled; TXEN0 enabled; UCSZ02 disabled; UDRIE0 disabled; 
    UCSR0B = 0x18;
    //UCPOL0 disabled; UCSZ0 3; UMSEL0 Asynchronous Mode; UPM0 Disabled; USBS0 1-bit; 
    UCSR0C = 0x6;

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
    OCR0A = servo_pos;
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
    char* string = "servo: ";
    char dest[12];
    strcpy( dest, string );
    strcat(string, itoa(servo_pos, buf, 10));
    USART_Transmit_string(string);
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