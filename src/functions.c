#include "../include/functions.h"

uint16_t pwm_top = (F_CPU / PWM_PRESC) / (2 * PWM_FREQ) - 1; // calculate PWM TOP value: ((16Mhz/1024)/(2*50Hz))-1

void initialize_system()
{
    cli();

    // Ext clock setup
    CLKPR = (1 << CLKPCE);  // enable change of clock prescaler
    CLKPR = 0x00;           // set clock prescaler to 1 (default)

    enable_on_off();

    // Set up servo PWM for timer 0
    DDRD |= (1 << SERVO_PIN);                                   // Enable Servo pin (PD5)
    OCR0A = 155;                                                // Set TOP value for OCR0A
    TCNT0 = 0x0;                                                //Set count to 0
    TCCR0A = 0x23;                                              //COM0A 0; COM0B 2; WGM0 3;
    TCCR0B = 0xD;                                               //CS0 RUNNING_CLK_1024; FOC0A disabled; FOC0B disabled; Fast Mode PWM with OCR0A as TOP; 
    TIMSK0 = 0x04;                                              //OCIE0A disabled; OCIE0B disabled; TOIE0 enabled for debugging; 
    uint16_t servo_dc = pwm_top * 0.09;                         // Calibrated duty cycle: 9%: 0 degrees, 21%: 180 degrees, set initial position to 0 degrees
    OCR0B = servo_dc;                                           // Set OCR0B at 7.5% duty cycle for middle position


    // // Set up LED PWM for timer 1
    DDRB |= (1 << LED_PIN);     // Enable led pin (PB1)
    ICR1 = 155;             // Use same TOP as for timer 0, but using ICR1 register
    TCNT1 = 0x0;                //Set count to 0
    TCCR1A = 0x82;              //COM1A 2; COM1B 0; WGM1 3; 
    TCCR1B = 0x1D;              //CS1 RUNNING_CLK_1024; ICES1 disabled; ICNC1 disabled; WGM1 3; 
    TCCR1C = 0x0;               //FOC1A disabled; FOC1B disabled; 
    TIMSK1 = 0x0;               //ICIE1 disabled; OCIE1A disabled; OCIE1B disabled; TOIE1 disabled; 
    OCR1A = 0;                  // Set the LED brightness to 0

    // Set up ADC
    ADMUX |= (1 << REFS0);          // Set reference voltage to AVCC
    ADMUX &= ~(1 << ADLAR);         // Right-justify ADC result
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Set ADC prescaler to 128
    ADCSRA |= (1 << ADEN);          // Enable ADC
    
	// USART Setup
    UBRR0 = UBRRVALUE;

    UCSR0A = 0x0;   //DOR0 disabled; FE0 disabled; MPCM0 disabled; RXC0 disabled; TXC0 disabled; U2X0 disabled; UDRE0 disabled; UPE0 disabled; 
    UCSR0B = 0x98;  //RXB80 disabled; RXCIE0 enabled; RXEN0 enabled; TXB80 disabled; TXCIE0 disabled; TXEN0 enabled; UCSZ02 disabled; UDRIE0 disabled;
    UCSR0C = 0x6;   //UCPOL0 disabled; UCSZ0 3; UMSEL0 Asynchronous Mode; UPM0 Disabled; USBS0 1-bit; 

    // Enable global interrups
    sei();
}

void enable_on_off() {
    // On/Off Switch set up
    DDRD &= ~(1 << SWITCH_PIN);    // set PD3 as input (for switch)
    PORTD |= (1 << SWITCH_PIN);    // enable pull-up resistor on PD3
    EICRA |= (1 << ISC01);  // set INT0 to trigger on falling edge
    EIMSK |= (1 << INT1);   // enable INT1
}


uint16_t read_adc(uint8_t adc_pin) {
    ADMUX &= 0xF0;                  // Clear previous ADC channel
    ADMUX |= adc_pin;               // Set new ADC channel
    ADCSRA |= (1 << ADSC);          // Start conversion
    while (ADCSRA & (1 << ADSC));   // Wait for conversion to complete
    return ADC;                     // Return the ADC result
}


// Updates the position of the servo motor
uint16_t update_servo_position(uint16_t pot_adc, int incr, int temp) {
    // Convert degrees to pulse width
    int zero_point = pwm_top*0.09;

    uint32_t pot_angle = (pot_adc*180)/500;
    uint16_t angle = ((pot_angle+temp)/2)+incr;
    uint16_t pulse_width = zero_point + (angle * 0.1035);
    
    // Set duty cycle based on pulse width
    OCR0B = pulse_width;

    return angle;
}

float read_temp() {
    uint16_t adc_val = read_adc(THERM_PIN);
    float voltage = (adc_val / 500.0) * 5.0; // Convert ADC value to voltage
    float resistance = (5.0 - voltage) / voltage * THERMISTOR_R0; // Calculate thermistor resistance
    float temperature = 1.0 / (log(resistance / THERMISTOR_R0) / THERMISTOR_B + 1 / 298.15) - 273.15; // Calculate temperature in Celsius
    return temperature;

    return temperature;
}

void enter_low_power_mode() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    sei();
    sleep_cpu();
  
    // entry-point after wake-up
    sleep_disable();
}

void update_led(uint16_t servo_pos) {
    // Map servo position to LED brightness (0-255)
    uint16_t brightness = (servo_pos / 180.0) * 255.0;

    // Set Timer 0 compare match value to control LED brightness
    OCR1A = brightness;
}


void send_UART(uint16_t servo_pos, float temp) {
    // Quick and dirty solution, fix if time
    char buf[10];
    char* servo_string = "\r\n Servo angle: ";
    USART_Transmit_string(servo_string);
    USART_Transmit_string(itoa(servo_pos, buf, 10));

    char* temp_string = "\r\n Temp: ";
    USART_Transmit_string(temp_string);
    USART_Transmit_string(itoa((uint16_t)temp, buf, 10));
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

int read_UART() {
    if (UCSR0A & (1 << RXC0)) {
        // Read input character from UART
        char input_char = UDR0;
        // Handle increment/decrement commands
        if (input_char == '+') {
            return 1;
        } else if (input_char == '-') {
            return -1;
        }
    }
    return 0;
}