#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define LED_PIN     PB1
#define POT_PIN     PC0
#define SWITCH_PIN  PD3

#define SERVO_PIN   PD5
#define PWM_FREQ    50      // PWM frequency in Hz
#define PWM_PRESC   1024    // PWM prescaler

#define THERM_PIN   PC5
#define THERMISTOR_R0 10000.0    // Thermistor resistance at 25°C KTY81 from datasheet
#define THERMISTOR_B 3950.0     // Thermistor beta value


#define UBRRVALUE (F_CPU/(16ul*9600)-1)

void initialize_system();
uint16_t update_servo_position(uint16_t degrees, int incr, int temp);
void enter_low_power_mode();
void update_led(uint16_t servo_pos);
void send_UART(uint16_t servo_pos, float temp);
float read_temp();
int read_UART();
uint16_t read_adc(uint8_t adc_pin);
void USART_Transmit_string(char *data );
