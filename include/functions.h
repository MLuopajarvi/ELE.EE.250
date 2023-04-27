#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define LED_PIN     PB1
#define POT_PIN     PC0
#define SWITCH_PIN  PD3

#define SERVO_PIN   PD5
#define PWM_FREQ    50      // PWM frequency in Hz
#define PWM_PRESC   1024    // PWM prescaler

#define THERM_PIN   PC5
#define THERMISTOR_R0 10000.0 // Thermistor resistance at 25°C
#define THERMISTOR_B 3950.0 // Thermistor beta value


// Function declarations
void initialize_system();
int update_servo_position(int increment);
void enter_low_power_mode();
void update_led(uint16_t servo_pos);
void send_UART(uint16_t servo_pos, float temp);
float read_temp();
void read_UART();
void USART_Transmit_string(char *data );
