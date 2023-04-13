#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdio.h>


#define LED_PIN     PB0
#define POT_PIN     PC0
#define INTERVAL_MS 500

#define THERM_PIN   PC5
#define THERMISTOR_R0 10000.0 // Thermistor resistance at 25°C
#define THERMISTOR_B 3950.0 // Thermistor beta value

// Function declarations
void initialize_system();
int update_servo_position(int amount);
void enter_low_power_mode();
void update_led(uint16_t servo_pos);
void send_UART(uint16_t servo_pos);
float read_temp();
