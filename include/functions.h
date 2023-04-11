#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

#define LED_PIN     PB0
#define START_PIN   PC0
#define STOP_PIN    PC1
#define INTERVAL_MS 500

// Constants
#define BAUDRATE 9600

// globals
volatile bool servo_tuned = false;
volatile int temperature = 0;
volatile uint8_t sleep_flag = 1;

// Function declarations
void initialize_system();
int update_servo_position(int amount);
void update_led_brightness(uint8_t servo_po);
void update_temperature();
void send_data_to_uart();
void wait_for_switch_on();
void enter_low_power_mode();
void send_uart_string(char *str);
