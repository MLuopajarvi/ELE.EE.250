#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED_PIN     PB0
#define START_PIN   PC0
#define STOP_PIN    PC1
#define INTERVAL_MS 500

// Constants
#define BAUDRATE 9600

// globals
volatile int temperature = 0;
volatile uint8_t sleep_flag = 1;

// Function declarations
void initialize_system();
void update_servo_position(int amount);
void enter_low_power_mode();
