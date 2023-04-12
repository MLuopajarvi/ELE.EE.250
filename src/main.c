#include "../include/functions.h"

// Globals
volatile uint8_t system_on = 0;
volatile int servo_position = 0;
volatile uint16_t pot_value = 0;

ISR(ADC_vect) {
    // Read ADC value
    pot_value = ADC;
}

ISR(PCINT2_vect) {
    // Check switch state
    if (PIND & (1 << PD2)) {
        system_on = 1;
    } else {
        system_on = 0;
        // Enter low power mode
        enter_low_power_mode();
    }
}

// Main function
int main()
{
    initialize_system();

    while (1) {
        // If switch is on, operate normally
        if (system_on == 1) {
            update_servo_position(pot_value);
        }
    }
    return 0;
    
}