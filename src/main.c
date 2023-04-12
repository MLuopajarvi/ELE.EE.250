#include "../include/functions.h"

// Globals
volatile bool system_on = false;
volatile int servo_position = 0;
volatile uint16_t adc_value = 0;

// Main function
int main()
{
    initialize_system();
    system_on = true;
    
    
}