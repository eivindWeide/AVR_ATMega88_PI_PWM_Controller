/**
 * PI controller to regulate the speed of a DC motor.
 *
 *
 * --- Pins ---
 * PD7 - Output: Power LED
 * PD6 - Output: Testing/Status LED
 * PB1 - Output: PWM signal for motor driver (OC1A)
 * PC3 - Input:  Analog signal for speed fine-tuning (ADC3)
 * PC4 - Input:  Encoder Channel B (PCINT20)
 * PC5 - Input:  Encoder Channel A (PCINT21)
 * PD0 - Input:  USART Receive (RX)
 * PD1 - Output: USART Transmit (TX)
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// --- Configuration Constants ---

// Serial Communication
#define BAUD_RATE 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD_RATE) - 1)

// Encoder and Motor Parameters
#define ENCODER_PPR 24 // Pulses (periods) per revolution from encoder spec
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR * 4) // 4 states per period (quadrature)

// Control Loop Timing
#define CONTROL_LOOP_FREQ 100
#define CONTROL_LOOP_PERIOD (1.0 / CONTROL_LOOP_FREQ)

// PI Controller Gains
#define KP 0.8f
#define KI 1.5f

// PI Controller Limits
#define PWM_MAX 249.0f // 100% duty cycle
#define INTEGRAL_MAX (PWM_MAX / KI)
#define INTEGRAL_MIN 0.0f

// --- Global Volatile Variables ---

// Speed Measurement
volatile int16_t encoder_count = 0; 
volatile float current_speed_rpm = 0.0f;

// Speed Setpoint
volatile float reference_speed_rpm = 0.0f;
volatile float fine_tune_rpm = 0.0f;

// PI Controller State
volatile float integral_term = 0.0f;

// --- Functions ---
void init_gpio(void);
void init_pwm(void);
void init_encoder_interrupts(void);
void init_control_timer(void);
void init_usart(void);
void init_adc(void);
void usart_transmit_string(const char* str);

// --- Main Program ---
int main(void) {
    // Initialize all hardware peripherals
    init_gpio();
    init_pwm();
    init_encoder_interrupts();
    init_adc();
    init_usart();
    init_control_timer(); // trigger the control loop

    // Power LED
    PORTD |= (1 << PD7);

    // Enable global interrupts
    sei();

    usart_transmit_string("\r\n--- AVR Motor PI Controller Initialized ---\r\n");
    usart_transmit_string("Enter target RPM and press Enter.\r\n");

    
    while (1) {
        // Blink LED
        PORTD ^= (1 << PD6);
        _delay_ms(500);
    }
}

/**
 * @brief Configures GPIO pins for their intended functions (input/output).
 */
void init_gpio(void) {
    // Port B: PWM (Output)
    DDRB |= (1 << DDB1);

    // Port C: Encoder and ADC (Inputs)
    DDRC &= ~((1 << DDC5) | (1 << DDC4) | (1 << DDC3));
    PORTC |= (1 << PORTC5) | (1 << PORTC4);

    // Port D: USART and LEDs
    DDRD |= (1 << DDD7) | (1 << DDD6);
    PORTD &= ~((1 << PORTD7) | (1 << PORTD6)); // Start with LEDs off
}

/**
 * @brief Configures Timer/Counter1 for Fast PWM generation on PB1 (OC1A).
 */
void init_pwm(void) {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    // Set TOP value for 4kHz frequency
    ICR1 = 249;

    // Start with 0% duty cycle
    OCR1A = 0;
}

/**
 * @brief Configures Pin Change Interrupts for the encoder on PC4 and PC5.
 */
void init_encoder_interrupts(void) {
    // Enable Pin Change Interrupt
    PCICR |= (1 << PCIE2);

    PCMSK2 |= (1 << PCINT21) | (1 << PCINT20);
}

/**
 * @brief Configures Timer/Counter2 to trigger the control loop ISR at a fixed rate.
 */
void init_control_timer(void) {
    // Set Timer2 to CTC mode
    TCCR2A = (1 << WGM21);

    // Set prescaler to 1024
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);

    // Set compare value for ~100Hz frequency
    OCR2A = 77;

    TIMSK2 |= (1 << OCIE2A);
}

/**
 * @brief Configures the USART for serial communication.
 */
void init_usart(void) {
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE);

    // Enable receiver, transmitter, and receive-complete interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/**
 * @brief Configures the ADC to read the analog value from PC3.
 */
void init_adc(void) {
    ADMUX = (1 << REFS0) | (1 << MUX1) | (1 << MUX0);

    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}


// --- Interrupt Service Routines ---

/**
 * @brief ISR for Pin Change on Port C (Encoder).
 */
ISR(PCINT2_vect) {
    encoder_count++;
}

/**
 * @brief ISR for Timer2 Compare Match - The Control Loop.
 */
ISR(TIMER2_COMPA_vect) { 
    // Atomically read and reset encoder_count
    cli();
    int16_t counts = encoder_count;
    encoder_count = 0;
    sei();

    // Calculate speed in RPM
    current_speed_rpm = ((float)counts / ENCODER_COUNTS_PER_REV) * CONTROL_LOOP_FREQ * 60.0f;

    ADCSRA |= (1 << ADSC); // ADC conversion
    while (ADCSRA & (1 << ADSC));
    uint16_t adc_value = ADC;

    // Map ADC value (0-1023) to a speed adjustment
    fine_tune_rpm = ((float)adc_value - 512.0f) / 512.0f * 5.0f; // Range: [-5, 5]

    float adjusted_ref_speed = reference_speed_rpm + fine_tune_rpm;
    if (adjusted_ref_speed < 0) adjusted_ref_speed = 0;

    // Calculate error
    float error = adjusted_ref_speed - current_speed_rpm;

    // Proportional Term
    float p_term = KP * error;

    // Integral Term with Anti-Windup
    integral_term += KI * error * CONTROL_LOOP_PERIOD;
    if (integral_term > INTEGRAL_MAX) {
        integral_term = INTEGRAL_MAX;
    } else if (integral_term < INTEGRAL_MIN) {
        integral_term = INTEGRAL_MIN;
    }

    // PI Controller Output
    float pi_output = p_term + integral_term;

    // Update PWM Duty Cycle
    if (pi_output > PWM_MAX) {
        pi_output = PWM_MAX;
    } else if (pi_output < 0) {
        pi_output = 0;
    }

    // Set the PWM duty cycle
    OCR1A = (uint16_t)pi_output;
}

/**
 * @brief ISR for USART Receive Complete.
 */
ISR(USART_RX_vect) {
    static char buffer[10];
    static uint8_t index = 0;
    char received_char = UDR0;

    // Echo character back
    // UDR0 = received_char;

    if ((received_char >= '0' && received_char <= '9') && (index < sizeof(buffer) - 1)) {
        buffer[index++] = received_char;
    } else {
        if (index > 0) {
            buffer[index] = '\0'; // Null-terminate the string
            reference_speed_rpm = atof(buffer); // Convert string to float
            
            // Send confirmation
            char confirm_msg[50];
            snprintf(confirm_msg, sizeof(confirm_msg), "\r\nSet Speed: %.2f RPM\r\n", reference_speed_rpm);
            usart_transmit_string(confirm_msg);
        }
        index = 0; // Reset for next command
    }
}

/**
 * @brief Transmits a null-terminated string over USART.
 * @param str The string to transmit.
 */
void usart_transmit_string(const char* str) {
    while (*str) {
        while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
        UDR0 = *str++; // Put data into buffer, sends the data
    }
}
