#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

// SSD1306 OLED display settings
#define SSD1306_ADDRESS 0x3C
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_COMMAND 0x00
#define SSD1306_DATA 0x40

// Thresholds for sync status determination
#define SYNC_THRESHOLD 20       // Phase drift speed within +/- this value is considered "in sync"
#define FAST_THRESHOLD 90      // Phase drift speed beyond this is "too fast"
#define SLOW_THRESHOLD -90     // Phase drift speed below this is "too slow"
#define NO_SIGNAL_TIMEOUT 3000 // 3 seconds timeout to determine "no signal"

// Sync status enum for better code readability
typedef enum {
    STATUS_NO_SIGNAL,
    STATUS_TOO_SLOW,
    STATUS_SLOW,
    STATUS_IN_SYNC,
    STATUS_FAST,
    STATUS_TOO_FAST
} SyncStatus;

// Global volatile variables for timing
volatile uint16_t refTime = 0;
volatile int16_t phaseDiff = 0;
volatile uint8_t newMeasurement = 0;
volatile uint16_t syncTime = 0;
volatile uint16_t lastPhaseDiff = 0;
volatile int16_t phaseDriftSpeed = 0;
volatile uint32_t lastMeasurementTime = 0;
volatile uint8_t refSignalDetected = 0;
volatile uint8_t syncSignalDetected = 0;

// Function prototypes
void twi_init(void);
void twi_start(void);
void twi_stop(void);
void twi_write(uint8_t data);
void ssd1306_command(uint8_t cmd);
void ssd1306_data(uint8_t data);
void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_set_cursor(uint8_t row, uint8_t col);
void ssd1306_string(const char *str);
void ssd1306_draw_animation(SyncStatus status, int16_t drift_speed);
void uart_init(unsigned int baud);
void uart_transmit(char data);
void uart_print(const char *str);
void uart_print_int16(int16_t num);
SyncStatus get_sync_status(int16_t drift_speed);

// I2C (TWI) functions for OLED communication
void twi_init(void) {
    // Set SCL frequency to 100kHz with 8MHz system clock
    TWSR = 0x00; // Prescaler value = 1
    TWBR = 32;   // Bit rate register = 32
    TWCR = (1 << TWEN); // Enable TWI
}

void twi_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

// SSD1306 OLED functions
void ssd1306_command(uint8_t cmd) {
    twi_start();
    twi_write(SSD1306_ADDRESS << 1); // Write address
    twi_write(SSD1306_COMMAND);      // Command mode
    twi_write(cmd);
    twi_stop();
}

void ssd1306_data(uint8_t data) {
    twi_start();
    twi_write(SSD1306_ADDRESS << 1); // Write address
    twi_write(SSD1306_DATA);         // Data mode
    twi_write(data);
    twi_stop();
}

void ssd1306_init(void) {
    _delay_ms(100); // Wait for VDD stabilization

    ssd1306_command(0xAE); // Display off
    ssd1306_command(0xD5); // Set display clock divide ratio/oscillator frequency
    ssd1306_command(0x80); // Recommended value
    ssd1306_command(0xA8); // Set multiplex ratio
    ssd1306_command(0x3F); // 1/64 duty
    ssd1306_command(0xD3); // Set display offset
    ssd1306_command(0x00); // No offset
    ssd1306_command(0x40); // Set start line to 0
    ssd1306_command(0x8D); // Charge pump setting
    ssd1306_command(0x14); // Enable charge pump
    ssd1306_command(0x20); // Memory addressing mode
    ssd1306_command(0x00); // Horizontal addressing mode
    ssd1306_command(0xA1); // Set segment re-map (A0 = normal, A1 = flipped)
    ssd1306_command(0xC8); // Set COM output scan direction (C0 = normal, C8 = flipped)
    ssd1306_command(0xDA); // Set COM pins hardware configuration
    ssd1306_command(0x12); // Alternative COM pin configuration
    ssd1306_command(0x81); // Set contrast control
    ssd1306_command(0xCF); // High contrast
    ssd1306_command(0xD9); // Set pre-charge period
    ssd1306_command(0xF1); //
    ssd1306_command(0xDB); // Set VCOMH deselect level
    ssd1306_command(0x40); //
    ssd1306_command(0xA4); // Entire display ON (resume from RAM)
    ssd1306_command(0xA6); // Normal display (A7 = inverse)
    ssd1306_command(0xAF); // Display on

    // Clear the display
    ssd1306_clear();
}

void ssd1306_clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_command(0xB0 + page); // Set page address
        ssd1306_command(0x00);        // Set column address lower 4 bits
        ssd1306_command(0x10);        // Set column address upper 4 bits

        for (uint8_t col = 0; col < 128; col++) {
            ssd1306_data(0x00);       // Clear all pixels in this page
        }
    }
}

void ssd1306_set_cursor(uint8_t page, uint8_t col) {
    ssd1306_command(0xB0 + page);                 // Set page address
    ssd1306_command(0x00 + (col & 0x0F));         // Set column address lower 4 bits
    ssd1306_command(0x10 + ((col >> 4) & 0x0F));  // Set column address upper 4 bits
}

void ssd1306_string(const char *str) {
    // Simple 5x7 font for basic characters (ASCII 32-127)
static const uint8_t font[][5] PROGMEM = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // "\"
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x10, 0x08, 0x08, 0x10, 0x08}, // ~
    {0x78, 0x46, 0x41, 0x46, 0x78}  // DEL
    };

    while (*str) {
        char c = *str++;
        if (c >= 32 && c <= 127) {
            // Draw character from font data
            for (uint8_t i = 0; i < 5; i++) {
                ssd1306_data(pgm_read_byte(&font[c - 32][i]));
            }
            ssd1306_data(0x00); // Add space between characters
        }
    }
}

// Draw sync status animation on OLED
void ssd1306_draw_animation(SyncStatus status, int16_t drift_speed) {
    ssd1306_clear();

    // Draw title
    ssd1306_set_cursor(0, 0);

    // Draw status text
    ssd1306_set_cursor(0, 0);
    switch(status) {
        case STATUS_NO_SIGNAL:
            // Draw "NO SIGNAL" text
            ssd1306_string("NO SIGNAL DETECTED");

            // Draw crossed-out signal icon
            ssd1306_set_cursor(3, 32);
            for (int i = 0; i < 64; i++) {
                ssd1306_data(0xAA); // Crossed pattern
            }
            break;

        case STATUS_TOO_SLOW:
            ssd1306_string("TOO SLOW");

            // Draw leftward moving arrows
            for (uint8_t page = 2; page < 6; page++) {
                ssd1306_set_cursor(page, 0);
                for (uint8_t i = 0; i < 16; i++) {
                    // Draw leftward arrow pattern
                    ssd1306_data(0x00);
                    ssd1306_data(0x04);
                    ssd1306_data(0x08);
                    ssd1306_data(0x10);
                    ssd1306_data(0x3F); // Arrow shaft
                    ssd1306_data(0x10);
                    ssd1306_data(0x08);
                    ssd1306_data(0x04);
                }
            }
            break;

        case STATUS_SLOW:
            ssd1306_string("SLOW");

            // Draw slow leftward moving dot pattern
            for (uint8_t page = 2; page < 6; page++) {
                ssd1306_set_cursor(page, 0);
                for (uint8_t i = 0; i < 16; i++) {
                    if (i % 4 == 0) {
                        ssd1306_data(0x3C); // Dot
                    } else {
                        ssd1306_data(0x00); // Space
                    }
                }
            }
            break;

        case STATUS_IN_SYNC:
            ssd1306_string("IN SYNC");

            // Draw equals sign or lock symbol
            ssd1306_set_cursor(3, 32);
            for (uint8_t i = 0; i < 64; i++) {
                // Create lock or equals symbol pattern
                if (i % 8 < 4) {
                    ssd1306_data(0xFF); // Filled
                } else {
                    ssd1306_data(0x00); // Empty
                }
            }
            break;

        case STATUS_FAST:
            ssd1306_string("FAST");

            // Draw fast rightward moving dot pattern
            for (uint8_t page = 2; page < 6; page++) {
                ssd1306_set_cursor(page, 0);
                for (uint8_t i = 0; i < 16; i++) {
                    if (i % 3 == 0) {
                        ssd1306_data(0x3C); // Dot
                    } else {
                        ssd1306_data(0x00); // Space
                    }
                }
            }
            break;

        case STATUS_TOO_FAST:
            ssd1306_string("TOO FAST");

            // Draw rightward moving arrows
            for (uint8_t page = 2; page < 6; page++) {
                ssd1306_set_cursor(page, 0);
                for (uint8_t i = 0; i < 16; i++) {
                    // Draw rightward arrow pattern
                    ssd1306_data(0x00);
                    ssd1306_data(0x20);
                    ssd1306_data(0x10);
                    ssd1306_data(0x08);
                    ssd1306_data(0x3F); // Arrow shaft
                    ssd1306_data(0x08);
                    ssd1306_data(0x10);
                    ssd1306_data(0x20);
                }
            }
            break;
    }

    // Draw drift speed value
    char buf[16];
    sprintf(buf, "Drift: %d", drift_speed);

    ssd1306_set_cursor(7, 0);
    ssd1306_string(buf);
}

// Simple UART routines for output (using USART0)
void uart_init(unsigned int baud) {
    uint16_t ubrr = F_CPU/16/baud - 1;  // This calculation now uses 8 MHz
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);  // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data format
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char *str) {
    while(*str) {
        uart_transmit(*str++);
    }
}

void uart_print_int16(int16_t num) {
    char buf[8];
    sprintf(buf, "%d", num);
    uart_print(buf);
}

// Get synchronization status based on phase drift speed
// Now includes NO_SIGNAL detection
SyncStatus get_sync_status(int16_t drift_speed) {
    uint32_t current_time = TCNT0; // Using Timer0 as a time counter

    // Check if signals are present
    if (!refSignalDetected || !syncSignalDetected ||
        (current_time - lastMeasurementTime) > NO_SIGNAL_TIMEOUT) {
        return STATUS_NO_SIGNAL;
    }

    // Return status based on drift speed
    if (abs(drift_speed) <= SYNC_THRESHOLD) {
        return STATUS_IN_SYNC;
    } else if (drift_speed > FAST_THRESHOLD) {
        return STATUS_TOO_FAST;
    } else if (drift_speed > SYNC_THRESHOLD) {
        return STATUS_FAST;
    } else if (drift_speed < SLOW_THRESHOLD) {
        return STATUS_TOO_SLOW;
    } else {
        return STATUS_SLOW;
    }
}

// INT0 ISR: Triggered by rising edge on the reference signal (PD2)
ISR(INT0_vect) {
    refTime = TCNT1;
    refSignalDetected = 1;
}

// INT1 ISR: Triggered by rising edge on the sync signal (PD3)
ISR(INT1_vect) {
    syncTime = TCNT1;
    syncSignalDetected = 1;

    // Compute difference accounting for potential overflow
    if(syncTime >= refTime)
      phaseDiff = syncTime - refTime;
    else
        phaseDiff = (0xFFFF - refTime) + syncTime;

    phaseDriftSpeed = phaseDiff - lastPhaseDiff;
    lastPhaseDiff = phaseDiff;

    // Update last measurement time for no-signal detection
    lastMeasurementTime = TCNT0;

    newMeasurement = 1;
}

// Timer0 Overflow ISR for tracking time
ISR(TIMER0_OVF_vect) {
    static uint16_t counter = 0;

    counter++;
    if (counter >= 1000) { // Check approximately every second
        counter = 0;

        // If no measurement received for a while, consider signals lost
        if ((TCNT0 - lastMeasurementTime) > NO_SIGNAL_TIMEOUT) {
            refSignalDetected = 0;
            syncSignalDetected = 0;
        }
    }
}

int main(void) {
    // Set PD2 (INT0) and PD3 (INT1) as input; enable internal pull-ups if needed
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3);

    // Configure external interrupts for rising edges on INT0 and INT1
    EICRA |= (1 << ISC01) | (1 << ISC00) | (1 << ISC11) | (1 << ISC10);
    EIMSK |= (1 << INT0) | (1 << INT1);  // Enable INT0 and INT1

    // Set up Timer1 in normal mode with prescaler = 8 (each tick = 1 µs)
    TCCR1A = 0;
    TCCR1B |= (1 << CS11); // Use prescaler 8

    // Set up Timer0 for timeout tracking
    TCCR0A = 0;
    TCCR0B |= (1 << CS02) | (1 << CS00); // Prescaler 1024
    TIMSK0 |= (1 << TOIE0); // Enable overflow interrupt

    // Initialize UART for serial output at 9600 baud
    uart_init(9600);

    // Initialize I2C interface
    twi_init();

    // Initialize SSD1306 OLED display
    ssd1306_init();

    // Display startup message
    ssd1306_set_cursor(0, 0);
    ssd1306_string("Synchroscope");
    ssd1306_set_cursor(1, 0);
    ssd1306_string("Starting...");
    _delay_ms(1000);

    uart_print("Synchroscope with OLED started\r\n");
    uart_print("-----------------------------\r\n");

    sei(); // Enable global interrupts

    SyncStatus current_status = STATUS_NO_SIGNAL;
    int16_t current_drift = 0;

    while (1) {
        if (newMeasurement) {
            newMeasurement = 0;

            // Convert timer ticks to microseconds (each tick = 0.5µs)
            int16_t phase_us = phaseDiff/2;
            int16_t drift_speed_us = phaseDriftSpeed/2;

            // Get sync status
            current_status = get_sync_status(drift_speed_us);
            current_drift = drift_speed_us;

            // Print drift speed and status
            uart_print("Phase Drift: ");
            uart_print_int16(drift_speed_us);
            uart_print(" us | Status: ");

            switch(current_status) {
                case STATUS_NO_SIGNAL:
                    uart_print("NO SIGNAL");
                    break;
                case STATUS_TOO_SLOW:
                    uart_print("TOO SLOW");
                    break;
                case STATUS_SLOW:
                    uart_print("SLOW");
                    break;
                case STATUS_IN_SYNC:
                    uart_print("IN SYNC");
                    break;
                case STATUS_FAST:
                    uart_print("FAST");
                    break;
                case STATUS_TOO_FAST:
                    uart_print("TOO FAST");
                    break;
            }

            uart_print("\r\n");
        }

        // Update OLED display with current status and drift
        static uint8_t animation_frame = 0;
        static uint8_t update_counter = 0;

        update_counter++;
        if (update_counter >= 10) { // Update display animations less frequently
            update_counter = 0;
            animation_frame++;

            // Draw the animation frame
            ssd1306_draw_animation(current_status, current_drift);
        }

        _delay_ms(10);
    }

    return 0;
}
