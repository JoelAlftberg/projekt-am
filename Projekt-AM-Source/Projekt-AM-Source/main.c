/*
 * Projekt-AM-Source.c
 *
 * Created: 2026-01-26 08:40:10
 * Author : joela
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 1000000UL
#define SI5351_ADDR 0x60
#define I2C_WRITE (SI5351_ADDR << 1)
#define I2C_READ SI5351_ADDR

volatile uint32_t freq = 0;

ISR(INT1_vect){
    
    uint8_t status_rotary_a = (PIND >> PIND3) & 1;
    uint8_t status_rotary_b = (PIND >> PIND7) & 1;
    
    // If Rotary B signal is lagging after Rotary A we turned clockwise)
    if (status_rotary_a != status_rotary_b){
        PORTD ^= (1 << PIND4);
        freq += 10;
    }
    else{
        PORTD ^= (1 << PIND5);
        if (freq >= 0)
            freq -= 10;
    }        

 
}

// Bit rate set to 50 kHz F_CPU/16+2*(TWBR*prescaler), 16 is the "cost" of an I2C transaction and 2 from the high and low pulse
void i2c_init(){
    
    TWSR = 0x00;
    TWBR |= (1 << TWBR1); 
    TWCR = (1 << TWEN);
    
}

void i2c_write(uint8_t reg, uint8_t data){
    
    // Send start signal
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    // Wait for ack/nack from bus (TWINT will be 1 when done)
    while (!(TWCR & (1 << TWINT)));
    
    // Send address to bus and wait for answer
    TWDR = I2C_WRITE;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    // Send reg to bus and wait for ack
    TWDR = reg;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    // Send data to bus and wait for ack
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    // Send stop signal
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    
}

int main(void)
{
    
    // Set PD4 and PD5 to output
    DDRD |= (1 << PORTD4) | (1 << PORTD5);
    
    // Set PD3 and PD7 to input and enable pull-up 
    DDRD &= ~((1 << PORTD3) | (1 << PORTD7));
    PORTD |= (1 << PORTD3) | (1 << PORTD7);
    
    // Enable interrupt on PD7 (INT1 and set it to rising edge)
    EIMSK |= (1 << INT1);
    EICRA |= (1 << ISC10);    
    sei();
    
    while (1) 
    {
        
    }
}

