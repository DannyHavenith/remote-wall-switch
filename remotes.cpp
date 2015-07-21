//
//  Copyright (C) 2015 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include <avr/interrupt.h>
#include <util/delay.h>

#include "avr_utilities/pin_definitions.hpp"

PIN_TYPE( B, 6)led;
PIN_TYPE( B, 7)transmit;

struct code
{
    /// how long to wait between sending the same signal again in units of
    /// 4 microseconds.
    uint16_t us4_between_repeats;

    /// how many bits in one transmission
    uint8_t bits;

    /// whether to invert the signal and if so: how long to wait after inversion
    /// in units of 4 microseconds
    uint16_t us4_delay_after_invert;

    /// describes a single symbol, usually a 1 or a 0
    typedef uint16_t Symbol[2];

    /// describes the symbols for a 0 bit and for a 1 bit
    Symbol alphabet[2];
};

constexpr code codes[] = {
    // quigg
    { 17000, 20, 175, { { 175, 350 }, { 350, 175 } } },

    // impuls
    { 1500, 25, 0, { { 140, 49 }, { 42, 147 } } } };

constexpr int quigg = 0;
constexpr int impuls = 1;

namespace
{

/**
 * create a loop that runs for delay * 4 clock ticks.
 */
void delay_4us(uint16_t delay)
{
    static_assert( F_CPU == 1000000, "This code assumes a 1 Mhz clock");
    asm volatile(
            "               SBIW %[counter], 1 \n"
            "               BRNE .-4           \n"
            :
            : [counter] "w" (delay)
    );
}

void send_symbol(const code::Symbol &symbol)
{
    toggle( transmit);
    delay_4us( symbol[0]);
    toggle( transmit);
    delay_4us( symbol[1]);
}

void send_code_once(const code &code, uint32_t value)
{
    if (code.us4_delay_after_invert)
    {
        set( transmit);
        delay_4us( code.us4_delay_after_invert);
    }

    uint8_t bitcounter = code.bits;
    while (bitcounter--)
    {
        send_symbol( code.alphabet[value & 0x01]);
        value >>= 1;
    }

    clear( transmit);
}

void send_code( const code &code, uint32_t value, uint8_t count = 4)
{
    for (; count; --count)
    {
        send_code_once(code, value);
        delay_4us( code.us4_between_repeats);
    }
}

void on2()
{
    send_code( codes[quigg], 0b11001001000000001101);
}

void off2()
{
    send_code( codes[quigg], 0b01000001000000001101);
}

void onA()
{
    send_code( codes[impuls], 0b1110101010101110000000000);
}

void offA()
{
    send_code( codes[impuls], 0b1011101010101110000000000);
}

void onC()
{
    send_code( codes[impuls], 0b1110101011101010000000000);
}

void offC()
{
    send_code( codes[impuls], 0b1011101011101010000000000);
}
}

int main(void)
{
    make_output( led);
    make_output( transmit);

    for (;;)
    {
        set( led);
        onC();
        _delay_ms( 300);
        onA();
        _delay_ms( 300);
        on2();
        _delay_ms( 1000);
        clear( led);
        offC();
        _delay_ms( 300);
        offA();
        _delay_ms( 300);
        off2();
        _delay_ms( 1000);
    }
}
