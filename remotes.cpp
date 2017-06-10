//
//  Copyright (C) 2015, 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include <avr_utilities/pin_definitions.hpp>
#include <avr_utilities/devices/uart.h>
#include <avr_utilities/esp-link/client.hpp>
#include <avr_utilities/esp-link/command.hpp>

PIN_TYPE( B, 6) led;
PIN_TYPE( D, 3) transmit;
PIN_TYPE( B, 3) pir;

serial::uart<> uart(19200);
IMPLEMENT_UART_INTERRUPT( uart);
esp_link::client esp( uart);




/**
 * Structure that describes the protocol to use for a specific brand of
 * RF controlled switches.
 *
 * The encoding consists of an alphabet of symbols (currently always 2, but may become more).
 * Each symbol consists of a "logical high" level that is maintained for a specified
 * amount of time and a "logical low" level that also has a defined duration. Signals
 * may be inverted, in which case "logical high" actually means a low voltage (and a 0
 * value sent to the GPIO pins). Inverted signals typically remain in a low voltage during rests,
 * but assert the output pin just before starting to transmit.
 *
 * Each protocol also has its own number of symbols to transmit.
 *
 */
struct Encoding
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

// two brands of RF controlled switches that are used here.
constexpr int quigg = 0;
constexpr int impuls = 1;

// describe the known protocols
constexpr Encoding symbols[] = {
    // quigg
    { 17000,    20,     175,{ { 175, 350 }, { 350, 175 } } },

    // impuls
    { 1500,     25,     0,  { { 140, 49 }, { 42, 147 } } } };

/**
 * Description of a switch that consists of an encoding
 * (= protocol to be used) and a bit sequence for "off" and
 * one for "on" signals.
 */
struct Switch
{
    uint8_t encoding;
    uint32_t signals[2];
};

/**
 * This implementation can control three switches.
 */
const Switch switches[] =
{
        { quigg,  { 0b01000001000000001101,     // off
                    0b11001001000000001101}     // on
        },
        { impuls, { 0b1011101010101110000000000,// off
                    0b1110101010101110000000000}// on
        },
        { impuls, { 0b1011101011101010000000000,// off
                    0b1110101011101010000000000}// on
        }
};

/**
 * Convenience function to get the size of an arry without
 * the sizeof/sizeof hassle.
 */
template<typename E, size_t size>
constexpr size_t Size( const E (&)[size])
{
    return size;
}


namespace
{

/**
 * create a loop that runs for delay * 4 clock ticks.
 */
void delay_4us(uint16_t delay)
{
    delay *= 2;
    static_assert( F_CPU == 8000000, "This code assumes a 8 Mhz clock");
    asm volatile(
            "loop:          SBIW %[counter], 1 \n"
            "               RJMP .-0           \n"
            "               RJMP .-0           \n"
            "               RJMP .-0           \n"
            "               RJMP .-0           \n"
            "               RJMP .-0           \n"
            "               RJMP .-0           \n"
            "               BRNE loop           \n"
            :
            : [counter] "w" (delay)
    );
}

/**
 * Send a single symbol.
 */
void send_symbol(const Encoding::Symbol &symbol)
{
    toggle( transmit);
    delay_4us( symbol[0]);
    toggle( transmit);
    delay_4us( symbol[1]);
}

/**
 * Send a command (as encoded by value) using the Encoding described by code
 */
void send_command_once(const Encoding &code, uint32_t value)
{
    set( led);
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
    clear( led);
}

/**
 * Send a command several times.
 *
 * In practice most RF transmitters send the code several times to increase the
 * chances of command reception.
 */
void send_command( const Encoding &code, uint32_t value, uint8_t count = 8)
{
    for (; count; --count)
    {
        send_command_once(code, value);
        delay_4us( code.us4_between_repeats);
    }
}

/**
 * Given a switchs number and an on/of code (0 or 1),
 * emit the RF command for the given switch.
 *
 * This function will have no effect if switch_index is
 * outside the range of available switches or if onoff is more
 * than 1.
 */
void sendcode( uint8_t switch_index, uint8_t onoff)
{
    if (switch_index < Size( switches) and onoff < Size( switches[switch_index].signals))
    {
        send_command(
                symbols[ switches[switch_index].encoding],
                switches[switch_index].signals[onoff]);
    }
}

/**
 * Given a pointer to a buffer in "input" and the end pointer of the buffer
 * in "end", parse the string in the buffer into a uint16_t.
 *
 * Since the parameter "input" is given by reference, this function will
 * actually advance the first parameter to point just beyond the last recognized
 * numerical character.
 */
uint16_t parse_uint16( const char *(&input), const char *end)
{
    uint16_t value = 0;
    while ( input < end and *input and *input <= '9' and *input >= '0')
    {
        value = 10 * value + (*input - '0');
        ++input;
    }
    return value;
}

/**
 * Consume as many charactres from the character array pointed to by "input" as
 * match the expectation.
 *
 * This function will increase "input" and stop consuming characters at the
 * first character that doesn't match the expectation string or when the
 * expectation string is exhausted.
 */
bool consume( const char *(&input), const char *end, const char *expectation)
{
    while (*expectation and input < end and *input++ == *expectation++) /* continue */;
    return *expectation == 0;
}

/**
 * empty the uarts input buffer.
 */
void clear_uart()
{
    while (uart.data_available()) uart.get();
}

/**
 * This function is called when an update is received on the subscribed MQTT topic.
 */
void update( const esp_link::packet *p, uint16_t size)
{
    using namespace esp_link;
    packet_parser parser{ p};

    string_ref topic;
    string_ref message;
    parser.get( topic);
    parser.get( message);

    const char *topic_ptr = topic.buffer;
    const char *topic_end = topic_ptr + topic.len;

    // if the topic is indeed the expected one...
    if (consume(topic_ptr, topic_end, "spider/switch/"))
    {
        // ...try to parse the switch number from the topic and the
        // on/off number from the message.
        uint8_t sw = parse_uint16( topic_ptr, topic_end);
        uint8_t onoff = parse_uint16(message.buffer, message.buffer + message.len);

        // ... and send the corresponding code.
        sendcode( sw, onoff);
    }
}

}

int main(void)
{
    using esp_link::mqtt::setup;
    using esp_link::mqtt::subscribe;

    make_output( led|transmit);
    make_input( pir);
    set( pir); // pull-up


    // get startup logging of the uart out of the way.
    _delay_ms( 5000); // wait for an eternity.
    clear_uart();    // then clear everything received on uart.

    while (not esp.sync()) toggle( led);
    esp.execute( setup, nullptr, nullptr, nullptr, &update);
    esp.execute( subscribe, "spider/switch/+", 0);


    for (;;)
    {
        esp.try_receive();
    }
}
