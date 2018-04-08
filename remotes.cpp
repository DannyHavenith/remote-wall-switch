//
//  Copyright (C) 2015, 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include "timer.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include <avr_utilities/pin_definitions.hpp>
#include <avr_utilities/devices/uart.h>
#include <avr_utilities/esp-link/client.hpp>
#include <avr_utilities/esp-link/command.hpp>

#define MQTT_BASE_NAME "spider/"

namespace {
PIN_TYPE( B, 6) led;
PIN_TYPE( D, 3) transmit;
PIN_TYPE( B, 3) pir;



Timer::TimerWaitValue motionTimeout = Timer::always;
esp_link::client::uart_type uart(19200);
esp_link::client esp( uart);
}

IMPLEMENT_UART_INTERRUPT( uart);

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

    /// start symbol: how long to stay high before starting
    /// a pulse train. If us4_start_high is non-zero and
    /// us4_start_low is zero, the signal will go up and all
    /// subsequent pulses will be low-active
    /// If us4_start_low is non-zero and
    uint16_t us4_start_high;

    /// how long to stay low before starting a pulse train.
    uint16_t us4_start_low;

    /// describes a single symbol, usually a 1 or a 0
    typedef uint16_t Symbol[2];

    /// describes the symbols for a 0 bit and for a 1 bit
    Symbol alphabet[2];
};

// two brands of RF controlled switches that are used here.
constexpr int quigg = 0;
constexpr int impuls = 1;
constexpr int globaltronic = 2;

// describe the known protocols
const Encoding symbols[] = {
    // quigg
    { 17000,    20,     175, 0, { { 175, 350 }, { 350, 175 } } },

    // impuls
    { 1500,     25,     0,   0, { { 140, 49 }, { 42, 147 } } },

	// globaltronic
	{ 1,        24,     756, 1784, {{ 131, 250}, {250, 131}}}
};

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
 * This implementation can control as many switches
 * as are listed here.
 */
const Switch switches[] =
{
        { quigg,  		{ 	0b01000001000000001101,     // off quigg2
                    		0b11001001000000001101}     // on
        },
        { impuls, 		{ 	0b1011101010101110000000000,// off
                    		0b1110101010101110000000000}// on
        },
        { impuls, 		{ 	0b1011101011101010000000000,// off
                    		0b1110101011101010000000000}// on
        },
		{ globaltronic, { 	0b111011000111011101001111, // off 4
				 	 	  	0b111010011011010000011111} // on
		},
		{ globaltronic, { 	0b011101101000000110001111, // off 3
				 	 	  	0b011100011101100000001111} // on
		},
		{ globaltronic, { 	0b001111011100010111101111, // off 1
				 	 	  	0b001101010110110010011111} // on
		},
        { quigg,  		{	0b00000000000000001101,     // off quigg 1
                    		0b10001000000000001101}     // on
        },
        { quigg,  		{ 	0b11000011000000001101,     // off quigg 3
                    		0b01001011000000001101}     // on
        },
        { quigg,  		{ 	0b10000010000000001101,     // off quigg 4
        		    		0b00001010000000001101}     // on
        },
		{ globaltronic, { 	0b101010111001111010111111, // off 2
						  	0b101000000100000001011111} // on
		}
};

/**
 * Convenience function to get the size of an array without
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
    if (code.us4_start_high)
    {
        set( transmit);
        delay_4us( code.us4_start_high);
    }

    if (code.us4_start_low)
    {
    	clear( transmit);
    	delay_4us( code.us4_start_low);
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
void send_command( const Encoding &code, uint32_t value, uint8_t count = 12)
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
    if (consume(topic_ptr, topic_end, MQTT_BASE_NAME "switch/"))
    {
        // ...try to parse the switch number from the topic and the
        // on/off number from the message.
        uint8_t sw = parse_uint16( topic_ptr, topic_end);
        uint8_t onoff = parse_uint16(message.buffer, message.buffer + message.len);

        // ... and send the corresponding code.
        sendcode( sw, onoff);
        motionTimeout = Timer::After( 4 * Timer::ticksPerSecond);
    }
}

const char *tohex( uint16_t value)
{
	static char hex[5] = {};
	static constexpr char digits[] = {
			'0', '1', '2', '3',
			'4', '5', '6', '7',
			'8', '9', 'A', 'B',
			'C', 'D', 'E', 'F'
	};

	hex[3] = digits[ value % 16];
	value /= 16;
	hex[2] = digits[ value % 16];
	value /= 16;
	hex[1] = digits[ value % 16];
	value /= 16;
	hex[0] = digits[ value % 16];

	return hex;
}

void connected( const esp_link::packet *p, uint16_t size)
{
    using esp_link::mqtt::subscribe;
    using esp_link::mqtt::publish;
    static uint16_t reconnect_count = 0;
	//esp.send("connected\n");
    esp.execute( subscribe, MQTT_BASE_NAME "switch/+", 0);
    esp.execute( publish,   MQTT_BASE_NAME "connects", tohex( ++reconnect_count), 0, true);
}

}

int main(void)
{
    using esp_link::mqtt::setup;
    using esp_link::mqtt::publish;

    make_output( led|transmit);
    make_input( pir);
    set( pir); // pull-up


    // get startup logging of the uart out of the way.
    _delay_ms( 5000); // wait for an eternity.
    clear_uart();    // then clear everything received on uart.

    while (not esp.sync()) toggle( led);
    esp.execute( setup, &connected, nullptr, nullptr, &update);
    connected(nullptr, 0);

    bool previous_pir_value = false;
    for (;;)
    {
    	bool pir_value = read( pir);
    	if (pir_value != previous_pir_value)
    	{
    		if (Timer::HasPassedOnce( motionTimeout))
    		{
    			esp.execute( publish, MQTT_BASE_NAME "motion", pir_value?"1":"0", 0, false);
    		}
    		previous_pir_value = pir_value;
    	}

        esp.try_receive();
    }
}
