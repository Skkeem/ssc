#include "SSC.h"
#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define _usToTicks(_us)     ((clockCyclesPerMicrosecond() * _us) / 256)

/* Serial Source */
static int pre, post;
static volatile int ser_buf;

/* PWM Source */
static int from_l, from_h;
static int to_l, to_h;
static int pwm_pin;

/* Source Selection */
static volatile int selection = 0;
static int idx = 0;
static int src_type[2];

/* Control Type */
static int ctrl_type;

SSCClass::SSCClass()
{
}

void SSCClass::attachSourceSERIAL(unsigned int _baud, int _pre, int _post)
{
    unsigned long baud = (F_CPU / (16UL * _baud)) - 1;

    /* Set baud rate */
    UBRR0H = (unsigned char) (0xf & (baud >> 8));
    UBRR0L = (unsigned char) (0xff & baud);

    pre = _pre;
    post = _post;

    src_type[idx++] = SERIAL;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = 3 << UCSZ00;
}

ISR(USART_RX_vect)
{
    static int val, pos = 0;
    int temp;

    temp = UDR0;
//    digitalWrite(7, HIGH);///

    if (pos == 0 && temp == pre) {
        pos++;
        return;
    }
    else if (pos == 1) {
        pos++;
        val = temp;
        return;
    }
    else if (pos == 2 && temp == post) {
        pos = 0;
        ser_buf = val;
        UDR0 = ser_buf;////
        return;
    }
    else {
        pos = 0;
        return;
    }
}

void SSCClass::attachSourcePWM(int pin, int fl, int fh, int tl, int th)
{
    pwm_pin = pin;
    from_l = fl;
    from_h = fh;
    to_l = tl;
    to_h = th;

    src_type[idx++] = PWM;    
}

void SSCClass::attachSelect()
{
    EICRA = 3 << ISC00;
    EIMSK = 1 << INT0;
}

ISR (INT0_vect)
{
    delay(25);
    selection = !selection;
    digitalWrite(7, selection);
}

void SSCClass::attachControlPWM(unsigned int freq)
{
    OCR1A = freq;
    OCR1B = 0;

    ctrl_type = PWM;

    TCCR1A = (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM13) | (1 << CS12);
    DDRB |= 1 << DDB2;
}

int SSCClass::getSource(void)
{
    if (src_type[selection] == SERIAL)
        return ser_buf;
    else {
        //return (int) map(pulseIn(9, HIGH), 980, 1850, 0, 180);
        unsigned long pwm_buf = pulseIn(pwm_pin, HIGH);
        //return (int) map(pulseIn(pwm_pin, HIGH), from_l, from_h, to_l, to_h);
        return pwm_buf ? (int) map(pwm_buf, from_l, from_h, to_l, to_h) : 0;
    }
}

void SSCClass::putControl(int value)
{
    if (ctrl_type == PWM)
        OCR1B = value;
}

SSCClass SSC;
