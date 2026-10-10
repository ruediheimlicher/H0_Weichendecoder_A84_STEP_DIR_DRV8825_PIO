#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include <util/delay.h>

#include "defines.h"



/* ==========================================================
   ATtiny84 @ 16 MHz

   PA0 = STEP
   PA1 = DIR
   PA2 = ENABLE

   DRV8825 ENABLE aktiv LOW
   ========================================================== */


#define STEP_PIN    PA0
#define DIR_PIN     PA1
#define ENABLE_PIN  PA2
#define STEP_FWD  4
#define STEP_BCK  3

uint16_t loupcount0 = 0;
uint16_t loupcount1 = 0;
uint16_t loupcount2 = 0;
/* ==========================================================
   Zustände
   ========================================================== */

volatile uint16_t step_remaining = 0;

volatile uint8_t stepper_running = 0;
volatile uint8_t step_pin_high = 0;


/*
 * Aktuelle STEP-Periode in µs
 */
volatile uint16_t current_period_us;


/*
 * Profil
 */
uint16_t start_period_us = 1500;
uint16_t max_period_us   = 300;
uint16_t acceleration_us = 50;


/* ==========================================================
   Timerperiode setzen
   ==========================================================

   Timer:

       16 MHz / 8 = 2 MHz

       1 Tick = 0,5 µs

   Ein STEP-Zyklus besteht aus:

       LOW  -> HIGH
       HIGH -> LOW

   Deshalb erzeugt jeder Timer-Interrupt eine Flanke.

   Die hier gesetzte Periode ist die halbe STEP-Periode.
   ========================================================== */

static inline void timer1_set_half_period(uint16_t period_us)
{
    uint32_t ticks;

    ticks = (uint32_t)period_us * 2UL;

    if (ticks < 2)
        ticks = 2;

    if (ticks > 65535UL)
        ticks = 65535UL;

    OCR1A = (uint16_t)(ticks - 1);
}


/* ==========================================================
   Timer1
   ========================================================== */

void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0;

    /*
     * CTC
     */
    TCCR1B |= (1 << WGM12);

    /*
     * Prescaler 8
     */
    TCCR1B |= (1 << CS11);

    /*
     * Startwert
     */
    OCR1A = 3999;

    /*
     * Compare A Interrupt
     */
    TIMSK1 |= (1 << OCIE1A);
}


/* ==========================================================
   Stepper Init
   ========================================================== */

void stepper_init(void)
{
    /*
     * Ausgänge
     */
    DDRA |= (1 << STEP_PIN);
    DDRA |= (1 << DIR_PIN);
    DDRA |= (1 << ENABLE_PIN);

    /*
     * STEP LOW
     */
    PORTA &= ~(1 << STEP_PIN);

    /*
     * DIR LOW
     */
    PORTA &= ~(1 << DIR_PIN);

    /*
     * DRV8825 aus
     */
    PORTA |= (1 << ENABLE_PIN);

    timer1_init();

    sei();
}


/* ==========================================================
   Profil einstellen
   ==========================================================

   start_us:
       Startgeschwindigkeit

   max_us:
       Maximalgeschwindigkeit

   acceleration_us:
       Änderung der Periode pro Schritt

   Kleinere Periode = höhere Geschwindigkeit.
   ========================================================== */

void stepper_set_profile(uint16_t start_us,
                         uint16_t max_us,
                         uint16_t acceleration)
{
    if (start_us < max_us)
        start_us = max_us;

    if (acceleration == 0)
        acceleration = 1;

    start_period_us = start_us;
    max_period_us = max_us;
    acceleration_us = acceleration;
}


/* ==========================================================
   Geschwindigkeit während der Bewegung
   ========================================================== */

static void stepper_update_speed(void)
{
    /*
     * Beschleunigen:
     *
     * Periode verkleinern.
     */
    if (current_period_us > max_period_us)
    {
        if (current_period_us > max_period_us + acceleration_us)
        {
            current_period_us -= acceleration_us;
        }
        else
        {
            current_period_us = max_period_us;
        }

        return;
    }

    /*
     * Bremsen wird separat von der ISR entschieden.
     */
}


/* ==========================================================
   ENABLE
   ========================================================== */

static inline void stepper_enable(void)
{
    PORTA &= ~(1 << ENABLE_PIN);
}


static inline void stepper_disable(void)
{
    PORTA |= (1 << ENABLE_PIN);
}


/* ==========================================================
   Bewegung starten
   ========================================================== */

void stepper_move(uint16_t steps, uint8_t direction)
{
    if (steps == 0)
        return;

    /*
     * Während einer Bewegung nicht neu starten.
     */
    if (stepper_running)
        return;

    /*
     * Richtung setzen
     */
    if (direction)
        PORTA |= (1 << DIR_PIN);
    else
        PORTA &= ~(1 << DIR_PIN);

    /*
     * Schrittzahl
     */
    step_remaining = steps;

    /*
     * Startgeschwindigkeit
     */
    current_period_us = start_period_us;

    /*
     * STEP LOW
     */
    PORTA &= ~(1 << STEP_PIN);

    step_pin_high = 0;

    /*
     * Timer auf Startgeschwindigkeit setzen
     */
    timer1_set_half_period(current_period_us);

    /*
     * Motor aktivieren
     */
    stepper_enable();

    stepper_running = 1;
}


/* ==========================================================
   Ein einzelner Schritt
   ========================================================== */

void stepper_step(uint8_t direction)
{
    stepper_move(1, direction);
}


/* ==========================================================
   Status
   ========================================================== */

uint8_t stepper_busy(void)
{
    return stepper_running;
}


/* ==========================================================
   Timer1 ISR
   ========================================================== */

ISR(TIM1_COMPA_vect)
{
    /*
     * Keine Bewegung
     */
    if (!stepper_running)
    {
        PORTA &= ~(1 << STEP_PIN);
        return;
    }


    /*
     * ----------------------------------------------
     * LOW -> HIGH
     * ----------------------------------------------
     */

    if (!step_pin_high)
    {
        PORTA |= (1 << STEP_PIN);

        step_pin_high = 1;

        return;
    }


    /*
     * ----------------------------------------------
     * HIGH -> LOW
     * ----------------------------------------------
     */

    PORTA &= ~(1 << STEP_PIN);

    step_pin_high = 0;


    /*
     * Ein vollständiger STEP
     */
    if (step_remaining > 0)
        step_remaining--;


    /*
     * ----------------------------------------------
     * Fertig
     * ----------------------------------------------
     */

    if (step_remaining == 0)
    {
        stepper_running = 0;

        PORTA &= ~(1 << STEP_PIN);

        /*
         * Motor stromlos
         */
        stepper_disable();

        return;
    }


    /*
     * ----------------------------------------------
     * Beschleunigen / Bremsen
     * ----------------------------------------------
     *
     * Wir betrachten die verbleibenden Schritte.
     *
     * Wenn noch genug Schritte vorhanden sind:
     *       beschleunigen.
     *
     * Wenn nicht mehr genug Schritte für das
     * Abbremsen vorhanden sind:
     *       abbremsen.
     */


    /*
     * Anzahl Schritte, die ungefähr zum
     * Abbremsen benötigt werden.
     */
    uint16_t brake_steps;

    if (acceleration_us != 0)
    {
        /*
         * Grobe Näherung:
         *
         * Unterschied zwischen aktueller
         * und Startperiode / Beschleunigung.
         */
        if (current_period_us < start_period_us)
        {
            brake_steps =
                (start_period_us - current_period_us)
                / acceleration_us;
        }
        else
        {
            brake_steps = 0;
        }
    }
    else
    {
        brake_steps = 0;
    }


    /*
     * ----------------------------------------------
     * Bremsphase
     * ----------------------------------------------
     */

    if (step_remaining <= brake_steps + 1)
    {
        /*
         * Periode erhöhen
         * = Geschwindigkeit reduzieren
         */

        if (current_period_us < start_period_us)
        {
            if (current_period_us >
                start_period_us - acceleration_us)
            {
                current_period_us = start_period_us;
            }
            else
            {
                current_period_us += acceleration_us;
            }
        }
    }

    /*
     * ----------------------------------------------
     * Beschleunigungsphase
     * ----------------------------------------------
     */

    else
    {
        if (current_period_us > max_period_us)
        {
            if (current_period_us >
                max_period_us + acceleration_us)
            {
                current_period_us -= acceleration_us;
            }
            else
            {
                current_period_us = max_period_us;
            }
        }
    }


    /*
     * Neue Timerperiode
     */
    timer1_set_half_period(current_period_us);
}


/* ==========================================================
   MAIN
   ========================================================== */

int main(void)
{
    stepper_init();

    DDRA &= ~(1<<STEP_FWD);
    PORTA |= (1<<STEP_FWD);

    DDRA &= ~(1<<STEP_BCK);
    PORTA |= (1<<STEP_BCK);
    LOOPLED_DDR |= (1<<LOOPLED);

    /*
     * ----------------------------------------------
     * Bewegungsprofil
     * ----------------------------------------------
     *
     * Start:
     *     500 Schritte/s
     *
     * Maximum:
     *     2000 Schritte/s
     *
     * Beschleunigung:
     *     20 µs Periodenänderung pro Schritt
     */

    stepper_set_profile(
        2500,
        300,
        50
    );


    /*
     * 1000 Schritte
     */
    //stepper_move(100, 1);
    //_delay_ms(1000);
    bool dir = false;
    while (1)
    {
        loupcount0++;
        if((loupcount0>0xFFFA) && (stepper_busy() == false))
        {
            loupcount0 = 0;
            loupcount1++;
            LOOPLED_PORT ^= (1<<LOOPLED);
            if(loupcount1 > 0xF)
            {
                loupcount1 = 0;
                stepper_move(1200, dir);
                if(dir == true)
                {
                    dir = false;
                }
                else
                {
                    dir = true;
                }
            }

        }
        {
            /*
                stepper_move(400, 0);
                _delay_ms(1500);
                stepper_move(400, 1);
                _delay_ms(1500);
            */

        }

        /*
        if((PINA & (1<<STEP_FWD)) == 0)
        {
            LOOPLED_PORT &= ~(1<<LOOPLED);
             _delay_ms(500);
            LOOPLED_PORT |= (1<<LOOPLED);

            stepper_move(100, 1);
            //_delay_ms(500);
        }

        if((PINA & (1<<STEP_BCK)) == 0)
        {
            stepper_move(100, 0);
            _delay_ms(500);
        }
*/

        if (!stepper_busy())
        {
            
        }
    }
}