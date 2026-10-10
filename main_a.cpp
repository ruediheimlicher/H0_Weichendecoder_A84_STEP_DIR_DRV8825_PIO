#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

/*
 * ATtiny84 @ 16 MHz
 *
 * Timer1:
 * Prescaler 8
 * Timer-Takt = 2 MHz
 * 1 Tick = 0,5 us
 *
 * Interrupt alle 100 us:
 * OCR1A = 199
 *
 * Der Timer erzeugt damit die STEP-Zeit.
 */

#define STEP_PORT   PORTA
#define STEP_DDR    DDRA
#define STEP_PIN    PA0

#define DIR_PORT    PORTA
#define DIR_DDR     DDRA
#define DIR_PIN     PA1

#define EN_PORT     PORTA
#define EN_DDR      DDRA
#define EN_PIN      PA2


/* ----------------------------------------------------------
   Motorparameter
   ---------------------------------------------------------- */

/*
 * Schrittintervall:
 *
 * 1000 = 100 ms  -> 10 Schritte/s
 *  100 = 10 ms   -> 100 Schritte/s
 *   20 = 2 ms    -> 500 Schritte/s
 *   10 = 1 ms    -> 1000 Schritte/s
 *
 * Einheit: 100 us
 */

#define RAMP_START     100       // 10 ms
#define RAMP_END        100       // 10 ms
#define STEP_INTERVAL   10       // 1 ms


/* ----------------------------------------------------------
   Motorstatus
   ---------------------------------------------------------- */

volatile uint32_t steps_remaining = 0;

volatile uint16_t step_timer = 0;
volatile uint16_t current_interval = RAMP_START;

volatile uint16_t ramp_steps = 0;
volatile uint16_t ramp_down_steps = 0;

volatile uint32_t total_steps = 0;
volatile uint32_t steps_done = 0;

volatile uint8_t motor_running = 0;
volatile uint8_t step_high = 0;


/* ----------------------------------------------------------
   Timer1 Initialisierung
   ---------------------------------------------------------- */

void timer1_init(void)
{
    /*
     * Timer1 CTC
     *
     * F_CPU = 16 MHz
     * Prescaler = 8
     *
     * 16 MHz / 8 = 2 MHz
     *
     * 2000 kHz / 200 = 10 kHz
     * => 100 us Interrupt
     */

    TCCR1A = 0;

    TCCR1B = 0;

    TCCR1B |= (1 << WGM12);       // CTC

    TCCR1B |= (1 << CS11);        // Prescaler 8

    OCR1A = 199;

    TIMSK1 |= (1 << OCIE1A);      // Compare Interrupt
}


/* ----------------------------------------------------------
   GPIO Initialisierung
   ---------------------------------------------------------- */

void motor_init(void)
{
    STEP_DDR |= (1 << STEP_PIN);
    DIR_DDR  |= (1 << DIR_PIN);
    EN_DDR   |= (1 << EN_PIN);

    /* STEP LOW */
    STEP_PORT &= ~(1 << STEP_PIN);

    /* DIR LOW */
    DIR_PORT &= ~(1 << DIR_PIN);

    /*
     * DRV8825 deaktiviert
     * EN HIGH
     */
    EN_PORT |= (1 << EN_PIN);

    timer1_init();

    sei();
}


/* ----------------------------------------------------------
   Motor aktivieren
   ---------------------------------------------------------- */

static inline void motor_enable(void)
{
    /*
     * DRV8825 EN ist active LOW
     */
    EN_PORT &= ~(1 << EN_PIN);
}


/* ----------------------------------------------------------
   Motor stromlos
   ---------------------------------------------------------- */

static inline void motor_disable(void)
{
    /*
     * STEP sicher LOW
     */
    STEP_PORT &= ~(1 << STEP_PIN);

    /*
     * DRV8825 deaktivieren
     */
    EN_PORT |= (1 << EN_PIN);
}


/* ----------------------------------------------------------
   Richtung setzen
   ---------------------------------------------------------- */

static inline void motor_direction(uint8_t direction)
{
    if (direction)
        DIR_PORT |= (1 << DIR_PIN);
    else
        DIR_PORT &= ~(1 << DIR_PIN);
}


/* ----------------------------------------------------------
   Ein Schritt starten
   ---------------------------------------------------------- */

void motor_step(uint8_t direction)
{
    /*
     * Nur starten, wenn Motor frei ist.
     *
     * Die eigentliche STEP-Flanke erzeugt Timer1.
     */

    if (motor_running)
        return;

    motor_direction(direction);

    motor_enable();

    steps_remaining = 1;
    total_steps = 1;
    steps_done = 0;

    ramp_steps = 0;
    ramp_down_steps = 0;

    current_interval = RAMP_START;
    step_timer = current_interval;

    motor_running = 1;
}


/* ----------------------------------------------------------
   N Schritte starten
   ---------------------------------------------------------- */

void motor_move(uint32_t steps, uint8_t direction)
{
    if (steps == 0)
    {
        motor_disable();
        return;
    }

    if (motor_running)
        return;

    motor_direction(direction);

    motor_enable();

    steps_remaining = steps;
    total_steps = steps;
    steps_done = 0;

    /*
     * Anzahl Schritte für Beschleunigung
     *
     * Bei kurzen Bewegungen darf die Rampe
     * nicht größer als die halbe Bewegung werden.
     */

    ramp_steps = 20;

    if (steps < (uint32_t)(ramp_steps * 2))
        ramp_steps = steps / 2;

    ramp_down_steps = ramp_steps;

    current_interval = RAMP_START;
    step_timer = current_interval;

    motor_running = 1;
}


/* ----------------------------------------------------------
   Timer1 Interrupt
   ---------------------------------------------------------- */

ISR(TIM1_COMPA_vect)
{
    if (!motor_running)
        return;


    /*
     * ------------------------------------------------------
     * STEP HIGH
     * ------------------------------------------------------
     *
     * Wir erzeugen eine STEP-Flanke.
     *
     * Der DRV8825 benötigt nur eine kurze HIGH-Zeit.
     */

    if (!step_high)
    {
        /*
         * Zeit bis zum nächsten Schritt
         */
        if (step_timer > 0)
        {
            step_timer--;
            return;
        }

        /*
         * STEP HIGH
         */
        STEP_PORT |= (1 << STEP_PIN);

        step_high = 1;

        /*
         * STEP HIGH für 100 us
         */
        step_timer = 1;

        return;
    }


    /*
     * ------------------------------------------------------
     * STEP LOW
     * ------------------------------------------------------
     */

    STEP_PORT &= ~(1 << STEP_PIN);

    step_high = 0;

    /*
     * Ein Schritt ist jetzt abgeschlossen.
     */

    if (steps_remaining > 0)
        steps_remaining--;

    steps_done++;


    /*
     * ------------------------------------------------------
     * Bewegung fertig?
     * ------------------------------------------------------
     */

    if (steps_remaining == 0)
    {
        motor_running = 0;

        motor_disable();

        return;
    }


    /*
     * ------------------------------------------------------
     * Rampe
     * ------------------------------------------------------
     */

    /*
     * Beschleunigung:
     *
     * Intervall wird kleiner
     * => Schrittfrequenz wird größer
     */

    if (steps_done < ramp_steps)
    {
        if (current_interval > STEP_INTERVAL)
            current_interval--;
    }


    /*
     * Abbremsen:
     *
     * Sobald nur noch ramp_down_steps
     * übrig sind, Intervall wieder erhöhen.
     */

    if (steps_remaining <= ramp_down_steps)
    {
        if (current_interval < RAMP_END)
            current_interval++;
    }


    /*
     * Timer für nächsten Schritt
     */

    step_timer = current_interval;
}
int main(void)
{
    motor_init();

    motor_move(200, 1);
    _delay_ms(200);
    while (1)
    {
        motor_move(200, 0);
        _delay_ms(200);
        motor_move(200, 1);
        _delay_ms(200);
        
    }
}