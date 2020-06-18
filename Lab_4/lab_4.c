/********************************************/
// Basic Alark clock with LCD 7-SEG display
// Push buttons for changing modes (set alarm, time, reset alarm, snooze etc.)
// 
// Two encoder dials for changing the time while in set time mode or alarm in set alarm mode. 
// (one by increments of 1 , the other 100)
// 
// Ran on ATMEGA128 with peripheral break out board.
// 7SEG LCD used to display time
// Using 32kHz oscillator for timing
// External speaker attached to PORTC for generating alarm tone
/********************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "hd44780.h" //LCD write library provided

#define CW 1 //Encoder rotation direction definitions
#define CCW 0
#define CWB 3
#define CCWB 2

volatile uint16_t cycles;
volatile uint8_t  mode = 0;
volatile int16_t  time = 0;

volatile uint8_t  clearBar = 0; 

volatile uint8_t  alarmtone = 0; 
volatile uint8_t  snoozed1 = 0; //snooze flag
volatile uint8_t  snoozed10 = 0; // snooze flag

volatile uint8_t  clear = 0;
volatile int16_t  alarm = 0; //store alarm value
volatile uint8_t  alarmset = 3;
volatile uint8_t  alarmcheck = 0; //alarm flag

char lcdwrite[33]    = "                                \0"; //Buffer Templates for LCD write
char alarmstring[33] = "ALARM SET                       ";
char alarmclear[33]  = "                                ";
uint8_t blink = 0;//flag to prevent blinking during setup, part of class spec
uint8_t segment_data[5];//segment buffer
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF}; //easy access for correct encodings for 7-seg

/***********************************************************************/
//                            spi_init
//Initalizes the SPI port on the mega128. Does not do any further
//external device specific initalizations.  Sets up SPI to be:
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void)
{
    DDRB   =   0xF7;                    //Turn on SS, MOSI, SCLK
    DDRE  |=   0xFF;
    DDRC   =   0xFF;
    PORTE  =   0x00;
    DDRD  |=   (1 << PD2);
    DDRA   =   0xFF;
    SPCR  |=   (1 << SPE) | (1 << MSTR);//set up SPI mode
  //  SPSR  |=   (1 << SPI2X);            // double speed operation
}//spi_init

/***********************************************************************/
//                              tcnt0_init
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void)
{
    TIMSK  |=  ((1 << TOIE0)|(1 << TOIE2)|(1 << OCIE1A)); //enable timer/counter0 overflow interrupt
    TCCR0  |=  (1<<CS00);  //normal mode, no prescale
    ASSR   |=  (1 << AS0);
    TCCR2  |=  ((1 << CS21)|(1 << CS20)|(1 << WGM21)|(1 << WGM20)|(1 << COM21)|(1  << COM20));//64 prescale
    ADMUX  |=  ((1 << REFS0)|(1 << ADLAR)|(1 << MUX2)|(1 << MUX1)|(1 << MUX0));//5v reference, Left Adjusted, Pin 7 of Port F
    ADCSRA |=  ((1 << ADEN));//Enable ADC
    TCCR1B |=  ((1 << 5)|(1 << WGM12)|(1 << CS12)|(1 << CS10));//1024 Prescale and PWM
    TCCR3A |=  ((1 << COM3A1)|(1 << COM3A0)|(1 << WGM30));
    TCCR3B |=  ((1 << WGM32)|(1 << CS30));
    OCR1A   =  50;
    OCR3A   =  100;
}

uint8_t chk_buttons(uint8_t button)//Button press check, no debouncing needed in this program
{
    static uint16_t state[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000;
    if(state[button] == 0xF000){return 1;}
    else{return 0;}
}

void segsum(uint16_t sum)//Converting timer count to human readable format
{
    segment_data[0] = dec_to_7seg[((sum % 100) % 10)];//index the preformated buffer based on the value at each place (1's ,10's etc.)
    segment_data[1] = dec_to_7seg[((sum % 100)/ 10)];
   if(mode == 2){segment_data[2] = 0xF8;}
    segment_data[3] = dec_to_7seg[((sum % 1000) / 100)];
    segment_data[4] = dec_to_7seg[(sum / 1000)];
}

ISR(TIMER2_OVF_vect)
{
    uint8_t progmode = 0;
    uint8_t direction = 7;
    ADCSRA |= (1 << ADSC);
    PORTB |= 0x70;
    DDRA = 0x00;//make PORTA an input port with pullups
    PORTA = 0xFF;
    if(chk_buttons(0))//Set the time
    {
        if(mode == 2){}
        else
        {
            alarmcheck = 0;
            if(mode == 1)//Latching to make sure alarm does not go off when setting it
            {
                alarmcheck = 1;
                clearBar = 1;
            }
            mode ^= 1;
        }
    }

    if(chk_buttons(1))//Set alarm
    {
        if(mode == 1){}
        else
        {
            if(mode == 2)//Latching nonsense to make sure the alarm does not go off when setting it
            {
                alarmcheck = 1;
                alarmset = 1;
                clearBar = 1;
            }
            else{alarmcheck = 0;}
            mode ^= 2;
        }
    }
    if(chk_buttons(2))//Clear the alarm on pressing the clear alarm button
    {
        segment_data[2] = 0xFF;
        clear = 1;
    }
    if(chk_buttons(3))//Snooze! 10min
    {
        if(alarmcheck)
        {
            snoozed10 = 1;
        }
    }
    DDRA = 0xFF;//make PORTA output
    PORTB &= ~(0x70);

    progmode = 0xAA;
    if(mode == 1)//Time set mode, ATrack encoder reading, dec/increments the time
    {
 
        PORTE &= ~(1 << PE6);
        PORTB |=  (1 << PB0);
        SPDR = progmode;
        while(bit_is_clear(SPSR,SPIF)){}
        PORTD |=  (1 << PD2);
        PORTD &= ~(1 << PD2);
        PORTB &= ~(1 << PB0);


        PORTE |=  (1 << PE6);
        static uint8_t a_past;
        static uint8_t a_past2;
        static uint8_t b_past;
        static uint8_t b_past2;

        uint8_t a_current  = SPDR & 0x01;
        uint8_t a_current2 = (SPDR >> 2) & 0x01;
        uint8_t b_current  = (SPDR >> 1) & 0x01;
        uint8_t b_current2 = (SPDR >> 3) & 0x01;
        if(a_past == a_current){
        if((a_current == 1) && (b_past < b_current)){direction = CW;}
        if((a_current == 1) && (b_past > b_current)){direction = CCW;}
        }
        if(a_past2 == a_current2){
        if((a_current2 == 1) && (b_past2 < b_current2)){direction = CWB;}
        if((a_current2 == 1) && (b_past2 > b_current2)){direction = CCWB;}
        }

        a_past = a_current;
        b_past = b_current;
        a_past2 = a_current2;
        b_past2 = b_current2;

        if(direction == CW){time += 1;}
        else if(direction == CCW){time -= 1;}
        else if(direction == CWB){time += 100;}
        else if(direction == CCWB){time -= 100;}
    }
    progmode = 0x55;
    if(mode == 2)//Alarm set mode, ATrack encoder reading, dec/increments the alarm
    {
      //  PORTB &= (1 << PB7);
        PORTE &= ~(1 << PE6);
        PORTB |=  (1 << PB0);
        SPDR = progmode;
        while(bit_is_clear(SPSR,SPIF)){} //Port writes for setup to read encoders
        PORTD |=  (1 << PD2);			
        PORTD &= ~(1 << PD2);
        PORTB &= ~(1 << PB0);
     //   PORTB |= (1 << PB7);

        PORTE |=  (1 << PE6);
        static uint8_t a_past;
        static uint8_t a_past2; //variables to track encoder states
        static uint8_t b_past;
        static uint8_t b_past2;

        uint8_t a_current  = SPDR & 0x01; //track current states
        uint8_t a_current2 = (SPDR >> 2) & 0x01; //write current states
        uint8_t b_current  = (SPDR >> 1) & 0x01;
        uint8_t b_current2 = (SPDR >> 3) & 0x01;
        if(a_past == a_current){
        if((a_current == 1) && (b_past < b_current)){direction = CW;}//comparisons to determine encoder direction 
        if((a_current == 1) && (b_past > b_current)){direction = CCW;}
        }
        if(a_past2 == a_current2){
        if((a_current2 == 1) && (b_past2 < b_current2)){direction = CWB;}
        if((a_current2 == 1) && (b_past2 > b_current2)){direction = CCWB;}
        }


        a_past = a_current;//Store states for next time
        b_past = b_current;
        a_past2 = a_current2;
        b_past2 = b_current2;

        if(direction == CW){alarm += 1;}//Increment/Decrement by 1 for dial A
        else if(direction == CCW){alarm -= 1;}
        else if(direction == CWB){alarm += 100;}//100 for dial B
        else if(direction == CCWB){alarm -= 100;}

    }
    if(clearBar == 1)
    {
        SPDR = 0x00;
        while(bit_is_clear(SPSR, SPIF)){}
        PORTD |=  (1 << PD2);
        PORTD &= ~(1 << PD2);
        clearBar = 0;
    }
    refresh_lcd(lcdwrite);
  if(alarmtone == 1){OCR2 = 255;}//No dimming during alarm tone
  else
  {
    while(bit_is_clear(ADCSRA, ADIF));//Wait for conversion to complete
    ADCSRA |= (1 << ADIF);//clear flag
    if(ADCH > 200){OCR2 = 255;}//Dimming and such
    else if(ADCH > 150){OCR2 = 150;}
    else if(ADCH > 130){OCR2 = 75;}
  }
}
ISR(TIMER0_OVF_vect)//Timer 0 interrupt, used for keeping time and alarm state
{
    cycles++;//Every interrupt increment these cycle counts to keep time
    blink++;
    if(cycles == 7680)//1 minute of time is 7680 cycles
    {
        time++;//increment time
        cycles = 0;
    }
    if(blink == 128)//Every second (128 cycles)
    {
        if(mode != 2){segment_data[2] ^= 0x3;}//XOR to blink colon
        blink = 0;//reset cycle
    }


    if(alarmcheck == 1)//If we have an alarm set
    {
        if(alarmset == 1)//and we haven't wrote the LCD already
        {
            set_cursor(1,7);//Write the LCD with a Yes to signify the alarm is indeed set
            sprintf(lcdwrite, alarmstring);
            alarmset = 0;//latch so we do not write Yes repeatedly
        }
    }

    if(alarmcheck)//Only check the alarm if an alarm is set
    {
            if(snoozed10)//SNooze for 10
            {
                if(time == alarm + 10)
                {
                    alarmtone = 1;//Signal that a tone should play
                    sprintf(lcdwrite, alarmclear);
                    alarmset = 1;//allows the alarm mode to be written on lcd again
                    segment_data[2] = 0xFF;//colon blanked
                    alarmcheck = 0;//do not check anymore
                    snoozed10 = 0;//not snoozed
                }
            }
            else if(time == alarm)
            {
                alarmtone = 1;//Same stuff as above, however no reset of the "Snooze" flag
                sprintf(lcdwrite, alarmclear);
                alarmset = 1;
                segment_data[2] = 0xFF;
                alarmcheck = 0;
            }
    }
    if(clear == 1)
    {
        snoozed10 = 0;//We are not snoozed if we have no alarm set
        sprintf(lcdwrite, alarmclear);
        alarmcheck = 0;//No alarm to check we cleared it
        alarmset = 1;//Allow the LCD to be checked again
        alarmtone = 0;//no tone!
        clear = 0;//we do not want to keep this latched
    }

}
ISR(TIMER1_COMPA_vect)//alarm interrupt
{

    if(alarmtone == 1)//only oscillate when we want a tone
    {
       PORTC ^= (1 << 4);//Toggle pin to oscillate
    }
    else
    {
      PORTC = 0x00;
    }
}
int main()
{
    spi_init();    //initalize SPI port
    lcd_init();//initalize LCD
    tcnt0_init();//Initalize all the timers and some other stuff
    uint8_t digit = 0;
    segment_data[2] = 0xFF;//clear colon
    while(1){
    sei();
        if(time >= 2400){time = 0;}//Block of conditionals to keep the time bounded in 24hr format
        if(time < 0){time = 2359; }
        uint8_t temp = (time % 100) / 10;
        if(temp == 6){time += 40;}
        if(temp > 6){time -= 40;}

        if(alarm >= 2400){alarm = 0;}//Block to keep alarm in 24hr format
        if(alarm < 0){alarm = 2359;}
        temp = (alarm % 100) / 10;
        if(temp == 6){alarm += 40;}
        if(temp > 6){alarm -= 40;}



        if(mode == 2){segsum(alarm);}//depending on the mode display time or alarm
        else {segsum(time);}//default to showing time
      
/* Test display
        while(digit < 5)//display on led
        {
            PORTA = segment_data[digit];
            PORTB = (0x10 * digit);
            digit++;
            _delay_ms(.08);
        }
        digit = 0;
*/    
    }
}//main
