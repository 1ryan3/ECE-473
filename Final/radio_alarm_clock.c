#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "si4734.h"
#include "uart_functions.h"

#define CW 1
#define CCW 0
#define CWB 3
#define CCWB 2
volatile uint16_t cycles;
volatile uint8_t  mode = 0;

volatile uint8_t  radio_on = 0;
volatile uint8_t  radio_off = 0;
volatile uint8_t  radio_pwr = 0;
volatile uint8_t  radio_tune = 0;
volatile uint8_t  radio_alarm = 0;

volatile uint8_t  freq_change = 0;
volatile uint8_t  freq_timeout = 0;
volatile uint8_t  disp_freq_timeout = 0;

volatile int16_t  time = 0;

volatile uint8_t  clearBar = 1;

volatile uint8_t  alarmtone = 0;
volatile uint8_t  snoozed1 = 0;
volatile uint8_t  snoozed10 = 0;

volatile uint8_t  clear = 0;
volatile int16_t  alarm = 0;
volatile uint8_t  alarmset = 3;
volatile uint8_t  alarmcheck = 0;
volatile uint16_t lm73_temp;
volatile int16_t volume = 200;


//                      012345678901234567890123456789012
char lcdwrite[33]    = "                Loc:    Ext:    \0";

                    //Alarm Set is in slots 0 - 8
                    //Local temp is in slots 21 22
                    //External temp is in slots 29 30

char ext_tempa;
char ext_tempb;
char exta;
char extb;

uint8_t blink = 0;
uint8_t segment_data[5];
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF};
uint8_t dec_to_7seg_freq[12] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10, 0x7F};

enum radio_band{FM, AM, SW};
volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;


uint16_t current_fm_freq;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

char uart1_tx_buf[40];
char uart1_rx_buf[40];

uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];

extern uint8_t STC_interrupt;

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
    PORTE |=   0x04;
    DDRD  |=   (1 << PD2);
    EICRB |=   ((1 << ISC71)|(1 << ISC71));
    EIMSK |=   (1 << INT7);
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
    OCR3A   =  220;
}

uint8_t chk_buttons(uint8_t button)
{
    static uint16_t state[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000;
    if(state[button] == 0xF000){return 1;}
    else{return 0;}
}

void segsum(uint16_t sum)
{
    if(freq_change == 1)//Frequency change needs a different display
    {
        segment_data[0] = dec_to_7seg[((sum % 100) / 10)];
        segment_data[1] = dec_to_7seg_freq[((sum % 1000)/ 100)];
        segment_data[2] = 0xFF;
        segment_data[3] = dec_to_7seg[((sum % 10000) / 1000)];
        segment_data[4] = dec_to_7seg[(sum / 10000)];
    }
    else
    {
        segment_data[0] = dec_to_7seg[((sum % 100) % 10)];
        segment_data[1] = dec_to_7seg[((sum % 100)/ 10)];
        if(mode == 2){segment_data[2] = 0xF8;}
        segment_data[3] = dec_to_7seg[((sum % 1000) / 100)];
        segment_data[4] = dec_to_7seg[(sum / 1000)];
    }
}

ISR(TIMER2_OVF_vect)
{
    uint8_t progmode = 0;
    uint8_t direction = 7;
    static uint8_t digit = 0;

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


        if(freq_change == 1){segsum(current_fm_freq);}//if encoders change then we display frequency
        else if(mode == 2){segsum(alarm);}//depending on the mode display time or alarm
        else {segsum(time);}


    ADCSRA |= (1 << ADSC);
    PORTB |= 0x70;
    DDRA = 0x00;//make PORTA an input port with pullups
    PORTA = 0xFF;

    if(chk_buttons(0))//set time
    {
        if(mode == 1)
        {
            clearBar = 1;
            mode = 0;
        }
        else if(mode == 0){mode = 1;}
    }
    if(chk_buttons(1))//set alarm
    {
        if(mode == 2)
        {
            alarmcheck = 1;
            alarmset = 1;
            clearBar= 1;
            mode = 0;
        }
        else if(mode == 0){mode = 2;}
    }
    if(chk_buttons(2))
    {
        if(mode == 0)
        {
            clear = 1;
        }
        else{}
    }
    if(chk_buttons(3))
    {
        if(mode == 0)
        {
        }
        else{}
    }
    if(chk_buttons(4))
    {
        if(mode == 0)//set volume
        {
            mode = 3;
        }
        else if(mode == 3)
        {
            mode = 0;
            clearBar = 1;
        }
    }
    if(chk_buttons(5))//radio on/off
    {
        if(radio_on == 0)
        {
            radio_pwr = 1;
        }
        if(radio_on == 1)
        {
            radio_on = 0;
            radio_off = 1;
        }
    }
    if(chk_buttons(6))//change alarm to radio
    {
        if(alarmcheck == 1)
        {
            radio_alarm ^= 1;
        }
    }
     DDRA = 0xFF;//make PORTA output
    PORTB &= ~(0x70);
    if(mode == 0)//freq set mode, ATrack encoder reading, dec/increments the time
    {
    //    PORTB &= (1 << PB7);
        PORTE &= ~(1 << PE6);
        PORTB |=  (1 << PB0);
        SPDR = progmode;
        while(bit_is_clear(SPSR,SPIF)){}
        PORTD |=  (1 << PD2);
        PORTD &= ~(1 << PD2);
        PORTB &= ~(1 << PB0);
   //     PORTB |= (1 << PB7);

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

        if(direction == CW)//frequency change
        {
            current_fm_freq += 20;
            freq_change = 1;
        }
        else if(direction == CCW)
        {
            current_fm_freq -= 20;
            freq_change = 1;
        }
    }

    progmode = 0xAA;
    if(mode == 1)//Time set mode, ATrack encoder reading, dec/increments the time
    {
    //    PORTB &= (1 << PB7);
        PORTE &= ~(1 << PE6);
        PORTB |=  (1 << PB0);
        SPDR = progmode;
        while(bit_is_clear(SPSR,SPIF)){}
        PORTD |=  (1 << PD2);
        PORTD &= ~(1 << PD2);
        PORTB &= ~(1 << PB0);
   //     PORTB |= (1 << PB7);

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
        while(bit_is_clear(SPSR,SPIF)){}
        PORTD |=  (1 << PD2);
        PORTD &= ~(1 << PD2);
        PORTB &= ~(1 << PB0);
     //   PORTB |= (1 << PB7);

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

        if(direction == CW){alarm += 1;}
        else if(direction == CCW){alarm -= 1;}
        else if(direction == CWB){alarm += 100;}
        else if(direction == CCWB){alarm -= 100;}

    }
    progmode = 0x99;
    if(mode == 3)//Volume set mode, ATrack encoder reading, dec/increments the alarm
    {
      //  PORTB &= (1 << PB7);
        PORTE &= ~(1 << PE6);
        PORTB |=  (1 << PB0);
        SPDR = progmode;
        while(bit_is_clear(SPSR,SPIF)){}
        PORTD |=  (1 << PD2);
        PORTD &= ~(1 << PD2);
        PORTB &= ~(1 << PB0);
     //   PORTB |= (1 << PB7);

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

        if(direction == CW){volume -= 5;}
        else if(direction == CCW){volume += 5;}
        if(volume < 0){volume = 0;}
        OCR3A = volume;
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

    lm73_temp  = ((uint16_t)lm73_rd_buf[0] << 8);
    lm73_temp |= lm73_rd_buf[1];
    lm73_temp >>= 7;
    lcdwrite[21] = lm73_temp / 10 + 48;
    lcdwrite[22] = lm73_temp % 10 + 48;
    lcdwrite[29] = ext_tempa;
    lcdwrite[30] = ext_tempb;

  if(alarmtone == 1){OCR2 = 255;}//No dimming during alarm tone
  else
  {
    while(bit_is_clear(ADCSRA, ADIF));//Wait for conversion to complete
    ADCSRA |= (1 << ADIF);//clear flag
    if(ADCH > 200){OCR2 = 255;}//Dimming and such
    else if(ADCH > 150){OCR2 = 200;}
    else if(ADCH > 130){OCR2 = 145;}
    else{OCR2 = 90;}
  }

  PORTA = segment_data[digit];
  PORTB = (0x10 * digit);
  digit++;
  if(digit > 5){digit = 0;}

}
ISR(TIMER0_OVF_vect)
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
        if((mode != 2) && (freq_change != 1)){segment_data[2] ^= 0x3;}//XOR to blink colon
        blink = 0;//reset cycle
        twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);
    }
    if(freq_change == 1)
    {
        freq_timeout = 1;
        radio_tune = 1;
    }
    if(freq_timeout == 1)
    {
        disp_freq_timeout++;
        if(disp_freq_timeout == 128)
        {
            freq_timeout = 0;
            freq_change = 0;
        }
    }


    if((alarmcheck) && (mode == 0))//Only check the alarm if an alarm is set
    {

           if(radio_alarm == 1)
           {
               lcdwrite[0] = 'R';//radio alarm set
               lcdwrite[1] = 'a';
               lcdwrite[2] = 'd';
               lcdwrite[3] = 'i';
               lcdwrite[4] = 'o';
           }
           else
            {
               lcdwrite[0] = 'A'; //alarm set
               lcdwrite[1] = 'l';
               lcdwrite[2] = 'a';
               lcdwrite[3] = 'r';
               lcdwrite[4] = 'm';
            }

            if(snoozed10)//SNooze for 10
            {
                if(time == alarm + 10)
                {
                    if(radio_alarm == 1){radio_pwr = 1;}
                    else{alarmtone = 1;}//Signal that a tone should play
                    lcdwrite[0] = ' '; //alarm clear
                    lcdwrite[1] = ' ';
                    lcdwrite[2] = ' ';
                    lcdwrite[3] = ' ';
                    lcdwrite[4] = ' ';
                    segment_data[2] = 0xFF;//colon blanked
                    alarmcheck = 0;//do not check anymore
                    snoozed10 = 0;//not snoozed
                }
            }
            else if(time == alarm)
            {
                if(radio_alarm == 1){radio_pwr = 1;}
                else{alarmtone = 1;}//Signal that a tone should play
                lcdwrite[0] = ' '; //alarm clear
                lcdwrite[1] = ' ';
                lcdwrite[2] = ' ';
                lcdwrite[3] = ' ';
                lcdwrite[4] = ' ';

                segment_data[2] = 0xFF;
                alarmcheck = 0;
            }
    }

    if(clear == 1)
    {
        if(radio_alarm == 1)//clear radio alarm
        {
            radio_off = 1;
            radio_alarm = 0;
            radio_on = 0;
        }
        snoozed10 = 0;//We are not snoozed if we have no alarm set
        lcdwrite[0] = ' '; //alarm clear
        lcdwrite[1] = ' ';
        lcdwrite[2] = ' ';
        lcdwrite[3] = ' ';
        lcdwrite[4] = ' ';

        segment_data[2]= 0xFF;
        alarmcheck = 0;//No alarm to check we cleared it
        alarmtone = 0;//no tone!
        alarmset = 0;
        clear = 0;//we do not want to clear all the time
        mode = 0;
    }

}
ISR(INT7_vect){STC_interrupt = TRUE;}
ISR(TIMER1_COMPA_vect)
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
    init_twi();//initialize twi
    uart_init();

    lm73_wr_buf[0] = LM73_PTR_TEMP;
    PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
    DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
    PORTE |=  (1<<PE2); //hardware reset Si4734
    _delay_us(200);     //hold for 200us, 100us by spec
    PORTE &= ~(1<<PE2); //release reset
    _delay_us(30);      //5us required because of my slow I2C translators I suspect
                                 //Si code in "low" has 30us delay...no explaination
    DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

    sei();
    current_fm_freq = 10630;

       segment_data[2] = 0xFF;//clear colon

    while(1){
           uart_putc('A');
           exta = uart_getc();
           if(exta != '?'){ext_tempa = exta;}
           extb = uart_getc();
           if(extb != '?'){ext_tempb = extb;}

           if(radio_pwr == 1)///radio on
           {
                fm_pwr_up();
                fm_tune_freq();
                radio_on = 1;
                radio_pwr = 0;
           }
           if(radio_tune == 1)//tune radio
           {
               fm_tune_freq();
               radio_tune = 0;
           }
           if(radio_off == 1)//turn off radio
           {
               radio_pwr_dwn();
               radio_off = 0;
           }
    }
}//main
