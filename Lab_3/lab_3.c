// bar_graph_demo_skel.c
// R. Traylor
// 10.22.14
// demos interrupts, counter timer and SPI

// !!! YOU MUST RENAME THIS FILE TO MATCH YOUR MAKEFILE   !!!
// !!! YOU MUST FILL IN THE AREAS MARKED WITH "@" SIGNS   !!!
// !!! DISCONNECT ALL OTHER CONNECTIONS TO YOUR AVR BOARD !!!

// This code implements a timer interrupt to update the bar graph display
// at 0.5 second intervals. Every half second, a new data value is sent to
// the bargraph via SPI. The value is displayed as single led in a climbing
// pattern.

// Expected Connections:
// Bargraph board           Mega128 board
// --------------      ----------------------
//     reglck            PORTB bit 0 (ss_n)
//     srclk             PORTB bit 1 (sclk)
//     sdin              PORTB bit 2 (mosi)
//     oe_n                   ground
//     gnd2                   ground
//     vdd2                     vcc
//     sd_out               no connect

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define CW 1
#define CCW 0
volatile uint8_t scalar = 1;
volatile uint8_t encoder_out = 0;
volatile int16_t sum = 0;
uint16_t encode_in;
uint8_t segment_data[5];
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF};

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
    DDRE  |=   (1 << PE6);
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
    TIMSK  |=  (1 << TOIE0); //enable timer/counter0 overflow interrupt
    TCCR0  |=  ((1 << CS02)|(1<<CS00));  //normal mode, no prescale
}

uint8_t chk_buttons(uint8_t button)
{
    static uint16_t state[4] = {0, 0, 0, 0};
    state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000;
    if(state[button] == 0xF000){return 1;}
    else{return 0;}
}

void segsum(uint16_t sum)
{
    segment_data[0] = dec_to_7seg[((sum % 100) % 10)];
    segment_data[1] = dec_to_7seg[((sum % 100)/ 10)];
    segment_data[2] = dec_to_7seg[10];
    segment_data[3] = dec_to_7seg[((sum % 1000) / 100)];
    segment_data[4] = dec_to_7seg[(sum / 1000)];

    if(segment_data[4] == 0xC0)
    {
        segment_data[4] = 0xFF;
        if(segment_data[3] == 0xC0)
        {
            segment_data[3] = 0xFF;
            if(segment_data[1] == 0xC0)
            {
                segment_data[1] = 0xFF;
            }
        }
    }
}
/*************************************************************************/
//                           timer/counter0 ISR
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is
//incremented.  Every 7680 interrupts the minutes counter is incremented.
//tcnt0 interrupts come at 7.8125ms internals.
// 1/32768         = 30.517578uS
//(1/32768)*256    = 7.8125ms
//(1/32768)*256*64 = 500mS
/*************************************************************************/
ISR(TIMER0_OVF_vect)
{
  //  uint8_t digit = 0;
    if(sum > 1023){sum = 0;}
    else if(sum < 0){sum = 1023;}
    uint8_t direction = 7;
    segsum(sum);
    //_delay_ms(1);

    PORTB |= 0x70;
    DDRA = 0x00;//make PORTA an input port with pullups
    PORTA = 0xFF;
    _delay_ms(.1);
    if(chk_buttons(0))//Increment by 2
    {
        if(scalar == 2){scalar = 1;}
        else if(scalar == 0){scalar = 4;}
        else if(scalar == 4){scalar = 0;}
        else{scalar = 2;}
    }

    if(chk_buttons(1))//Increment by 4
    {
        if(scalar == 4){scalar = 1;}
        else if(scalar == 0){scalar = 2;}
        else if(scalar == 2){scalar = 0;}
        else{scalar = 4;}
    }
    DDRA = 0xFF;
    PORTB &= ~0x70;
   /* while(digit < 5)
    {
        PORTA = segment_data[digit];
        PORTB = ((0x10 * digit));
        digit++;
        _delay_ms(.25);
    }
    digit = 0;*/
    PORTB &= ~(1 << PB7);
    PORTE &= ~(1 << PE6);
    PORTB |=  (1 << PB0);
    SPDR = scalar;
    while(bit_is_clear(SPSR,SPIF)){}
    PORTD |=  (1 << PD2);
    PORTD &= ~(1 << PD2);
    PORTB &= ~(1 << PB0);
    encoder_out = SPDR;
    PORTB |= (1 << PB7);

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
    if((a_current2 == 1) && (b_past2 < b_current2)){direction = CW;}
    if((a_current2 == 1) && (b_past2 > b_current2)){direction = CCW;}
    }

    /*if((a_past < a_current) && ((b_past | b_current) == 0)){direction=CW;}
    if((a_past < a_current) && ((b_past | b_current) == 1)){direction = CCW;}
    if((a_past > a_current) && ((b_past | b_current) == 1)){direction = CW;}
    if((a_past > a_current) && ((b_past | b_current) == 0)){direction = CCW;}*/

    a_past = a_current;
    b_past = b_current;
    a_past2 = a_current2;
    b_past2 = b_current2;


    if(direction == CW){sum += scalar;}
    else if(direction == CCW){sum -= scalar;}
    else if(direction == 7){sum = sum;}

}


/***********************************************************************/
//                                main
/***********************************************************************/
int main()
{
    uint8_t digit = 0;
    tcnt0_init();  //initalize counter timer zero
    spi_init();    //initalize SPI port
//    sei();         //enable interrupts before entering loop
    while(1){
        cli();
        while(digit < 5)
        {
            PORTA = segment_data[digit];
            PORTB = ((0x10 * digit));
            digit++;
            _delay_ms(.25);
        }
        digit = 0;
        sei();
    }
}//main
