// lab2_skel.c
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#define BLANK 11111111
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12];

uint8_t encode_to7(uint16_t BCD)
{
    switch(BCD)
    {
        case 0 :
            return 0x00000001;
            break;
        case 1 :
            return 0x10011111;
            break;
        case 2 :
            return 0x00100101;
            break;
        case 3 :
            return 0x00001101;
            break;
        case 4 :
            return 0x10011001;
            break;
        case 5 :
            return 0x01001001;
            break;
        case 6 :
            return 0x01000001;
            break;
        case 7 :
            return 0x00011111;
            break;

        case 8 :
            return 0x00000001;
            break;
        case 9 :
            return 0x00011001;
            break;

        default:
            return 0x11111111;
            break;
    }
}
//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//
uint8_t chk_buttons(uint8_t button) {
//******************************************************************************
    static uint16_t state[8] = {0,0,0,0,0,0,0,0};
    state[button] = (state[button] << 1) | (! bit_is_clear(PINA,button)) | 0xE000;
    if(state[button] == 0xF000){return 1;}
    else{return 0;}
}
//***********************************************************************************
//                                   segment_sum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {

    segment_data[4] = encode_to7((sum % 100) % 10);//Ones
    segment_data[3] = encode_to7(((sum % 100) - segment_data[4]) / 10);//Tens
    segment_data[2] = 0x11111111;
    segment_data[1] = encode_to7(((sum - segment_data[4] - (segment_data[3] * 10)) % 1000) / 100);//Hundo
    segment_data[0] = encode_to7((sum - segment_data[4] - (segment_data[3] * 10) - (segment_data[1] * 100)) / 1000);//Thousands

    int i = 0;
    while(i < 5)
    {
        if(segment_data[i] == 0x00000001)
        {
            segment_data[i] = 0x11111111;
        }
        else
        {
            break;
        }
        i++;
    }
}//segment_sum
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
DDRB = 0xFF; //set port bits 4-7 B as outputs
static uint16_t counter = 0;
static uint8_t  digit = 0;
segment_data[0] = 0x00;
segment_data[1] = 0x00;
segment_data[2] = 0x00;
segment_data[3] = 0x00;
while(1){
 _delay_ms(2); //insert loop delay for debounce
 DDRA = 0x00; //make PORTA an input port with pullups
 PORTA = 0x00;
 DDRB = 0xFF;
 PORTB = 0x00; //enable tristate buffer for pushbutton switches

 if(chk_buttons(0)){ counter += 1;}//now check each button and increment the count as needed
 if(chk_buttons(1)){ counter += 2;}
 if(chk_buttons(2)){ counter += 4;}
 if(chk_buttons(3)){ counter += 8;}
 if(chk_buttons(4)){ counter += 16;}
 if(chk_buttons(5)){ counter += 32;}
 if(chk_buttons(6)){ counter += 64;}
 if(chk_buttons(7)){ counter += 128;}

 PORTB  = 0xFF;//disable tristate buffer for pushbutton switches
 if(counter > 1023){ counter  = 1;}//bound the count to 0 - 1023
 segsum(counter);//break up the disp_value to 4, BCD digits in the array: call (segsum)
 int i = 0;
 while(i < 4)
 {
 DDRA = 0xFF;//make PORTA an output
 PORTA = digit;//send 7 segment code to LED segments
 PORTB = digit;//(digit | 0x10000000);//send PORTB the digit to display
 digit++;
 i++;
 } //update digit to display
 digit = 0;
 i = 0;
  }//while
return 0;
}
