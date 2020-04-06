#include <avr/interrupt.h>
#include <avr/io.h>
void init_PWM_timer0() {       //PWM motoare
  cli();
  TCCR0A = TCCR0B = 0;
  TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);
  TCCR0B |= (1 << CS00);
  sei();
}
void PWM(int timeON_A, int timeON_B, int stare) {
  PORTB&=~0x07;
  PORTD&=~0x38;
  if(stare==0){//frana
    if(timeON_A == 0 && timeON_B == 0) {
      PORTB |= (1<<2);
      PORTD |= (1<<5);
      PORTD &=~ (1<<3);//in1
      PORTD &=~ (1<<4);//in2
      PORTB &=~ (1<<0);//in3
      PORTB &=~ (1<<1);//in4
      OCR0A = timeON_A;
      OCR0B = timeON_B;
    }
  }
  else if(stare==1){//stanga
    if(timeON_A > 0 && timeON_B > 0) {
      PORTB |= (1<<2);
      PORTD |= (1<<5);
      PORTD &=~ (1<<3);//in1
      PORTD &=~ (1<<4);//in2
      PORTB &=~ (1<<0);//in3
      PORTB &=~ (1<<1);//in4
      OCR0A = timeON_A;
      OCR0B = timeON_B;
    }
  }
  else if(stare==2){//dreapta
    if(timeON_A > 0 && timeON_B > 0) {
      PORTB |= (1<<2);
      PORTD |= (1<<5);
      PORTD &=~ (1<<3);//in1
      PORTD |= (1<<4);//in2
      PORTB &=~ (1<<1);//in3
      PORTB |= (1<<0);//in4
      OCR0A = timeON_A;
      OCR0B = timeON_B;
    }
  }
  else if(stare==3){//spate
    if(timeON_A > 0 && timeON_B > 0) {
      PORTB |= (1<<2);
      PORTD |= (1<<5);
      PORTD &=~ (1<<3);//in1
      PORTD |= (1<<4);//in2
      PORTB |= (1<<1);//in3
      PORTB &=~(1<<0);//in4
      
      OCR0A = timeON_A;
      OCR0B = timeON_B;
    }
  }
  else if(stare==4){//fata
    if(timeON_A > 0 && timeON_B > 0) {
      PORTB |= (1<<2);
      PORTD |= (1<<5);
      PORTD |= (1<<3);//in1
      PORTD &=~ (1<<4);//in2
      PORTB &=~ (1<<1);//in3
      PORTB |= (1<<0);//in4
      
      OCR0A = timeON_A;
      OCR0B = timeON_B;
    }
  }
}
int main()
{
  DDRD |= 0x38;
  DDRB |= 0x07;
  DDRB |= (1<<3) //trg
  init_PWM_timer0();
  PWM(100,100,4);
}
