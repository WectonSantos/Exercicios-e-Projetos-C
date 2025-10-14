/* gbm
 * simulado timer1.c
 *
 * Created: 05/10/2025 22:25:05
 * Author : Wecton
 
 
timer1 => 16bits => 665536 passos
Crystal 16mhz

C = 64mHZ/16 => C= 4us
NP = 61ms/4us = 15250passos
 
 
 */ 
#define F_CPU 16000000              //define o clock de trabalho em 16Mhz

#include <avr/io.h>                 // importa todas as definições de IO
#include <avr/interrupt.h>          // importa todas as definições de Interrupção


#define SETA_BIT(PORT,BIT) PORT |= (1<<BIT)    //seta o determinado bit
#define LIMPA_BIT(PORT,BIT) PORT &= ~(1<<BIT)  //limpa o determiando bit
#define ALTERNA_BIT(PORT,BIT) PORT ^= (1<<BIT) //alterna o estado lógico do bit da porta que quiser

ISR(TIMER1_COMPA_vect){
	
	ALTERNA_BIT(PORTB,PORTB0);
	
}

void timer1_init(){
	TCCR1A |= 0x00;
	TCCR1B |= (1<<WGM12)|(1<<CS11)|(1<<CS10);//configura o prescale em 64
	TCNT1 = 0;
	OCR1A = 15249;    //inicio o Contador
	TIMSK1 = (1<<OCIE1A);//habilita interrupcao de interrupção Comp A (verficiar qual vetor de interrupção correspondente do datasheet)
	sei();        //Habilitar as interrupções globais
}

      
int main(void){
	DDRB |= (1 << PORTB0);
	timer1_init();    
	while (1){}              
    }

