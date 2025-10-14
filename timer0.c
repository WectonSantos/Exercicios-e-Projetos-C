/*
 * simulado timer0.c
 *
 * Created: 05/10/2025 22:39:11
 * Author : Wecton
gbm
74 = 16M / 256 * (OCR0A+1)
OCR0A + 1 = 16M / 256 * 74
OCR0A=3405.4?1 => OCR0A = 3404.04 => ajustando para 3404 p trabalhar c numeros inteiros
 */ 

#define F_CPU 16000000    //define o clock de trabalho em 16Mhz           

#include <avr/io.h>                 // importa todas as definições de IO
#include <avr/interrupt.h>          // importa todas as definições de Interrupção    


#define SETA_BIT(PORT,BIT) PORT |= (1<<BIT)    //seta o determinado bit
#define LIMPA_BIT(PORT,BIT) PORT &= ~(1<<BIT)  //limpa o determiando bit
#define ALTERNA_BIT(PORT,BIT) PORT ^= (1<<BIT) //alterna o estado lógico do bit da porta que quiser

void timer0_init() {

	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);  
	TCCR0B |= (1 << WGM02) | (1 << CS01) | (1 << CS00);     

	OCR0A = 3404;  
	
    DDRD |= (1 << PORTD6);

	sei();
}

int main(void) {
	timer0_init();    
	
	while (1) {
	}
}
