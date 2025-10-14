#include <avr/io.h>
#include <avr/interrupt.h> // carrego a biblioteca de interrupções

ISR(INT0_vect) {
	PORTB |= (1<< PORTB0); // atribuo o valor 1 no pino B0
}
ISR(INT1_vect) {
	PORTB &= ~(1<< PORTB0); // atribuo o valor 0 no pino B0
}

int main(void)
{
	DDRD &= ~(1<<PORTD2); // seto o pino D2 como entrada (botao 1)
	DDRD &= ~(1<<PORTD3); // seto o pino D3 como entrada (botao 2)
	
	DDRB |= (1<<PORTB0); // seto o pino B0 como saida (motor) 
	
	//pull-ups dos botões
	PORTD |= (1<<PORTD2);
	PORTD |= (1<<PORTD3);
	
	//configuro a interrupção INT0 -> PORTD2
	EICRA |= (1<<ISC01);
	EICRA &= ~(1<<ISC00); 
	
	EIMSK |= (1<<INT0);
	
	//configuro a interrupção INT1 -> PORTD3
	EICRA |= (1<<ISC11);
	EICRA &= ~(1<<ISC10);
	EIMSK |= (1<<INT1);
	
	// habilito as interrupções
	sei();
    while (1) 
    {
    }
}

