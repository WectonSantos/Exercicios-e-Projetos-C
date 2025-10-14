#include <avr/io.h>
#include <avr/interrupt.h> // carrego a biblioteca de interrup��es

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
	
	//pull-ups dos bot�es
	PORTD |= (1<<PORTD2);
	PORTD |= (1<<PORTD3);
	
	//configuro a interrup��o INT0 -> PORTD2
	EICRA |= (1<<ISC01);
	EICRA &= ~(1<<ISC00); 
	
	EIMSK |= (1<<INT0);
	
	//configuro a interrup��o INT1 -> PORTD3
	EICRA |= (1<<ISC11);
	EICRA &= ~(1<<ISC10);
	EIMSK |= (1<<INT1);
	
	// habilito as interrup��es
	sei();
    while (1) 
    {
    }
}

