#include <avr/io.h>

void timer0_init(){
	// Configura o timer sem usar prescaler
	TCCR0B |= (1 << CS00);
	
	// inicializa o contador do timer 0
	TCNT0 = 0;
}

int main(void)
{
	//Setando o pino C0 como saida
	DDRC |= (1 << PINC0);
	
	//inicia o timer
	timer0_init();
	
	
	while (1)
	{
		//verifica se o timer chegou em 191, ou seja, 6ms
		if(TCNT0 >= 191){
			PORTC ^= (1<< PINC0); //alterna o estado do LED
			TCNT0 = 0; //zera o timer
		}
		
	}
}
