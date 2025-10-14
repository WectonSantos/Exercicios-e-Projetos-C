/*
 * GccApplication1.c
 *
 * Created: 05/10/2025 20:52:00
 * Author : Wecton
 */ 

/*
	=> 23 segundos

	Crystal 16mhz .:. T = 1/16m => T= 62,5ns
	Timer 2 possui 8 bits .:. 255 ciclos de clock => 255*62,5ns = 15,93ms
	p/prescaler 1024 => máximo 16,32ms
		 prescaler           
			  8			2mhz		500ns	127,5us p/estouro
			  64		250mhz		4us		1,02ms 
			  256		62khz		16us	4,08ms
			  1024		15,63khz	64us	16,32ms
	    p/cd prescaler
		      8			180392156.863x     (180392156 * 127,5us = 22.99999989s) => 110ns p/compensar == +1160 ciclos de clock
			  64        22549.0196078x     (22549 * 1,02ms = 22,99998s) => 20us p/compensar == +51 ciclos
			  256		5637.25490196x	   (5637 * 4,08ms = 22,99996s) => 40us p/compensar == +102 ciclos
			  1024	    1409.31372549x     (1409 * 16,32ms = 22,99888s) => 1,12ms p/compensar == +15 ciclos p/compensar
			  
	o prescaler mais indicado neste caso seria o de 1024 pois precisa de menos estouros	contando o erro

				

*/
#define F_CPU 16000000
#include <avr/interrupt.h>
#include <avr/io.h>

volatile uint16_t tot_overflow = 0;

ISR(TIMER2_OVF_vect){
	tot_overflow++;
}

void timer0_init(){
	// timer2 normal, prescaler 1024
	TCCR2A = 0;
	TCCR2B = 0;
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // CS22:0 = 111 -> prescaler 1024

	TCNT2  = 0;
	TIMSK2 = (1 << TOIE2); // habilita interrupcao de overflow do Timer2

	sei(); // habilita interrupções globais
	tot_overflow = 0;
}

int main(void)
{
	DDRB |= (1 << PORTB0);
	timer0_init();

	while (1)
	{
		if (tot_overflow >= 1049) {
			if (TCNT2 >= 15) {
				PORTB ^= (1 << PORTB0);
				TCNT2 = 0;
				tot_overflow = 0;
			}
		}
	}
}


