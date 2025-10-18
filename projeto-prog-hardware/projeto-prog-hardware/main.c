/*
 * projeto-prog-hardware.c
 *
 * Created: 13/10/2025 22:14:15
 * Author : Wecton
 gbm GBM
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t vidas = 3;
// LIMITE DE LUZ LDR
#define LIMIAR_LUZ 700

// ROTINA QUE CONFIGURA OS LEDS DE VIDA
void ledsVida(){
	DDRC |= (1 << PORTC0); // LED 1 (SA�DA)
	DDRC |= (1 << PORTC1); // LED 2 (SA�DA)
	DDRC |= (1 << PORTC2); // LED 3	(SA�DA)
	
	PORTC |= (1<< PORTC0); // LIGANDO LED 1
	PORTC |= (1<< PORTC1); // LIGANDO LED 1
	PORTC |= (1<< PORTC2); // LIGANDO LED 1

}
// ROTINA QUE CONFIGURA OS MOTORES
void motores(){
	DDRB |= (1 << PB7); // MOTOR 1 SENTIDO HOR�RIO
	DDRB |= (1 << PB4); // MOTOR 1 SENTIDO ANTI-HOR�RIO
	DDRD |= (1 << PD5); // MOTOR 1 PWM
	
	DDRB |= (1 << PB5); // MOTOR 2 SENTIDO HOR�RIO
	DDRB |= (1 << PB0); // MOTOR 2 SENTIDO ANTI-HOR�RIO
	DDRD |= (1 << PD6); // MOTOR 2 PWM	
	
	//TESTE
    // IN1�IN4 como sa�da
    DDRB |= (1<<PB7)|(1<<PB4)|(1<<PB5)|(1<<PB0);
    // ENA (PD5) e ENB (PD6) como sa�da, mas sem setar PORTD ainda
    DDRD |= (1<<PD5)|(1<<PD6);
	//TESTE
}

// BOT�ES PREVIAMENTE QUE SER�O TROCADOS PELO VALOR RECEBIDO DO CONTROLE
void botoes(){
	DDRB &= ~(1 << PORTB1);
	DDRB &= ~(1 << PORTB2);
	DDRD &= ~(1 << PORTD4);
}

// ROTINA QUE CONFIGURA O LASER
void laser(){
	DDRB |= (1 << PORTB3); // LASER PORTA B3 (CTC)
}

// ROTINA LDR
void adc_init(void) {
	// Refer�ncia AVcc (5V), canal ADC4, right adjust
	ADMUX = (1 << REFS0); // AVcc como refer�ncia, canal ser� configurado na leitura
	ADCSRA = (1 << ADEN)  // habilita ADC
	| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128 (16MHz/128=125kHz)
	DIDR0 = (1 << ADC4D); // desativa entrada digital no PC4
}

uint16_t adc_read(uint8_t canal) {
	// seleciona o canal (0�7)
	ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);
	// inicia convers�o
	ADCSRA |= (1 << ADSC);
	// espera terminar
	while (ADCSRA & (1 << ADSC));
	// retorna valor 0�1023
	return ADC;
}

void atualizaLedsVida() {
	if (vidas >= 3){
		PORTC |= (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2);
	}else if (vidas == 2) {
		PORTC |= (1 << PORTC1) | (1 << PORTC2);
		PORTC &= ~(1 << PORTC0);
	}else if (vidas == 1) {
		PORTC |= (1 << PORTC2);
		PORTC &= ~((1 << PORTC0) | (1 << PORTC1));
	}else{
		PORTC &= ~((1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2));
	}
}
// FIM ROTINA LDR

// ROTINA DOS MOTORES
void ligaMotoresHorario(){
	if (PIND & (1<<PD4)) {
			
		// DEFINE DIRE��O: IN1=1, IN2=0 ; IN3=1, IN4=0
		PORTB |=  (1<<PB7)|(1<<PB5);
		PORTB &= ~((1<<PB4)|(1<<PB0));

		// LIGA PWM
		OCR0B = 200;   // duty motor 1 (ajuste entre 0�255)
		OCR0A = 200;   // duty motor 2
	}
	else {
		// DESLIGA PWM (MOTORES PARADOS)
		OCR0B = 0;
		OCR0A = 0;

		PORTB &= ~((1<<PB7)|(1<<PB5));
	}	
}

void ligaMotoresAntiHorario(){
	if (PINB & (1<<PB1)) {
		
		// DEFINE DIRE��O: IN1=1, IN2=0 ; IN3=1, IN4=0
		PORTB |=  (1<<PB4)|(1<<PB0);
		PORTB &= ~((1<<PB7)|(1<<PB5));

		// LIGA PWM
		OCR0B = 200; 
		OCR0A = 200; 
	}
	else {
		// DESLIGA PWM (MOTORES PARADOS)
		OCR0B = 0;
		OCR0A = 0;
		PORTB &= ~((1<<PB7)|(1<<PB5));
	}
}

void ligaMotoresMeiaForca(){
	if (PINB & (1 << PB2)) {

		// define dire��o padr�o (por exemplo, sentido hor�rio)
		PORTB |=  (1<<PB7)|(1<<PB5);
		PORTB &= ~((1<<PB4)|(1<<PB0));

		// APLICANDO 50% DE FATOR DE CICLO
		OCR0B = 128;   // motor 1
		OCR0A = 128;   // motor 2
	}
	else {
		// desliga PWM (motores parados)
		OCR0B = 0;
		OCR0A = 0;
		PORTB &= ~((1<<PB7)|(1<<PB5));
	}
}


void verificaSentido(){
	if (PIND & (1 << PD4)) {
		ligaMotoresHorario();
	}
	else if (PINB & (1 << PB1)) {
		ligaMotoresAntiHorario();
	}else if (PINB & (1 << PB2)){
		ligaMotoresMeiaForca();  
	}
	else {
		// Nenhum bot�o pressionado ? garantir que motores est�o parados
		OCR0A = 0;
		OCR0B = 0;
		PORTB &= ~((1 << PB7) | (1 << PB5) | (1 << PB4) | (1 << PB0));
	}	
}
//FIM ROTINA DOS MOTORES

//ROTINA LASER MODO CTC
void timer1_init() {
	TCCR1B |= (1 << WGM12);  // CTC
	OCR1A = 15624;           // 1 SEGUNDO P/ PRESCALER 1024
	TIMSK1 |= (1 << OCIE1A); 
	TCCR1B |= (1 << CS12) | (1 << CS10); 
	sei();
}

ISR(TIMER1_COMPA_vect) {
	PORTB ^= (1 << PB3);  // ALTERNA ESTADO DO LED
}




int main(void)
{
	// SETANDO TODAS AS PORTAS
	ledsVida(); 
	motores(); 
	botoes(); 
	laser(); 
	
	// ROTINA LDR
	adc_init();
	uint16_t valorLDR;
	uint8_t luzAlta = 0; 
	// FIM ROTINA LDR


	//PORTB &= ~((1 << PB7) | (1 << PB5) | (1 << PB4) | (1 << PB0));
    // FAST PWM NO TIMER 0
    TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);
    TCCR0B = (1<<CS01);    // PRESCALER = 8

    //GARANTE QUE VAI COME�AR COM MOTORES DESLIGADOS
    OCR0A = 0; 
    OCR0B = 0;  

	
	//LASER
	timer1_init();

    while (1) 
    {
		//ROTINA LDR
		valorLDR = adc_read(4); // L� O ADC4 (PC4)

		if (valorLDR > LIMIAR_LUZ) {
			luzAlta = 1; // LDR RECEBEU VALOR ALTO
			PORTB |= (1 << PORTB3);
			} else {
			luzAlta = 0; 
		}
		if (luzAlta){
			if(vidas>0){
			vidas = vidas -1;
			}
		}
		atualizaLedsVida();
		verificaSentido();								
		
    }
}

