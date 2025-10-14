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
	DDRC |= (1 << PORTC0); // LED 1 (SAÍDA)
	DDRC |= (1 << PORTC1); // LED 2 (SAÍDA)
	DDRC |= (1 << PORTC2); // LED 3	(SAÍDA)
	
	PORTC |= (1<< PORTC0); // LIGANDO LED 1
	PORTC |= (1<< PORTC1); // LIGANDO LED 1
	PORTC |= (1<< PORTC2); // LIGANDO LED 1

}
// ROTINA QUE CONFIGURA OS MOTORES
void motores(){
	DDRB |= (1 << PB7); // MOTOR 1 SENTIDO HORÁRIO
	DDRB |= (1 << PB4); // MOTOR 1 SENTIDO ANTI-HORÁRIO
	DDRD |= (1 << PD5); // MOTOR 1 PWM
	
	DDRB |= (1 << PB5); // MOTOR 2 SENTIDO HORÁRIO
	DDRB |= (1 << PB0); // MOTOR 2 SENTIDO ANTI-HORÁRIO
	DDRD |= (1 << PD6); // MOTOR 2 PWM	
	
	//TESTE
    // IN1–IN4 como saída
    DDRB |= (1<<PB7)|(1<<PB4)|(1<<PB5)|(1<<PB0);
    // ENA (PD5) e ENB (PD6) como saída, mas sem setar PORTD ainda
    DDRD |= (1<<PD5)|(1<<PD6);
	//TESTE
}

// BOTÕES PREVIAMENTE QUE SERÃO TROCADOS PELO VALOR RECEBIDO DO CONTROLE
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
	// Referência AVcc (5V), canal ADC4, right adjust
	ADMUX = (1 << REFS0); // AVcc como referência, canal será configurado na leitura
	ADCSRA = (1 << ADEN)  // habilita ADC
	| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128 (16MHz/128=125kHz)
	DIDR0 = (1 << ADC4D); // desativa entrada digital no PC4
}

uint16_t adc_read(uint8_t canal) {
	// seleciona o canal (0–7)
	ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);
	// inicia conversão
	ADCSRA |= (1 << ADSC);
	// espera terminar
	while (ADCSRA & (1 << ADSC));
	// retorna valor 0–1023
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


void ligaMotoresHorario(){
	if (PIND & (1<<PD4)) {
			
		// define direção: IN1=1, IN2=0 ; IN3=1, IN4=0
		PORTB |=  (1<<PB7)|(1<<PB5);
		PORTB &= ~((1<<PB4)|(1<<PB0));

		// liga PWM nos enables
		OCR0B = 200;   // duty motor 1 (ajuste entre 0–255)
		OCR0A = 200;   // duty motor 2
	}
	else {
		// desliga PWM (motores parados)
		OCR0B = 0;
		OCR0A = 0;

		// opcional: deixa INx em 0 para garantir motor livre
		PORTB &= ~((1<<PB7)|(1<<PB5));
	}	
}

void ligaMotoresAntiHorario(){
	if (PINB & (1<<PB1)) {
		
		// define direção: IN1=1, IN2=0 ; IN3=1, IN4=0
		PORTB |=  (1<<PB4)|(1<<PB0);
		PORTB &= ~((1<<PB7)|(1<<PB5));

		// liga PWM nos enables
		OCR0B = 200;   // duty motor 1 (ajuste entre 0–255)
		OCR0A = 200;   // duty motor 2
	}
	else {
		// desliga PWM (motores parados)
		OCR0B = 0;
		OCR0A = 0;

		// opcional: deixa INx em 0 para garantir motor livre
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
		//50%();
	}
	else {
		// Nenhum botão pressionado ? garantir que motores estão parados
		OCR0A = 0;
		OCR0B = 0;
		PORTB &= ~((1 << PB7) | (1 << PB5) | (1 << PB4) | (1 << PB0));
	}	
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

	//TESTE
	PORTB &= ~((1 << PB7) | (1 << PB5) | (1 << PB4) | (1 << PB0));
    // configura Fast PWM no Timer0
    TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);
    TCCR0B = (1<<CS01);    // prescaler = 8

    // começa com duty = 0 (motores desligados)
    OCR0A = 0;  // canal OC0A ? motor 2
    OCR0B = 0;  // canal OC0B ? motor 1
	//TESTE

    while (1) 
    {
		//ROTINA LDR
		valorLDR = adc_read(4); // lê o ADC4 (PC4)

		if (valorLDR > LIMIAR_LUZ) {
			luzAlta = 1; // LDR recebeu valor alto
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
		//FIM ROTINA LDR
		
		
		
		//LIGANDO MOTORES SENTIDO HORÁRIO
		/*if(PIND & (1 << PORTD4)){
			PORTB |= (1 << PORTB7);
			PORTB |= (1 << PORTB5);
		}*/
		
		//TESTE
		verificaSentido();								
		//TESTE
		
		
    }
}

