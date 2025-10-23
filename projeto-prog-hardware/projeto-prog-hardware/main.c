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

// ROTINA QUE CONFIGURA OS LEDS DE VIDA
void ledsVida(){
	DDRC |= (1 << PC0); // LED 1 (SAÍDA)
	DDRC |= (1 << PC1); // LED 2 (SAÍDA)
	DDRC |= (1 << PC2); // LED 3	(SAÍDA)
	
	PORTC |= (1<< PC0); // LIGANDO LED 1
	PORTC |= (1<< PC1); // LIGANDO LED 1
	PORTC |= (1<< PC2); // LIGANDO LED 1
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
	DDRB &= ~(1 << PB1);
	DDRB &= ~(1 << PB2);
	DDRD &= ~(1 << PD4);
}

// ROTINA QUE CONFIGURA O LASER
void laser(){
	DDRB |= (1 << PB3); // LASER PORTA B3 (CTC)
}

// ROTINA LDR

void adc_init(void) {
	ADMUX = (1 << REFS0); 
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128 (16MHz/128=125kHz)
	//DIDR0 = (1 << ADC4D);
	//DDRC &= ~(1 << PC4); 
}

uint16_t adc_read(uint8_t canal) {
	//ADMUX = (1 << REFS0) | (canal & 0x0F);
	ADMUX = (ADMUX & 0xF0) | (canal & 0x0f);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC; 
}


void atualizaLedsVida() {
	if (vidas >= 3){
		PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2);
	}else if (vidas == 2) {
		PORTC |= (1 << PC1) | (1 << PC2);
		PORTC &= ~(1 << PC0);
	}else if (vidas == 1) {
		PORTC |= (1 << PC2);
		PORTC &= ~((1 << PC0) | (1 << PC1));
	}else{
		PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2));
	}
}
// FIM ROTINA LDR

// ROTINA DOS MOTORES
void ligaMotoresHorario(){		
	// DEFINE DIREÇÃO: IN1=1, IN2=0 ; IN3=1, IN4=0
	PORTB |=  (1<<PB7)|(1<<PB5);
	PORTB &= ~((1<<PB4)|(1<<PB0));

	// LIGA PWM
	OCR0B = 200;  
	OCR0A = 200;   
}

void ligaMotoresAntiHorario(){
	// DEFINE DIREÇÃO: IN1=1, IN2=0 ; IN3=1, IN4=0
	PORTB |=  (1<<PB4)|(1<<PB0);
	PORTB &= ~((1<<PB7)|(1<<PB5));

	// LIGA PWM
	OCR0B = 200; 
	OCR0A = 200; 	
}

void ligaMotoresMeiaForca(){
	// DEFINE DIREÇÃO
	PORTB |=  (1<<PB7)|(1<<PB5);
	PORTB &= ~((1<<PB4)|(1<<PB0));

	// APLICANDO 50% DE FATOR DE CICLO
	OCR0B = 128;   // motor 1
	OCR0A = 128;   // motor 2
}

void giraEsquerda() {
	// MOTOR 1: ANTI-HORÁRIO (PB4)
	// MOTOR 2: HORÁRIO (PB5)
	PORTB |= (1 << PB4) | (1 << PB5);
	PORTB &= ~((1 << PB7) | (1 << PB0));

	// LIGA PWM
	OCR0B = 200;   // motor 1
	OCR0A = 200;   // motor 2
}

void giraDireita() {
	// MOTOR 1: HORÁRIO (PB7)
	// MOTOR 2: ANTI-HORÁRIO (PB0)
	PORTB |= (1 << PB7) | (1 << PB0);
	PORTB &= ~((1 << PB4) | (1 << PB5));

	// LIGA PWM
	OCR0B = 200;   // motor 1
	OCR0A = 200;   // motor 2
}

void pararMotores() {
	OCR0A = 0;
	OCR0B = 0;
	PORTB &= ~((1 << PB7) | (1 << PB5) | (1 << PB4) | (1 << PB0));
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

//ROTINA CONTROLE BLUETOOTH
// INICIAALIZA UART EM 9600 BPS
void UART_Init(void) {
	uint16_t ubrr = 103; // 9600 BPS p/ 16 MHz
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);       // HABILITA RX E TX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // 8 BITS DE DADOS
}
// RECEBE UM CARACTERA
char UART_Receive(void) {
	while (!(UCSR0A & (1 << RXC0))); // ESPERA DADO CHEGAR
	return UDR0;
}
// ENVIA 1 CARACTERE PELA UART
void UART_Transmit(char data) {
	while (!(UCSR0A & (1 << UDRE0))); // ESPERA BUFFER ESVAZIAR
	UDR0 = data;
}

// ENVIA STRING PELA UART
void UART_SendString(const char *str) {
	while (*str) {
		UART_Transmit(*str++);
	}
}

// ENVIA INT COMO TEXTO PARA UART

void UART_SendInt(uint16_t value) {
	char buffer[10];
	itoa(buffer, "%u", value);  // FUNÇÃO QUE CONVERTE INT PARA STRING 
	UART_SendString(buffer);
}


void controle(){
	char comando = UART_Receive(); // ESPERA COMANDO DO BLUETOOTH
    switch (comando) {
	    case 'F': // PARA CIMA
	    case 'f':
			ligaMotoresHorario();
			PORTD |= (1 << PD2);
			break;

	    case 'B': // PARA BAIXO
	    case 'b':
			ligaMotoresAntiHorario();
			PORTD |= (1 << PD2);
			break;

	    case 'L': // PARA ESQUERDA
	    case 'l':
			giraEsquerda();
			PORTD |= (1 << PD2);
			break;

	    case 'R': // PARA DIREITA
	    case 'r':
			giraDireita();
			PORTD |= (1 << PD2);

			break;

	    default:
			pararMotores();
	    break;
    }		
	
}

//FIM ROTINA CONTROLE BLUETOOTH


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

    //GARANTE QUE VAI COMEÇAR COM MOTORES DESLIGADOS
    OCR0A = 0; 
    OCR0B = 0;  

	
	//LASER
	timer1_init();

	// CONTROLE 
	UART_Init();
	
	DDRD |= (1 << PD2); // teste ldr

    while (1) 
    {
		//ROTINA LDR
		valorLDR = adc_read(4); // LÊ O ADC4 (PC4)
		// debug ldr
		UART_SendString("LDR = ");
		UART_SendInt(valorLDR);
		UART_SendString("\r\n");	
		// debug ldr

		if (valorLDR > 300) {
			luzAlta = 1; // LDR RECEBEU VALOR ALTO
			PORTD |= (1 << PD2);
		} else{
			luzAlta = 0;
			PORTD &= ~(1 << PD2);
		}
		if (luzAlta){
			if(vidas>0){
			vidas = vidas -1;
			}
		}
		atualizaLedsVida();
		verificaSentido();								
		
		if (UCSR0A & (1 << RXC0)) { // SE HOUVER ALGO EM UART
			controle(); // CHAMA CONTROLE
		}
    }
}

