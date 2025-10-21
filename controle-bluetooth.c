#include <avr/io.h>

// ================== DEFINIÇÕES ==================
#define LED PD2   // LED no pino digital 2 (PORTD bit 2)

// ================== UART ==================

// Inicializa UART em 9600 bps
void UART_Init(void) {
    uint16_t ubrr = 103; // 9600 bps @ 16 MHz
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);       // habilita RX e TX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // 8 bits de dados
}

// Transmite um caractere
void UART_Transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // espera buffer livre
    UDR0 = data;
}

// Envia uma string
void UART_SendString(const char *str) {
    while (*str) UART_Transmit(*str++);
}

// Recebe um caractere
char UART_Receive(void) {
    while (!(UCSR0A & (1 << RXC0))); // espera dado chegar
    return UDR0;
}

// ================== AÇÕES ==================
void ligarLED(void) {
    PORTD |= (1 << LED);
}

void desligarLED(void) {
    PORTD &= ~(1 << LED);
}

// ================== PROGRAMA PRINCIPAL ==================
int main(void) {
    // Configura pino D2 como saída e apaga LED inicialmente
    DDRD |= (1 << LED);
    desligarLED();

    UART_Init();
    UART_SendString("=== HC-06 Controle (C Puro) ===\r\n");
    UART_SendString("Use F, B, L, R para acionar o LED D2\r\n");

    while (1) {
        char comando = UART_Receive(); // Espera comando do Bluetooth

        switch (comando) {
            case 'F': // Forward (CIMA)
            case 'f':
                ligarLED();
                UART_SendString(">> CIMA (F)\r\n");
                break;

            case 'B': // Backward (BAIXO)
            case 'b':
                ligarLED();
                UART_SendString(">> BAIXO (B)\r\n");
                break;

            case 'L': // Left (ESQUERDA)
            case 'l':
                ligarLED();
                UART_SendString(">> ESQUERDA (L)\r\n");
                break;

            case 'R': // Right (DIREITA)
            case 'r':
                ligarLED();
                UART_SendString(">> DIREITA (R)\r\n");
                break;

            default:
                desligarLED();
                UART_SendString(">> Comando invalido\r\n");
                break;
        }
    }
}