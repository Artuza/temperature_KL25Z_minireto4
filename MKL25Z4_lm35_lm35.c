#include "MKL25Z4.h"

#define RS 0x02 // Define el pin RS del LCD (PTA1)
#define RW 0x04 // Define el pin RW del LCD (PTA2)
#define EN 0x10 // Define el pin EN del LCD (PTA4)
#define LCD_DATA GPIOD // Define el puerto de datos del LCD
#define ADC_RESOLUTION 1023.0
#define VREF 3.3  // Voltaje de referencia de 3.3V
#define NUM_READINGS 10  // Número de lecturas para el promedio


void delayMs(int n);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void ADC_Init(void);
uint32_t ADC_Read(void);
float ConvertToTemperature(uint32_t adcValue);
float calculateAverageTemperature(void);

int main(void) {
    LCD_init();  // Inicializa el LCD
    ADC_Init();  // Inicializa el ADC

    while (1) {
        uint32_t adcValue = ADC_Read(); // Lee el valor actual del ADC
        float currentTemperature = ConvertToTemperature(adcValue); // Convierte a temperatura

        LCD_command(1);  // Limpia la pantalla
        delayMs(1);

        // Formatea y muestra la temperatura actual
        char buffer[20];
        sprintf(buffer, "%.2fC", currentTemperature);
        LCD_command(0x80);  // Cursor en la primera línea
        char* ptr = buffer;
        while (*ptr) {
            LCD_data(*ptr++);  // Escribe cada carácter en el LCD
        }

        delayMs(500);  // Espera 0.5 segundos antes de la próxima lectura
    }
}



float calculateAverageTemperature(void) {
    float sumTemperatures = 0;  // Suma de las temperaturas

    // Recolectar múltiples lecturas
    for (int i = 0; i < NUM_READINGS; i++) {
        uint32_t adcValue = ADC_Read();
        float temperature = ConvertToTemperature(adcValue);
        sumTemperatures += temperature;  // Sumar las temperaturas
        delayMs(100);  // Pequeña pausa entre lecturas
    }

    return sumTemperatures / NUM_READINGS;  // Calcular promedio
}

void LCD_init(void) {
	SIM->SCGC5 |= 0x200;  // Habilita el reloj para el puerto A
	SIM->SCGC5 |= 0x1000; // Habilita el reloj para el puerto D
	PORTA->PCR[1] = 0x100;  // Pone el pin PTA1 como GPIO para RS
	PORTA->PCR[2] = 0x100;  // Pone el pin PTA2 como GPIO para RW
	PORTA->PCR[4] = 0x100;  // Pone el pin PTA4 como GPIO para EN
	PORTD->PCR[0] = 0x100;  // Pone el pin PTD0 como GPIO para D4
	PORTD->PCR[1] = 0x100;  // Pone el pin PTD1 como GPIO para D5
	PORTD->PCR[2] = 0x100;  // Pone el pin PTD2 como GPIO para D6
	PORTD->PCR[3] = 0x100;  // Pone el pin PTD3 como GPIO para D7
	GPIOD->PDDR |= 0x0F;    // Pone los pines PTD0-PTD3 como salida para datos
	GPIOA->PDDR |= RS | RW | EN; // Pone los pines PTA1, PTA2 y PTA4 como salida para control
    delayMs(30);  // Espera por más de 15 ms después de VCC sube a 4.5V
    LCD_command(0x03);  // Función set: modo de 8 bits
    delayMs(10);  // Espera 5ms
    LCD_command(0x03);  // Función set: modo de 8 bits
    delayMs(1);  // Espera 160us
    LCD_command(0x03);  // Función set: modo de 8 bits
    LCD_command(0x02);  // Función set: cambia a modo de 4 bits

    // Configuración del LCD
    LCD_command(0x28);  // DL=0 (4 bits), N=1 (2 líneas), F=0 (5x8 puntos)
    LCD_command(0x06);  // ID=1 (incrementa cursor), S=0 (no desplaza pantalla)
    LCD_command(0x01);  // Limpia pantalla
    LCD_command(0x0F);  // Enciende display, cursor parpadeante
}

void LCD_command(unsigned char command) {
    LCD_DATA->PDOR &= ~0xF;  // Limpia los datos
    LCD_DATA->PDOR |= (command & 0xF0) >> 4;  // Envía los 4 bits más significativos
    GPIOA->PCOR = RS | RW;  // RS = 0, RW = 0
    GPIOA->PSOR = EN;  // EN = 1
    delayMs(0);  // Espera 1ms
    GPIOA->PCOR = EN;  // EN = 0
    delayMs(1);  // Espera 1ms

    LCD_DATA->PDOR &= ~0xF;  // Limpia los datos
    LCD_DATA->PDOR |= (command & 0x0F);  // Envía los 4 bits menos significativos
    GPIOA->PSOR = EN;  // EN = 1
    delayMs(0);  // Espera 1ms
    GPIOA->PCOR = EN;  // EN = 0
    delayMs(1);  // Espera 1ms
}

void LCD_data(unsigned char data) {
    LCD_DATA->PDOR &= ~0xF;  // Limpia los datos
    LCD_DATA->PDOR |= (data & 0xF0) >> 4;  // Envía los 4 bits más significativos
    GPIOA->PSOR = RS;  // RS = 1
    GPIOA->PCOR = RW;  // RW = 0
    GPIOA->PSOR = EN;  // EN = 1
    delayMs(0);  // Espera 1ms
    GPIOA->PCOR = EN;  // EN = 0

    LCD_DATA->PDOR &= ~0xF;  // Limpia los datos
    LCD_DATA->PDOR |= (data & 0x0F);  // Envía los 4 bits menos significativos
    GPIOA->PSOR = EN;  // EN = 1
    delayMs(0);  // Espera 1ms
    GPIOA->PCOR = EN;  // EN = 0
    delayMs(1);  // Espera 1ms
}


void delayMs(int n) {
    int i;
    int j;
    for(i = 0 ; i < n; i++)
        for(j = 0 ; j < 7000; j++) {}
}


void ADC_Init(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  // Habilitar reloj para ADC0
    ADC0->CFG1 = ADC_CFG1_ADICLK(0) | ADC_CFG1_MODE(3);  // Bus clock, 16-bit mode
    ADC0->SC2 = 0; // Software trigger, default voltage reference

    // Calibrar el ADC
    ADC0->SC3 |= ADC_SC3_CAL_MASK;  // Iniciar calibración
    while (ADC0->SC3 & ADC_SC3_CAL_MASK) {
        // Esperar a que la calibración finalice
    }
}



uint32_t ADC_Read(void) {
    ADC0->SC1[0] = ADC_SC1_ADCH(26);  // Seleccionar canal 26 para sensor de temperatura
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
        // Esperar a que la conversión esté completa
    }
    return ADC0->R[0];  // Devolver el resultado de la conversión
}




float ConvertToTemperature(uint32_t adcValue) {
    // Voltaje = (valor ADC / máximo valor ADC) * VREF
    float voltage = adcValue * (VREF / 65535.0);  // Ajusta 65535 para resolución de 16 bits
    // La fórmula exacta puede variar, revisar el datasheet para detalles
    float temperature = 25.0 - ((voltage - 0.716) / 0.00162);
    return temperature;
}

//



