/*
******************************************************************************
* @file main.c
* @designers Cristian Gabriel Escude - Nicolas Emiliano Ivani - ME1
* @board STM32F407G-DISCOVERY
* @version 1.2
* @date 11-Noviembre-2021
*
* Este programa realiza un muestreo mediante dos canales del ADC1.
* Se configura el DMA para guardar los valores del ADC automáticamente.
* También, se configuran dos Timers. El primero controla los tiempos del ADC
para convertir
* los valores analógicos a discretos, con una frecuencia de 5kHz. Y el
segundo es el encargado
* de activar el DMA cada 2 segundos. Cuando el DMA termina de obtener los
valores, se calculan las potencias
* y los valores eficaces tanto de tensión como de corriente. Luego se
procede a mostrar dichos valores en el display LCD.
* Y luego, se esperan dos segundos para volver a repetir todo el proceso.
*
* PINES UTILIZADO PARA EL LCD
* PE5 - RS
* PE6 - E
* PE7 - D4
* PE8 - D5
* PE9 - D6
* PE10 - D7
*
* PIN UTILIZADO PARA EL BACKLIGHT
* PD11
*
* PINES UTILIZADO PARA EL ADC 1
* PA1
* PC5
*
* PIN PARA PULSADOR
* PC9
*
******************************************************************************

/* Includes */
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_spi.h"
#include "stdio.h"
#include "strings.h"
#include "stdlib.h"
#include "math.h"
#include "stm32_ub_lcd_2x16.h"
/*
 * Frecuencia Analógica = 50 Hz.
 * Frecuencia Muestreo = 5 kHz.
 */
#define FactorTrafo 30.4 // N = 228V/7.5V; Factor de transformación del transformador
#define FactorAtenuacion 7.5 // 1/(RV2/R13) = 33k/4.9k (La atenuación producida por el AO-U1:A)
#define R_Toroide 27 // Impedancia colocada en paralelo con el Toroide
#define FactorToroide 333.33    // N= 10A / (10mA * 3 vueltas) 
                                //Factor de transformación del Toroide
#define F 0.01 // Frecuencia de muestras --> T =50Hz / 5kHz = 0.01 --> 1 / 100 muestras.
void Inicializar_Puertos (void); //Configuración e Inicialización de los
                                 //pines PC5 y PA1 como entradas para el ADC 
void Inicializar_ADC(void); // Inicialización del ADC y Timer
void TIM_Config(void);      // Inicialización del Timer
void DMA_Config(void);      // Configuración del DMA
void Calcular_Potencias(void);
void Discriminar_Datos(void);
void Imprimir_LCD(void);
uint16_t Decimal(float_t Var);
float_t ValorEficaz(float_t vect[]);
float_t PotenciaActiva_P(float_t vect_V[], float_t vect_I[]);
typedef struct
{
    float_t Aparente;
    float_t Activa;
    float_t Reactiva;
    float_t Phi;
} Potencia;
char fila1[16] = "";
char fila2[16] = "";
uint16_t PrescalerValue_Tim2 = 0;
uint16_t PrescalerValue_Tim3 = 0;
uint16_t mediciones[1000]; // Arreglo para guardar los datos del ADC, 500 elementos para tensión y 500 para corriente
uint16_t i = 0;
uint16_t j = 0;
uint16_t flag_DMA = 0;
uint16_t decimaltension = 0;
uint16_t decimalcorriente = 0;
uint32_t delay = 0;
float_t Vrms = 0;       // Valor eficaz de Tensión
float_t Irms = 0;       // Valor eficaz de Corriente
Potencia P;             // Variable de tipo Potencia. Estructura para guardar los datos calculados.
float_t voltaje[500];   // Arreglo para las mediciones de tensión
float_t corriente[500]; // Arreglo para las mediciones de corriente
uint16_t flag_pulsador = 0;
uint8_t flag_pulsador_selec = 0;
int main(void)
{
    SystemInit();
    // Inicialización Display
    UB_LCD_2x16_Init();
    UB_LCD_2x16_Clear();
    // Inicialización de los puertos PA1, PC5, PC9 y PD11
    Inicializar_Puertos();
    // Se enciende el BackLight del LCD
    GPIO_SetBits(GPIOD, GPIO_Pin_11);
    // Configuración de los Timers
    TIM_Config();
    // Configuración del DMA
    DMA_Config();
    // Inicialización del ADC1
    Inicializar_ADC();
    while (1)
    {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) && flag_pulsador == 1) // Verifico si el pulsador está activo
            {
                delay = 200000; // Si el pulsador está activo, se espera un tiempo para evitar el rebote
                while (delay)
                {
                    delay--;
                }
                if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)) // Si sigue activo después del retardo se modifican los flags
                {
                    flag_pulsador_selec++;
                    if (flag_pulsador_selec == 3)
                    {
                        flag_pulsador_selec = 0;
                    }
                    flag_pulsador = 0;
                }
            }
    }
}
void Inicializar_Puertos(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // Habilitación de la señal de reloj para el periférico GPIOA
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // PA1 ADC para tensión
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct); // Se aplica la configuración definida anteriormente al puerto A
    // Habilitación de la señal de reloj para el periférico GPIOC
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // PC5 ADC para corriente
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct); // Se aplica la configuración definida anteriormente al puerto C
                                        // PC9 Pulsador
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct); // Se aplica la configuración definida  anteriormente al puerto C
    // Habilitación de la señal de reloj para el periférico GPIOD
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    // Ahora se configura el pin PD11 (backlight del LCD)
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // Salida
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStruct); // Se aplica la configuración definida anteriormente al puerto D
}
void Inicializar_ADC(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; // Para habilitar pedidos de DMA
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    // Scan Mode significa que se van a convertir varios canales simultáneamente
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    // Modo continuo es para que el ADC se dispare constantemente sin control, se desactiva para dispararlo solo con el TIM2
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; // Esto define con que se dispara el ADC, en este caso es 
                                                                            //el TMRGO(timer go) del TIMER 2 
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 2;
    // Cantidad de datos que se van a convertir cada vez que se dispara el ADC. Son dos, uno por tensión y otro por corriente
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    // Habilitar requerimiento de DMA luego de cada conversión
    ADC_DMACmd(ADC1, ENABLE); // Se activa ADC1 con DMA
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1,
                             ADC_SampleTime_480Cycles); // Entrada PA1, canal 1 del ADC1, para tensión
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2,
                             ADC_SampleTime_480Cycles); // Entrada PC5, canal 15 del ADC1, para corriente
    ADC_Cmd(ADC1, ENABLE);
}
void TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    NVIC_InitTypeDef NVIC_InitStruct_TIM3;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    // Configuración del TIM2 para el muestreo
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 40; // 5kHz
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    PrescalerValue_Tim2 = (uint16_t)(249);
    TIM_PrescalerConfig(TIM2, PrescalerValue_Tim2, TIM_PSCReloadMode_Immediate);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
    TIM_Cmd(TIM2, DISABLE);
    // Configuración del TIM3 para el intervalo de espera entre muestreos
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStruct.TIM_Period = 5000; // 0,5Hz
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
    PrescalerValue_Tim3 = (uint16_t)(9999);
    TIM_PrescalerConfig(TIM3, PrescalerValue_Tim3, TIM_PSCReloadMode_Immediate);
    // Se configura la interrupción para el TIM3
    NVIC_InitStruct_TIM3.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct_TIM3.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct_TIM3.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStruct_TIM3);
    TIM_Cmd(TIM3, ENABLE);
}
void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStruct_DMA;
    DMA_StructInit(&DMA_InitStructure);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    // Registro donde se guardan los valores convertidos por el ADC
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&mediciones[0];
    // Dirección del arreglo donde se guardan las muestras
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    // El DMA transmite de periférico a memoria
    DMA_InitStructure.DMA_BufferSize = 1000;
    // Cantidad de valores que se van a convertir
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    // Modo Circular: cuando se llega al final del arreglo vuelva a arrancar desde el primer valor(sobreescribiendo)
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure); // Se inicializa el DMA2
    Stream0
        DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
    NVIC_InitStruct_DMA.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStruct_DMA.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct_DMA.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStruct_DMA);
    DMA_Cmd(DMA2_Stream0, DISABLE); // Se desactiva el DMA2 - Stream 0
}
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
        // Cuando se produce la interrupción del DMA se desactiva el Timer 2 para muestreo
            TIM_Cmd(TIM2, DISABLE);
        // Cuando se produce la interrupción del DMA se desactiva el DMA
        DMA_Cmd(DMA2_Stream0, DISABLE);
        // Del arreglo de mil muestras se separan las muestras de tensión y las de corriente en dos arreglos distintos
        Discriminar_Datos();
        // Con los nuevos arreglos se calculan las potencias y los valores eficaces de tensión y corriente
        Calcular_Potencias();
        // Se imprimen los parámetros en el display LCD
        Imprimir_LCD();
        // Comienza el conteo para el intervalo entre muestras
        TIM_Cmd(TIM3, ENABLE);
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    }
}
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1))
    {
        // Cuando termina el intervalo entre muestras se vuelven a activar los periféricos
        TIM_Cmd(TIM2, ENABLE);
        DMA_Cmd(DMA2_Stream0, ENABLE); // Enable the DMA2 - Stream 0
        TIM_Cmd(TIM3, DISABLE);
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    }
}
void Calcular_Potencias(void)
{
    Vrms = ValorEficaz(voltaje);
    Irms = ValorEficaz(corriente);
    P.Aparente = Vrms * Irms;
    P.Activa = PotenciaActiva_P(voltaje, corriente);
    P.Phi = (P.Activa / P.Aparente);
    P.Reactiva = (sqrt(pow(P.Aparente, 2) - (pow(P.Activa, 2))));
}
void Discriminar_Datos(void)
{
    int32_t k = 0;
    float_t aux_v = 0;
    float_t aux_i = 0;
    for (k = 0; k < 1000; k++)
    {
        if (k % 2 == 0) // Si la posición es par, el valor es de tensión
        {
            aux_v = (float_t)((mediciones[k] * 3.0 / 4095)); //Convierto las cuentas de tensión del ADC a valores en Volts
                voltaje[k / 2] = (aux_v - 1.41) * FactorTrafo *
                                 FactorAtenuacion; // Calculo la tensión equivalente al valor convertido
        }
        else
        {
            aux_i = (((float_t)mediciones[k]) * 3.0 / 4095); //Convierto las cuentas de corriente del ADC a valores en Volts
                corriente[((k - 1) / 2)] = (aux_i - 1.41) * FactorToroide /
                                           R_Toroide / 3 * 80; // Calculo la corriente equivalente al valor convertido
        }
    }
}
float_t ValorEficaz(float_t arr[])
{
    int16_t n = 0;
    float_t aux = 0;
    float_t val_eficaz = 0;
    for (n = 0; n < 500; n++)
    {
        aux += pow(arr[n], 2);
        switch (n)
        {
        case 99: // Calculo el valor eficaz del primer periodo muestreado
        {
            val_eficaz += sqrt(F * aux);
            aux = 0;
            break;
        }
        case 199: // Calculo el valor eficaz del segundo periodo muestreado
        {
            val_eficaz += sqrt(F * aux);
            aux = 0;
            break;
        }
        case 299: // Calculo el valor eficaz del tercer periodo muestreado
        {
            val_eficaz += sqrt(F * aux);
            aux = 0;
            break;
        }
        case 399: // Calculo el valor eficaz del cuarto periodo muestreado
        {
            val_eficaz += sqrt(F * aux);
            aux = 0;
            break;
        }
        case 499: // Calculo el valor eficaz del quinto periodo muestreado
        {
            val_eficaz += sqrt(F * aux);
            aux = 0;
            break;
        }
        default:
            break;
        }
    }
    val_eficaz = val_eficaz / 5; // Calculo el promedio del valor eficaz de los cinco periodos muestreados 
    return val_eficaz;
}
float_t PotenciaActiva_P(float_t arr_V[], float_t arr_I[])
{
    int16_t n = 0;
    float_t aux = 0;
    float_t pot_aux = 0;
    for (n = 0; n < 500; n++)
    {
        aux += arr_V[n] * arr_I[n];
        switch (n)
        {
        case 99:
        {
            pot_aux += aux * F;
            aux = 0;
        }
        break;
        case 199:
        {
            pot_aux += aux * F;
            aux = 0;
        }
        break;
        case 299:
        {
            pot_aux += aux * F;
            aux = 0;
        }
        break;
        case 399:
        {
            pot_aux += aux * F;
            aux = 0;
        }
        break;
        case 499:
        {
            pot_aux += aux * F;
            aux = 0;
        }
        break;
        default:
            break;
        }
    }
    pot_aux = pot_aux / 5;
    pot_aux = abs(pot_aux);
    return pot_aux;
}
void Imprimir_LCD(void)
{
    UB_LCD_2x16_Clear();
    if (flag_pulsador_selec == 0)
    {
        sprintf(fila1, "Vrms: %.2lf", Vrms);
        sprintf(fila2, "Irms: %.4lf", Irms);
        // Primera línea del LCD.
        UB_LCD_2x16_String(0, 0, fila1);
        UB_LCD_2x16_String(14, 0, "V");
        // Segunda línea del LCD.
        UB_LCD_2x16_String(0, 1, fila2);
        UB_LCD_2x16_String(14, 1, "A");
        flag_pulsador = 1;
    }
    else if (flag_pulsador_selec == 1)
    {
        sprintf(fila1, "P: %.2lf", P.Activa);
        sprintf(fila2, "Q: %.2lf", P.Reactiva);
        // Primera línea del LCD.
        UB_LCD_2x16_String(0, 0, fila1);
        UB_LCD_2x16_String(13, 0, "W");
        // Segunda línea del LCD.
        UB_LCD_2x16_String(0, 1, fila2);
        UB_LCD_2x16_String(12, 1, "VAr");
        flag_pulsador = 1;
    }
    else
    {
        sprintf(fila1, "Ps: %.2lf", P.Aparente);
        sprintf(fila2, "cos(phi): %.2lf", P.Phi);
        // Primera línea del LCD.
        UB_LCD_2x16_String(0, 0, fila1);
        UB_LCD_2x16_String(13, 0, "VA");
        // Segunda línea del LCD.
        UB_LCD_2x16_String(0, 1, fila2);
        flag_pulsador = 1;
    }
}