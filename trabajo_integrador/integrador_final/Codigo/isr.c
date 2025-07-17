#include "isr.h"

/**
 * @brief Callback para interrupción por flancos del infrarojo
 * @param pintr numero de interrupción
 */



/**
 * @brief Handler para la interrupcion del ADC Sequence A
 */
void ADC0_SEQA_IRQHandler(void) {
	// Variable de cambio de contexto
	int32_t higher_task = 0;
	// Verifico que se haya terminado la conversion correctamente
	if(kADC_ConvSeqAInterruptFlag == (kADC_ConvSeqAInterruptFlag & ADC_GetStatusFlags(ADC0))) {
		// Limpio flag de interrupcion
		ADC_ClearStatusFlags(ADC0, kADC_ConvSeqAInterruptFlag);
		// Resultado de conversion
		adc_result_info_t temp_info, ref_info;
		// Leo el valor del ADC
		ADC_GetChannelConversionResult(ADC0, REF_POT_CH, &ref_info);
		// Leo el valor del ADC
		ADC_GetChannelConversionResult(ADC0, LM35_CH, &temp_info);
		// Estructura de datos para mandar
		adc_data_t data = {
			.temp_raw = (uint16_t) 4095 - temp_info.result,
			.ref_raw = (uint16_t) 4095 - ref_info.result
		};
		// Mando por la cola los datos
		xQueueOverwriteFromISR(queue_adc, &data, &higher_task);
		// Veo si hace falta un cambio de contexto
		portYIELD_FROM_ISR(higher_task);
	}
}

