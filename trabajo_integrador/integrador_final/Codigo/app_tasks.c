#include "app_tasks.h"

// Cola para datos del ADC
xQueueHandle queue_adc;
// Cola para datos de luminosidad
xQueueHandle queue_lux;
// Cola para datos del display
xQueueHandle queue_display;

xQueueHandle queue_display_control;

// Cola para el valor de setpoint
xQueueHandle queue_setpoint;

// Semáforo para interrupción del infrarojo
xSemaphoreHandle semphr_buzz;
// Semáforo para contador
xSemaphoreHandle semphr_counter;
xSemaphoreHandle semphr_user;

// Handler para la tarea de display write
TaskHandle_t handle_display;

/**
 * @brief Inicializa todos los perifericos y colas
 */
void task_init(void *params) {
	// Inicializo semáforos
	semphr_buzz = xSemaphoreCreateBinary();
	semphr_user = xSemaphoreCreateBinary();
	semphr_counter = xSemaphoreCreateCounting(50, 25); //conteo maximo, conteo inicial

	// Inicializo colas
	queue_adc = xQueueCreate(1, sizeof(adc_data_t));
	queue_lux = xQueueCreate(1, sizeof(uint16_t));
	queue_setpoint = xQueueCreate(1, sizeof(uint16_t));
	queue_display = xQueueCreate(1, sizeof(uint8_t));

	queue_display_control = xQueueCreate(1, sizeof(bool));
	
	// Inicializacion de GPIO
	wrapper_gpio_init(0);
	wrapper_gpio_init(1);
	// Inicialización del LED
	wrapper_output_init((gpio_t){LED}, true);
	// Inicialización del buzzer
	wrapper_output_init((gpio_t){BUZZER}, false);
	// Inicialización del enable del CNY70
	wrapper_output_init((gpio_t){CNY70_EN}, true);
	// Configuro el ADC
	wrapper_adc_init();
	// Configuro el display
	wrapper_display_init();
	// Configuro botones
	wrapper_btn_init();
	// Inicializo el PWM
	wrapper_pwm_init();
	// Inicializo I2C y Bh1750
	wrapper_i2c_init();
	wrapper_bh1750_init();
	// Elimino tarea para liberar recursos
	vTaskDelete(NULL);

}

/**
 * @brief Activa una secuencia de conversion cada 0.25 segundos
 */
void task_adc(void *params) {

	while(1) {
		// Inicio una conversion
		ADC_DoSoftwareTriggerConvSeqA(ADC0);
		// Bloqueo la tarea por 250 ms
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

/**
 * @brief Tarea que escribe un número en el display
 */
void task_display_change(void *params) {
	bool estado= false;

	while(1) {
		xSemaphoreTake(semphr_user, portMAX_DELAY);
		xQueuePeek(queue_display_control, &estado, pdMS_TO_TICKS(100));
		bool nuevo_estado = !estado;
		xQueueOverwrite(queue_display_control, &nuevo_estado);
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

/**
 * @brief Actualiza el duty del PWM
 */
void task_pwm(void *params) {
	// Variable para guardar los datos del ADC
	adc_data_t info = {0};

	while(1) {
		// Esperar a los datos nuevos del ADC
		xQueuePeek(queue_adc, &info, portMAX_DELAY);
		// Duty potenciometro
		uint8_t duty = (100 * info.ref_raw) / 4095;
		// Duty del Ledmulticolor
		if(duty < 100 && duty > 0){ 
    		SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_0, duty, 0);
    	}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}


/**
 * @brief Lee periodicamente el valor de intensidad luminica
 */
void task_bh1750(void *params) {
	// Valor de intensidad luminica
	uint16_t lux = 0;
	while(1) {
		// Bloqueo por 160 ms (requisito)
		vTaskDelay(pdMS_TO_TICKS(200));
		// Leo luz con wrapper
		lux = wrapper_bh1750_read();
		uint16_t TERMporcentaje = lux * 100 / 30000; //porcentaje
		// Muestro por consola
		xQueueOverwrite(queue_lux, &TERMporcentaje);
	}
}

/**
 * @brief Lee los valores de los botones para definir que valor mostrar
 */
void task_buzzer(void *params) {

	while(1){
		
		if(!GPIO_PinRead(CNY70)) {
			vTaskDelay(pdMS_TO_TICKS(50));
			if(GPIO_PinRead(CNY70)) {
				// Flanco ascendente
				wrapper_output_toggle((gpio_t){BUZZER});
			}
		} else {
			vTaskDelay(pdMS_TO_TICKS(50));
			if(!GPIO_PinRead(CNY70)) {
				// Flanco descendente
				wrapper_output_toggle((gpio_t){BUZZER});
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

/**
 * @brief Tarea que decrementa el contador
 */
 void task_display(void *params) {
	// Variable con el dato para escribir
	uint16_t info;
	bool control = false;
	while(1) {
		// Mira el dato que haya en la cola
		xQueuePeek(queue_display_control, &control, pdMS_TO_TICKS(100));
		if(control){
			xQueuePeek(queue_setpoint, &info, pdMS_TO_TICKS(100));
		} 
		else {
			xQueuePeek(queue_lux, &info, pdMS_TO_TICKS(100));}
		// Muestro el número
		wrapper_display_off();
		wrapper_display_write((uint8_t)(info / 10), control);
		wrapper_display_on((gpio_t){COM_1});
		vTaskDelay(pdMS_TO_TICKS(10));
		wrapper_display_off();
		wrapper_display_write((uint8_t)(info % 10), control);
		wrapper_display_on((gpio_t){COM_2});
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}



/**
 * @brief Tarea que manualmente controla el contador
 */
void task_counter_btns(void *params) {

	while(1) {
		if(wrapper_btn_get_with_debouncing_with_pull_up((gpio_t){S1_BTN})) {
			// Decrementa
			xSemaphoreTake(semphr_counter, 0);	
		}
		else if(wrapper_btn_get_with_debouncing_with_pull_up((gpio_t){S2_BTN})) {
			// Incrementa 
			xSemaphoreGive(semphr_counter);
		}
		// Escribe en el display
		uint16_t info = uxSemaphoreGetCount(semphr_counter) + 25;
		xQueueOverwrite(queue_setpoint, &info);
		
		vTaskDelay(pdMS_TO_TICKS(100));

	}
}

// Las nuevas tareas:
void task_user(void *params) {

	while(1){
		
		if(!GPIO_PinRead(USR_BTN)) {
			vTaskDelay(pdMS_TO_TICKS(50));
			if(GPIO_PinRead(USR_BTN)) {
				// Flanco ascendente
				xSemaphoreGive(semphr_user);
			}
		} else {continue;}

		vTaskDelay(pdMS_TO_TICKS(10));
	}

}


void task_ledmulti(void *params) {
	// Variable para guardar los datos del ADC
	uint16_t info;
	uint16_t setpoint;

	while(1) {
		// Bloqueo hasta que haya algo que leer
		xQueuePeek(queue_lux, &info, portMAX_DELAY);
		xQueuePeek(queue_setpoint, &setpoint, portMAX_DELAY);
		int16_t err = info - setpoint;
		// Actualizo el duty
		if(err >0) {
			// para valores mayores LED rojo
			wrapper_pwm_update_bled(0);
			wrapper_pwm_update_rled(err);
		}
		else {
			//para valores menores LED azul
			wrapper_pwm_update_rled(0);
			wrapper_pwm_update_bled(-err);
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}


void task_terminal(void *params){
	uint8_t porcentaje = 0;
	adc_data_t info = {0};
	uint16_t control = 0;
	while(1){
		xQueuePeek(queue_setpoint, &control, pdMS_TO_TICKS(100));
		xQueuePeek(queue_lux, &porcentaje, portMAX_DELAY);
		xQueuePeek(queue_adc, &info, portMAX_DELAY);
		TickType_t tiempoActual = xTaskGetTickCount();
		uint32_t tiempo_ms = tiempoActual * 1000;
		uint16_t duty = 100- (100.0 * info.ref_raw) / 4095.0;
		// print
		PRINTF("Cantiadad de ticks: %ld ms\n", tiempo_ms);
		PRINTF("Porcentaje de luz: %d\n", porcentaje);
		PRINTF("Porcentaje de setpoint: %d\n", control);
		PRINTF("Brillo del LED D1: %d\n", duty);
		// delay
		vTaskDelay(500);
	}

}