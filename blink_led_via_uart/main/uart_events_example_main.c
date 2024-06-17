/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/


#define LF (0x0A)
#define CR (0x0D)
#define CMD_SIZE (32)
#define RESP_SIZE (32)
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
static QueueHandle_t command_queue;
static QueueHandle_t led_command_queue;

typedef struct {
    char ch_arr_command[CMD_SIZE];
}command_struct_t;


/*
"0"Off
"1"On
"2"Blink state extra slow (0.5Hz)
"3"Blink state slow (1Hz)
"4"Blink state fast (2Hz)
"5"Blink state extra fast (4Hz)
*/

typedef enum{
    OFF =               (int)'0',
    ON =                (int)'1',
    BLINK_SLOW =        (int)'2',
    BLINK_NORMAL =      (int)'3',
    BLINK_FAST =        (int)'4',
    BLINK_EXTRA_FAST =  (int)'5',
    NO_CHANGE =   (int)'X'
}led_state_t;

// Define the blink formats. Minimum time frame is 125ms
#define BLINK_MASK_OFF          ((led_shift_register_t)(0x0000))
#define BLINK_MASK_ON           ((led_shift_register_t)(0xFFFF))
#define BLINK_MASK_SLOW         ((led_shift_register_t)(0xFF00))
#define BLINK_MASK_NORMAL       ((led_shift_register_t)(0xF0F0))
#define BLINK_MASK_FAST         ((led_shift_register_t)(0xCCCC))
#define BLINK_MASK_EXTRA_FAST   ((led_shift_register_t)(0xAAAA))


typedef enum{
    UNKNOWN_COMMAND,
    LED_COMMAND,
    STATUS_REQUEST_COMMAND
}command_type_t;

#define LED_COUNT 8
typedef uint16_t led_shift_register_t;

led_shift_register_t global_led_state[LED_COUNT];


command_type_t get_command_type(const command_struct_t* p_command){
    command_type_t command_type = UNKNOWN_COMMAND;
    if(p_command->ch_arr_command[0] == 'Z' && p_command->ch_arr_command[1] == ' '){
        command_type = LED_COMMAND;
    }else if(p_command->ch_arr_command[0] == 'S'){
        command_type = STATUS_REQUEST_COMMAND;
    }else if(p_command->ch_arr_command[0] == 'R'){

    }
    return command_type;
}

int create_answer_string(char* p_answer, const led_shift_register_t* p_led_state){
    int pos = 2;
    sprintf(p_answer, "z ");
    for(int i = 0; i < LED_COUNT; i++){
        if(p_led_state[i] == BLINK_MASK_OFF){
            p_answer[pos++] = (char) OFF;
        }else if(p_led_state[i] == BLINK_MASK_ON){
            p_answer[pos++] = (char) ON;
        }else if(p_led_state[i] == BLINK_MASK_SLOW){
            p_answer[pos++] = (char) BLINK_SLOW;
        }else if(p_led_state[i] == BLINK_MASK_NORMAL){
            p_answer[pos++] = (char) BLINK_NORMAL;
        }else if(p_led_state[i] == BLINK_MASK_FAST){
            p_answer[pos++] = (char) BLINK_FAST;
        }else if(p_led_state[i] == BLINK_MASK_EXTRA_FAST){
            p_answer[pos++] = (char) BLINK_EXTRA_FAST;
        }else{
            p_answer[pos++] = (char) 'E';
        }
    }
    return pos;
}

static void cyclic_blink_task(void *pvParameters){
    ESP_LOGI(TAG, "Cyclic blink task started");
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 0x00FF;
        io_conf.pull_down_en = 1;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);
    }
    uint16_t active_bit = 0x0001;
    for(;;){
        for(int i = 0; i < LED_COUNT; i+=1){
            if((global_led_state[i] & active_bit) == active_bit){
                gpio_set_level(i, 1);
            }else{
                gpio_set_level(i, 0);
            }
        }
        if (active_bit == 0x8000){
            active_bit = 0x0001;
        }else{
            active_bit = active_bit << 1;
        }
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}


static void led_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED control task started");
    command_struct_t command;
    for (;;) {
        if (xQueueReceive(led_command_queue, &command, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Command received LED Control task: %s\nSetting LED states", command.ch_arr_command);
            // Command has the following format: Z X12X34 
            // X means no change, every other number is interpreted as LED state
            // Write into global state memory
            for (int i = 2; i < CMD_SIZE; i++) {
                if (command.ch_arr_command[i] == 0) {
                    break;
                }
                switch (command.ch_arr_command[i]) {
                    case OFF:
                        // Clear the bit
                        global_led_state[i-2] = BLINK_MASK_OFF;
                        break;
                    case ON:
                        // Set the bit
                        global_led_state[i-2] = BLINK_MASK_ON;
                        break;
                    case BLINK_SLOW:
                        // Set the bit and set the blink mask
                        global_led_state[i-2] = BLINK_MASK_SLOW;
                        break;
                    case BLINK_NORMAL:
                        // Set the bit and set the blink mask
                        global_led_state[i-2] = BLINK_MASK_NORMAL;
                        break;
                    case BLINK_FAST:
                        // Set the bit and set the blink mask
                        global_led_state[i-2] = BLINK_MASK_FAST;
                        break;
                    case BLINK_EXTRA_FAST:
                        // Set the bit and set the blink mask
                        global_led_state[i-2] = BLINK_MASK_EXTRA_FAST;
                        break;
                    case NO_CHANGE:
                        // Do nothing
                        break;
                    default:
                        ESP_LOGE(TAG, "Unknown LED state received: %c", command.ch_arr_command[i]);
                        break;
                }
            }
            char answer[RESP_SIZE] = { 0 };
            create_answer_string(answer, global_led_state);
            // NOt implemented, but maybe instead of writing directly to uart, create a 
            // Queue and send all commands in queue so we dont have to worry about accessing the driver
            uart_write_bytes(EX_UART_NUM, answer, strlen(answer));
        }
    }
    vTaskDelete(NULL);
}

static void command_event_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Command event task started");
    command_struct_t command;
    for (;;) {
        if (xQueueReceive(command_queue, &command, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Command received in task: %s", command.ch_arr_command);
            command_type_t cmd_type = get_command_type(&command);
            if(cmd_type == LED_COMMAND){
                xQueueSend(led_command_queue, &command, 0);
            }else if(cmd_type == STATUS_REQUEST_COMMAND){
                for(int i = 0; i < LED_COUNT; i++){
                    ESP_LOGI(TAG, "LED %d: 0x%X", i, global_led_state[i]);
                }
            }
            else{
                ESP_LOGE(TAG, "Unknown command received: %s", command.ch_arr_command);
            }
        }
    }
    vTaskDelete(NULL);
}


static void uart_event_task(void *pvParameters)
{
    ESP_LOGI(TAG, "uart_event_task started");
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    command_struct_t cmd_from_uart;
    int pos = 0;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            switch (event.type) {
            //Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                // We have too long command detected. Ignore and send failure
                if (else if(pos >= CMD_SIZE){
                    // Maybe add to queue
                    const char error_msg[] = "Command too long, ignoring"
                    ESP_LOGE(TAG, error_msg);
                    pos = 0;
                    uart_write_bytes(EX_UART_NUM, error_msg , strlen(error_msg));
                    memset(cmd_from_uart.ch_arr_command, 0, CMD_SIZE);
                } 

                // Create command string, add chars till CR is received
                for (int i = 0; i < event.size; i++) {
                    if (dtmp[i] == CR) {
                        cmd_from_uart.ch_arr_command[pos] = 0;
                        ESP_LOGI(TAG, "Command: %s", cmd_from_uart.ch_arr_command);
                        xQueueSend(command_queue, &cmd_from_uart, 0);
                        pos = 0;
                        memset(cmd_from_uart.ch_arr_command, 0, CMD_SIZE);
                    }
                    else {
                        cmd_from_uart.ch_arr_command[pos++] = dtmp[i];
                    }
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    command_queue = xQueueCreate(10, sizeof(command_struct_t));
    if (command_queue == NULL) {
        ESP_LOGE(TAG, "Error creating command queue");
        assert(0);
    }
    led_command_queue = xQueueCreate(10, sizeof(command_struct_t));
    if (led_command_queue == NULL) {
        ESP_LOGE(TAG, "Error creating led command queue");
        assert(0);
    }

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(command_event_task, "command_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 12, NULL);
    xTaskCreate(cyclic_blink_task, "cyclic_blink_task", 2048, NULL, 12, NULL);
        
}
