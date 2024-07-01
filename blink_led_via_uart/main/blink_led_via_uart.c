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

#define UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define LF (0x0A)
#define CR (0x0D)
#define CMD_SIZE (32)
#define RESP_SIZE (32)
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define SW_TIME_MS (125) // Switch time in ms. Approx time cyclic task suspends LED switching.
#define SW_TIME_MIN_MS (80) // Min switch time in ms.
#define OUTPUT_BIT_MASK (0x00FF)

static QueueHandle_t uart0_queue;
static QueueHandle_t command_queue;

typedef enum{
    CMD_UNKNOWN,
    CMD_LED_STATE_SET   = (int)'Z',
    CMD_STATE_REQ       = (int)'S',
    CMD_SET_BLINK_MS    = (int)'B'  // Blink period in ms 
}command_type_t;

typedef struct {
    command_type_t type;
    char ch_arr_command[CMD_SIZE];
}command_struct_t;


/*
"0"Off
"1"On
"2"Blink state extra slow 
"3"Blink state slow 
"4"Blink state fast 
"5"Blink state extra fast 
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
#define BLINK_MASK_UNDEFINED    ((led_shift_register_t)(0xDEAD))


#define LED_COUNT 8
typedef uint16_t led_shift_register_t;

led_shift_register_t global_led_state[LED_COUNT];

static command_type_t parse_command(command_struct_t* p_command){
    command_type_t command_type = CMD_UNKNOWN;
    if(p_command->ch_arr_command[0] == (char)CMD_LED_STATE_SET){
        command_type = CMD_LED_STATE_SET;
    }else if(p_command->ch_arr_command[0] == (char)CMD_STATE_REQ){
        command_type = CMD_STATE_REQ;
    }else if(p_command->ch_arr_command[0] == (char)CMD_SET_BLINK_MS){
        command_type = CMD_SET_BLINK_MS;
    }
    p_command->type = command_type;
    // Shift command string 2 to the left. Copies everything, does not check for null termination.
    for(int i = 0; i < CMD_SIZE - 2; i += 1){
        p_command->ch_arr_command[i] = p_command->ch_arr_command[i + 2];
    }
    return command_type;
}

int create_led_answer_string(char* p_answer, const led_shift_register_t* p_led_state){
    int pos = 2;
    sprintf(p_answer, "z ");
    for(int i = 0; i < LED_COUNT; i += 1){
        if(p_led_state[i] == BLINK_MASK_OFF){
            p_answer[pos] = (char) OFF;
        }else if(p_led_state[i] == BLINK_MASK_ON){
            p_answer[pos] = (char) ON;
        }else if(p_led_state[i] == BLINK_MASK_SLOW){
            p_answer[pos] = (char) BLINK_SLOW;
        }else if(p_led_state[i] == BLINK_MASK_NORMAL){
            p_answer[pos] = (char) BLINK_NORMAL;
        }else if(p_led_state[i] == BLINK_MASK_FAST){
            p_answer[pos] = (char) BLINK_FAST;
        }else if(p_led_state[i] == BLINK_MASK_EXTRA_FAST){
            p_answer[pos] = (char) BLINK_EXTRA_FAST;
        }else{
            p_answer[pos] = (char) 'E'; // Add 'E' for unknown error
        }
        pos += 1;
    }
    p_answer[pos] = CR;
    pos += 1;
    p_answer[pos] = LF;
    pos += 1;
    p_answer[pos] = 0;
    pos += 1;
    return pos;
}

static void send_actual_led_state_to_uart(){
    char answer[RESP_SIZE] = { 0 };
    create_led_answer_string(answer, global_led_state);
    // Maybe instead of writing directly to uart, create a 
    // Queue and send all commands in queue so we dont have to worry about accessing the driver
    uart_write_bytes(UART_NUM, answer, strlen(answer));
}

static void send_bp_to_uart(uint32_t blink_period_ms){
    char answer[RESP_SIZE] = { 0 };
    sprintf(answer, "b %ld\r\n", blink_period_ms);
    uart_write_bytes(UART_NUM, answer, strlen(answer));
}

static led_shift_register_t set_global_led_state(char led_state, int led_number){
    led_shift_register_t mask = BLINK_MASK_UNDEFINED;
    switch (led_state) {
        case OFF:
            mask = BLINK_MASK_OFF;
            break;
        case ON:
            mask = BLINK_MASK_ON;
            break;
        case BLINK_SLOW:
            mask = BLINK_MASK_SLOW;
            break;
        case BLINK_NORMAL:
            mask = BLINK_MASK_NORMAL;
            break;
        case BLINK_FAST:
            mask = BLINK_MASK_FAST;
            break;
        case BLINK_EXTRA_FAST:
            mask = BLINK_MASK_EXTRA_FAST;
            break;
        case NO_CHANGE:
            // Do nothing
            break;
        default:
            break;
    }
    if (BLINK_MASK_UNDEFINED != mask){
        global_led_state[led_number] = mask;
    }
    return mask;
}

static void cyclic_blink_task(void *pvParameters){
    ESP_LOGI(TAG, "Cyclic blink task started");
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = OUTPUT_BIT_MASK; // set the pins 0-15 to output
        io_conf.pull_down_en = 1;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);
    }
    uint16_t active_bit = 0x0001;
    uint32_t blink_period_ms = SW_TIME_MS;
    command_struct_t command;
    for(;;){
        if (xQueuePeek(command_queue, &command, 0) == pdPASS){
            if (command.type == CMD_SET_BLINK_MS){
                xQueueReceive(command_queue, &command, 0);
            }
            // Interpret command string as blink period
            blink_period_ms = atoi(command.ch_arr_command);
            if (blink_period_ms < SW_TIME_MIN_MS){
                blink_period_ms = SW_TIME_MIN_MS;
            }
            send_bp_to_uart(blink_period_ms);
        }

        for(int i = 0; i < LED_COUNT; i+=1){
            // if((global_led_state[i] & active_bit) == active_bit){ // Check before deletion
            if(global_led_state[i] & active_bit){
                gpio_set_level(i, 1);
            }else{
                gpio_set_level(i, 0);
            }
        }
        if (active_bit == 0x8000){
            // Overflow, reset to first bit
            active_bit = 0x0001;
        }else{
            active_bit = active_bit << 1;
        }
        vTaskDelay(blink_period_ms / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void led_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED control task started");
    command_struct_t command;
    for (;;) {
        if (xQueuePeek(command_queue, &command, portMAX_DELAY)){
            if (command.type == CMD_LED_STATE_SET || command.type == CMD_STATE_REQ ){
                xQueueReceive(command_queue, &command, portMAX_DELAY);
            }else{
                continue;
            }
            ESP_LOGI(TAG, "Command received LED Control task: %s\nSetting LED states", command.ch_arr_command);

            if(command.type == CMD_LED_STATE_SET){
                // Command has the following format: Z X12X34 
                // X means no change, every other number is interpreted as LED state
                // Write into global state memory
                for (int i = 0; i < CMD_SIZE; i += 1) {
                    set_global_led_state(command.ch_arr_command[i], i);
                }
                send_actual_led_state_to_uart();
            }else if(command.type == CMD_STATE_REQ){
                send_actual_led_state_to_uart();
            }
        }
    }   
    vTaskDelete(NULL);
}

static void uart_event_task(void *pvParameters)
{
    ESP_LOGI(TAG, "uart_event_task started");
    uart_event_t event;
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
                uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                
                // Create command string, add chars till CR is received
                for (int i = 0; i < event.size; i += 1) {
                    // We have detected a too long command. Ignore and send failure
                    if (pos >= CMD_SIZE){
                        ESP_LOGE(TAG, "Error detected: Command too long, ignoring");
                        pos = 0;
                        const char error_msg[] = "Command too long, ignoring";
                        uart_write_bytes(UART_NUM, error_msg , strlen(error_msg));
                        memset(cmd_from_uart.ch_arr_command, 0, CMD_SIZE);
                        break;
                    } else if (dtmp[i] == CR) {
                        cmd_from_uart.ch_arr_command[pos] = 0; // Null terminate string
                        ESP_LOGI(TAG, "Command: %s", cmd_from_uart.ch_arr_command);
                        parse_command(&cmd_from_uart); 
                        if (cmd_from_uart.type == CMD_UNKNOWN){
                            ESP_LOGE(TAG, "Error detected: Unknown command, ignoring");
                        }else{
                            xQueueSend(command_queue, &cmd_from_uart, 0);
                        }
                        pos = 0;
                        memset(cmd_from_uart.ch_arr_command, 0, CMD_SIZE);
                    }
                    else {
                        cmd_from_uart.ch_arr_command[pos] = dtmp[i];
                        pos += 1;
                    }
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM);
                xQueueReset(uart0_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM);
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
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_NUM, 20);

    command_queue = xQueueCreate(10, sizeof(command_struct_t));
    if (command_queue == NULL) {
        ESP_LOGE(TAG, "Error creating command queue");
        assert(0);
    }

    // Init LEDs with slow blink to message that the system is ready
    for(int i = 0; i < LED_COUNT; i += 1){
        global_led_state[i] = BLINK_MASK_SLOW;
    }

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 12, NULL);
    xTaskCreate(cyclic_blink_task, "cyclic_blink_task", 2048, NULL, 12, NULL);
        
}
