#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include "driver/periph_ctrl.h"
#include "freertos/portmacro.h"
#include "driver/periph_ctrl.h"
#include "soc/rtc_wdt.h"



#define latchPin  25  // Latch pin (STCP腳位)
#define latchPin1  26  // Latch pin (STCP腳位)
#define latchPin2  16  // Latch pin (STCP腳位)
#define latchPin3  17  // Latch pin (STCP腳位)
#define clockPin  18 // Clock pin (SHCP腳位)
#define dataPin  19  // Data pin (DS腳位)
#define dataPin1  21  // Data pin (DS腳位)
#define dataPin2  22  // Data pin (DS腳位)
#define dataPin3  23  // Data pin (DS腳位)
#define DelayTime 0.001
#define TIMER_DIVIDER         16
#define TEST_WITHOUT_RELOAD   0
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<latchPin) | (1ULL<<latchPin1)| (1ULL<<latchPin2)| (1ULL<<latchPin3)| (1ULL<<clockPin)| (1ULL<<dataPin)| (1ULL<<dataPin1)| (1ULL<<dataPin2)| (1ULL<<dataPin3))
#define GPIO_ISR_CORE_ID_UNINIT    (3)
#define GPIO_HAL_GET_HW(num) GPIO_LL_GET_HW(num)
#define GPIO_LL_GET_HW(num) (((num) == 0) ? (&GPIO) : NULL)
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
int temp;
double duration; 
void my_gpio_set_level(gpio_num_t gpio_num, uint32_t level);
void my_shiftOut(uint8_t DataPin, uint8_t ClockPin, uint8_t val);
void init_gpio_to_output();
void init_uart();
static void init_timer(int timer_idx,bool auto_reload);
void angle_0();
void angle_10();
void angle_20();
void angle_30();
void angle_40();
void angle_50();
void angle_60();
void angle_inverse_10();
void angle_inverse_20();
void angle_inverse_30();
void angle_inverse_40();
void angle_inverse_50();
void angle_inverse_60();
void test();
typedef struct {
    gpio_isr_t fn;   
    void *args;      
} gpio_isr_func_t;


typedef struct {
    gpio_dev_t *dev;
    uint32_t version;
} gpio_hal_context_t;

typedef struct {
    int source;               
    int intr_alloc_flags;     
    void (*fn)(void*);        
    void *arg;                
    void *handle;             
    esp_err_t ret;
} gpio_isr_alloc_t;

static gpio_hal_context_t _gpio_hal = {
    .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};

typedef struct {
    gpio_hal_context_t *gpio_hal;
    portMUX_TYPE gpio_spinlock;
    uint32_t isr_core_id;
    gpio_isr_func_t *gpio_isr_func;
    gpio_isr_handle_t gpio_isr_handle;
} gpio_context_t;

static gpio_context_t gpio_context = {
    .gpio_hal = &_gpio_hal,
    .gpio_spinlock = portMUX_INITIALIZER_UNLOCKED,
    .isr_core_id = GPIO_ISR_CORE_ID_UNINIT,
    .gpio_isr_func = NULL,
};

void init_gpio_to_output(){
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void init_uart(){
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);    
}

static void init_timer(int timer_idx,bool auto_reload)
{
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = auto_reload,
    }; 
    timer_init(TIMER_GROUP_0, timer_idx, &config);
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
}
void app_main(void)
{
    init_gpio_to_output();


    //init_uart();
    //init_timer(TIMER_0, TEST_WITHOUT_RELOAD);
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    while(1){
    //test();
    for(temp = 0 ; temp < 10000000 ; temp++);
    angle_0();
    for(temp = 0 ; temp < 10000000 ; temp++);
    angle_10();
    for(temp = 0 ; temp < 10000000 ; temp++);    
    }

/*
    while(1){
    int angle = -1;        
    printf("\ninput angle:");
    scanf("%d" , &angle);
    switch (angle){
        case 0:
        angle_0();
        break; 

        case 10:
        angle_10();
        break;  

        case 20:
        angle_20();
        break;

        case 30:
        angle_30();
        break;  

        case 40:
        angle_40();
        break; 

        case 50:
        angle_50();
        break; 

        case 60:
        angle_60();
        break; 

        case -10:
        angle_inverse_10();
        break;

        case -20:
        angle_inverse_20();
        break; 

        case -30:
        angle_inverse_30();
        break; 

        case -40:
        angle_inverse_40();
        break;

        case -50:
        angle_inverse_50();
        break; 

        case -60:
        angle_inverse_60();
        break;
        
        case 100:
        test();
        break;

        default:
        break;
    }
    }*/
}
inline void my_gpio_set_level(gpio_num_t gpio_num, uint32_t level){//gpio_num<32
    if(level)
     gpio_context.gpio_hal->dev->out_w1ts = (1<<gpio_num);
    else{
     gpio_context.gpio_hal->dev->out_w1tc = (1<<gpio_num);
    }
    __asm__ __volatile__ ("nop\n\t");
}

inline void my_shiftOut(uint8_t DataPin, uint8_t ClockPin, uint8_t val)
{
        //MSB
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
                

}
void test(){ //simply test gpio work or not
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    my_shiftOut(dataPin, clockPin,  170 );
    printf("Testing\n\n");
}
void angle_0(){
    //printf("The angle of reflection is 0 deg\n");
    //timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    //timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  129 );//第1排
    my_shiftOut(dataPin, clockPin,  255 ); //第2排
    my_shiftOut(dataPin, clockPin,  255 );//第3排
    my_shiftOut(dataPin, clockPin,  255 ); //第4排
    my_shiftOut(dataPin, clockPin,  129 );//第5排
    my_shiftOut(dataPin, clockPin,  3 );//第6排
    my_shiftOut(dataPin, clockPin,  255 );//第7排
    my_shiftOut(dataPin, clockPin,  195 );//第8排
    my_shiftOut(dataPin, clockPin,  255 );//第9排
    my_shiftOut(dataPin, clockPin,  192 );//第10排
    my_shiftOut(dataPin, clockPin,  7 );//第40排
    my_shiftOut(dataPin, clockPin,  252 ); //第39排
    my_shiftOut(dataPin, clockPin,  0 );//第38排
    my_shiftOut(dataPin, clockPin,  63 ); //第37排
    my_shiftOut(dataPin, clockPin,  224 );//第36排
    my_shiftOut(dataPin, clockPin,  15 );//第35排
    my_shiftOut(dataPin, clockPin,  224 );//第34排
    my_shiftOut(dataPin, clockPin,  0 );//第33排
    my_shiftOut(dataPin, clockPin,  7 );//第32排
    my_shiftOut(dataPin, clockPin,  240 );//第31排
    my_shiftOut(dataPin, clockPin,  31 ); //第30排
    my_shiftOut(dataPin, clockPin,  192 ); //第29排
    my_shiftOut(dataPin, clockPin,  0 ); //第28排
    my_shiftOut(dataPin, clockPin,  3 );//第27排
    my_shiftOut(dataPin, clockPin,  248 );//第26排
    my_shiftOut(dataPin, clockPin,  63 );//第25排
    my_shiftOut(dataPin, clockPin,  0 );//第24排
    my_shiftOut(dataPin, clockPin,  0 );//第23排
    my_shiftOut(dataPin, clockPin,  0 );//第22排
    my_shiftOut(dataPin, clockPin,  252 ); //第21排
    my_shiftOut(dataPin, clockPin,  126 ); //第20排
    my_shiftOut(dataPin, clockPin,  0 );//第19排
    my_shiftOut(dataPin, clockPin,  0 );//第18排
    my_shiftOut(dataPin, clockPin,  0 );//第17排
    my_shiftOut(dataPin, clockPin,  126 );//第16排
    my_shiftOut(dataPin, clockPin,  252 );//第15排
    my_shiftOut(dataPin, clockPin,  0 );//第14排
    my_shiftOut(dataPin, clockPin,  0 ); //第13排
    my_shiftOut(dataPin, clockPin,  0 );//第12排
    my_shiftOut(dataPin, clockPin,  63 );//第11排
    my_shiftOut(dataPin, clockPin,  248 );//第10排
    my_shiftOut(dataPin, clockPin,  1 );//第9排
    my_shiftOut(dataPin, clockPin,  255 );//第8排
    my_shiftOut(dataPin, clockPin,  128 );//第7排
    my_shiftOut(dataPin, clockPin,  31 );//第6排
    my_shiftOut(dataPin, clockPin,  248 );//第5排
    my_shiftOut(dataPin, clockPin,  7 );//第4排
    my_shiftOut(dataPin, clockPin,  255 );//第3排
    my_shiftOut(dataPin, clockPin,  224 );//第2排
    my_shiftOut(dataPin, clockPin,  31 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  240 );//第50排
    my_shiftOut(dataPin1, clockPin,  15 ); //第49排
    my_shiftOut(dataPin1, clockPin,  255 );//第48排
    my_shiftOut(dataPin1, clockPin,  240 ); //第47排
    my_shiftOut(dataPin1, clockPin,  15 );//第46排
    my_shiftOut(dataPin1, clockPin,  224 );//第45排
    my_shiftOut(dataPin1, clockPin,  31 );//第44排
    my_shiftOut(dataPin1, clockPin,  255 );//第43排
    my_shiftOut(dataPin1, clockPin,  248 );//第42排
    my_shiftOut(dataPin1, clockPin,  7 );//第41排
    my_shiftOut(dataPin1, clockPin,  224 );//第40排
    my_shiftOut(dataPin1, clockPin,  63 ); //第39排
    my_shiftOut(dataPin1, clockPin,  255 );//第38排
    my_shiftOut(dataPin1, clockPin,  252 ); //第37排
    my_shiftOut(dataPin1, clockPin,  7 );//第36排
    my_shiftOut(dataPin1, clockPin,  224 );//第35排
    my_shiftOut(dataPin1, clockPin,  127 );//第34排
    my_shiftOut(dataPin1, clockPin,  255 );//第33排
    my_shiftOut(dataPin1, clockPin,  254 );//第32排
    my_shiftOut(dataPin1, clockPin,  7 );//第31排
    my_shiftOut(dataPin1, clockPin,  192 ); //第30排
    my_shiftOut(dataPin1, clockPin,  127 ); //第29排
    my_shiftOut(dataPin1, clockPin,  255 ); //第28排
    my_shiftOut(dataPin1, clockPin,  254 );//第27排
    my_shiftOut(dataPin1, clockPin,  3 );//第26排
    my_shiftOut(dataPin1, clockPin,  192 );//第25排
    my_shiftOut(dataPin1, clockPin,  255 );//第24排
    my_shiftOut(dataPin1, clockPin,  255 );//第23排
    my_shiftOut(dataPin1, clockPin,  255 );//第22排
    my_shiftOut(dataPin1, clockPin,  3 ); //第21排
    my_shiftOut(dataPin1, clockPin,  192 ); //第20排
    my_shiftOut(dataPin1, clockPin,  255 );//第19排
    my_shiftOut(dataPin1, clockPin,  255 );//第18排
    my_shiftOut(dataPin1, clockPin,  255 );//第17排
    my_shiftOut(dataPin1, clockPin,  3 );//第16排
    my_shiftOut(dataPin1, clockPin,  192 );//第15排
    my_shiftOut(dataPin1, clockPin,  255 );//第14排
    my_shiftOut(dataPin1, clockPin,  255 ); //第13排
    my_shiftOut(dataPin1, clockPin,  255 );//第12排
    my_shiftOut(dataPin1, clockPin,  3 );//第11排
    my_shiftOut(dataPin1, clockPin,  128 );//第10排
    my_shiftOut(dataPin1, clockPin,  255 );//第9排
    my_shiftOut(dataPin1, clockPin,  255 );//第8排
    my_shiftOut(dataPin1, clockPin,  255 );//第7排
    my_shiftOut(dataPin1, clockPin,  1 );//第6排
    my_shiftOut(dataPin1, clockPin,  128 );//第5排
    my_shiftOut(dataPin1, clockPin,  255 );//第4排
    my_shiftOut(dataPin1, clockPin,  255 );//第3排
    my_shiftOut(dataPin1, clockPin,  255 );//第2排
    my_shiftOut(dataPin1, clockPin,  1 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  128 );//第50排
    my_shiftOut(dataPin2, clockPin,  255 ); //第49排
    my_shiftOut(dataPin2, clockPin,  255 );//第48排
    my_shiftOut(dataPin2, clockPin,  255 ); //第47排
    my_shiftOut(dataPin2, clockPin,  1 );//第46排
    my_shiftOut(dataPin2, clockPin,  128 );//第45排
    my_shiftOut(dataPin2, clockPin,  255 );//第44排
    my_shiftOut(dataPin2, clockPin,  255 );//第43排
    my_shiftOut(dataPin2, clockPin,  255 );//第42排
    my_shiftOut(dataPin2, clockPin,  1 );//第41排
    my_shiftOut(dataPin2, clockPin,  192 );//第40排
    my_shiftOut(dataPin2, clockPin,  255 ); //第39排
    my_shiftOut(dataPin2, clockPin,  255 );//第38排
    my_shiftOut(dataPin2, clockPin,  255 ); //第37排
    my_shiftOut(dataPin2, clockPin,  3 );//第36排
    my_shiftOut(dataPin2, clockPin,  192 );//第35排
    my_shiftOut(dataPin2, clockPin,  255 );//第34排
    my_shiftOut(dataPin2, clockPin,  255 );//第33排
    my_shiftOut(dataPin2, clockPin,  255 );//第32排
    my_shiftOut(dataPin2, clockPin,  3 );//第31排
    my_shiftOut(dataPin2, clockPin,  192 ); //第30排
    my_shiftOut(dataPin2, clockPin,  255 ); //第29排
    my_shiftOut(dataPin2, clockPin,  255 ); //第28排
    my_shiftOut(dataPin2, clockPin,  255 );//第27排
    my_shiftOut(dataPin2, clockPin,  3 );//第26排
    my_shiftOut(dataPin2, clockPin,  192 );//第25排
    my_shiftOut(dataPin2, clockPin,  127 );//第24排
    my_shiftOut(dataPin2, clockPin,  255 );//第23排
    my_shiftOut(dataPin2, clockPin,  254 );//第22排
    my_shiftOut(dataPin2, clockPin,  3 ); //第21排
    my_shiftOut(dataPin2, clockPin,  224 ); //第20排
    my_shiftOut(dataPin2, clockPin,  127 );//第19排
    my_shiftOut(dataPin2, clockPin,  255 );//第18排
    my_shiftOut(dataPin2, clockPin,  254 );//第17排
    my_shiftOut(dataPin2, clockPin,  7 );//第16排
    my_shiftOut(dataPin2, clockPin,  224 );//第15排
    my_shiftOut(dataPin2, clockPin,  63 );//第14排
    my_shiftOut(dataPin2, clockPin,  255 ); //第13排
    my_shiftOut(dataPin2, clockPin,  252 );//第12排
    my_shiftOut(dataPin2, clockPin,  7 );//第11排
    my_shiftOut(dataPin2, clockPin,  224 );//第10排
    my_shiftOut(dataPin2, clockPin,  31 );//第9排
    my_shiftOut(dataPin2, clockPin,  255 );//第8排
    my_shiftOut(dataPin2, clockPin,  248 );//第7排
    my_shiftOut(dataPin2, clockPin,  7 );//第6排
    my_shiftOut(dataPin2, clockPin,  240 );//第5排
    my_shiftOut(dataPin2, clockPin,  15 );//第4排
    my_shiftOut(dataPin2, clockPin,  255 );//第3排
    my_shiftOut(dataPin2, clockPin,  240 );//第2排
    my_shiftOut(dataPin2, clockPin,  15 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  248 );//第50排
    my_shiftOut(dataPin3, clockPin,  7 ); //第49排
    my_shiftOut(dataPin3, clockPin,  255 );//第48排
    my_shiftOut(dataPin3, clockPin,  224 ); //第47排
    my_shiftOut(dataPin3, clockPin,  31 );//第46排
    my_shiftOut(dataPin3, clockPin,  248 );//第45排
    my_shiftOut(dataPin3, clockPin,  1 );//第44排
    my_shiftOut(dataPin3, clockPin,  255 );//第43排
    my_shiftOut(dataPin3, clockPin,  128 );//第42排
    my_shiftOut(dataPin3, clockPin,  31 );//第41排
    my_shiftOut(dataPin3, clockPin,  252 );//第40排
    my_shiftOut(dataPin3, clockPin,  0 ); //第39排
    my_shiftOut(dataPin3, clockPin,  0 );//第38排
    my_shiftOut(dataPin3, clockPin,  0 ); //第37排
    my_shiftOut(dataPin3, clockPin,  63 );//第36排
    my_shiftOut(dataPin3, clockPin,  126 );//第35排
    my_shiftOut(dataPin3, clockPin,  0 );//第34排
    my_shiftOut(dataPin3, clockPin,  0 );//第33排
    my_shiftOut(dataPin3, clockPin,  0 );//第32排
    my_shiftOut(dataPin3, clockPin,  126 );//第31排
    my_shiftOut(dataPin3, clockPin,  63 ); //第30排
    my_shiftOut(dataPin3, clockPin,  0 ); //第29排
    my_shiftOut(dataPin3, clockPin,  0 ); //第28排
    my_shiftOut(dataPin3, clockPin,  0 );//第27排
    my_shiftOut(dataPin3, clockPin,  252 );//第26排
    my_shiftOut(dataPin3, clockPin,  31 );//第25排
    my_shiftOut(dataPin3, clockPin,  192 );//第24排
    my_shiftOut(dataPin3, clockPin,  0 );//第23排
    my_shiftOut(dataPin3, clockPin,  3 );//第22排
    my_shiftOut(dataPin3, clockPin,  248 ); //第21排
    my_shiftOut(dataPin3, clockPin,  15 ); //第20排
    my_shiftOut(dataPin3, clockPin,  224 );//第19排
    my_shiftOut(dataPin3, clockPin,  0 );//第18排
    my_shiftOut(dataPin3, clockPin,  7 );//第17排
    my_shiftOut(dataPin3, clockPin,  240 );//第16排
    my_shiftOut(dataPin3, clockPin,  7 );//第15排
    my_shiftOut(dataPin3, clockPin,  252 );//第14排
    my_shiftOut(dataPin3, clockPin,  0 ); //第13排
    my_shiftOut(dataPin3, clockPin,  63 );//第12排
    my_shiftOut(dataPin3, clockPin,  224 );//第11排
    my_shiftOut(dataPin3, clockPin,  3 );//第10排
    my_shiftOut(dataPin3, clockPin,  255 );//第9排
    my_shiftOut(dataPin3, clockPin,  195 );//第8排
    my_shiftOut(dataPin3, clockPin,  255 );//第7排
    my_shiftOut(dataPin3, clockPin,  192 );//第6排
    my_shiftOut(dataPin3, clockPin,  129 );//第5排
    my_shiftOut(dataPin3, clockPin,  255 );//第4排
    my_shiftOut(dataPin3, clockPin,  255 );//第3排
    my_shiftOut(dataPin3, clockPin,  255 );//第2排
    my_shiftOut(dataPin3, clockPin,  129 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    //timer_pause(TIMER_GROUP_0, 0);
    //timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    //timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    //printf("Running time: ");
    //printf("%.2f" , duration*1000000);
    //printf("us\n\n");
}
void angle_10(){    
    //start=millis();
    printf("The angle of reflection is 10 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  56 );//第1排
    my_shiftOut(dataPin, clockPin,  240 ); //第2排
    my_shiftOut(dataPin, clockPin,  248 );//第3排
    my_shiftOut(dataPin, clockPin,  7 ); //第4排
    my_shiftOut(dataPin, clockPin,  255 );//第5排
    my_shiftOut(dataPin, clockPin,  113 );//第6排
    my_shiftOut(dataPin, clockPin,  225 );//第7排
    my_shiftOut(dataPin, clockPin,  240 );//第8排
    my_shiftOut(dataPin, clockPin,  31 );//第9排
    my_shiftOut(dataPin, clockPin,  255 );//第10排
    my_shiftOut(dataPin, clockPin,  113 );//第40排
    my_shiftOut(dataPin, clockPin,  195 ); //第39排
    my_shiftOut(dataPin, clockPin,  224 );//第38排
    my_shiftOut(dataPin, clockPin,  127 );//第37排
    my_shiftOut(dataPin, clockPin,  255 );//第36排
    my_shiftOut(dataPin, clockPin,  227 );//第35排
    my_shiftOut(dataPin, clockPin,  199 );//第34排
    my_shiftOut(dataPin, clockPin,  192 );//第33排
    my_shiftOut(dataPin, clockPin,  255 );//第32排
    my_shiftOut(dataPin, clockPin,  255 );//第31排
    my_shiftOut(dataPin, clockPin,  227 ); //第30排
    my_shiftOut(dataPin, clockPin,  135 ); //第29排
    my_shiftOut(dataPin, clockPin,  129 ); //第28排
    my_shiftOut(dataPin, clockPin,  255 );//第27排
    my_shiftOut(dataPin, clockPin,  225 );//第26排
    my_shiftOut(dataPin, clockPin,  199 );//第25排
    my_shiftOut(dataPin, clockPin,  15 );//第24排
    my_shiftOut(dataPin, clockPin,  3 );//第23排
    my_shiftOut(dataPin, clockPin,  254 );//第22排
    my_shiftOut(dataPin, clockPin,  0 ); //第21排
    my_shiftOut(dataPin, clockPin,  199 ); //第20排
    my_shiftOut(dataPin, clockPin,  15 );//第19排
    my_shiftOut(dataPin, clockPin,  7 );//第18排
    my_shiftOut(dataPin, clockPin,  248 );//第17排
    my_shiftOut(dataPin, clockPin,  0 );//第16排
    my_shiftOut(dataPin, clockPin,  142 );//第15排
    my_shiftOut(dataPin, clockPin,  30 );//第14排
    my_shiftOut(dataPin, clockPin,  15 ); //第13排
    my_shiftOut(dataPin, clockPin,  240 );//第12排
    my_shiftOut(dataPin, clockPin,  0 );//第11排
    my_shiftOut(dataPin, clockPin,  142 );//第10排
    my_shiftOut(dataPin, clockPin,  30 );//第9排
    my_shiftOut(dataPin, clockPin,  31 );//第8排
    my_shiftOut(dataPin, clockPin,  192 );//第7排
    my_shiftOut(dataPin, clockPin,  0 );//第6排
    my_shiftOut(dataPin, clockPin,  142 );//第5排
    my_shiftOut(dataPin, clockPin,  60 );//第4排
    my_shiftOut(dataPin, clockPin,  31 );//第3排
    my_shiftOut(dataPin, clockPin,  128 );//第2排
    my_shiftOut(dataPin, clockPin,  0 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  156 );//第50排
    my_shiftOut(dataPin1, clockPin,  60 ); //第49排
    my_shiftOut(dataPin1, clockPin,  63 );//第48排
    my_shiftOut(dataPin1, clockPin,  0 ); //第47排
    my_shiftOut(dataPin1, clockPin,  0 );//第46排
    my_shiftOut(dataPin1, clockPin,  28 );//第45排
    my_shiftOut(dataPin1, clockPin,  120 );//第44排
    my_shiftOut(dataPin1, clockPin,  63 );//第43排
    my_shiftOut(dataPin1, clockPin,  0 );//第42排
    my_shiftOut(dataPin1, clockPin,  0 );//第41排
    my_shiftOut(dataPin1, clockPin,  28 );//第40排
    my_shiftOut(dataPin1, clockPin,  120 ); //第39排
    my_shiftOut(dataPin1, clockPin,  126 );//第38排
    my_shiftOut(dataPin1, clockPin,  0 ); //第37排
    my_shiftOut(dataPin1, clockPin,  0 );//第36排
    my_shiftOut(dataPin1, clockPin,  28 );//第35排
    my_shiftOut(dataPin1, clockPin,  120 );//第34排
    my_shiftOut(dataPin1, clockPin,  126 );//第33排
    my_shiftOut(dataPin1, clockPin,  0 );//第32排
    my_shiftOut(dataPin1, clockPin,  62 );//第31排
    my_shiftOut(dataPin1, clockPin,  56 ); //第30排
    my_shiftOut(dataPin1, clockPin,  112 ); //第29排
    my_shiftOut(dataPin1, clockPin,  124 ); //第28排
    my_shiftOut(dataPin1, clockPin,  0 );//第27排
    my_shiftOut(dataPin1, clockPin,  127 );//第26排
    my_shiftOut(dataPin1, clockPin,  56 );//第25排
    my_shiftOut(dataPin1, clockPin,  240 );//第24排
    my_shiftOut(dataPin1, clockPin,  252 );//第23排
    my_shiftOut(dataPin1, clockPin,  0 );//第22排
    my_shiftOut(dataPin1, clockPin,  255 ); //第21排
    my_shiftOut(dataPin1, clockPin,  56 ); //第20排
    my_shiftOut(dataPin1, clockPin,  240 );//第19排
    my_shiftOut(dataPin1, clockPin,  252 );//第18排
    my_shiftOut(dataPin1, clockPin,  1 );//第17排
    my_shiftOut(dataPin1, clockPin,  255 );//第16排
    my_shiftOut(dataPin1, clockPin,  56 );//第15排
    my_shiftOut(dataPin1, clockPin,  240 );//第14排
    my_shiftOut(dataPin1, clockPin,  248 ); //第13排
    my_shiftOut(dataPin1, clockPin,  3 );//第12排
    my_shiftOut(dataPin1, clockPin,  255 );//第11排
    my_shiftOut(dataPin1, clockPin,  56 );//第10排
    my_shiftOut(dataPin1, clockPin,  240 );//第9排
    my_shiftOut(dataPin1, clockPin,  248 );//第8排
    my_shiftOut(dataPin1, clockPin,  3 );//第7排
    my_shiftOut(dataPin1, clockPin,  255 );//第6排
    my_shiftOut(dataPin1, clockPin,  56 );//第5排
    my_shiftOut(dataPin1, clockPin,  240 );//第4排
    my_shiftOut(dataPin1, clockPin,  248 );//第3排
    my_shiftOut(dataPin1, clockPin,  3 );//第2排
    my_shiftOut(dataPin1, clockPin,  255 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  56 );//第50排
    my_shiftOut(dataPin2, clockPin,  240 ); //第49排
    my_shiftOut(dataPin2, clockPin,  248 );//第48排
    my_shiftOut(dataPin2, clockPin,  3 ); //第47排
    my_shiftOut(dataPin2, clockPin,  255 );//第46排
    my_shiftOut(dataPin2, clockPin,  56 );//第45排
    my_shiftOut(dataPin2, clockPin,  240 );//第44排
    my_shiftOut(dataPin2, clockPin,  248 );//第43排
    my_shiftOut(dataPin2, clockPin,  3 );//第42排
    my_shiftOut(dataPin2, clockPin,  255 );//第41排
    my_shiftOut(dataPin2, clockPin,  56 );//第40排
    my_shiftOut(dataPin2, clockPin,  240 ); //第39排
    my_shiftOut(dataPin2, clockPin,  248 );//第38排
    my_shiftOut(dataPin2, clockPin,  3 ); //第37排
    my_shiftOut(dataPin2, clockPin,  255 );//第36排
    my_shiftOut(dataPin2, clockPin,  56 );//第35排
    my_shiftOut(dataPin2, clockPin,  240 );//第34排
    my_shiftOut(dataPin2, clockPin,  252 );//第33排
    my_shiftOut(dataPin2, clockPin,  1 );//第32排
    my_shiftOut(dataPin2, clockPin,  255 );//第31排
    my_shiftOut(dataPin2, clockPin,  56 ); //第30排
    my_shiftOut(dataPin2, clockPin,  240 ); //第29排
    my_shiftOut(dataPin2, clockPin,  252 ); //第28排
    my_shiftOut(dataPin2, clockPin,  0 );//第27排
    my_shiftOut(dataPin2, clockPin,  255 );//第26排
    my_shiftOut(dataPin2, clockPin,  56 );//第25排
    my_shiftOut(dataPin2, clockPin,  112 );//第24排
    my_shiftOut(dataPin2, clockPin,  124 );//第23排
    my_shiftOut(dataPin2, clockPin,  0 );//第22排
    my_shiftOut(dataPin2, clockPin,  127 ); //第21排
    my_shiftOut(dataPin2, clockPin,  28 ); //第20排
    my_shiftOut(dataPin2, clockPin,  120 );//第19排
    my_shiftOut(dataPin2, clockPin,  126 );//第18排
    my_shiftOut(dataPin2, clockPin,  0 );//第17排
    my_shiftOut(dataPin2, clockPin,  62 );//第16排
    my_shiftOut(dataPin2, clockPin,  28 );//第15排
    my_shiftOut(dataPin2, clockPin,  120 );//第14排
    my_shiftOut(dataPin2, clockPin,  126 ); //第13排
    my_shiftOut(dataPin2, clockPin,  0 );//第12排
    my_shiftOut(dataPin2, clockPin,  0 );//第11排
    my_shiftOut(dataPin2, clockPin,  28 );//第10排
    my_shiftOut(dataPin2, clockPin,  120 );//第9排
    my_shiftOut(dataPin2, clockPin,  63 );//第8排
    my_shiftOut(dataPin2, clockPin,  0 );//第7排
    my_shiftOut(dataPin2, clockPin,  0 );//第6排
    my_shiftOut(dataPin2, clockPin,  156 );//第5排
    my_shiftOut(dataPin2, clockPin,  60 );//第4排
    my_shiftOut(dataPin2, clockPin,  63 );//第3排
    my_shiftOut(dataPin2, clockPin,  0 );//第2排
    my_shiftOut(dataPin2, clockPin,  0 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  142 );//第50排
    my_shiftOut(dataPin3, clockPin,  60 ); //第49排
    my_shiftOut(dataPin3, clockPin,  31 );//第48排
    my_shiftOut(dataPin3, clockPin,  128 ); //第47排
    my_shiftOut(dataPin3, clockPin,  0 );//第46排
    my_shiftOut(dataPin3, clockPin,  142 );//第45排
    my_shiftOut(dataPin3, clockPin,  30 );//第44排
    my_shiftOut(dataPin3, clockPin,  31 );//第43排
    my_shiftOut(dataPin3, clockPin,  192 );//第42排
    my_shiftOut(dataPin3, clockPin,  0 );//第41排
    my_shiftOut(dataPin3, clockPin,  142 );//第40排
    my_shiftOut(dataPin3, clockPin,  30 ); //第39排
    my_shiftOut(dataPin3, clockPin,  15 );//第38排
    my_shiftOut(dataPin3, clockPin,  240 ); //第37排
    my_shiftOut(dataPin3, clockPin,  0 );//第36排
    my_shiftOut(dataPin3, clockPin,  199 );//第35排
    my_shiftOut(dataPin3, clockPin,  15 );//第34排
    my_shiftOut(dataPin3, clockPin,  7 );//第33排
    my_shiftOut(dataPin3, clockPin,  248 );//第32排
    my_shiftOut(dataPin3, clockPin,  0 );//第31排
    my_shiftOut(dataPin3, clockPin,  199 ); //第30排
    my_shiftOut(dataPin3, clockPin,  15 ); //第29排
    my_shiftOut(dataPin3, clockPin,  3 ); //第28排
    my_shiftOut(dataPin3, clockPin,  254 );//第27排
    my_shiftOut(dataPin3, clockPin,  0 );//第26排
    my_shiftOut(dataPin3, clockPin,  227 );//第25排
    my_shiftOut(dataPin3, clockPin,  135 );//第24排
    my_shiftOut(dataPin3, clockPin,  129 );//第23排
    my_shiftOut(dataPin3, clockPin,  255 );//第22排
    my_shiftOut(dataPin3, clockPin,  225 ); //第21排
    my_shiftOut(dataPin3, clockPin,  227 ); //第20排
    my_shiftOut(dataPin3, clockPin,  199 );//第19排
    my_shiftOut(dataPin3, clockPin,  192 );//第18排
    my_shiftOut(dataPin3, clockPin,  255 );//第17排
    my_shiftOut(dataPin3, clockPin,  255 );//第16排
    my_shiftOut(dataPin3, clockPin,  113 );//第15排
    my_shiftOut(dataPin3, clockPin,  195 );//第14排
    my_shiftOut(dataPin3, clockPin,  224 ); //第13排
    my_shiftOut(dataPin3, clockPin,  127 );//第12排
    my_shiftOut(dataPin3, clockPin,  255 );//第11排
    my_shiftOut(dataPin3, clockPin,  113 );//第10排
    my_shiftOut(dataPin3, clockPin,  225 );//第9排
    my_shiftOut(dataPin3, clockPin,  240 );//第8排
    my_shiftOut(dataPin3, clockPin,  31 );//第7排
    my_shiftOut(dataPin3, clockPin,  255 );//第6排
    my_shiftOut(dataPin3, clockPin,  56 );//第5排
    my_shiftOut(dataPin3, clockPin,  240 );//第4排
    my_shiftOut(dataPin3, clockPin,  248 );//第3排
    my_shiftOut(dataPin3, clockPin,  7 );//第2排
    my_shiftOut(dataPin3, clockPin,  255 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");  
    }
void angle_20(){
    printf("The angle of reflection is 20 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  204 );//第1排
    my_shiftOut(dataPin, clockPin,  198 ); //第2排
    my_shiftOut(dataPin, clockPin,  56 );//第3排
    my_shiftOut(dataPin, clockPin,  225 ); //第4排
    my_shiftOut(dataPin, clockPin,  240 );//第5排
    my_shiftOut(dataPin, clockPin,  204 );//第6排
    my_shiftOut(dataPin, clockPin,  206 );//第7排
    my_shiftOut(dataPin, clockPin,  113 );//第8排
    my_shiftOut(dataPin, clockPin,  195 );//第9排
    my_shiftOut(dataPin, clockPin,  224 );//第10排
    my_shiftOut(dataPin, clockPin,  153 );//第40排
    my_shiftOut(dataPin, clockPin,  140 ); //第39排
    my_shiftOut(dataPin, clockPin,  99 );//第38排
    my_shiftOut(dataPin, clockPin,  195 ); //第37排
    my_shiftOut(dataPin, clockPin,  192 );//第36排
    my_shiftOut(dataPin, clockPin,  153 );//第35排
    my_shiftOut(dataPin, clockPin,  156 );//第34排
    my_shiftOut(dataPin, clockPin,  227 );//第33排
    my_shiftOut(dataPin, clockPin,  135 );//第32排
    my_shiftOut(dataPin, clockPin,  129 );//第31排
    my_shiftOut(dataPin, clockPin,  145 ); //第30排
    my_shiftOut(dataPin, clockPin,  152 ); //第29排
    my_shiftOut(dataPin, clockPin,  199 ); //第28排
    my_shiftOut(dataPin, clockPin,  143 );//第27排
    my_shiftOut(dataPin, clockPin,  3 );//第26排
    my_shiftOut(dataPin, clockPin,  51 );//第25排
    my_shiftOut(dataPin, clockPin,  25 );//第24排
    my_shiftOut(dataPin, clockPin,  199 );//第23排
    my_shiftOut(dataPin, clockPin,  15 );//第22排
    my_shiftOut(dataPin, clockPin,  7 ); //第21排
    my_shiftOut(dataPin, clockPin,  51 ); //第20排
    my_shiftOut(dataPin, clockPin,  57 );//第19排
    my_shiftOut(dataPin, clockPin,  207 );//第18排
    my_shiftOut(dataPin, clockPin,  30 );//第17排
    my_shiftOut(dataPin, clockPin,  15 );//第16排
    my_shiftOut(dataPin, clockPin,  51 );//第15排
    my_shiftOut(dataPin, clockPin,  49 );//第14排
    my_shiftOut(dataPin, clockPin,  142 ); //第13排
    my_shiftOut(dataPin, clockPin,  28 );//第12排
    my_shiftOut(dataPin, clockPin,  31 );//第11排
    my_shiftOut(dataPin, clockPin,  102 );//第10排
    my_shiftOut(dataPin, clockPin,  51 );//第9排
    my_shiftOut(dataPin, clockPin,  142 );//第8排
    my_shiftOut(dataPin, clockPin,  60 );//第7排
    my_shiftOut(dataPin, clockPin,  31 );//第6排
    my_shiftOut(dataPin, clockPin,  102 );//第5排
    my_shiftOut(dataPin, clockPin,  115 );//第4排
    my_shiftOut(dataPin, clockPin,  156 );//第3排
    my_shiftOut(dataPin, clockPin,  56 );//第2排
    my_shiftOut(dataPin, clockPin,  63 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  102 );//第50排
    my_shiftOut(dataPin1, clockPin,  99 ); //第49排
    my_shiftOut(dataPin1, clockPin,  28 );//第48排
    my_shiftOut(dataPin1, clockPin,  120 ); //第47排
    my_shiftOut(dataPin1, clockPin,  62 );//第46排
    my_shiftOut(dataPin1, clockPin,  102 );//第45排
    my_shiftOut(dataPin1, clockPin,  99 );//第44排
    my_shiftOut(dataPin1, clockPin,  28 );//第43排
    my_shiftOut(dataPin1, clockPin,  120 );//第42排
    my_shiftOut(dataPin1, clockPin,  126 );//第41排
    my_shiftOut(dataPin1, clockPin,  102 );//第40排
    my_shiftOut(dataPin1, clockPin,  103 ); //第39排
    my_shiftOut(dataPin1, clockPin,  24 );//第38排
    my_shiftOut(dataPin1, clockPin,  112 ); //第37排
    my_shiftOut(dataPin1, clockPin,  124 );//第36排
    my_shiftOut(dataPin1, clockPin,  76 );//第35排
    my_shiftOut(dataPin1, clockPin,  103 );//第34排
    my_shiftOut(dataPin1, clockPin,  56 );//第33排
    my_shiftOut(dataPin1, clockPin,  240 );//第32排
    my_shiftOut(dataPin1, clockPin,  252 );//第31排
    my_shiftOut(dataPin1, clockPin,  76 ); //第30排
    my_shiftOut(dataPin1, clockPin,  231 ); //第29排
    my_shiftOut(dataPin1, clockPin,  56 ); //第28排
    my_shiftOut(dataPin1, clockPin,  240 );//第27排
    my_shiftOut(dataPin1, clockPin,  248 );//第26排
    my_shiftOut(dataPin1, clockPin,  204 );//第25排
    my_shiftOut(dataPin1, clockPin,  230 );//第24排
    my_shiftOut(dataPin1, clockPin,  56 );//第23排
    my_shiftOut(dataPin1, clockPin,  240 );//第22排
    my_shiftOut(dataPin1, clockPin,  248 ); //第21排
    my_shiftOut(dataPin1, clockPin,  204 ); //第20排
    my_shiftOut(dataPin1, clockPin,  198 );//第19排
    my_shiftOut(dataPin1, clockPin,  56 );//第18排
    my_shiftOut(dataPin1, clockPin,  224 );//第17排
    my_shiftOut(dataPin1, clockPin,  248 );//第16排
    my_shiftOut(dataPin1, clockPin,  204 );//第15排
    my_shiftOut(dataPin1, clockPin,  198 );//第14排
    my_shiftOut(dataPin1, clockPin,  56 ); //第13排
    my_shiftOut(dataPin1, clockPin,  225 );//第12排
    my_shiftOut(dataPin1, clockPin,  248 );//第11排
    my_shiftOut(dataPin1, clockPin,  204 );//第10排
    my_shiftOut(dataPin1, clockPin,  198 );//第9排
    my_shiftOut(dataPin1, clockPin,  56 );//第8排
    my_shiftOut(dataPin1, clockPin,  225 );//第7排
    my_shiftOut(dataPin1, clockPin,  240 );//第6排
    my_shiftOut(dataPin1, clockPin,  204);//第5排
    my_shiftOut(dataPin1, clockPin,  198 );//第4排
    my_shiftOut(dataPin1, clockPin,  56 );//第3排
    my_shiftOut(dataPin1, clockPin,  225 );//第2排
    my_shiftOut(dataPin1, clockPin,  240 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  204 );//第50排
    my_shiftOut(dataPin2, clockPin,  198 ); //第49排
    my_shiftOut(dataPin2, clockPin,  56 );//第48排
    my_shiftOut(dataPin2, clockPin,  225 ); //第47排
    my_shiftOut(dataPin2, clockPin,  240 );//第46排
    my_shiftOut(dataPin2, clockPin,  204 );//第45排
    my_shiftOut(dataPin2, clockPin,  198 );//第44排
    my_shiftOut(dataPin2, clockPin,  56 );//第43排
    my_shiftOut(dataPin2, clockPin,  225 );//第42排
    my_shiftOut(dataPin2, clockPin,  240 );//第41排
    my_shiftOut(dataPin2, clockPin,  204 );//第40排
    my_shiftOut(dataPin2, clockPin,  198 ); //第39排
    my_shiftOut(dataPin2, clockPin,  56 );//第38排
    my_shiftOut(dataPin2, clockPin,  225 ); //第37排
    my_shiftOut(dataPin2, clockPin,  248 );//第36排
    my_shiftOut(dataPin2, clockPin,  204 );//第35排
    my_shiftOut(dataPin2, clockPin,  198 );//第34排
    my_shiftOut(dataPin2, clockPin,  56 );//第33排
    my_shiftOut(dataPin2, clockPin,  224 );//第32排
    my_shiftOut(dataPin2, clockPin,  248 );//第31排
    my_shiftOut(dataPin2, clockPin,  204 ); //第30排
    my_shiftOut(dataPin2, clockPin,  230 ); //第29排
    my_shiftOut(dataPin2, clockPin,  56 ); //第28排
    my_shiftOut(dataPin2, clockPin,  240 );//第27排
    my_shiftOut(dataPin2, clockPin,  248 );//第26排
    my_shiftOut(dataPin2, clockPin,  76 );//第25排
    my_shiftOut(dataPin2, clockPin,  231 );//第24排
    my_shiftOut(dataPin2, clockPin,  56 );//第23排
    my_shiftOut(dataPin2, clockPin,  240 );//第22排
    my_shiftOut(dataPin2, clockPin,  248 ); //第21排
    my_shiftOut(dataPin2, clockPin,  76 ); //第20排
    my_shiftOut(dataPin2, clockPin,  103 );//第19排
    my_shiftOut(dataPin2, clockPin,  56 );//第18排
    my_shiftOut(dataPin2, clockPin,  240 );//第17排
    my_shiftOut(dataPin2, clockPin,  252 );//第16排
    my_shiftOut(dataPin2, clockPin,  102 );//第15排
    my_shiftOut(dataPin2, clockPin,  103 );//第14排
    my_shiftOut(dataPin2, clockPin,  24 ); //第13排
    my_shiftOut(dataPin2, clockPin,  112 );//第12排
    my_shiftOut(dataPin2, clockPin,  124 );//第11排
    my_shiftOut(dataPin2, clockPin,  102 );//第10排
    my_shiftOut(dataPin2, clockPin,  99 );//第9排
    my_shiftOut(dataPin2, clockPin,  28 );//第8排
    my_shiftOut(dataPin2, clockPin,  120 );//第7排
    my_shiftOut(dataPin2, clockPin,  126 );//第6排
    my_shiftOut(dataPin2, clockPin,  102 );//第5排
    my_shiftOut(dataPin2, clockPin,  99 );//第4排
    my_shiftOut(dataPin2, clockPin,  28 );//第3排
    my_shiftOut(dataPin2, clockPin,  120 );//第2排
    my_shiftOut(dataPin2, clockPin,  62 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  102 );//第50排
    my_shiftOut(dataPin3, clockPin,  115 ); //第49排
    my_shiftOut(dataPin3, clockPin,  156 );//第48排
    my_shiftOut(dataPin3, clockPin,  56 ); //第47排
    my_shiftOut(dataPin3, clockPin,  63 );//第46排
    my_shiftOut(dataPin3, clockPin,  102 );//第45排
    my_shiftOut(dataPin3, clockPin,  51 );//第44排
    my_shiftOut(dataPin3, clockPin,  142 );//第43排
    my_shiftOut(dataPin3, clockPin,  60 );//第42排
    my_shiftOut(dataPin3, clockPin,  31 );//第41排
    my_shiftOut(dataPin3, clockPin,  51 );//第40排
    my_shiftOut(dataPin3, clockPin,  49 ); //第39排
    my_shiftOut(dataPin3, clockPin,  142 );//第38排
    my_shiftOut(dataPin3, clockPin,  28 ); //第37排
    my_shiftOut(dataPin3, clockPin,  31 );//第36排
    my_shiftOut(dataPin3, clockPin,  51 );//第35排
    my_shiftOut(dataPin3, clockPin,  57 );//第34排
    my_shiftOut(dataPin3, clockPin,  207 );//第33排
    my_shiftOut(dataPin3, clockPin,  30 );//第32排
    my_shiftOut(dataPin3, clockPin,  15 );//第31排
    my_shiftOut(dataPin3, clockPin,  51 ); //第30排
    my_shiftOut(dataPin3, clockPin,  25 ); //第29排
    my_shiftOut(dataPin3, clockPin,  199 ); //第28排
    my_shiftOut(dataPin3, clockPin,  15 );//第27排
    my_shiftOut(dataPin3, clockPin,  7 );//第26排
    my_shiftOut(dataPin3, clockPin,  145 );//第25排
    my_shiftOut(dataPin3, clockPin,  152 );//第24排
    my_shiftOut(dataPin3, clockPin,  199 );//第23排
    my_shiftOut(dataPin3, clockPin,  143 );//第22排
    my_shiftOut(dataPin3, clockPin,  3 ); //第21排
    my_shiftOut(dataPin3, clockPin,  153 ); //第20排
    my_shiftOut(dataPin3, clockPin,  156 );//第19排
    my_shiftOut(dataPin3, clockPin,  227 );//第18排
    my_shiftOut(dataPin3, clockPin,  135 );//第17排
    my_shiftOut(dataPin3, clockPin,  129 );//第16排
    my_shiftOut(dataPin3, clockPin,  153 );//第15排
    my_shiftOut(dataPin3, clockPin,  140 );//第14排
    my_shiftOut(dataPin3, clockPin,  99 ); //第13排
    my_shiftOut(dataPin3, clockPin,  195 );//第12排
    my_shiftOut(dataPin3, clockPin,  192 );//第11排
    my_shiftOut(dataPin3, clockPin,  204 );//第10排
    my_shiftOut(dataPin3, clockPin,  206 );//第9排
    my_shiftOut(dataPin3, clockPin,  113 );//第8排
    my_shiftOut(dataPin3, clockPin,  195 );//第7排
    my_shiftOut(dataPin3, clockPin,  224 );//第6排
    my_shiftOut(dataPin3, clockPin,  204 );//第5排
    my_shiftOut(dataPin3, clockPin,  198 );//第4排
    my_shiftOut(dataPin3, clockPin,  56 );//第3排
    my_shiftOut(dataPin3, clockPin,  225 );//第2排
    my_shiftOut(dataPin3, clockPin,  240 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");     
}
void angle_30(){
    //start=millis();
    printf("The angle of reflection is 30 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  36 );//第1排
    my_shiftOut(dataPin, clockPin,  153 ); //第2排
    my_shiftOut(dataPin, clockPin,  51 );//第3排
    my_shiftOut(dataPin, clockPin,  152 ); //第4排
    my_shiftOut(dataPin, clockPin,  231 );//第5排
    my_shiftOut(dataPin, clockPin,  108 );//第6排
    my_shiftOut(dataPin, clockPin,  155 );//第7排
    my_shiftOut(dataPin, clockPin,  51 );//第8排
    my_shiftOut(dataPin, clockPin,  57 );//第9排
    my_shiftOut(dataPin, clockPin,  199 );//第10排
    my_shiftOut(dataPin, clockPin,  109 );//第40排
    my_shiftOut(dataPin, clockPin,  147 ); //第39排
    my_shiftOut(dataPin, clockPin,  51 );//第38排
    my_shiftOut(dataPin, clockPin,  49 ); //第37排
    my_shiftOut(dataPin, clockPin,  142 );//第36排
    my_shiftOut(dataPin, clockPin,  73 );//第35排
    my_shiftOut(dataPin, clockPin,  178 );//第34排
    my_shiftOut(dataPin, clockPin,  102 );//第33排
    my_shiftOut(dataPin, clockPin,  51 );//第32排
    my_shiftOut(dataPin, clockPin,  142 );//第31排
    my_shiftOut(dataPin, clockPin,  73 ); //第30排
    my_shiftOut(dataPin, clockPin,  54 ); //第29排
    my_shiftOut(dataPin, clockPin,  102 ); //第28排
    my_shiftOut(dataPin, clockPin,  99 );//第27排
    my_shiftOut(dataPin, clockPin,  156 );//第26排
    my_shiftOut(dataPin, clockPin,  91 );//第25排
    my_shiftOut(dataPin, clockPin,  38 );//第24排
    my_shiftOut(dataPin, clockPin,  102 );//第23排
    my_shiftOut(dataPin, clockPin,  99 );//第22排
    my_shiftOut(dataPin, clockPin,  28 ); //第21排
    my_shiftOut(dataPin, clockPin,  219 ); //第20排
    my_shiftOut(dataPin, clockPin,  102 );//第19排
    my_shiftOut(dataPin, clockPin,  204 );//第18排
    my_shiftOut(dataPin, clockPin,  103 );//第17排
    my_shiftOut(dataPin, clockPin,  56 );//第16排
    my_shiftOut(dataPin, clockPin,  219 );//第15排
    my_shiftOut(dataPin, clockPin,  100 );//第14排
    my_shiftOut(dataPin, clockPin,  204 ); //第13排
    my_shiftOut(dataPin, clockPin,  198 );//第12排
    my_shiftOut(dataPin, clockPin,  56 );//第11排
    my_shiftOut(dataPin, clockPin,  146 );//第10排
    my_shiftOut(dataPin, clockPin,  108 );//第9排
    my_shiftOut(dataPin, clockPin,  204 );//第8排
    my_shiftOut(dataPin, clockPin,  198 );//第7排
    my_shiftOut(dataPin, clockPin,  56 );//第6排
    my_shiftOut(dataPin, clockPin,  146 );//第5排
    my_shiftOut(dataPin, clockPin,  76 );//第4排
    my_shiftOut(dataPin, clockPin,  204 );//第3排
    my_shiftOut(dataPin, clockPin,  206 );//第2排
    my_shiftOut(dataPin, clockPin,  113 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  146 );//第50排
    my_shiftOut(dataPin1, clockPin,  76 ); //第49排
    my_shiftOut(dataPin1, clockPin,  153 );//第48排
    my_shiftOut(dataPin1, clockPin,  204 ); //第47排
    my_shiftOut(dataPin1, clockPin,  113 );//第46排
    my_shiftOut(dataPin1, clockPin,  182 );//第45排
    my_shiftOut(dataPin1, clockPin,  77 );//第44排
    my_shiftOut(dataPin1, clockPin,  153 );//第43排
    my_shiftOut(dataPin1, clockPin,  140 );//第42排
    my_shiftOut(dataPin1, clockPin,  113 );//第41排
    my_shiftOut(dataPin1, clockPin,  182 );//第40排
    my_shiftOut(dataPin1, clockPin,  201 ); //第39排
    my_shiftOut(dataPin1, clockPin,  153 );//第38排
    my_shiftOut(dataPin1, clockPin,  140 ); //第37排
    my_shiftOut(dataPin1, clockPin,  97 );//第36排
    my_shiftOut(dataPin1, clockPin,  182 );//第35排
    my_shiftOut(dataPin1, clockPin,  201 );//第34排
    my_shiftOut(dataPin1, clockPin,  153 );//第33排
    my_shiftOut(dataPin1, clockPin,  156 );//第32排
    my_shiftOut(dataPin1, clockPin,  227 );//第31排
    my_shiftOut(dataPin1, clockPin,  182 ); //第30排
    my_shiftOut(dataPin1, clockPin,  217 ); //第29排
    my_shiftOut(dataPin1, clockPin,  153 ); //第28排
    my_shiftOut(dataPin1, clockPin,  156 );//第27排
    my_shiftOut(dataPin1, clockPin,  227 );//第26排
    my_shiftOut(dataPin1, clockPin,  166 );//第25排
    my_shiftOut(dataPin1, clockPin,  217 );//第24排
    my_shiftOut(dataPin1, clockPin,  153 );//第23排
    my_shiftOut(dataPin1, clockPin,  156 );//第22排
    my_shiftOut(dataPin1, clockPin,  227 ); //第21排
    my_shiftOut(dataPin1, clockPin,  36 ); //第20排
    my_shiftOut(dataPin1, clockPin,  217 );//第19排
    my_shiftOut(dataPin1, clockPin,  153 );//第18排
    my_shiftOut(dataPin1, clockPin,  152 );//第17排
    my_shiftOut(dataPin1, clockPin,  227 );//第16排
    my_shiftOut(dataPin1, clockPin,  36 );//第15排
    my_shiftOut(dataPin1, clockPin,  217 );//第14排
    my_shiftOut(dataPin1, clockPin,  177 ); //第13排
    my_shiftOut(dataPin1, clockPin,  152 );//第12排
    my_shiftOut(dataPin1, clockPin,  227 );//第11排
    my_shiftOut(dataPin1, clockPin,  36 );//第10排
    my_shiftOut(dataPin1, clockPin,  217 );//第9排
    my_shiftOut(dataPin1, clockPin,  179 );//第8排
    my_shiftOut(dataPin1, clockPin,  152 );//第7排
    my_shiftOut(dataPin1, clockPin,  227 );//第6排
    my_shiftOut(dataPin1, clockPin,  36 );//第5排
    my_shiftOut(dataPin1, clockPin,  217 );//第4排
    my_shiftOut(dataPin1, clockPin,  179 );//第3排
    my_shiftOut(dataPin1, clockPin,  152 );//第2排
    my_shiftOut(dataPin1, clockPin,  227 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  36 );//第50排
    my_shiftOut(dataPin2, clockPin,  217 ); //第49排
    my_shiftOut(dataPin2, clockPin,  179 );//第48排
    my_shiftOut(dataPin2, clockPin,  152 ); //第47排
    my_shiftOut(dataPin2, clockPin,  227 );//第46排
    my_shiftOut(dataPin2, clockPin,  36 );//第45排
    my_shiftOut(dataPin2, clockPin,  217 );//第44排
    my_shiftOut(dataPin2, clockPin,  179 );//第43排
    my_shiftOut(dataPin2, clockPin,  152 );//第42排
    my_shiftOut(dataPin2, clockPin,  227 );//第41排
    my_shiftOut(dataPin2, clockPin,  36 );//第40排
    my_shiftOut(dataPin2, clockPin,  217 ); //第39排
    my_shiftOut(dataPin2, clockPin,  177 );//第38排
    my_shiftOut(dataPin2, clockPin,  152 ); //第37排
    my_shiftOut(dataPin2, clockPin,  227 );//第36排
    my_shiftOut(dataPin2, clockPin,  36 );//第35排
    my_shiftOut(dataPin2, clockPin,  217 );//第34排
    my_shiftOut(dataPin2, clockPin,  153 );//第33排
    my_shiftOut(dataPin2, clockPin,  152 );//第32排
    my_shiftOut(dataPin2, clockPin,  227 );//第31排
    my_shiftOut(dataPin2, clockPin,  166 ); //第30排
    my_shiftOut(dataPin2, clockPin,  217 ); //第29排
    my_shiftOut(dataPin2, clockPin,  153 ); //第28排
    my_shiftOut(dataPin2, clockPin,  156 );//第27排
    my_shiftOut(dataPin2, clockPin,  227 );//第26排
    my_shiftOut(dataPin2, clockPin,  182 );//第25排
    my_shiftOut(dataPin2, clockPin,  217 );//第24排
    my_shiftOut(dataPin2, clockPin,  153 );//第23排
    my_shiftOut(dataPin2, clockPin,  156 );//第22排
    my_shiftOut(dataPin2, clockPin,  227 ); //第21排
    my_shiftOut(dataPin2, clockPin,  182 ); //第20排
    my_shiftOut(dataPin2, clockPin,  201 );//第19排
    my_shiftOut(dataPin2, clockPin,  153 );//第18排
    my_shiftOut(dataPin2, clockPin,  156 );//第17排
    my_shiftOut(dataPin2, clockPin,  227 );//第16排
    my_shiftOut(dataPin2, clockPin,  182 );//第15排
    my_shiftOut(dataPin2, clockPin,  201 );//第14排
    my_shiftOut(dataPin2, clockPin,  153 ); //第13排
    my_shiftOut(dataPin2, clockPin,  140 );//第12排
    my_shiftOut(dataPin2, clockPin,  97 );//第11排
    my_shiftOut(dataPin2, clockPin,  182 );//第10排
    my_shiftOut(dataPin2, clockPin,  77 );//第9排
    my_shiftOut(dataPin2, clockPin,  153 );//第8排
    my_shiftOut(dataPin2, clockPin,  140 );//第7排
    my_shiftOut(dataPin2, clockPin,  113 );//第6排
    my_shiftOut(dataPin2, clockPin,  146 );//第5排
    my_shiftOut(dataPin2, clockPin,  76 );//第4排
    my_shiftOut(dataPin2, clockPin,  153 );//第3排
    my_shiftOut(dataPin2, clockPin,  204 );//第2排
    my_shiftOut(dataPin2, clockPin,  113 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  146 );//第50排
    my_shiftOut(dataPin3, clockPin,  76 ); //第49排
    my_shiftOut(dataPin3, clockPin,  204 );//第48排
    my_shiftOut(dataPin3, clockPin,  206 ); //第47排
    my_shiftOut(dataPin3, clockPin,  113 );//第46排
    my_shiftOut(dataPin3, clockPin,  146 );//第45排
    my_shiftOut(dataPin3, clockPin,  108 );//第44排
    my_shiftOut(dataPin3, clockPin,  204 );//第43排
    my_shiftOut(dataPin3, clockPin,  198 );//第42排
    my_shiftOut(dataPin3, clockPin,  56 );//第41排
    my_shiftOut(dataPin3, clockPin,  219 );//第40排
    my_shiftOut(dataPin3, clockPin,  100 ); //第39排
    my_shiftOut(dataPin3, clockPin,  204 );//第38排
    my_shiftOut(dataPin3, clockPin,  198 ); //第37排
    my_shiftOut(dataPin3, clockPin,  56 );//第36排
    my_shiftOut(dataPin3, clockPin,  219 );//第35排
    my_shiftOut(dataPin3, clockPin,  102 );//第34排
    my_shiftOut(dataPin3, clockPin,  204 );//第33排
    my_shiftOut(dataPin3, clockPin,  103 );//第32排
    my_shiftOut(dataPin3, clockPin,  56 );//第31排
    my_shiftOut(dataPin3, clockPin,  91 ); //第30排
    my_shiftOut(dataPin3, clockPin,  38 ); //第29排
    my_shiftOut(dataPin3, clockPin,  102 ); //第28排
    my_shiftOut(dataPin3, clockPin,  99 );//第27排
    my_shiftOut(dataPin3, clockPin,  28 );//第26排
    my_shiftOut(dataPin3, clockPin,  73 );//第25排
    my_shiftOut(dataPin3, clockPin,  54 );//第24排
    my_shiftOut(dataPin3, clockPin,  102 );//第23排
    my_shiftOut(dataPin3, clockPin,  99 );//第22排
    my_shiftOut(dataPin3, clockPin,  156 ); //第21排
    my_shiftOut(dataPin3, clockPin,  73 ); //第20排
    my_shiftOut(dataPin3, clockPin,  178 );//第19排
    my_shiftOut(dataPin3, clockPin,  102 );//第18排
    my_shiftOut(dataPin3, clockPin,  51 );//第17排
    my_shiftOut(dataPin3, clockPin,  142 );//第16排
    my_shiftOut(dataPin3, clockPin,  109 );//第15排
    my_shiftOut(dataPin3, clockPin,  147 );//第14排
    my_shiftOut(dataPin3, clockPin,  51 ); //第13排
    my_shiftOut(dataPin3, clockPin,  49 );//第12排
    my_shiftOut(dataPin3, clockPin,  142 );//第11排
    my_shiftOut(dataPin3, clockPin,  108 );//第10排
    my_shiftOut(dataPin3, clockPin,  155 );//第9排
    my_shiftOut(dataPin3, clockPin,  51 );//第8排
    my_shiftOut(dataPin3, clockPin,  57 );//第7排
    my_shiftOut(dataPin3, clockPin,  199 );//第6排
    my_shiftOut(dataPin3, clockPin,  36 );//第5排
    my_shiftOut(dataPin3, clockPin,  153 );//第4排
    my_shiftOut(dataPin3, clockPin,  51 );//第3排
    my_shiftOut(dataPin3, clockPin,  152 );//第2排
    my_shiftOut(dataPin3, clockPin,  231 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}
void angle_40(){
    //start=millis();
    printf("The angle of reflection is 40 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  180 );//第1排
    my_shiftOut(dataPin, clockPin,  180 ); //第2排
    my_shiftOut(dataPin, clockPin,  146 );//第3排
    my_shiftOut(dataPin, clockPin,  100 ); //第4排
    my_shiftOut(dataPin, clockPin,  206 );//第5排
    my_shiftOut(dataPin, clockPin,  165 );//第6排
    my_shiftOut(dataPin, clockPin,  165 );//第7排
    my_shiftOut(dataPin, clockPin,  178 );//第8排
    my_shiftOut(dataPin, clockPin,  108 );//第9排
    my_shiftOut(dataPin, clockPin,  204 );//第10排
    my_shiftOut(dataPin, clockPin,  173 );//第40排
    my_shiftOut(dataPin, clockPin,  45 ); //第39排
    my_shiftOut(dataPin, clockPin,  182 );//第38排
    my_shiftOut(dataPin, clockPin,  76 ); //第37排
    my_shiftOut(dataPin, clockPin,  204 );//第36排
    my_shiftOut(dataPin, clockPin,  173 );//第35排
    my_shiftOut(dataPin, clockPin,  109 );//第34排
    my_shiftOut(dataPin, clockPin,  38 );//第33排
    my_shiftOut(dataPin, clockPin,  201 );//第32排
    my_shiftOut(dataPin, clockPin,  152 );//第31排
    my_shiftOut(dataPin, clockPin,  41 ); //第30排
    my_shiftOut(dataPin, clockPin,  105 ); //第29排
    my_shiftOut(dataPin, clockPin,  36 ); //第28排
    my_shiftOut(dataPin, clockPin,  217 );//第27排
    my_shiftOut(dataPin, clockPin,  153 );//第26排
    my_shiftOut(dataPin, clockPin,  107 );//第25排
    my_shiftOut(dataPin, clockPin,  75 );//第24排
    my_shiftOut(dataPin, clockPin,  108 );//第23排
    my_shiftOut(dataPin, clockPin,  153 );//第22排
    my_shiftOut(dataPin, clockPin,  153 ); //第21排
    my_shiftOut(dataPin, clockPin,  74 ); //第20排
    my_shiftOut(dataPin, clockPin,  91 );//第19排
    my_shiftOut(dataPin, clockPin,  109 );//第18排
    my_shiftOut(dataPin, clockPin,  155 );//第17排
    my_shiftOut(dataPin, clockPin,  49 );//第16排
    my_shiftOut(dataPin, clockPin,  74 );//第15排
    my_shiftOut(dataPin, clockPin,  91 );//第14排
    my_shiftOut(dataPin, clockPin,  77 ); //第13排
    my_shiftOut(dataPin, clockPin,  147 );//第12排
    my_shiftOut(dataPin, clockPin,  51 );//第11排
    my_shiftOut(dataPin, clockPin,  90 );//第10排
    my_shiftOut(dataPin, clockPin,  218 );//第9排
    my_shiftOut(dataPin, clockPin,  73 );//第8排
    my_shiftOut(dataPin, clockPin,  179 );//第7排
    my_shiftOut(dataPin, clockPin,  51 );//第6排
    my_shiftOut(dataPin, clockPin,  82 );//第5排
    my_shiftOut(dataPin, clockPin,  210 );//第4排
    my_shiftOut(dataPin, clockPin,  73 );//第3排
    my_shiftOut(dataPin, clockPin,  51 );//第2排
    my_shiftOut(dataPin, clockPin,  51 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  82 );//第50排
    my_shiftOut(dataPin1, clockPin,  146 ); //第49排
    my_shiftOut(dataPin1, clockPin,  217 );//第48排
    my_shiftOut(dataPin1, clockPin,  50 ); //第47排
    my_shiftOut(dataPin1, clockPin,  99 );//第46排
    my_shiftOut(dataPin1, clockPin,  86 );//第45排
    my_shiftOut(dataPin1, clockPin,  150 );//第44排
    my_shiftOut(dataPin1, clockPin,  217 );//第43排
    my_shiftOut(dataPin1, clockPin,  54 );//第42排
    my_shiftOut(dataPin1, clockPin,  103 );//第41排
    my_shiftOut(dataPin1, clockPin,  214 );//第40排
    my_shiftOut(dataPin1, clockPin,  150 ); //第39排
    my_shiftOut(dataPin1, clockPin,  219 );//第38排
    my_shiftOut(dataPin1, clockPin,  38 ); //第37排
    my_shiftOut(dataPin1, clockPin,  102 );//第36排
    my_shiftOut(dataPin1, clockPin,  214 );//第35排
    my_shiftOut(dataPin1, clockPin,  182 );//第34排
    my_shiftOut(dataPin1, clockPin,  219 );//第33排
    my_shiftOut(dataPin1, clockPin,  38 );//第32排
    my_shiftOut(dataPin1, clockPin,  102 );//第31排
    my_shiftOut(dataPin1, clockPin,  212 ); //第30排
    my_shiftOut(dataPin1, clockPin,  182 ); //第29排
    my_shiftOut(dataPin1, clockPin,  155 ); //第28排
    my_shiftOut(dataPin1, clockPin,  102 );//第27排
    my_shiftOut(dataPin1, clockPin,  102 );//第26排
    my_shiftOut(dataPin1, clockPin,  148 );//第25排
    my_shiftOut(dataPin1, clockPin,  182 );//第24排
    my_shiftOut(dataPin1, clockPin,  147 );//第23排
    my_shiftOut(dataPin1, clockPin,  102 );//第22排
    my_shiftOut(dataPin1, clockPin,  102 ); //第21排
    my_shiftOut(dataPin1, clockPin,  148 ); //第20排
    my_shiftOut(dataPin1, clockPin,  180 );//第19排
    my_shiftOut(dataPin1, clockPin,  147 );//第18排
    my_shiftOut(dataPin1, clockPin,  102 );//第17排
    my_shiftOut(dataPin1, clockPin,  102 );//第16排
    my_shiftOut(dataPin1, clockPin,  148 );//第15排
    my_shiftOut(dataPin1, clockPin,  180 );//第14排
    my_shiftOut(dataPin1, clockPin,  147 ); //第13排
    my_shiftOut(dataPin1, clockPin,  102 );//第12排
    my_shiftOut(dataPin1, clockPin,  102 );//第11排
    my_shiftOut(dataPin1, clockPin,  148 );//第10排
    my_shiftOut(dataPin1, clockPin,  180 );//第9排
    my_shiftOut(dataPin1, clockPin,  147 );//第8排
    my_shiftOut(dataPin1, clockPin,  102 );//第7排
    my_shiftOut(dataPin1, clockPin,  198 );//第6排
    my_shiftOut(dataPin1, clockPin,  148 );//第5排
    my_shiftOut(dataPin1, clockPin,  180 );//第4排
    my_shiftOut(dataPin1, clockPin,  147 );//第3排
    my_shiftOut(dataPin1, clockPin,  102 );//第2排
    my_shiftOut(dataPin1, clockPin,  198 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  148 );//第50排
    my_shiftOut(dataPin2, clockPin,  180 ); //第49排
    my_shiftOut(dataPin2, clockPin,  147 );//第48排
    my_shiftOut(dataPin2, clockPin,  102 ); //第47排
    my_shiftOut(dataPin2, clockPin,  198 );//第46排
    my_shiftOut(dataPin2, clockPin,  148 );//第45排
    my_shiftOut(dataPin2, clockPin,  180 );//第44排
    my_shiftOut(dataPin2, clockPin,  147 );//第43排
    my_shiftOut(dataPin2, clockPin,  102 );//第42排
    my_shiftOut(dataPin2, clockPin,  198 );//第41排
    my_shiftOut(dataPin2, clockPin,  148 );//第40排
    my_shiftOut(dataPin2, clockPin,  180 ); //第39排
    my_shiftOut(dataPin2, clockPin,  147 );//第38排
    my_shiftOut(dataPin2, clockPin,  102 ); //第37排
    my_shiftOut(dataPin2, clockPin,  102 );//第36排
    my_shiftOut(dataPin2, clockPin,  148 );//第35排
    my_shiftOut(dataPin2, clockPin,  180 );//第34排
    my_shiftOut(dataPin2, clockPin,  147 );//第33排
    my_shiftOut(dataPin2, clockPin,  102 );//第32排
    my_shiftOut(dataPin2, clockPin,  102 );//第31排
    my_shiftOut(dataPin2, clockPin,  148 ); //第30排
    my_shiftOut(dataPin2, clockPin,  182 ); //第29排
    my_shiftOut(dataPin2, clockPin,  147 ); //第28排
    my_shiftOut(dataPin2, clockPin,  102 );//第27排
    my_shiftOut(dataPin2, clockPin,  102 );//第26排
    my_shiftOut(dataPin2, clockPin,  212 );//第25排
    my_shiftOut(dataPin2, clockPin,  182 );//第24排
    my_shiftOut(dataPin2, clockPin,  155 );//第23排
    my_shiftOut(dataPin2, clockPin,  102 );//第22排
    my_shiftOut(dataPin2, clockPin,  102 ); //第21排
    my_shiftOut(dataPin2, clockPin,  214 ); //第20排
    my_shiftOut(dataPin2, clockPin,  182 );//第19排
    my_shiftOut(dataPin2, clockPin,  219 );//第18排
    my_shiftOut(dataPin2, clockPin,  38 );//第17排
    my_shiftOut(dataPin2, clockPin,  102 );//第16排
    my_shiftOut(dataPin2, clockPin,  214 );//第15排
    my_shiftOut(dataPin2, clockPin,  150 );//第14排
    my_shiftOut(dataPin2, clockPin,  219 ); //第13排
    my_shiftOut(dataPin2, clockPin,  38 );//第12排
    my_shiftOut(dataPin2, clockPin,  102 );//第11排
    my_shiftOut(dataPin2, clockPin,  86 );//第10排
    my_shiftOut(dataPin2, clockPin,  150 );//第9排
    my_shiftOut(dataPin2, clockPin,  217 );//第8排
    my_shiftOut(dataPin2, clockPin,  54 );//第7排
    my_shiftOut(dataPin2, clockPin,  103 );//第6排
    my_shiftOut(dataPin2, clockPin,  82 );//第5排
    my_shiftOut(dataPin2, clockPin,  146 );//第4排
    my_shiftOut(dataPin2, clockPin,  217 );//第3排
    my_shiftOut(dataPin2, clockPin,  50 );//第2排
    my_shiftOut(dataPin2, clockPin,  99 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  82 );//第50排
    my_shiftOut(dataPin3, clockPin,  210 ); //第49排
    my_shiftOut(dataPin3, clockPin,  73 );//第48排
    my_shiftOut(dataPin3, clockPin,  51 ); //第47排
    my_shiftOut(dataPin3, clockPin,  51 );//第46排
    my_shiftOut(dataPin3, clockPin,  90 );//第45排
    my_shiftOut(dataPin3, clockPin,  218 );//第44排
    my_shiftOut(dataPin3, clockPin,  73 );//第43排
    my_shiftOut(dataPin3, clockPin,  179 );//第42排
    my_shiftOut(dataPin3, clockPin,  51 );//第41排
    my_shiftOut(dataPin3, clockPin,  74 );//第40排
    my_shiftOut(dataPin3, clockPin,  91 ); //第39排
    my_shiftOut(dataPin3, clockPin,  77 );//第38排
    my_shiftOut(dataPin3, clockPin,  147 ); //第37排
    my_shiftOut(dataPin3, clockPin,  51 );//第36排
    my_shiftOut(dataPin3, clockPin,  74 );//第35排
    my_shiftOut(dataPin3, clockPin,  91 );//第34排
    my_shiftOut(dataPin3, clockPin,  109 );//第33排
    my_shiftOut(dataPin3, clockPin,  155 );//第32排
    my_shiftOut(dataPin3, clockPin,  49 );//第31排
    my_shiftOut(dataPin3, clockPin,  107 ); //第30排
    my_shiftOut(dataPin3, clockPin,  75 ); //第29排
    my_shiftOut(dataPin3, clockPin,  108 ); //第28排
    my_shiftOut(dataPin3, clockPin,  153 );//第27排
    my_shiftOut(dataPin3, clockPin,  153 );//第26排
    my_shiftOut(dataPin3, clockPin,  41 );//第25排
    my_shiftOut(dataPin3, clockPin,  105 );//第24排
    my_shiftOut(dataPin3, clockPin,  36 );//第23排
    my_shiftOut(dataPin3, clockPin,  217 );//第22排
    my_shiftOut(dataPin3, clockPin,  153 ); //第21排
    my_shiftOut(dataPin3, clockPin,  173 ); //第20排
    my_shiftOut(dataPin3, clockPin,  109 );//第19排
    my_shiftOut(dataPin3, clockPin,  38 );//第18排
    my_shiftOut(dataPin3, clockPin,  201 );//第17排
    my_shiftOut(dataPin3, clockPin,  152 );//第16排
    my_shiftOut(dataPin3, clockPin,  173 );//第15排
    my_shiftOut(dataPin3, clockPin,  45 );//第14排
    my_shiftOut(dataPin3, clockPin,  182 ); //第13排
    my_shiftOut(dataPin3, clockPin,  76 );//第12排
    my_shiftOut(dataPin3, clockPin,  204 );//第11排
    my_shiftOut(dataPin3, clockPin,  165 );//第10排
    my_shiftOut(dataPin3, clockPin,  165 );//第9排
    my_shiftOut(dataPin3, clockPin,  178 );//第8排
    my_shiftOut(dataPin3, clockPin,  108 );//第7排
    my_shiftOut(dataPin3, clockPin,  204 );//第6排
    my_shiftOut(dataPin3, clockPin,  180 );//第5排
    my_shiftOut(dataPin3, clockPin,  180 );//第4排
    my_shiftOut(dataPin3, clockPin,  146 );//第3排
    my_shiftOut(dataPin3, clockPin,  100 );//第2排
    my_shiftOut(dataPin3, clockPin,  206 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}
void angle_50(){
    //start=millis();
    printf("The angle of reflection is 50 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  85 );//第1排
    my_shiftOut(dataPin, clockPin,  74 ); //第2排
    my_shiftOut(dataPin, clockPin,  210 );//第3排
    my_shiftOut(dataPin, clockPin,  219 ); //第4排
    my_shiftOut(dataPin, clockPin,  108 );//第5排
    my_shiftOut(dataPin, clockPin,  181 );//第6排
    my_shiftOut(dataPin, clockPin,  90 );//第7排
    my_shiftOut(dataPin, clockPin,  150 );//第8排
    my_shiftOut(dataPin, clockPin,  146 );//第9排
    my_shiftOut(dataPin, clockPin,  77 );//第10排
    my_shiftOut(dataPin, clockPin,  169 );//第40排
    my_shiftOut(dataPin, clockPin,  86 ); //第39排
    my_shiftOut(dataPin, clockPin,  180 );//第38排
    my_shiftOut(dataPin, clockPin,  150 ); //第37排
    my_shiftOut(dataPin, clockPin,  73 );//第36排
    my_shiftOut(dataPin, clockPin,  171 );//第35排
    my_shiftOut(dataPin, clockPin,  84 );//第34排
    my_shiftOut(dataPin, clockPin,  180 );//第33排
    my_shiftOut(dataPin, clockPin,  182 );//第32排
    my_shiftOut(dataPin, clockPin,  217 );//第31排
    my_shiftOut(dataPin, clockPin,  170 ); //第30排
    my_shiftOut(dataPin, clockPin,  212 ); //第29排
    my_shiftOut(dataPin, clockPin,  165 ); //第28排
    my_shiftOut(dataPin, clockPin,  164 );//第27排
    my_shiftOut(dataPin, clockPin,  217 );//第26排
    my_shiftOut(dataPin, clockPin,  170 );//第25排
    my_shiftOut(dataPin, clockPin,  149 );//第24排
    my_shiftOut(dataPin, clockPin,  173 );//第23排
    my_shiftOut(dataPin, clockPin,  36 );//第22排
    my_shiftOut(dataPin, clockPin,  155 ); //第21排
    my_shiftOut(dataPin, clockPin,  170 ); //第20排
    my_shiftOut(dataPin, clockPin,  181 );//第19排
    my_shiftOut(dataPin, clockPin,  45 );//第18排
    my_shiftOut(dataPin, clockPin,  36 );//第17排
    my_shiftOut(dataPin, clockPin,  147 );//第16排
    my_shiftOut(dataPin, clockPin,  106 );//第15排
    my_shiftOut(dataPin, clockPin,  165 );//第14排
    my_shiftOut(dataPin, clockPin,  105 ); //第13排
    my_shiftOut(dataPin, clockPin,  109 );//第12排
    my_shiftOut(dataPin, clockPin,  179 );//第11排
    my_shiftOut(dataPin, clockPin,  90 );//第10排
    my_shiftOut(dataPin, clockPin,  173 );//第9排
    my_shiftOut(dataPin, clockPin,  105 );//第8排
    my_shiftOut(dataPin, clockPin,  109 );//第7排
    my_shiftOut(dataPin, clockPin,  178 );//第6排
    my_shiftOut(dataPin, clockPin,  86 );//第5排
    my_shiftOut(dataPin, clockPin,  169 );//第4排
    my_shiftOut(dataPin, clockPin,  75 );//第3排
    my_shiftOut(dataPin, clockPin,  105 );//第2排
    my_shiftOut(dataPin, clockPin,  182 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  84 );//第50排
    my_shiftOut(dataPin1, clockPin,  169 ); //第49排
    my_shiftOut(dataPin1, clockPin,  75 );//第48排
    my_shiftOut(dataPin1, clockPin,  73 ); //第47排
    my_shiftOut(dataPin1, clockPin,  54 );//第46排
    my_shiftOut(dataPin1, clockPin,  85 );//第45排
    my_shiftOut(dataPin1, clockPin,  171 );//第44排
    my_shiftOut(dataPin1, clockPin,  90 );//第43排
    my_shiftOut(dataPin1, clockPin,  73 );//第42排
    my_shiftOut(dataPin1, clockPin,  38 );//第41排
    my_shiftOut(dataPin1, clockPin,  85 );//第40排
    my_shiftOut(dataPin1, clockPin,  43 ); //第39排
    my_shiftOut(dataPin1, clockPin,  90 );//第38排
    my_shiftOut(dataPin1, clockPin,  73 ); //第37排
    my_shiftOut(dataPin1, clockPin,  38 );//第36排
    my_shiftOut(dataPin1, clockPin,  85 );//第35排
    my_shiftOut(dataPin1, clockPin,  42 );//第34排
    my_shiftOut(dataPin1, clockPin,  90 );//第33排
    my_shiftOut(dataPin1, clockPin,  91 );//第32排
    my_shiftOut(dataPin1, clockPin,  38 );//第31排
    my_shiftOut(dataPin1, clockPin,  85 ); //第30排
    my_shiftOut(dataPin1, clockPin,  106 ); //第29排
    my_shiftOut(dataPin1, clockPin,  82 ); //第28排
    my_shiftOut(dataPin1, clockPin,  91 );//第27排
    my_shiftOut(dataPin1, clockPin,  100 );//第26排
    my_shiftOut(dataPin1, clockPin,  85 );//第25排
    my_shiftOut(dataPin1, clockPin,  106 );//第24排
    my_shiftOut(dataPin1, clockPin,  82 );//第23排
    my_shiftOut(dataPin1, clockPin,  219 );//第22排
    my_shiftOut(dataPin1, clockPin,  100 ); //第21排
    my_shiftOut(dataPin1, clockPin,  85 ); //第20排
    my_shiftOut(dataPin1, clockPin,  106 );//第19排
    my_shiftOut(dataPin1, clockPin,  82 );//第18排
    my_shiftOut(dataPin1, clockPin,  219 );//第17排
    my_shiftOut(dataPin1, clockPin,  100 );//第16排
    my_shiftOut(dataPin1, clockPin,  85 );//第15排
    my_shiftOut(dataPin1, clockPin,  74 );//第14排
    my_shiftOut(dataPin1, clockPin,  210 ); //第13排
    my_shiftOut(dataPin1, clockPin,  219 );//第12排
    my_shiftOut(dataPin1, clockPin,  100 );//第11排
    my_shiftOut(dataPin1, clockPin,  85 );//第10排
    my_shiftOut(dataPin1, clockPin,  74 );//第9排
    my_shiftOut(dataPin1, clockPin,  210 );//第8排
    my_shiftOut(dataPin1, clockPin,  219 );//第7排
    my_shiftOut(dataPin1, clockPin,  108 );//第6排
    my_shiftOut(dataPin1, clockPin,  85 );//第5排
    my_shiftOut(dataPin1, clockPin,  74 );//第4排
    my_shiftOut(dataPin1, clockPin,  210 );//第3排
    my_shiftOut(dataPin1, clockPin,  219 );//第2排
    my_shiftOut(dataPin1, clockPin,  108 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  85 );//第50排
    my_shiftOut(dataPin2, clockPin,  74 ); //第49排
    my_shiftOut(dataPin2, clockPin,  210 );//第48排
    my_shiftOut(dataPin2, clockPin,  219 ); //第47排
    my_shiftOut(dataPin2, clockPin,  108 );//第46排
    my_shiftOut(dataPin2, clockPin,  85 );//第45排
    my_shiftOut(dataPin2, clockPin,  74 );//第44排
    my_shiftOut(dataPin2, clockPin,  210 );//第43排
    my_shiftOut(dataPin2, clockPin,  219 );//第42排
    my_shiftOut(dataPin2, clockPin,  108 );//第41排
    my_shiftOut(dataPin2, clockPin,  85 );//第40排
    my_shiftOut(dataPin2, clockPin,  74 ); //第39排
    my_shiftOut(dataPin2, clockPin,  210 );//第38排
    my_shiftOut(dataPin2, clockPin,  219 ); //第37排
    my_shiftOut(dataPin2, clockPin,  100 );//第36排
    my_shiftOut(dataPin2, clockPin,  85 );//第35排
    my_shiftOut(dataPin2, clockPin,  106 );//第34排
    my_shiftOut(dataPin2, clockPin,  82 );//第33排
    my_shiftOut(dataPin2, clockPin,  219 );//第32排
    my_shiftOut(dataPin2, clockPin,  100 );//第31排
    my_shiftOut(dataPin2, clockPin,  85 ); //第30排
    my_shiftOut(dataPin2, clockPin,  106 ); //第29排
    my_shiftOut(dataPin2, clockPin,  82 ); //第28排
    my_shiftOut(dataPin2, clockPin,  219 );//第27排
    my_shiftOut(dataPin2, clockPin,  100 );//第26排
    my_shiftOut(dataPin2, clockPin,  85 );//第25排
    my_shiftOut(dataPin2, clockPin,  106 );//第24排
    my_shiftOut(dataPin2, clockPin,  82 );//第23排
    my_shiftOut(dataPin2, clockPin,  91 );//第22排
    my_shiftOut(dataPin2, clockPin,  100 ); //第21排
    my_shiftOut(dataPin2, clockPin,  85 ); //第20排
    my_shiftOut(dataPin2, clockPin,  42 );//第19排
    my_shiftOut(dataPin2, clockPin,  90 );//第18排
    my_shiftOut(dataPin2, clockPin,  91 );//第17排
    my_shiftOut(dataPin2, clockPin,  38 );//第16排
    my_shiftOut(dataPin2, clockPin,  85 );//第15排
    my_shiftOut(dataPin2, clockPin,  43 );//第14排
    my_shiftOut(dataPin2, clockPin,  90 ); //第13排
    my_shiftOut(dataPin2, clockPin,  73 );//第12排
    my_shiftOut(dataPin2, clockPin,  38 );//第11排
    my_shiftOut(dataPin2, clockPin,  85 );//第10排
    my_shiftOut(dataPin2, clockPin,  171 );//第9排
    my_shiftOut(dataPin2, clockPin,  90 );//第8排
    my_shiftOut(dataPin2, clockPin,  73 );//第7排
    my_shiftOut(dataPin2, clockPin,  38 );//第6排
    my_shiftOut(dataPin2, clockPin,  84 );//第5排
    my_shiftOut(dataPin2, clockPin,  169 );//第4排
    my_shiftOut(dataPin2, clockPin,  75 );//第3排
    my_shiftOut(dataPin2, clockPin,  73 );//第2排
    my_shiftOut(dataPin2, clockPin,  54 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  86 );//第50排
    my_shiftOut(dataPin3, clockPin,  169 ); //第49排
    my_shiftOut(dataPin3, clockPin,  75 );//第48排
    my_shiftOut(dataPin3, clockPin,  105 ); //第47排
    my_shiftOut(dataPin3, clockPin,  182 );//第46排
    my_shiftOut(dataPin3, clockPin,  90 );//第45排
    my_shiftOut(dataPin3, clockPin,  173 );//第44排
    my_shiftOut(dataPin3, clockPin,  105 );//第43排
    my_shiftOut(dataPin3, clockPin,  109 );//第42排
    my_shiftOut(dataPin3, clockPin,  178 );//第41排
    my_shiftOut(dataPin3, clockPin,  106 );//第40排
    my_shiftOut(dataPin3, clockPin,  165 ); //第39排
    my_shiftOut(dataPin3, clockPin,  105 );//第38排
    my_shiftOut(dataPin3, clockPin,  109 ); //第37排
    my_shiftOut(dataPin3, clockPin,  179 );//第36排
    my_shiftOut(dataPin3, clockPin,  170 );//第35排
    my_shiftOut(dataPin3, clockPin,  181 );//第34排
    my_shiftOut(dataPin3, clockPin,  45 );//第33排
    my_shiftOut(dataPin3, clockPin,  36 );//第32排
    my_shiftOut(dataPin3, clockPin,  147 );//第31排
    my_shiftOut(dataPin3, clockPin,  170 ); //第30排
    my_shiftOut(dataPin3, clockPin,  149 ); //第29排
    my_shiftOut(dataPin3, clockPin,  173 ); //第28排
    my_shiftOut(dataPin3, clockPin,  36 );//第27排
    my_shiftOut(dataPin3, clockPin,  155 );//第26排
    my_shiftOut(dataPin3, clockPin,  170 );//第25排
    my_shiftOut(dataPin3, clockPin,  212 );//第24排
    my_shiftOut(dataPin3, clockPin,  165 );//第23排
    my_shiftOut(dataPin3, clockPin,  164 );//第22排
    my_shiftOut(dataPin3, clockPin,  217 ); //第21排
    my_shiftOut(dataPin3, clockPin,  171 ); //第20排
    my_shiftOut(dataPin3, clockPin,  84 );//第19排
    my_shiftOut(dataPin3, clockPin,  180 );//第18排
    my_shiftOut(dataPin3, clockPin,  182 );//第17排
    my_shiftOut(dataPin3, clockPin,  217 );//第16排
    my_shiftOut(dataPin3, clockPin,  169 );//第15排
    my_shiftOut(dataPin3, clockPin,  86 );//第14排
    my_shiftOut(dataPin3, clockPin,  180 ); //第13排
    my_shiftOut(dataPin3, clockPin,  150 );//第12排
    my_shiftOut(dataPin3, clockPin,  73 );//第11排
    my_shiftOut(dataPin3, clockPin,  181 );//第10排
    my_shiftOut(dataPin3, clockPin,  90 );//第9排
    my_shiftOut(dataPin3, clockPin,  150 );//第8排
    my_shiftOut(dataPin3, clockPin,  146 );//第7排
    my_shiftOut(dataPin3, clockPin,  77 );//第6排
    my_shiftOut(dataPin3, clockPin,  85 );//第5排
    my_shiftOut(dataPin3, clockPin,  74 );//第4排
    my_shiftOut(dataPin3, clockPin,  210 );//第3排
    my_shiftOut(dataPin3, clockPin,  219 );//第2排
    my_shiftOut(dataPin3, clockPin,  108 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");     
}
void angle_60(){
    //start=millis();
    printf("The angle of reflection is 60 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  42 );//第1排
    my_shiftOut(dataPin, clockPin,  173 ); //第2排
    my_shiftOut(dataPin, clockPin,  86 );//第3排
    my_shiftOut(dataPin, clockPin,  165 ); //第4排
    my_shiftOut(dataPin, clockPin,  164 );//第5排
    my_shiftOut(dataPin, clockPin,  170 );//第6排
    my_shiftOut(dataPin, clockPin,  170 );//第7排
    my_shiftOut(dataPin, clockPin,  84 );//第8排
    my_shiftOut(dataPin, clockPin,  173 );//第9排
    my_shiftOut(dataPin, clockPin,  36 );//第10排
    my_shiftOut(dataPin, clockPin,  170 );//第40排
    my_shiftOut(dataPin, clockPin,  170 ); //第39排
    my_shiftOut(dataPin, clockPin,  149 );//第38排
    my_shiftOut(dataPin, clockPin,  173 ); //第37排
    my_shiftOut(dataPin, clockPin,  45 );//第36排
    my_shiftOut(dataPin, clockPin,  170 );//第35排
    my_shiftOut(dataPin, clockPin,  170 );//第34排
    my_shiftOut(dataPin, clockPin,  181 );//第33排
    my_shiftOut(dataPin, clockPin,  41 );//第32排
    my_shiftOut(dataPin, clockPin,  109 );//第31排
    my_shiftOut(dataPin, clockPin,  170 ); //第30排
    my_shiftOut(dataPin, clockPin,  170 ); //第29排
    my_shiftOut(dataPin, clockPin,  165 ); //第28排
    my_shiftOut(dataPin, clockPin,  107 );//第27排
    my_shiftOut(dataPin, clockPin,  73 );//第26排
    my_shiftOut(dataPin, clockPin,  181 );//第25排
    my_shiftOut(dataPin, clockPin,  74 );//第24排
    my_shiftOut(dataPin, clockPin,  173 );//第23排
    my_shiftOut(dataPin, clockPin,  75 );//第22排
    my_shiftOut(dataPin, clockPin,  73 ); //第21排
    my_shiftOut(dataPin, clockPin,  85 ); //第20排
    my_shiftOut(dataPin, clockPin,  82 );//第19排
    my_shiftOut(dataPin, clockPin,  169 );//第18排
    my_shiftOut(dataPin, clockPin,  90 );//第17排
    my_shiftOut(dataPin, clockPin,  91 );//第16排
    my_shiftOut(dataPin, clockPin,  85 );//第15排
    my_shiftOut(dataPin, clockPin,  84 );//第14排
    my_shiftOut(dataPin, clockPin,  171 ); //第13排
    my_shiftOut(dataPin, clockPin,  90 );//第12排
    my_shiftOut(dataPin, clockPin,  91 );//第11排
    my_shiftOut(dataPin, clockPin,  85 );//第10排
    my_shiftOut(dataPin, clockPin,  85 );//第9排
    my_shiftOut(dataPin, clockPin,  42 );//第8排
    my_shiftOut(dataPin, clockPin,  82 );//第7排
    my_shiftOut(dataPin, clockPin,  218 );//第6排
    my_shiftOut(dataPin, clockPin,  85 );//第5排
    my_shiftOut(dataPin, clockPin,  85 );//第4排
    my_shiftOut(dataPin, clockPin,  106 );//第3排
    my_shiftOut(dataPin, clockPin,  210 );//第2排
    my_shiftOut(dataPin, clockPin,  210 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  85 );//第50排
    my_shiftOut(dataPin1, clockPin,  85 ); //第49排
    my_shiftOut(dataPin1, clockPin,  74 );//第48排
    my_shiftOut(dataPin1, clockPin,  214 ); //第47排
    my_shiftOut(dataPin1, clockPin,  146 );//第46排
    my_shiftOut(dataPin1, clockPin,  85 );//第45排
    my_shiftOut(dataPin1, clockPin,  85 );//第44排
    my_shiftOut(dataPin1, clockPin,  74 );//第43排
    my_shiftOut(dataPin1, clockPin,  150 );//第42排
    my_shiftOut(dataPin1, clockPin,  146 );//第41排
    my_shiftOut(dataPin1, clockPin,  85 );//第40排
    my_shiftOut(dataPin1, clockPin,  85 ); //第39排
    my_shiftOut(dataPin1, clockPin,  90 );//第38排
    my_shiftOut(dataPin1, clockPin,  148 ); //第37排
    my_shiftOut(dataPin1, clockPin,  150 );//第36排
    my_shiftOut(dataPin1, clockPin,  85 );//第35排
    my_shiftOut(dataPin1, clockPin,  85 );//第34排
    my_shiftOut(dataPin1, clockPin,  82 );//第33排
    my_shiftOut(dataPin1, clockPin,  148 );//第32排
    my_shiftOut(dataPin1, clockPin,  182 );//第31排
    my_shiftOut(dataPin1, clockPin,  84 ); //第30排
    my_shiftOut(dataPin1, clockPin,  85 ); //第29排
    my_shiftOut(dataPin1, clockPin,  82 ); //第28排
    my_shiftOut(dataPin1, clockPin,  180 );//第27排
    my_shiftOut(dataPin1, clockPin,  182 );//第26排
    my_shiftOut(dataPin1, clockPin,  90 );//第25排
    my_shiftOut(dataPin1, clockPin,  149 );//第24排
    my_shiftOut(dataPin1, clockPin,  82 );//第23排
    my_shiftOut(dataPin1, clockPin,  180 );//第22排
    my_shiftOut(dataPin1, clockPin,  182 ); //第21排
    my_shiftOut(dataPin1, clockPin,  74 ); //第20排
    my_shiftOut(dataPin1, clockPin,  181 );//第19排
    my_shiftOut(dataPin1, clockPin,  86 );//第18排
    my_shiftOut(dataPin1, clockPin,  181 );//第17排
    my_shiftOut(dataPin1, clockPin,  182 );//第16排
    my_shiftOut(dataPin1, clockPin,  106 );//第15排
    my_shiftOut(dataPin1, clockPin,  165 );//第14排
    my_shiftOut(dataPin1, clockPin,  86 ); //第13排
    my_shiftOut(dataPin1, clockPin,  181 );//第12排
    my_shiftOut(dataPin1, clockPin,  180 );//第11排
    my_shiftOut(dataPin1, clockPin,  106 );//第10排
    my_shiftOut(dataPin1, clockPin,  165 );//第9排
    my_shiftOut(dataPin1, clockPin,  86 );//第8排
    my_shiftOut(dataPin1, clockPin,  181 );//第7排
    my_shiftOut(dataPin1, clockPin,  180 );//第6排
    my_shiftOut(dataPin1, clockPin,  106 );//第5排
    my_shiftOut(dataPin1, clockPin,  173 );//第4排
    my_shiftOut(dataPin1, clockPin,  86 );//第3排
    my_shiftOut(dataPin1, clockPin,  181 );//第2排
    my_shiftOut(dataPin1, clockPin,  180 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  106 );//第50排
    my_shiftOut(dataPin2, clockPin,  173 ); //第49排
    my_shiftOut(dataPin2, clockPin,  86  );//第48排
    my_shiftOut(dataPin2, clockPin,  181 ); //第47排
    my_shiftOut(dataPin2, clockPin,  180 );//第46排
    my_shiftOut(dataPin2, clockPin,  106 );//第45排
    my_shiftOut(dataPin2, clockPin,  165 );//第44排
    my_shiftOut(dataPin2, clockPin,  86  );//第43排
    my_shiftOut(dataPin2, clockPin,  181 );//第42排
    my_shiftOut(dataPin2, clockPin,  180 );//第41排
    my_shiftOut(dataPin2, clockPin,  106 );//第40排
    my_shiftOut(dataPin2, clockPin,  165 ); //第39排
    my_shiftOut(dataPin2, clockPin,  86  );//第38排
    my_shiftOut(dataPin2, clockPin,  181 ); //第37排
    my_shiftOut(dataPin2, clockPin,  180 );//第36排
    my_shiftOut(dataPin2, clockPin,  74  );//第35排
    my_shiftOut(dataPin2, clockPin,  181 );//第34排
    my_shiftOut(dataPin2, clockPin,  86  );//第33排
    my_shiftOut(dataPin2, clockPin,  181 );//第32排
    my_shiftOut(dataPin2, clockPin,  182 );//第31排
    my_shiftOut(dataPin2, clockPin,  90  ); //第30排
    my_shiftOut(dataPin2, clockPin,  149 ); //第29排
    my_shiftOut(dataPin2, clockPin,  82  ); //第28排
    my_shiftOut(dataPin2, clockPin,  180 );//第27排
    my_shiftOut(dataPin2, clockPin,  182 );//第26排
    my_shiftOut(dataPin2, clockPin,  84  );//第25排
    my_shiftOut(dataPin2, clockPin,  85  );//第24排
    my_shiftOut(dataPin2, clockPin,  82  );//第23排
    my_shiftOut(dataPin2, clockPin,  180 );//第22排
    my_shiftOut(dataPin2, clockPin,  182 ); //第21排
    my_shiftOut(dataPin2, clockPin,  85  ); //第20排
    my_shiftOut(dataPin2, clockPin,  85  );//第19排
    my_shiftOut(dataPin2, clockPin,  82  );//第18排
    my_shiftOut(dataPin2, clockPin,  148 );//第17排
    my_shiftOut(dataPin2, clockPin,  182 );//第16排
    my_shiftOut(dataPin2, clockPin,  85  );//第15排
    my_shiftOut(dataPin2, clockPin,  85  );//第14排
    my_shiftOut(dataPin2, clockPin,  90  ); //第13排
    my_shiftOut(dataPin2, clockPin,  148 );//第12排
    my_shiftOut(dataPin2, clockPin,  150 );//第11排
    my_shiftOut(dataPin2, clockPin,  85  );//第10排
    my_shiftOut(dataPin2, clockPin,  85  );//第9排
    my_shiftOut(dataPin2, clockPin,  74  );//第8排
    my_shiftOut(dataPin2, clockPin,  150 );//第7排
    my_shiftOut(dataPin2, clockPin,  146 );//第6排
    my_shiftOut(dataPin2, clockPin,  85  );//第5排
    my_shiftOut(dataPin2, clockPin,  85  );//第4排
    my_shiftOut(dataPin2, clockPin,  74  );//第3排
    my_shiftOut(dataPin2, clockPin,  214 );//第2排
    my_shiftOut(dataPin2, clockPin,  146 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  85  );//第50排
    my_shiftOut(dataPin3, clockPin,  85  ); //第49排
    my_shiftOut(dataPin3, clockPin,  106 );//第48排
    my_shiftOut(dataPin3, clockPin,  210 ); //第47排
    my_shiftOut(dataPin3, clockPin,  210 );//第46排
    my_shiftOut(dataPin3, clockPin,  85  );//第45排
    my_shiftOut(dataPin3, clockPin,  85  );//第44排
    my_shiftOut(dataPin3, clockPin,  42  );//第43排
    my_shiftOut(dataPin3, clockPin,  82  );//第42排
    my_shiftOut(dataPin3, clockPin,  218 );//第41排
    my_shiftOut(dataPin3, clockPin,  85  );//第40排
    my_shiftOut(dataPin3, clockPin,  84  ); //第39排
    my_shiftOut(dataPin3, clockPin,  171 );//第38排
    my_shiftOut(dataPin3, clockPin,  90  ); //第37排
    my_shiftOut(dataPin3, clockPin,  91  );//第36排
    my_shiftOut(dataPin3, clockPin,  85  );//第35排
    my_shiftOut(dataPin3, clockPin,  82  );//第34排
    my_shiftOut(dataPin3, clockPin,  169 );//第33排
    my_shiftOut(dataPin3, clockPin,  90  );//第32排
    my_shiftOut(dataPin3, clockPin,  91  );//第31排
    my_shiftOut(dataPin3, clockPin,  181 ); //第30排
    my_shiftOut(dataPin3, clockPin,  74  ); //第29排
    my_shiftOut(dataPin3, clockPin,  173 ); //第28排
    my_shiftOut(dataPin3, clockPin,  75  );//第27排
    my_shiftOut(dataPin3, clockPin,  73  );//第26排
    my_shiftOut(dataPin3, clockPin,  170 );//第25排
    my_shiftOut(dataPin3, clockPin,  170 );//第24排
    my_shiftOut(dataPin3, clockPin,  165 );//第23排
    my_shiftOut(dataPin3, clockPin,  107 );//第22排
    my_shiftOut(dataPin3, clockPin,  73  ); //第21排
    my_shiftOut(dataPin3, clockPin,  170 ); //第20排
    my_shiftOut(dataPin3, clockPin,  170 );//第19排
    my_shiftOut(dataPin3, clockPin,  181 );//第18排
    my_shiftOut(dataPin3, clockPin,  41  );//第17排
    my_shiftOut(dataPin3, clockPin,  109 );//第16排
    my_shiftOut(dataPin3, clockPin,  170 );//第15排
    my_shiftOut(dataPin3, clockPin,  170 );//第14排
    my_shiftOut(dataPin3, clockPin,  149 ); //第13排
    my_shiftOut(dataPin3, clockPin,  173 );//第12排
    my_shiftOut(dataPin3, clockPin,  45  );//第11排
    my_shiftOut(dataPin3, clockPin,  170 );//第10排
    my_shiftOut(dataPin3, clockPin,  170 );//第9排
    my_shiftOut(dataPin3, clockPin,  84  );//第8排
    my_shiftOut(dataPin3, clockPin,  173 );//第7排
    my_shiftOut(dataPin3, clockPin,  36  );//第6排
    my_shiftOut(dataPin3, clockPin,  42  );//第5排
    my_shiftOut(dataPin3, clockPin,  173 );//第4排
    my_shiftOut(dataPin3, clockPin,  86  );//第3排
    my_shiftOut(dataPin3, clockPin,  165 );//第2排
    my_shiftOut(dataPin3, clockPin,  164 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}
void angle_inverse_10(){
    //start=millis();
    printf("The angle of reflection is -10 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  255 );//第1排
    my_shiftOut(dataPin, clockPin,  224 ); //第2排
    my_shiftOut(dataPin, clockPin,  31 );//第3排
    my_shiftOut(dataPin, clockPin,  15 ); //第4排
    my_shiftOut(dataPin, clockPin,  28 );//第5排
    my_shiftOut(dataPin, clockPin,  255 );//第6排
    my_shiftOut(dataPin, clockPin,  248 );//第7排
    my_shiftOut(dataPin, clockPin,  15 );//第8排
    my_shiftOut(dataPin, clockPin,  135 );//第9排
    my_shiftOut(dataPin, clockPin,  142 );//第10排
    my_shiftOut(dataPin, clockPin,  255 );//第40排
    my_shiftOut(dataPin, clockPin,  254 ); //第39排
    my_shiftOut(dataPin, clockPin,  7 );//第38排
    my_shiftOut(dataPin, clockPin,  195 ); //第37排
    my_shiftOut(dataPin, clockPin,  142 );//第36排
    my_shiftOut(dataPin, clockPin,  255 );//第35排
    my_shiftOut(dataPin, clockPin,  255 );//第34排
    my_shiftOut(dataPin, clockPin,  3 );//第33排
    my_shiftOut(dataPin, clockPin,  227 );//第32排
    my_shiftOut(dataPin, clockPin,  199 );//第31排
    my_shiftOut(dataPin, clockPin,  135 ); //第30排
    my_shiftOut(dataPin, clockPin,  255 ); //第29排
    my_shiftOut(dataPin, clockPin,  129 ); //第28排
    my_shiftOut(dataPin, clockPin,  225 );//第27排
    my_shiftOut(dataPin, clockPin,  199 );//第26排
    my_shiftOut(dataPin, clockPin,  0 );//第25排
    my_shiftOut(dataPin, clockPin,  127 );//第24排
    my_shiftOut(dataPin, clockPin,  192 );//第23排
    my_shiftOut(dataPin, clockPin,  240 );//第22排
    my_shiftOut(dataPin, clockPin,  227 ); //第21排
    my_shiftOut(dataPin, clockPin,  0 ); //第20排
    my_shiftOut(dataPin, clockPin,  31 );//第19排
    my_shiftOut(dataPin, clockPin,  224 );//第18排
    my_shiftOut(dataPin, clockPin,  240 );//第17排
    my_shiftOut(dataPin, clockPin,  227 );//第16排
    my_shiftOut(dataPin, clockPin,  0 );//第15排
    my_shiftOut(dataPin, clockPin,  15 );//第14排
    my_shiftOut(dataPin, clockPin,  240 ); //第13排
    my_shiftOut(dataPin, clockPin,  120 );//第12排
    my_shiftOut(dataPin, clockPin,  113 );//第11排
    my_shiftOut(dataPin, clockPin,  0 );//第10排
    my_shiftOut(dataPin, clockPin,  3 );//第9排
    my_shiftOut(dataPin, clockPin,  248 );//第8排
    my_shiftOut(dataPin, clockPin,  120 );//第7排
    my_shiftOut(dataPin, clockPin,  113 );//第6排
    my_shiftOut(dataPin, clockPin,  0 );//第5排
    my_shiftOut(dataPin, clockPin,  1 );//第4排
    my_shiftOut(dataPin, clockPin,  248 );//第3排
    my_shiftOut(dataPin, clockPin,  60 );//第2排
    my_shiftOut(dataPin, clockPin,  113 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  0 );//第50排
    my_shiftOut(dataPin1, clockPin,  0 ); //第49排
    my_shiftOut(dataPin1, clockPin,  252 );//第48排
    my_shiftOut(dataPin1, clockPin,  60 ); //第47排
    my_shiftOut(dataPin1, clockPin,  57 );//第46排
    my_shiftOut(dataPin1, clockPin,  0 );//第45排
    my_shiftOut(dataPin1, clockPin,  0 );//第44排
    my_shiftOut(dataPin1, clockPin,  252 );//第43排
    my_shiftOut(dataPin1, clockPin,  30 );//第42排
    my_shiftOut(dataPin1, clockPin,  56 );//第41排
    my_shiftOut(dataPin1, clockPin,  0 );//第40排
    my_shiftOut(dataPin1, clockPin,  0 ); //第39排
    my_shiftOut(dataPin1, clockPin,  126 );//第38排
    my_shiftOut(dataPin1, clockPin,  30 ); //第37排
    my_shiftOut(dataPin1, clockPin,  56 );//第36排
    my_shiftOut(dataPin1, clockPin,  124 );//第35排
    my_shiftOut(dataPin1, clockPin,  0 );//第34排
    my_shiftOut(dataPin1, clockPin,  126 );//第33排
    my_shiftOut(dataPin1, clockPin,  30 );//第32排
    my_shiftOut(dataPin1, clockPin,  56 );//第31排
    my_shiftOut(dataPin1, clockPin,  254 ); //第30排
    my_shiftOut(dataPin1, clockPin,  0 ); //第29排
    my_shiftOut(dataPin1, clockPin,  62 ); //第28排
    my_shiftOut(dataPin1, clockPin,  14 );//第27排
    my_shiftOut(dataPin1, clockPin,  28 );//第26排
    my_shiftOut(dataPin1, clockPin,  255 );//第25排
    my_shiftOut(dataPin1, clockPin,  0 );//第24排
    my_shiftOut(dataPin1, clockPin,  63 );//第23排
    my_shiftOut(dataPin1, clockPin,  15 );//第22排
    my_shiftOut(dataPin1, clockPin,  28 ); //第21排
    my_shiftOut(dataPin1, clockPin,  255 ); //第20排
    my_shiftOut(dataPin1, clockPin,  128 );//第19排
    my_shiftOut(dataPin1, clockPin,  63 );//第18排
    my_shiftOut(dataPin1, clockPin,  15 );//第17排
    my_shiftOut(dataPin1, clockPin,  28 );//第16排
    my_shiftOut(dataPin1, clockPin,  255 );//第15排
    my_shiftOut(dataPin1, clockPin,  192 );//第14排
    my_shiftOut(dataPin1, clockPin,  31 ); //第13排
    my_shiftOut(dataPin1, clockPin,  15 );//第12排
    my_shiftOut(dataPin1, clockPin,  28 );//第11排
    my_shiftOut(dataPin1, clockPin,  255 );//第10排
    my_shiftOut(dataPin1, clockPin,  192 );//第9排
    my_shiftOut(dataPin1, clockPin,  31 );//第8排
    my_shiftOut(dataPin1, clockPin,  15 );//第7排
    my_shiftOut(dataPin1, clockPin,  28 );//第6排
    my_shiftOut(dataPin1, clockPin,  255 );//第5排
    my_shiftOut(dataPin1, clockPin,  192 );//第4排
    my_shiftOut(dataPin1, clockPin,  31 );//第3排
    my_shiftOut(dataPin1, clockPin,  15 );//第2排
    my_shiftOut(dataPin1, clockPin,  28 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  255 );//第50排
    my_shiftOut(dataPin2, clockPin,  192 ); //第49排
    my_shiftOut(dataPin2, clockPin,  31 );//第48排
    my_shiftOut(dataPin2, clockPin,  15 ); //第47排
    my_shiftOut(dataPin2, clockPin,  28 );//第46排
    my_shiftOut(dataPin2, clockPin,  255 );//第45排
    my_shiftOut(dataPin2, clockPin,  192 );//第44排
    my_shiftOut(dataPin2, clockPin,  31 );//第43排
    my_shiftOut(dataPin2, clockPin,  15 );//第42排
    my_shiftOut(dataPin2, clockPin,  28 );//第41排
    my_shiftOut(dataPin2, clockPin,  255 );//第40排
    my_shiftOut(dataPin2, clockPin,  192 ); //第39排
    my_shiftOut(dataPin2, clockPin,  31 );//第38排
    my_shiftOut(dataPin2, clockPin,  15 ); //第37排
    my_shiftOut(dataPin2, clockPin,  28 );//第36排
    my_shiftOut(dataPin2, clockPin,  255 );//第35排
    my_shiftOut(dataPin2, clockPin,  128 );//第34排
    my_shiftOut(dataPin2, clockPin,  63 );//第33排
    my_shiftOut(dataPin2, clockPin,  15 );//第32排
    my_shiftOut(dataPin2, clockPin,  28 );//第31排
    my_shiftOut(dataPin2, clockPin,  255 ); //第30排
    my_shiftOut(dataPin2, clockPin,  0 ); //第29排
    my_shiftOut(dataPin2, clockPin,  63 ); //第28排
    my_shiftOut(dataPin2, clockPin,  15 );//第27排
    my_shiftOut(dataPin2, clockPin,  28 );//第26排
    my_shiftOut(dataPin2, clockPin,  254 );//第25排
    my_shiftOut(dataPin2, clockPin,  0 );//第24排
    my_shiftOut(dataPin2, clockPin,  62 );//第23排
    my_shiftOut(dataPin2, clockPin,  14 );//第22排
    my_shiftOut(dataPin2, clockPin,  28 ); //第21排
    my_shiftOut(dataPin2, clockPin,  124 ); //第20排
    my_shiftOut(dataPin2, clockPin,  0 );//第19排
    my_shiftOut(dataPin2, clockPin,  126 );//第18排
    my_shiftOut(dataPin2, clockPin,  30 );//第17排
    my_shiftOut(dataPin2, clockPin,  56 );//第16排
    my_shiftOut(dataPin2, clockPin,  0 );//第15排
    my_shiftOut(dataPin2, clockPin,  0 );//第14排
    my_shiftOut(dataPin2, clockPin,  126 ); //第13排
    my_shiftOut(dataPin2, clockPin,  30 );//第12排
    my_shiftOut(dataPin2, clockPin,  56 );//第11排
    my_shiftOut(dataPin2, clockPin,  0 );//第10排
    my_shiftOut(dataPin2, clockPin,  0 );//第9排
    my_shiftOut(dataPin2, clockPin,  252 );//第8排
    my_shiftOut(dataPin2, clockPin,  30 );//第7排
    my_shiftOut(dataPin2, clockPin,  56 );//第6排
    my_shiftOut(dataPin2, clockPin,  0 );//第5排
    my_shiftOut(dataPin2, clockPin,  0 );//第4排
    my_shiftOut(dataPin2, clockPin,  252 );//第3排
    my_shiftOut(dataPin2, clockPin,  60 );//第2排
    my_shiftOut(dataPin2, clockPin,  57 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  0 );//第50排
    my_shiftOut(dataPin3, clockPin,  1 ); //第49排
    my_shiftOut(dataPin3, clockPin,  248 );//第48排
    my_shiftOut(dataPin3, clockPin,  60 ); //第47排
    my_shiftOut(dataPin3, clockPin,  113 );//第46排
    my_shiftOut(dataPin3, clockPin,  0 );//第45排
    my_shiftOut(dataPin3, clockPin,  3 );//第44排
    my_shiftOut(dataPin3, clockPin,  248 );//第43排
    my_shiftOut(dataPin3, clockPin,  120 );//第42排
    my_shiftOut(dataPin3, clockPin,  113 );//第41排
    my_shiftOut(dataPin3, clockPin,  0 );//第40排
    my_shiftOut(dataPin3, clockPin,  15 ); //第39排
    my_shiftOut(dataPin3, clockPin,  240 );//第38排
    my_shiftOut(dataPin3, clockPin,  120 ); //第37排
    my_shiftOut(dataPin3, clockPin,  113 );//第36排
    my_shiftOut(dataPin3, clockPin,  0 );//第35排
    my_shiftOut(dataPin3, clockPin,  31 );//第34排
    my_shiftOut(dataPin3, clockPin,  224 );//第33排
    my_shiftOut(dataPin3, clockPin,  240 );//第32排
    my_shiftOut(dataPin3, clockPin,  227 );//第31排
    my_shiftOut(dataPin3, clockPin,  0 ); //第30排
    my_shiftOut(dataPin3, clockPin,  127 ); //第29排
    my_shiftOut(dataPin3, clockPin,  192 ); //第28排
    my_shiftOut(dataPin3, clockPin,  240 );//第27排
    my_shiftOut(dataPin3, clockPin,  227 );//第26排
    my_shiftOut(dataPin3, clockPin,  135 );//第25排
    my_shiftOut(dataPin3, clockPin,  255 );//第24排
    my_shiftOut(dataPin3, clockPin,  129 );//第23排
    my_shiftOut(dataPin3, clockPin,  225 );//第22排
    my_shiftOut(dataPin3, clockPin,  199 ); //第21排
    my_shiftOut(dataPin3, clockPin,  255 ); //第20排
    my_shiftOut(dataPin3, clockPin,  255 );//第19排
    my_shiftOut(dataPin3, clockPin,  3 );//第18排
    my_shiftOut(dataPin3, clockPin,  227 );//第17排
    my_shiftOut(dataPin3, clockPin,  199 );//第16排
    my_shiftOut(dataPin3, clockPin,  255 );//第15排
    my_shiftOut(dataPin3, clockPin,  254 );//第14排
    my_shiftOut(dataPin3, clockPin,  7 ); //第13排
    my_shiftOut(dataPin3, clockPin,  195 );//第12排
    my_shiftOut(dataPin3, clockPin,  142 );//第11排
    my_shiftOut(dataPin3, clockPin,  255 );//第10排
    my_shiftOut(dataPin3, clockPin,  248 );//第9排
    my_shiftOut(dataPin3, clockPin,  15 );//第8排
    my_shiftOut(dataPin3, clockPin,  135 );//第7排
    my_shiftOut(dataPin3, clockPin,  142 );//第6排
    my_shiftOut(dataPin3, clockPin,  255 );//第5排
    my_shiftOut(dataPin3, clockPin,  224 );//第4排
    my_shiftOut(dataPin3, clockPin,  31 );//第3排
    my_shiftOut(dataPin3, clockPin,  15 );//第2排
    my_shiftOut(dataPin3, clockPin,  28 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}
void angle_inverse_20(){
    //start=millis();
    printf("The angle of reflection is -20 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  15 );//第1排
    my_shiftOut(dataPin, clockPin,  135 ); //第2排
    my_shiftOut(dataPin, clockPin,  28 );//第3排
    my_shiftOut(dataPin, clockPin,  99 ); //第4排
    my_shiftOut(dataPin, clockPin,  51 );//第5排
    my_shiftOut(dataPin, clockPin,  7 );//第6排
    my_shiftOut(dataPin, clockPin,  195 );//第7排
    my_shiftOut(dataPin, clockPin,  142 );//第8排
    my_shiftOut(dataPin, clockPin,  115 );//第9排
    my_shiftOut(dataPin, clockPin,  51 );//第10排
    my_shiftOut(dataPin, clockPin,  3 );//第40排
    my_shiftOut(dataPin, clockPin,  195 ); //第39排
    my_shiftOut(dataPin, clockPin,  198 );//第38排
    my_shiftOut(dataPin, clockPin,  49 ); //第37排
    my_shiftOut(dataPin, clockPin,  153 );//第36排
    my_shiftOut(dataPin, clockPin,  129 );//第35排
    my_shiftOut(dataPin, clockPin,  225 );//第34排
    my_shiftOut(dataPin, clockPin,  199 );//第33排
    my_shiftOut(dataPin, clockPin,  57 );//第32排
    my_shiftOut(dataPin, clockPin,  153 );//第31排
    my_shiftOut(dataPin, clockPin,  192 ); //第30排
    my_shiftOut(dataPin, clockPin,  241 ); //第29排
    my_shiftOut(dataPin, clockPin,  227 ); //第28排
    my_shiftOut(dataPin, clockPin,  25 );//第27排
    my_shiftOut(dataPin, clockPin,  137 );//第26排
    my_shiftOut(dataPin, clockPin,  224 );//第25排
    my_shiftOut(dataPin, clockPin,  240 );//第24排
    my_shiftOut(dataPin, clockPin,  227 );//第23排
    my_shiftOut(dataPin, clockPin,  152 );//第22排
    my_shiftOut(dataPin, clockPin,  204 ); //第21排
    my_shiftOut(dataPin, clockPin,  240 ); //第20排
    my_shiftOut(dataPin, clockPin,  120 );//第19排
    my_shiftOut(dataPin, clockPin,  243 );//第18排
    my_shiftOut(dataPin, clockPin,  156 );//第17排
    my_shiftOut(dataPin, clockPin,  204 );//第16排
    my_shiftOut(dataPin, clockPin,  248 );//第15排
    my_shiftOut(dataPin, clockPin,  56 );//第14排
    my_shiftOut(dataPin, clockPin,  113 ); //第13排
    my_shiftOut(dataPin, clockPin,  140 );//第12排
    my_shiftOut(dataPin, clockPin,  204 );//第11排
    my_shiftOut(dataPin, clockPin,  248 );//第10排
    my_shiftOut(dataPin, clockPin,  60 );//第9排
    my_shiftOut(dataPin, clockPin,  113 );//第8排
    my_shiftOut(dataPin, clockPin,  204 );//第7排
    my_shiftOut(dataPin, clockPin,  102 );//第6排
    my_shiftOut(dataPin, clockPin,  252 );//第5排
    my_shiftOut(dataPin, clockPin,  28 );//第4排
    my_shiftOut(dataPin, clockPin,  57 );//第3排
    my_shiftOut(dataPin, clockPin,  206 );//第2排
    my_shiftOut(dataPin, clockPin,  102 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  124 );//第50排
    my_shiftOut(dataPin1, clockPin,  30 ); //第49排
    my_shiftOut(dataPin1, clockPin,  56 );//第48排
    my_shiftOut(dataPin1, clockPin,  198 ); //第47排
    my_shiftOut(dataPin1, clockPin,  102 );//第46排
    my_shiftOut(dataPin1, clockPin,  126 );//第45排
    my_shiftOut(dataPin1, clockPin,  30 );//第44排
    my_shiftOut(dataPin1, clockPin,  56 );//第43排
    my_shiftOut(dataPin1, clockPin,  198 );//第42排
    my_shiftOut(dataPin1, clockPin,  102 );//第41排
    my_shiftOut(dataPin1, clockPin,  62 );//第40排
    my_shiftOut(dataPin1, clockPin,  14 ); //第39排
    my_shiftOut(dataPin1, clockPin,  24 );//第38排
    my_shiftOut(dataPin1, clockPin,  230 ); //第37排
    my_shiftOut(dataPin1, clockPin,  102 );//第36排
    my_shiftOut(dataPin1, clockPin,  63 );//第35排
    my_shiftOut(dataPin1, clockPin,  15 );//第34排
    my_shiftOut(dataPin1, clockPin,  28 );//第33排
    my_shiftOut(dataPin1, clockPin,  230 );//第32排
    my_shiftOut(dataPin1, clockPin,  50 );//第31排
    my_shiftOut(dataPin1, clockPin,  31 ); //第30排
    my_shiftOut(dataPin1, clockPin,  15 ); //第29排
    my_shiftOut(dataPin1, clockPin,  28 ); //第28排
    my_shiftOut(dataPin1, clockPin,  231 );//第27排
    my_shiftOut(dataPin1, clockPin,  50 );//第26排
    my_shiftOut(dataPin1, clockPin,  31 );//第25排
    my_shiftOut(dataPin1, clockPin,  15 );//第24排
    my_shiftOut(dataPin1, clockPin,  28 );//第23排
    my_shiftOut(dataPin1, clockPin,  103 );//第22排
    my_shiftOut(dataPin1, clockPin,  51 ); //第21排
    my_shiftOut(dataPin1, clockPin,  31 ); //第20排
    my_shiftOut(dataPin1, clockPin,  7 );//第19排
    my_shiftOut(dataPin1, clockPin,  28 );//第18排
    my_shiftOut(dataPin1, clockPin,  99 );//第17排
    my_shiftOut(dataPin1, clockPin,  51 );//第16排
    my_shiftOut(dataPin1, clockPin,  31 );//第15排
    my_shiftOut(dataPin1, clockPin,  135 );//第14排
    my_shiftOut(dataPin1, clockPin,  28 ); //第13排
    my_shiftOut(dataPin1, clockPin,  99 );//第12排
    my_shiftOut(dataPin1, clockPin,  51 );//第11排
    my_shiftOut(dataPin1, clockPin,  15 );//第10排
    my_shiftOut(dataPin1, clockPin,  135 );//第9排
    my_shiftOut(dataPin1, clockPin,  28 );//第8排
    my_shiftOut(dataPin1, clockPin,  99 );//第7排
    my_shiftOut(dataPin1, clockPin,  51 );//第6排
    my_shiftOut(dataPin1, clockPin,  15 );//第5排
    my_shiftOut(dataPin1, clockPin,  135 );//第4排
    my_shiftOut(dataPin1, clockPin,  28 );//第3排
    my_shiftOut(dataPin1, clockPin,  99 );//第2排
    my_shiftOut(dataPin1, clockPin,  51 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  15 );//第50排
    my_shiftOut(dataPin2, clockPin,  135 ); //第49排
    my_shiftOut(dataPin2, clockPin,  28 );//第48排
    my_shiftOut(dataPin2, clockPin,  99 ); //第47排
    my_shiftOut(dataPin2, clockPin,  51 );//第46排
    my_shiftOut(dataPin2, clockPin,  15 );//第45排
    my_shiftOut(dataPin2, clockPin,  135 );//第44排
    my_shiftOut(dataPin2, clockPin,  28 );//第43排
    my_shiftOut(dataPin2, clockPin,  99 );//第42排
    my_shiftOut(dataPin2, clockPin,  51 );//第41排
    my_shiftOut(dataPin2, clockPin,  31 );//第40排
    my_shiftOut(dataPin2, clockPin,  135 ); //第39排
    my_shiftOut(dataPin2, clockPin,  28 );//第38排
    my_shiftOut(dataPin2, clockPin,  99 ); //第37排
    my_shiftOut(dataPin2, clockPin,  51 );//第36排
    my_shiftOut(dataPin2, clockPin,  31 );//第35排
    my_shiftOut(dataPin2, clockPin,  7 );//第34排
    my_shiftOut(dataPin2, clockPin,  28 );//第33排
    my_shiftOut(dataPin2, clockPin,  99 );//第32排
    my_shiftOut(dataPin2, clockPin,  51 );//第31排
    my_shiftOut(dataPin2, clockPin,  31 ); //第30排
    my_shiftOut(dataPin2, clockPin,  15 ); //第29排
    my_shiftOut(dataPin2, clockPin,  28 ); //第28排
    my_shiftOut(dataPin2, clockPin,  103 );//第27排
    my_shiftOut(dataPin2, clockPin,  51 );//第26排
    my_shiftOut(dataPin2, clockPin,  31 );//第25排
    my_shiftOut(dataPin2, clockPin,  15 );//第24排
    my_shiftOut(dataPin2, clockPin,  28 );//第23排
    my_shiftOut(dataPin2, clockPin,  231 );//第22排
    my_shiftOut(dataPin2, clockPin,  50 ); //第21排
    my_shiftOut(dataPin2, clockPin,  63 ); //第20排
    my_shiftOut(dataPin2, clockPin,  15 );//第19排
    my_shiftOut(dataPin2, clockPin,  28 );//第18排
    my_shiftOut(dataPin2, clockPin,  230 );//第17排
    my_shiftOut(dataPin2, clockPin,  50 );//第16排
    my_shiftOut(dataPin2, clockPin,  62 );//第15排
    my_shiftOut(dataPin2, clockPin,  14 );//第14排
    my_shiftOut(dataPin2, clockPin,  24 ); //第13排
    my_shiftOut(dataPin2, clockPin,  230 );//第12排
    my_shiftOut(dataPin2, clockPin,  102 );//第11排
    my_shiftOut(dataPin2, clockPin,  126 );//第10排
    my_shiftOut(dataPin2, clockPin,  30 );//第9排
    my_shiftOut(dataPin2, clockPin,  56 );//第8排
    my_shiftOut(dataPin2, clockPin,  198 );//第7排
    my_shiftOut(dataPin2, clockPin,  102 );//第6排
    my_shiftOut(dataPin2, clockPin,  124 );//第5排
    my_shiftOut(dataPin2, clockPin,  30 );//第4排
    my_shiftOut(dataPin2, clockPin,  56 );//第3排
    my_shiftOut(dataPin2, clockPin,  198 );//第2排
    my_shiftOut(dataPin2, clockPin,  102 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  252 );//第50排
    my_shiftOut(dataPin3, clockPin,  28 ); //第49排
    my_shiftOut(dataPin3, clockPin,  57 );//第48排
    my_shiftOut(dataPin3, clockPin,  206 ); //第47排
    my_shiftOut(dataPin3, clockPin,  102 );//第46排
    my_shiftOut(dataPin3, clockPin,  248 );//第45排
    my_shiftOut(dataPin3, clockPin,  60 );//第44排
    my_shiftOut(dataPin3, clockPin,  113 );//第43排
    my_shiftOut(dataPin3, clockPin,  204 );//第42排
    my_shiftOut(dataPin3, clockPin,  102 );//第41排
    my_shiftOut(dataPin3, clockPin,  248 );//第40排
    my_shiftOut(dataPin3, clockPin,  56 ); //第39排
    my_shiftOut(dataPin3, clockPin,  113 );//第38排
    my_shiftOut(dataPin3, clockPin,  140 ); //第37排
    my_shiftOut(dataPin3, clockPin,  204 );//第36排
    my_shiftOut(dataPin3, clockPin,  240 );//第35排
    my_shiftOut(dataPin3, clockPin,  120 );//第34排
    my_shiftOut(dataPin3, clockPin,  243 );//第33排
    my_shiftOut(dataPin3, clockPin,  156 );//第32排
    my_shiftOut(dataPin3, clockPin,  204 );//第31排
    my_shiftOut(dataPin3, clockPin,  224 ); //第30排
    my_shiftOut(dataPin3, clockPin,  240 ); //第29排
    my_shiftOut(dataPin3, clockPin,  227 ); //第28排
    my_shiftOut(dataPin3, clockPin,  152 );//第27排
    my_shiftOut(dataPin3, clockPin,  204 );//第26排
    my_shiftOut(dataPin3, clockPin,  192 );//第25排
    my_shiftOut(dataPin3, clockPin,  241 );//第24排
    my_shiftOut(dataPin3, clockPin,  227 );//第23排
    my_shiftOut(dataPin3, clockPin,  25 );//第22排
    my_shiftOut(dataPin3, clockPin,  137 ); //第21排
    my_shiftOut(dataPin3, clockPin,  129 ); //第20排
    my_shiftOut(dataPin3, clockPin,  225 );//第19排
    my_shiftOut(dataPin3, clockPin,  199 );//第18排
    my_shiftOut(dataPin3, clockPin,  57 );//第17排
    my_shiftOut(dataPin3, clockPin,  153 );//第16排
    my_shiftOut(dataPin3, clockPin,  3 );//第15排
    my_shiftOut(dataPin3, clockPin,  195 );//第14排
    my_shiftOut(dataPin3, clockPin,  198 ); //第13排
    my_shiftOut(dataPin3, clockPin,  49 );//第12排
    my_shiftOut(dataPin3, clockPin,  153 );//第11排
    my_shiftOut(dataPin3, clockPin,  7 );//第10排
    my_shiftOut(dataPin3, clockPin,  195 );//第9排
    my_shiftOut(dataPin3, clockPin,  142 );//第8排
    my_shiftOut(dataPin3, clockPin,  115 );//第7排
    my_shiftOut(dataPin3, clockPin,  51 );//第6排
    my_shiftOut(dataPin3, clockPin,  15 );//第5排
    my_shiftOut(dataPin3, clockPin,  135 );//第4排
    my_shiftOut(dataPin3, clockPin,  28 );//第3排
    my_shiftOut(dataPin3, clockPin,  99 );//第2排
    my_shiftOut(dataPin3, clockPin,  51 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}
void angle_inverse_30(){
    //start=millis();
    printf("The angle of reflection is -30 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  231 );//第1排
    my_shiftOut(dataPin, clockPin,  25 ); //第2排
    my_shiftOut(dataPin, clockPin,  204 );//第3排
    my_shiftOut(dataPin, clockPin,  153 ); //第4排
    my_shiftOut(dataPin, clockPin,  36 );//第5排
    my_shiftOut(dataPin, clockPin,  227 );//第6排
    my_shiftOut(dataPin, clockPin,  156 );//第7排
    my_shiftOut(dataPin, clockPin,  204 );//第8排
    my_shiftOut(dataPin, clockPin,  217 );//第9排
    my_shiftOut(dataPin, clockPin,  54 );//第10排
    my_shiftOut(dataPin, clockPin,  113 );//第40排
    my_shiftOut(dataPin, clockPin,  140 ); //第39排
    my_shiftOut(dataPin, clockPin,  204 );//第38排
    my_shiftOut(dataPin, clockPin,  201 ); //第37排
    my_shiftOut(dataPin, clockPin,  182 );//第36排
    my_shiftOut(dataPin, clockPin,  113 );//第35排
    my_shiftOut(dataPin, clockPin,  204 );//第34排
    my_shiftOut(dataPin, clockPin,  102 );//第33排
    my_shiftOut(dataPin, clockPin,  77 );//第32排
    my_shiftOut(dataPin, clockPin,  146 );//第31排
    my_shiftOut(dataPin, clockPin,  57 ); //第30排
    my_shiftOut(dataPin, clockPin,  198 ); //第29排
    my_shiftOut(dataPin, clockPin,  102 ); //第28排
    my_shiftOut(dataPin, clockPin,  108 );//第27排
    my_shiftOut(dataPin, clockPin,  146 );//第26排
    my_shiftOut(dataPin, clockPin,  56 );//第25排
    my_shiftOut(dataPin, clockPin,  198 );//第24排
    my_shiftOut(dataPin, clockPin,  102 );//第23排
    my_shiftOut(dataPin, clockPin,  100 );//第22排
    my_shiftOut(dataPin, clockPin,  218 ); //第21排
    my_shiftOut(dataPin, clockPin,  28 ); //第20排
    my_shiftOut(dataPin, clockPin,  230 );//第19排
    my_shiftOut(dataPin, clockPin,  51 );//第18排
    my_shiftOut(dataPin, clockPin,  102 );//第17排
    my_shiftOut(dataPin, clockPin,  219 );//第16排
    my_shiftOut(dataPin, clockPin,  28 );//第15排
    my_shiftOut(dataPin, clockPin,  99 );//第14排
    my_shiftOut(dataPin, clockPin,  51 ); //第13排
    my_shiftOut(dataPin, clockPin,  38 );//第12排
    my_shiftOut(dataPin, clockPin,  219 );//第11排
    my_shiftOut(dataPin, clockPin,  28 );//第10排
    my_shiftOut(dataPin, clockPin,  99 );//第9排
    my_shiftOut(dataPin, clockPin,  51 );//第8排
    my_shiftOut(dataPin, clockPin,  54 );//第7排
    my_shiftOut(dataPin, clockPin,  73 );//第6排
    my_shiftOut(dataPin, clockPin,  142 );//第5排
    my_shiftOut(dataPin, clockPin,  115 );//第4排
    my_shiftOut(dataPin, clockPin,  51 );//第3排
    my_shiftOut(dataPin, clockPin,  50 );//第2排
    my_shiftOut(dataPin, clockPin,  73 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  142 );//第50排
    my_shiftOut(dataPin1, clockPin,  51 ); //第49排
    my_shiftOut(dataPin1, clockPin,  153 );//第48排
    my_shiftOut(dataPin1, clockPin,  50 ); //第47排
    my_shiftOut(dataPin1, clockPin,  73 );//第46排
    my_shiftOut(dataPin1, clockPin,  142 );//第45排
    my_shiftOut(dataPin1, clockPin,  49 );//第44排
    my_shiftOut(dataPin1, clockPin,  153 );//第43排
    my_shiftOut(dataPin1, clockPin,  178 );//第42排
    my_shiftOut(dataPin1, clockPin,  109 );//第41排
    my_shiftOut(dataPin1, clockPin,  134 );//第40排
    my_shiftOut(dataPin1, clockPin,  49 ); //第39排
    my_shiftOut(dataPin1, clockPin,  153 );//第38排
    my_shiftOut(dataPin1, clockPin,  147 ); //第37排
    my_shiftOut(dataPin1, clockPin,  109 );//第36排
    my_shiftOut(dataPin1, clockPin,  199 );//第35排
    my_shiftOut(dataPin1, clockPin,  57 );//第34排
    my_shiftOut(dataPin1, clockPin,  153 );//第33排
    my_shiftOut(dataPin1, clockPin,  147 );//第32排
    my_shiftOut(dataPin1, clockPin,  109 );//第31排
    my_shiftOut(dataPin1, clockPin,  199 ); //第30排
    my_shiftOut(dataPin1, clockPin,  57 ); //第29排
    my_shiftOut(dataPin1, clockPin,  153 ); //第28排
    my_shiftOut(dataPin1, clockPin,  155 );//第27排
    my_shiftOut(dataPin1, clockPin,  109 );//第26排
    my_shiftOut(dataPin1, clockPin,  199 );//第25排
    my_shiftOut(dataPin1, clockPin,  57 );//第24排
    my_shiftOut(dataPin1, clockPin,  153 );//第23排
    my_shiftOut(dataPin1, clockPin,  155 );//第22排
    my_shiftOut(dataPin1, clockPin,  101 ); //第21排
    my_shiftOut(dataPin1, clockPin,  199 ); //第20排
    my_shiftOut(dataPin1, clockPin,  25 );//第19排
    my_shiftOut(dataPin1, clockPin,  153 );//第18排
    my_shiftOut(dataPin1, clockPin,  155 );//第17排
    my_shiftOut(dataPin1, clockPin,  36 );//第16排
    my_shiftOut(dataPin1, clockPin,  199 );//第15排
    my_shiftOut(dataPin1, clockPin,  25 );//第14排
    my_shiftOut(dataPin1, clockPin,  141 ); //第13排
    my_shiftOut(dataPin1, clockPin,  155 );//第12排
    my_shiftOut(dataPin1, clockPin,  36 );//第11排
    my_shiftOut(dataPin1, clockPin,  199 );//第10排
    my_shiftOut(dataPin1, clockPin,  25 );//第9排
    my_shiftOut(dataPin1, clockPin,  205 );//第8排
    my_shiftOut(dataPin1, clockPin,  155 );//第7排
    my_shiftOut(dataPin1, clockPin,  36 );//第6排
    my_shiftOut(dataPin1, clockPin,  199 );//第5排
    my_shiftOut(dataPin1, clockPin,  25 );//第4排
    my_shiftOut(dataPin1, clockPin,  205 );//第3排
    my_shiftOut(dataPin1, clockPin,  155 );//第2排
    my_shiftOut(dataPin1, clockPin,  36 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  199 );//第50排
    my_shiftOut(dataPin2, clockPin,  25 ); //第49排
    my_shiftOut(dataPin2, clockPin,  205 );//第48排
    my_shiftOut(dataPin2, clockPin,  155 ); //第47排
    my_shiftOut(dataPin2, clockPin,  36 );//第46排
    my_shiftOut(dataPin2, clockPin,  199 );//第45排
    my_shiftOut(dataPin2, clockPin,  25 );//第44排
    my_shiftOut(dataPin2, clockPin,  205 );//第43排
    my_shiftOut(dataPin2, clockPin,  155 );//第42排
    my_shiftOut(dataPin2, clockPin,  36 );//第41排
    my_shiftOut(dataPin2, clockPin,  199 );//第40排
    my_shiftOut(dataPin2, clockPin,  25 ); //第39排
    my_shiftOut(dataPin2, clockPin,  141 );//第38排
    my_shiftOut(dataPin2, clockPin,  155 ); //第37排
    my_shiftOut(dataPin2, clockPin,  36 );//第36排
    my_shiftOut(dataPin2, clockPin,  199 );//第35排
    my_shiftOut(dataPin2, clockPin,  25 );//第34排
    my_shiftOut(dataPin2, clockPin,  153 );//第33排
    my_shiftOut(dataPin2, clockPin,  155 );//第32排
    my_shiftOut(dataPin2, clockPin,  36 );//第31排
    my_shiftOut(dataPin2, clockPin,  199 ); //第30排
    my_shiftOut(dataPin2, clockPin,  57 ); //第29排
    my_shiftOut(dataPin2, clockPin,  153 ); //第28排
    my_shiftOut(dataPin2, clockPin,  155 );//第27排
    my_shiftOut(dataPin2, clockPin,  101 );//第26排
    my_shiftOut(dataPin2, clockPin,  199 );//第25排
    my_shiftOut(dataPin2, clockPin,  57 );//第24排
    my_shiftOut(dataPin2, clockPin,  153 );//第23排
    my_shiftOut(dataPin2, clockPin,  155 );//第22排
    my_shiftOut(dataPin2, clockPin,  109 ); //第21排
    my_shiftOut(dataPin2, clockPin,  199 ); //第20排
    my_shiftOut(dataPin2, clockPin,  57 );//第19排
    my_shiftOut(dataPin2, clockPin,  153 );//第18排
    my_shiftOut(dataPin2, clockPin,  147 );//第17排
    my_shiftOut(dataPin2, clockPin,  109 );//第16排
    my_shiftOut(dataPin2, clockPin,  134 );//第15排
    my_shiftOut(dataPin2, clockPin,  49 );//第14排
    my_shiftOut(dataPin2, clockPin,  153 ); //第13排
    my_shiftOut(dataPin2, clockPin,  147 );//第12排
    my_shiftOut(dataPin2, clockPin,  109 );//第11排
    my_shiftOut(dataPin2, clockPin,  142 );//第10排
    my_shiftOut(dataPin2, clockPin,  49 );//第9排
    my_shiftOut(dataPin2, clockPin,  153 );//第8排
    my_shiftOut(dataPin2, clockPin,  178 );//第7排
    my_shiftOut(dataPin2, clockPin,  109 );//第6排
    my_shiftOut(dataPin2, clockPin,  142 );//第5排
    my_shiftOut(dataPin2, clockPin,  51 );//第4排
    my_shiftOut(dataPin2, clockPin,  153 );//第3排
    my_shiftOut(dataPin2, clockPin,  50 );//第2排
    my_shiftOut(dataPin2, clockPin,  73 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  142 );//第50排
    my_shiftOut(dataPin3, clockPin,  115 ); //第49排
    my_shiftOut(dataPin3, clockPin,  51 );//第48排
    my_shiftOut(dataPin3, clockPin,  50 ); //第47排
    my_shiftOut(dataPin3, clockPin,  73 );//第46排
    my_shiftOut(dataPin3, clockPin,  28 );//第45排
    my_shiftOut(dataPin3, clockPin,  99 );//第44排
    my_shiftOut(dataPin3, clockPin,  51 );//第43排
    my_shiftOut(dataPin3, clockPin,  54 );//第42排
    my_shiftOut(dataPin3, clockPin,  73 );//第41排
    my_shiftOut(dataPin3, clockPin,  28 );//第40排
    my_shiftOut(dataPin3, clockPin,  99 ); //第39排
    my_shiftOut(dataPin3, clockPin,  51 );//第38排
    my_shiftOut(dataPin3, clockPin,  38 ); //第37排
    my_shiftOut(dataPin3, clockPin,  219 );//第36排
    my_shiftOut(dataPin3, clockPin,  28 );//第35排
    my_shiftOut(dataPin3, clockPin,  230 );//第34排
    my_shiftOut(dataPin3, clockPin,  51 );//第33排
    my_shiftOut(dataPin3, clockPin,  102 );//第32排
    my_shiftOut(dataPin3, clockPin,  219 );//第31排
    my_shiftOut(dataPin3, clockPin,  56 ); //第30排
    my_shiftOut(dataPin3, clockPin,  198 ); //第29排
    my_shiftOut(dataPin3, clockPin,  102 ); //第28排
    my_shiftOut(dataPin3, clockPin,  100 );//第27排
    my_shiftOut(dataPin3, clockPin,  218 );//第26排
    my_shiftOut(dataPin3, clockPin,  57 );//第25排
    my_shiftOut(dataPin3, clockPin,  198 );//第24排
    my_shiftOut(dataPin3, clockPin,  102 );//第23排
    my_shiftOut(dataPin3, clockPin,  108 );//第22排
    my_shiftOut(dataPin3, clockPin,  146 ); //第21排
    my_shiftOut(dataPin3, clockPin,  113 ); //第20排
    my_shiftOut(dataPin3, clockPin,  204 );//第19排
    my_shiftOut(dataPin3, clockPin,  102 );//第18排
    my_shiftOut(dataPin3, clockPin,  77 );//第17排
    my_shiftOut(dataPin3, clockPin,  146 );//第16排
    my_shiftOut(dataPin3, clockPin,  113 );//第15排
    my_shiftOut(dataPin3, clockPin,  140 );//第14排
    my_shiftOut(dataPin3, clockPin,  204 ); //第13排
    my_shiftOut(dataPin3, clockPin,  201 );//第12排
    my_shiftOut(dataPin3, clockPin,  182 );//第11排
    my_shiftOut(dataPin3, clockPin,  227 );//第10排
    my_shiftOut(dataPin3, clockPin,  156 );//第9排
    my_shiftOut(dataPin3, clockPin,  204 );//第8排
    my_shiftOut(dataPin3, clockPin,  217 );//第7排
    my_shiftOut(dataPin3, clockPin,  54 );//第6排
    my_shiftOut(dataPin3, clockPin,  231 );//第5排
    my_shiftOut(dataPin3, clockPin,  25 );//第4排
    my_shiftOut(dataPin3, clockPin,  204 );//第3排
    my_shiftOut(dataPin3, clockPin,  153 );//第2排
    my_shiftOut(dataPin3, clockPin,  36 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}
void angle_inverse_40(){
    //start=millis();
    printf("The angle of reflection is -40 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  115 );//第1排
    my_shiftOut(dataPin, clockPin,  38 ); //第2排
    my_shiftOut(dataPin, clockPin,  73 );//第3排
    my_shiftOut(dataPin, clockPin,  45 ); //第4排
    my_shiftOut(dataPin, clockPin,  45 );//第5排
    my_shiftOut(dataPin, clockPin,  51 );//第6排
    my_shiftOut(dataPin, clockPin,  54 );//第7排
    my_shiftOut(dataPin, clockPin,  77 );//第8排
    my_shiftOut(dataPin, clockPin,  165 );//第9排
    my_shiftOut(dataPin, clockPin,  165 );//第10排
    my_shiftOut(dataPin, clockPin,  51 );//第40排
    my_shiftOut(dataPin, clockPin,  50 ); //第39排
    my_shiftOut(dataPin, clockPin,  109 );//第38排
    my_shiftOut(dataPin, clockPin,  180 ); //第37排
    my_shiftOut(dataPin, clockPin,  181 );//第36排
    my_shiftOut(dataPin, clockPin,  25 );//第35排
    my_shiftOut(dataPin, clockPin,  147 );//第34排
    my_shiftOut(dataPin, clockPin,  100 );//第33排
    my_shiftOut(dataPin, clockPin,  182 );//第32排
    my_shiftOut(dataPin, clockPin,  181 );//第31排
    my_shiftOut(dataPin, clockPin,  153 ); //第30排
    my_shiftOut(dataPin, clockPin,  155 ); //第29排
    my_shiftOut(dataPin, clockPin,  36 ); //第28排
    my_shiftOut(dataPin, clockPin,  150 );//第27排
    my_shiftOut(dataPin, clockPin,  148 );//第26排
    my_shiftOut(dataPin, clockPin,  153 );//第25排
    my_shiftOut(dataPin, clockPin,  153 );//第24排
    my_shiftOut(dataPin, clockPin,  54 );//第23排
    my_shiftOut(dataPin, clockPin,  210 );//第22排
    my_shiftOut(dataPin, clockPin,  214 ); //第21排
    my_shiftOut(dataPin, clockPin,  140 ); //第20排
    my_shiftOut(dataPin, clockPin,  217 );//第19排
    my_shiftOut(dataPin, clockPin,  182 );//第18排
    my_shiftOut(dataPin, clockPin,  218 );//第17排
    my_shiftOut(dataPin, clockPin,  82 );//第16排
    my_shiftOut(dataPin, clockPin,  204 );//第15排
    my_shiftOut(dataPin, clockPin,  201 );//第14排
    my_shiftOut(dataPin, clockPin,  178 ); //第13排
    my_shiftOut(dataPin, clockPin,  218 );//第12排
    my_shiftOut(dataPin, clockPin,  82 );//第11排
    my_shiftOut(dataPin, clockPin,  204 );//第10排
    my_shiftOut(dataPin, clockPin,  205 );//第9排
    my_shiftOut(dataPin, clockPin,  146 );//第8排
    my_shiftOut(dataPin, clockPin,  91 );//第7排
    my_shiftOut(dataPin, clockPin,  90 );//第6排
    my_shiftOut(dataPin, clockPin,  204 );//第5排
    my_shiftOut(dataPin, clockPin,  204 );//第4排
    my_shiftOut(dataPin, clockPin,  146 );//第3排
    my_shiftOut(dataPin, clockPin,  75 );//第2排
    my_shiftOut(dataPin, clockPin,  74 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  198 );//第50排
    my_shiftOut(dataPin1, clockPin,  76 ); //第49排
    my_shiftOut(dataPin1, clockPin,  155 );//第48排
    my_shiftOut(dataPin1, clockPin,  73 ); //第47排
    my_shiftOut(dataPin1, clockPin,  74 );//第46排
    my_shiftOut(dataPin1, clockPin,  230 );//第45排
    my_shiftOut(dataPin1, clockPin,  108 );//第44排
    my_shiftOut(dataPin1, clockPin,  155 );//第43排
    my_shiftOut(dataPin1, clockPin,  105 );//第42排
    my_shiftOut(dataPin1, clockPin,  106 );//第41排
    my_shiftOut(dataPin1, clockPin,  102 );//第40排
    my_shiftOut(dataPin1, clockPin,  100 ); //第39排
    my_shiftOut(dataPin1, clockPin,  219 );//第38排
    my_shiftOut(dataPin1, clockPin,  105 ); //第37排
    my_shiftOut(dataPin1, clockPin,  107 );//第36排
    my_shiftOut(dataPin1, clockPin,  102 );//第35排
    my_shiftOut(dataPin1, clockPin,  100 );//第34排
    my_shiftOut(dataPin1, clockPin,  219 );//第33排
    my_shiftOut(dataPin1, clockPin,  109 );//第32排
    my_shiftOut(dataPin1, clockPin,  107 );//第31排
    my_shiftOut(dataPin1, clockPin,  102 ); //第30排
    my_shiftOut(dataPin1, clockPin,  102 ); //第29排
    my_shiftOut(dataPin1, clockPin,  217 ); //第28排
    my_shiftOut(dataPin1, clockPin,  109 );//第27排
    my_shiftOut(dataPin1, clockPin,  43 );//第26排
    my_shiftOut(dataPin1, clockPin,  102 );//第25排
    my_shiftOut(dataPin1, clockPin,  102 );//第24排
    my_shiftOut(dataPin1, clockPin,  201 );//第23排
    my_shiftOut(dataPin1, clockPin,  109 );//第22排
    my_shiftOut(dataPin1, clockPin,  41 ); //第21排
    my_shiftOut(dataPin1, clockPin,  102 ); //第20排
    my_shiftOut(dataPin1, clockPin,  102 );//第19排
    my_shiftOut(dataPin1, clockPin,  201 );//第18排
    my_shiftOut(dataPin1, clockPin,  45 );//第17排
    my_shiftOut(dataPin1, clockPin,  41 );//第16排
    my_shiftOut(dataPin1, clockPin,  102 );//第15排
    my_shiftOut(dataPin1, clockPin,  102 );//第14排
    my_shiftOut(dataPin1, clockPin,  201 ); //第13排
    my_shiftOut(dataPin1, clockPin,  45 );//第12排
    my_shiftOut(dataPin1, clockPin,  41 );//第11排
    my_shiftOut(dataPin1, clockPin,  99 );//第10排
    my_shiftOut(dataPin1, clockPin,  102 );//第9排
    my_shiftOut(dataPin1, clockPin,  201 );//第8排
    my_shiftOut(dataPin1, clockPin,  45 );//第7排
    my_shiftOut(dataPin1, clockPin,  41 );//第6排
    my_shiftOut(dataPin1, clockPin,  99 );//第5排
    my_shiftOut(dataPin1, clockPin,  102 );//第4排
    my_shiftOut(dataPin1, clockPin,  201 );//第3排
    my_shiftOut(dataPin1, clockPin,  45 );//第2排
    my_shiftOut(dataPin1, clockPin,  41 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  99 );//第50排
    my_shiftOut(dataPin2, clockPin,  102 ); //第49排
    my_shiftOut(dataPin2, clockPin,  201 );//第48排
    my_shiftOut(dataPin2, clockPin,  45 ); //第47排
    my_shiftOut(dataPin2, clockPin,  41 );//第46排
    my_shiftOut(dataPin2, clockPin,  99 );//第45排
    my_shiftOut(dataPin2, clockPin,  102 );//第44排
    my_shiftOut(dataPin2, clockPin,  201 );//第43排
    my_shiftOut(dataPin2, clockPin,  45 );//第42排
    my_shiftOut(dataPin2, clockPin,  41 );//第41排
    my_shiftOut(dataPin2, clockPin,  102 );//第40排
    my_shiftOut(dataPin2, clockPin,  102 ); //第39排
    my_shiftOut(dataPin2, clockPin,  201 );//第38排
    my_shiftOut(dataPin2, clockPin,  45 ); //第37排
    my_shiftOut(dataPin2, clockPin,  41 );//第36排
    my_shiftOut(dataPin2, clockPin,  102 );//第35排
    my_shiftOut(dataPin2, clockPin,  102 );//第34排
    my_shiftOut(dataPin2, clockPin,  201 );//第33排
    my_shiftOut(dataPin2, clockPin,  45 );//第32排
    my_shiftOut(dataPin2, clockPin,  41 );//第31排
    my_shiftOut(dataPin2, clockPin,  102 ); //第30排
    my_shiftOut(dataPin2, clockPin,  102 ); //第29排
    my_shiftOut(dataPin2, clockPin,  201 ); //第28排
    my_shiftOut(dataPin2, clockPin,  109 );//第27排
    my_shiftOut(dataPin2, clockPin,  41 );//第26排
    my_shiftOut(dataPin2, clockPin,  102 );//第25排
    my_shiftOut(dataPin2, clockPin,  102 );//第24排
    my_shiftOut(dataPin2, clockPin,  217 );//第23排
    my_shiftOut(dataPin2, clockPin,  109 );//第22排
    my_shiftOut(dataPin2, clockPin,  43 ); //第21排
    my_shiftOut(dataPin2, clockPin,  102 ); //第20排
    my_shiftOut(dataPin2, clockPin,  100 );//第19排
    my_shiftOut(dataPin2, clockPin,  219 );//第18排
    my_shiftOut(dataPin2, clockPin,  109 );//第17排
    my_shiftOut(dataPin2, clockPin,  107 );//第16排
    my_shiftOut(dataPin2, clockPin,  102 );//第15排
    my_shiftOut(dataPin2, clockPin,  100 );//第14排
    my_shiftOut(dataPin2, clockPin,  219 ); //第13排
    my_shiftOut(dataPin2, clockPin,  105 );//第12排
    my_shiftOut(dataPin2, clockPin,  107 );//第11排
    my_shiftOut(dataPin2, clockPin,  230 );//第10排
    my_shiftOut(dataPin2, clockPin,  108 );//第9排
    my_shiftOut(dataPin2, clockPin,  155 );//第8排
    my_shiftOut(dataPin2, clockPin,  105 );//第7排
    my_shiftOut(dataPin2, clockPin,  106 );//第6排
    my_shiftOut(dataPin2, clockPin,  198 );//第5排
    my_shiftOut(dataPin2, clockPin,  76 );//第4排
    my_shiftOut(dataPin2, clockPin,  155 );//第3排
    my_shiftOut(dataPin2, clockPin,  73 );//第2排
    my_shiftOut(dataPin2, clockPin,  74 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  204 );//第50排
    my_shiftOut(dataPin3, clockPin,  204 ); //第49排
    my_shiftOut(dataPin3, clockPin,  146 );//第48排
    my_shiftOut(dataPin3, clockPin,  75 ); //第47排
    my_shiftOut(dataPin3, clockPin,  74 );//第46排
    my_shiftOut(dataPin3, clockPin,  204 );//第45排
    my_shiftOut(dataPin3, clockPin,  205 );//第44排
    my_shiftOut(dataPin3, clockPin,  146 );//第43排
    my_shiftOut(dataPin3, clockPin,  91 );//第42排
    my_shiftOut(dataPin3, clockPin,  90 );//第41排
    my_shiftOut(dataPin3, clockPin,  204 );//第40排
    my_shiftOut(dataPin3, clockPin,  201 ); //第39排
    my_shiftOut(dataPin3, clockPin,  178 );//第38排
    my_shiftOut(dataPin3, clockPin,  218 ); //第37排
    my_shiftOut(dataPin3, clockPin,  82 );//第36排
    my_shiftOut(dataPin3, clockPin,  140 );//第35排
    my_shiftOut(dataPin3, clockPin,  217 );//第34排
    my_shiftOut(dataPin3, clockPin,  182 );//第33排
    my_shiftOut(dataPin3, clockPin,  218 );//第32排
    my_shiftOut(dataPin3, clockPin,  82 );//第31排
    my_shiftOut(dataPin3, clockPin,  153 ); //第30排
    my_shiftOut(dataPin3, clockPin,  153 ); //第29排
    my_shiftOut(dataPin3, clockPin,  54 ); //第28排
    my_shiftOut(dataPin3, clockPin,  210 );//第27排
    my_shiftOut(dataPin3, clockPin,  214 );//第26排
    my_shiftOut(dataPin3, clockPin,  153 );//第25排
    my_shiftOut(dataPin3, clockPin,  155 );//第24排
    my_shiftOut(dataPin3, clockPin,  36 );//第23排
    my_shiftOut(dataPin3, clockPin,  150 );//第22排
    my_shiftOut(dataPin3, clockPin,  148 ); //第21排
    my_shiftOut(dataPin3, clockPin,  25 ); //第20排
    my_shiftOut(dataPin3, clockPin,  147 );//第19排
    my_shiftOut(dataPin3, clockPin,  100 );//第18排
    my_shiftOut(dataPin3, clockPin,  182 );//第17排
    my_shiftOut(dataPin3, clockPin,  181 );//第16排
    my_shiftOut(dataPin3, clockPin,  51 );//第15排
    my_shiftOut(dataPin3, clockPin,  50 );//第14排
    my_shiftOut(dataPin3, clockPin,  109 ); //第13排
    my_shiftOut(dataPin3, clockPin,  180 );//第12排
    my_shiftOut(dataPin3, clockPin,  181 );//第11排
    my_shiftOut(dataPin3, clockPin,  51 );//第10排
    my_shiftOut(dataPin3, clockPin,  54 );//第9排
    my_shiftOut(dataPin3, clockPin,  77 );//第8排
    my_shiftOut(dataPin3, clockPin,  165 );//第7排
    my_shiftOut(dataPin3, clockPin,  165 );//第6排
    my_shiftOut(dataPin3, clockPin,  115 );//第5排
    my_shiftOut(dataPin3, clockPin,  38 );//第4排
    my_shiftOut(dataPin3, clockPin,  73 );//第3排
    my_shiftOut(dataPin3, clockPin,  45 );//第2排
    my_shiftOut(dataPin3, clockPin,  45 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");     
}
void angle_inverse_50(){
    //start=millis();
    printf("The angle of reflection is -50 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  54 );//第1排
    my_shiftOut(dataPin, clockPin,  219 ); //第2排
    my_shiftOut(dataPin, clockPin,  75 );//第3排
    my_shiftOut(dataPin, clockPin,  82 ); //第4排
    my_shiftOut(dataPin, clockPin,  170 );//第5排
    my_shiftOut(dataPin, clockPin,  178 );//第6排
    my_shiftOut(dataPin, clockPin,  73 );//第7排
    my_shiftOut(dataPin, clockPin,  105 );//第8排
    my_shiftOut(dataPin, clockPin,  90 );//第9排
    my_shiftOut(dataPin, clockPin,  173 );//第10排
    my_shiftOut(dataPin, clockPin,  146 );//第40排
    my_shiftOut(dataPin, clockPin,  105 ); //第39排
    my_shiftOut(dataPin, clockPin,  45 );//第38排
    my_shiftOut(dataPin, clockPin,  106 ); //第37排
    my_shiftOut(dataPin, clockPin,  149 );//第36排
    my_shiftOut(dataPin, clockPin,  155 );//第35排
    my_shiftOut(dataPin, clockPin,  109 );//第34排
    my_shiftOut(dataPin, clockPin,  45 );//第33排
    my_shiftOut(dataPin, clockPin,  42 );//第32排
    my_shiftOut(dataPin, clockPin,  213 );//第31排
    my_shiftOut(dataPin, clockPin,  155 ); //第30排
    my_shiftOut(dataPin, clockPin,  37 ); //第29排
    my_shiftOut(dataPin, clockPin,  165 ); //第28排
    my_shiftOut(dataPin, clockPin,  43 );//第27排
    my_shiftOut(dataPin, clockPin,  85 );//第26排
    my_shiftOut(dataPin, clockPin,  217 );//第25排
    my_shiftOut(dataPin, clockPin,  36 );//第24排
    my_shiftOut(dataPin, clockPin,  181 );//第23排
    my_shiftOut(dataPin, clockPin,  169 );//第22排
    my_shiftOut(dataPin, clockPin,  85 ); //第21排
    my_shiftOut(dataPin, clockPin,  201 ); //第20排
    my_shiftOut(dataPin, clockPin,  36 );//第19排
    my_shiftOut(dataPin, clockPin,  180 );//第18排
    my_shiftOut(dataPin, clockPin,  173 );//第17排
    my_shiftOut(dataPin, clockPin,  85 );//第16排
    my_shiftOut(dataPin, clockPin,  205 );//第15排
    my_shiftOut(dataPin, clockPin,  182 );//第14排
    my_shiftOut(dataPin, clockPin,  150 ); //第13排
    my_shiftOut(dataPin, clockPin,  165 );//第12排
    my_shiftOut(dataPin, clockPin,  86 );//第11排
    my_shiftOut(dataPin, clockPin,  77 );//第10排
    my_shiftOut(dataPin, clockPin,  182 );//第9排
    my_shiftOut(dataPin, clockPin,  150 );//第8排
    my_shiftOut(dataPin, clockPin,  181 );//第7排
    my_shiftOut(dataPin, clockPin,  90 );//第6排
    my_shiftOut(dataPin, clockPin,  109 );//第5排
    my_shiftOut(dataPin, clockPin,  150 );//第4排
    my_shiftOut(dataPin, clockPin,  210 );//第3排
    my_shiftOut(dataPin, clockPin,  149 );//第2排
    my_shiftOut(dataPin, clockPin,  106 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  108 );//第50排
    my_shiftOut(dataPin1, clockPin,  146 ); //第49排
    my_shiftOut(dataPin1, clockPin,  210 );//第48排
    my_shiftOut(dataPin1, clockPin,  149 ); //第47排
    my_shiftOut(dataPin1, clockPin,  42 );//第46排
    my_shiftOut(dataPin1, clockPin,  100 );//第45排
    my_shiftOut(dataPin1, clockPin,  146 );//第44排
    my_shiftOut(dataPin1, clockPin,  90 );//第43排
    my_shiftOut(dataPin1, clockPin,  213 );//第42排
    my_shiftOut(dataPin1, clockPin,  170 );//第41排
    my_shiftOut(dataPin1, clockPin,  100 );//第40排
    my_shiftOut(dataPin1, clockPin,  146 ); //第39排
    my_shiftOut(dataPin1, clockPin,  90 );//第38排
    my_shiftOut(dataPin1, clockPin,  212 ); //第37排
    my_shiftOut(dataPin1, clockPin,  170 );//第36排
    my_shiftOut(dataPin1, clockPin,  100 );//第35排
    my_shiftOut(dataPin1, clockPin,  218 );//第34排
    my_shiftOut(dataPin1, clockPin,  90 );//第33排
    my_shiftOut(dataPin1, clockPin,  84 );//第32排
    my_shiftOut(dataPin1, clockPin,  170 );//第31排
    my_shiftOut(dataPin1, clockPin,  38 ); //第30排
    my_shiftOut(dataPin1, clockPin,  218 ); //第29排
    my_shiftOut(dataPin1, clockPin,  74 ); //第28排
    my_shiftOut(dataPin1, clockPin,  86 );//第27排
    my_shiftOut(dataPin1, clockPin,  170 );//第26排
    my_shiftOut(dataPin1, clockPin,  38 );//第25排
    my_shiftOut(dataPin1, clockPin,  219 );//第24排
    my_shiftOut(dataPin1, clockPin,  74 );//第23排
    my_shiftOut(dataPin1, clockPin,  86 );//第22排
    my_shiftOut(dataPin1, clockPin,  170 ); //第21排
    my_shiftOut(dataPin1, clockPin,  38 ); //第20排
    my_shiftOut(dataPin1, clockPin,  219 );//第19排
    my_shiftOut(dataPin1, clockPin,  74 );//第18排
    my_shiftOut(dataPin1, clockPin,  86 );//第17排
    my_shiftOut(dataPin1, clockPin,  170 );//第16排
    my_shiftOut(dataPin1, clockPin,  38 );//第15排
    my_shiftOut(dataPin1, clockPin,  219 );//第14排
    my_shiftOut(dataPin1, clockPin,  75 ); //第13排
    my_shiftOut(dataPin1, clockPin,  82 );//第12排
    my_shiftOut(dataPin1, clockPin,  170 );//第11排
    my_shiftOut(dataPin1, clockPin,  54 );//第10排
    my_shiftOut(dataPin1, clockPin,  219 );//第9排
    my_shiftOut(dataPin1, clockPin,  75 );//第8排
    my_shiftOut(dataPin1, clockPin,  82 );//第7排
    my_shiftOut(dataPin1, clockPin,  170 );//第6排
    my_shiftOut(dataPin1, clockPin,  54 );//第5排
    my_shiftOut(dataPin1, clockPin,  219 );//第4排
    my_shiftOut(dataPin1, clockPin,  75 );//第3排
    my_shiftOut(dataPin1, clockPin,  82 );//第2排
    my_shiftOut(dataPin1, clockPin,  170 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  54 );//第50排
    my_shiftOut(dataPin2, clockPin,  219 ); //第49排
    my_shiftOut(dataPin2, clockPin,  75 );//第48排
    my_shiftOut(dataPin2, clockPin,  82 ); //第47排
    my_shiftOut(dataPin2, clockPin,  170 );//第46排
    my_shiftOut(dataPin2, clockPin,  54 );//第45排
    my_shiftOut(dataPin2, clockPin,  219 );//第44排
    my_shiftOut(dataPin2, clockPin,  75 );//第43排
    my_shiftOut(dataPin2, clockPin,  82 );//第42排
    my_shiftOut(dataPin2, clockPin,  170 );//第41排
    my_shiftOut(dataPin2, clockPin,  38 );//第40排
    my_shiftOut(dataPin2, clockPin,  219 ); //第39排
    my_shiftOut(dataPin2, clockPin,  75 );//第38排
    my_shiftOut(dataPin2, clockPin,  82 ); //第37排
    my_shiftOut(dataPin2, clockPin,  170 );//第36排
    my_shiftOut(dataPin2, clockPin,  38 );//第35排
    my_shiftOut(dataPin2, clockPin,  219 );//第34排
    my_shiftOut(dataPin2, clockPin,  74 );//第33排
    my_shiftOut(dataPin2, clockPin,  86 );//第32排
    my_shiftOut(dataPin2, clockPin,  170 );//第31排
    my_shiftOut(dataPin2, clockPin,  38 ); //第30排
    my_shiftOut(dataPin2, clockPin,  219 ); //第29排
    my_shiftOut(dataPin2, clockPin,  74 ); //第28排
    my_shiftOut(dataPin2, clockPin,  86 );//第27排
    my_shiftOut(dataPin2, clockPin,  170 );//第26排
    my_shiftOut(dataPin2, clockPin,  38 );//第25排
    my_shiftOut(dataPin2, clockPin,  218 );//第24排
    my_shiftOut(dataPin2, clockPin,  74 );//第23排
    my_shiftOut(dataPin2, clockPin,  86 );//第22排
    my_shiftOut(dataPin2, clockPin,  170 ); //第21排
    my_shiftOut(dataPin2, clockPin,  100 ); //第20排
    my_shiftOut(dataPin2, clockPin,  218 );//第19排
    my_shiftOut(dataPin2, clockPin,  90 );//第18排
    my_shiftOut(dataPin2, clockPin,  84 );//第17排
    my_shiftOut(dataPin2, clockPin,  170 );//第16排
    my_shiftOut(dataPin2, clockPin,  100 );//第15排
    my_shiftOut(dataPin2, clockPin,  146 );//第14排
    my_shiftOut(dataPin2, clockPin,  90 ); //第13排
    my_shiftOut(dataPin2, clockPin,  212 );//第12排
    my_shiftOut(dataPin2, clockPin,  170 );//第11排
    my_shiftOut(dataPin2, clockPin,  100 );//第10排
    my_shiftOut(dataPin2, clockPin,  146 );//第9排
    my_shiftOut(dataPin2, clockPin,  90 );//第8排
    my_shiftOut(dataPin2, clockPin,  213 );//第7排
    my_shiftOut(dataPin2, clockPin,  170 );//第6排
    my_shiftOut(dataPin2, clockPin,  108 );//第5排
    my_shiftOut(dataPin2, clockPin,  146 );//第4排
    my_shiftOut(dataPin2, clockPin,  210 );//第3排
    my_shiftOut(dataPin2, clockPin,  149 );//第2排
    my_shiftOut(dataPin2, clockPin,  42 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  109 );//第50排
    my_shiftOut(dataPin3, clockPin,  150 ); //第49排
    my_shiftOut(dataPin3, clockPin,  210 );//第48排
    my_shiftOut(dataPin3, clockPin,  149 ); //第47排
    my_shiftOut(dataPin3, clockPin,  106 );//第46排
    my_shiftOut(dataPin3, clockPin,  77 );//第45排
    my_shiftOut(dataPin3, clockPin,  182 );//第44排
    my_shiftOut(dataPin3, clockPin,  150 );//第43排
    my_shiftOut(dataPin3, clockPin,  181 );//第42排
    my_shiftOut(dataPin3, clockPin,  90 );//第41排
    my_shiftOut(dataPin3, clockPin,  205 );//第40排
    my_shiftOut(dataPin3, clockPin,  182 ); //第39排
    my_shiftOut(dataPin3, clockPin,  150 );//第38排
    my_shiftOut(dataPin3, clockPin,  165 ); //第37排
    my_shiftOut(dataPin3, clockPin,  86 );//第36排
    my_shiftOut(dataPin3, clockPin,  201 );//第35排
    my_shiftOut(dataPin3, clockPin,  36 );//第34排
    my_shiftOut(dataPin3, clockPin,  180 );//第33排
    my_shiftOut(dataPin3, clockPin,  173 );//第32排
    my_shiftOut(dataPin3, clockPin,  85 );//第31排
    my_shiftOut(dataPin3, clockPin,  217 ); //第30排
    my_shiftOut(dataPin3, clockPin,  36 ); //第29排
    my_shiftOut(dataPin3, clockPin,  181 ); //第28排
    my_shiftOut(dataPin3, clockPin,  169 );//第27排
    my_shiftOut(dataPin3, clockPin,  85 );//第26排
    my_shiftOut(dataPin3, clockPin,  155 );//第25排
    my_shiftOut(dataPin3, clockPin,  37 );//第24排
    my_shiftOut(dataPin3, clockPin,  165 );//第23排
    my_shiftOut(dataPin3, clockPin,  43 );//第22排
    my_shiftOut(dataPin3, clockPin,  85 ); //第21排
    my_shiftOut(dataPin3, clockPin,  155 ); //第20排
    my_shiftOut(dataPin3, clockPin,  109 );//第19排
    my_shiftOut(dataPin3, clockPin,  45 );//第18排
    my_shiftOut(dataPin3, clockPin,  42 );//第17排
    my_shiftOut(dataPin3, clockPin,  213 );//第16排
    my_shiftOut(dataPin3, clockPin,  146 );//第15排
    my_shiftOut(dataPin3, clockPin,  105 );//第14排
    my_shiftOut(dataPin3, clockPin,  45 ); //第13排
    my_shiftOut(dataPin3, clockPin,  106 );//第12排
    my_shiftOut(dataPin3, clockPin,  149 );//第11排
    my_shiftOut(dataPin3, clockPin,  178 );//第10排
    my_shiftOut(dataPin3, clockPin,  73 );//第9排
    my_shiftOut(dataPin3, clockPin,  105 );//第8排
    my_shiftOut(dataPin3, clockPin,  90 );//第7排
    my_shiftOut(dataPin3, clockPin,  173 );//第6排
    my_shiftOut(dataPin3, clockPin,  54 );//第5排
    my_shiftOut(dataPin3, clockPin,  219 );//第4排
    my_shiftOut(dataPin3, clockPin,  75 );//第3排
    my_shiftOut(dataPin3, clockPin,  82 );//第2排
    my_shiftOut(dataPin3, clockPin,  170 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}
void angle_inverse_60(){
    //start=millis();
    printf("The angle of reflection is -60 deg\n");
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, 0);
    my_gpio_set_level(latchPin, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin, clockPin,  37 );//第1排
    my_shiftOut(dataPin, clockPin,  165 ); //第2排
    my_shiftOut(dataPin, clockPin,  106 );//第3排
    my_shiftOut(dataPin, clockPin,  181 ); //第4排
    my_shiftOut(dataPin, clockPin,  84 );//第5排
    my_shiftOut(dataPin, clockPin,  36 );//第6排
    my_shiftOut(dataPin, clockPin,  181 );//第7排
    my_shiftOut(dataPin, clockPin,  42 );//第8排
    my_shiftOut(dataPin, clockPin,  85 );//第9排
    my_shiftOut(dataPin, clockPin,  85 );//第10排
    my_shiftOut(dataPin, clockPin,  180 );//第40排
    my_shiftOut(dataPin, clockPin,  181 ); //第39排
    my_shiftOut(dataPin, clockPin,  169 );//第38排
    my_shiftOut(dataPin, clockPin,  85 ); //第37排
    my_shiftOut(dataPin, clockPin,  85 );//第36排
    my_shiftOut(dataPin, clockPin,  182 );//第35排
    my_shiftOut(dataPin, clockPin,  148 );//第34排
    my_shiftOut(dataPin, clockPin,  173 );//第33排
    my_shiftOut(dataPin, clockPin,  85 );//第32排
    my_shiftOut(dataPin, clockPin,  85 );//第31排
    my_shiftOut(dataPin, clockPin,  146 ); //第30排
    my_shiftOut(dataPin, clockPin,  214 ); //第29排
    my_shiftOut(dataPin, clockPin,  165 ); //第28排
    my_shiftOut(dataPin, clockPin,  85 );//第27排
    my_shiftOut(dataPin, clockPin,  85 );//第26排
    my_shiftOut(dataPin, clockPin,  146 );//第25排
    my_shiftOut(dataPin, clockPin,  210 );//第24排
    my_shiftOut(dataPin, clockPin,  181 );//第23排
    my_shiftOut(dataPin, clockPin,  82 );//第22排
    my_shiftOut(dataPin, clockPin,  173 ); //第21排
    my_shiftOut(dataPin, clockPin,  218 ); //第20排
    my_shiftOut(dataPin, clockPin,  90 );//第19排
    my_shiftOut(dataPin, clockPin,  149 );//第18排
    my_shiftOut(dataPin, clockPin,  74 );//第17排
    my_shiftOut(dataPin, clockPin,  170 );//第16排
    my_shiftOut(dataPin, clockPin,  218 );//第15排
    my_shiftOut(dataPin, clockPin,  90 );//第14排
    my_shiftOut(dataPin, clockPin,  213 ); //第13排
    my_shiftOut(dataPin, clockPin,  42 );//第12排
    my_shiftOut(dataPin, clockPin,  170 );//第11排
    my_shiftOut(dataPin, clockPin,  91 );//第10排
    my_shiftOut(dataPin, clockPin,  74 );//第9排
    my_shiftOut(dataPin, clockPin,  84 );//第8排
    my_shiftOut(dataPin, clockPin,  170 );//第7排
    my_shiftOut(dataPin, clockPin,  170 );//第6排
    my_shiftOut(dataPin, clockPin,  75 );//第5排
    my_shiftOut(dataPin, clockPin,  75 );//第4排
    my_shiftOut(dataPin, clockPin,  86 );//第3排
    my_shiftOut(dataPin, clockPin,  170 );//第2排
    my_shiftOut(dataPin, clockPin,  170 );//第1排
    my_gpio_set_level(latchPin, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin1, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin1, clockPin,  73 );//第50排
    my_shiftOut(dataPin1, clockPin,  107 ); //第49排
    my_shiftOut(dataPin1, clockPin,  82 );//第48排
    my_shiftOut(dataPin1, clockPin,  170 ); //第47排
    my_shiftOut(dataPin1, clockPin,  170 );//第46排
    my_shiftOut(dataPin1, clockPin,  73 );//第45排
    my_shiftOut(dataPin1, clockPin,  105 );//第44排
    my_shiftOut(dataPin1, clockPin,  82 );//第43排
    my_shiftOut(dataPin1, clockPin,  170 );//第42排
    my_shiftOut(dataPin1, clockPin,  170 );//第41排
    my_shiftOut(dataPin1, clockPin,  105 );//第40排
    my_shiftOut(dataPin1, clockPin,  41 ); //第39排
    my_shiftOut(dataPin1, clockPin,  90 );//第38排
    my_shiftOut(dataPin1, clockPin,  170 ); //第37排
    my_shiftOut(dataPin1, clockPin,  170 );//第36排
    my_shiftOut(dataPin1, clockPin,  109 );//第35排
    my_shiftOut(dataPin1, clockPin,  41 );//第34排
    my_shiftOut(dataPin1, clockPin,  74 );//第33排
    my_shiftOut(dataPin1, clockPin,  170 );//第32排
    my_shiftOut(dataPin1, clockPin,  170 );//第31排
    my_shiftOut(dataPin1, clockPin,  109 ); //第30排
    my_shiftOut(dataPin1, clockPin,  45 ); //第29排
    my_shiftOut(dataPin1, clockPin,  74 ); //第28排
    my_shiftOut(dataPin1, clockPin,  170 );//第27排
    my_shiftOut(dataPin1, clockPin,  42 );//第26排
    my_shiftOut(dataPin1, clockPin,  109 );//第25排
    my_shiftOut(dataPin1, clockPin,  45 );//第24排
    my_shiftOut(dataPin1, clockPin,  74 );//第23排
    my_shiftOut(dataPin1, clockPin,  169 );//第22排
    my_shiftOut(dataPin1, clockPin,  90 ); //第21排
    my_shiftOut(dataPin1, clockPin,  109 ); //第20排
    my_shiftOut(dataPin1, clockPin,  173 );//第19排
    my_shiftOut(dataPin1, clockPin,  106 );//第18排
    my_shiftOut(dataPin1, clockPin,  173 );//第17排
    my_shiftOut(dataPin1, clockPin,  82 );//第16排
    my_shiftOut(dataPin1, clockPin,  45 );//第15排
    my_shiftOut(dataPin1, clockPin,  173 );//第14排
    my_shiftOut(dataPin1, clockPin,  106 ); //第13排
    my_shiftOut(dataPin1, clockPin,  165 );//第12排
    my_shiftOut(dataPin1, clockPin,  86 );//第11排
    my_shiftOut(dataPin1, clockPin,  45 );//第10排
    my_shiftOut(dataPin1, clockPin,  173 );//第9排
    my_shiftOut(dataPin1, clockPin,  106 );//第8排
    my_shiftOut(dataPin1, clockPin,  165 );//第7排
    my_shiftOut(dataPin1, clockPin,  86 );//第6排
    my_shiftOut(dataPin1, clockPin,  45 );//第5排
    my_shiftOut(dataPin1, clockPin,  173 );//第4排
    my_shiftOut(dataPin1, clockPin,  106 );//第3排
    my_shiftOut(dataPin1, clockPin,  181 );//第2排
    my_shiftOut(dataPin1, clockPin,  86 );//第1排
    my_gpio_set_level(latchPin1, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin2, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin2, clockPin,  45 );//第50排
    my_shiftOut(dataPin2, clockPin,  173 ); //第49排
    my_shiftOut(dataPin2, clockPin,  106  );//第48排
    my_shiftOut(dataPin2, clockPin,  181 ); //第47排
    my_shiftOut(dataPin2, clockPin,  86 );//第46排
    my_shiftOut(dataPin2, clockPin,  45 );//第45排
    my_shiftOut(dataPin2, clockPin,  173 );//第44排
    my_shiftOut(dataPin2, clockPin,  106  );//第43排
    my_shiftOut(dataPin2, clockPin,  165 );//第42排
    my_shiftOut(dataPin2, clockPin,  86 );//第41排
    my_shiftOut(dataPin2, clockPin,  45 );//第40排
    my_shiftOut(dataPin2, clockPin,  173 ); //第39排
    my_shiftOut(dataPin2, clockPin,  106  );//第38排
    my_shiftOut(dataPin2, clockPin,  165 ); //第37排
    my_shiftOut(dataPin2, clockPin,  86 );//第36排
    my_shiftOut(dataPin2, clockPin,  109  );//第35排
    my_shiftOut(dataPin2, clockPin,  173 );//第34排
    my_shiftOut(dataPin2, clockPin,  106  );//第33排
    my_shiftOut(dataPin2, clockPin,  173 );//第32排
    my_shiftOut(dataPin2, clockPin,  82 );//第31排
    my_shiftOut(dataPin2, clockPin,  109  ); //第30排
    my_shiftOut(dataPin2, clockPin,  45 ); //第29排
    my_shiftOut(dataPin2, clockPin,  74  ); //第28排
    my_shiftOut(dataPin2, clockPin,  169 );//第27排
    my_shiftOut(dataPin2, clockPin,  90 );//第26排
    my_shiftOut(dataPin2, clockPin,  109  );//第25排
    my_shiftOut(dataPin2, clockPin,  45 );//第24排
    my_shiftOut(dataPin2, clockPin,  74  );//第23排
    my_shiftOut(dataPin2, clockPin,  170 );//第22排
    my_shiftOut(dataPin2, clockPin,  42 ); //第21排
    my_shiftOut(dataPin2, clockPin,  109  ); //第20排
    my_shiftOut(dataPin2, clockPin,  41 );//第19排
    my_shiftOut(dataPin2, clockPin,  74  );//第18排
    my_shiftOut(dataPin2, clockPin,  170 );//第17排
    my_shiftOut(dataPin2, clockPin,  170 );//第16排
    my_shiftOut(dataPin2, clockPin,  105  );//第15排
    my_shiftOut(dataPin2, clockPin,  41  );//第14排
    my_shiftOut(dataPin2, clockPin,  90  ); //第13排
    my_shiftOut(dataPin2, clockPin,  170 );//第12排
    my_shiftOut(dataPin2, clockPin,  170 );//第11排
    my_shiftOut(dataPin2, clockPin,  73  );//第10排
    my_shiftOut(dataPin2, clockPin,  105  );//第9排
    my_shiftOut(dataPin2, clockPin,  82  );//第8排
    my_shiftOut(dataPin2, clockPin,  170 );//第7排
    my_shiftOut(dataPin2, clockPin,  170 );//第6排
    my_shiftOut(dataPin2, clockPin,  73  );//第5排
    my_shiftOut(dataPin2, clockPin,  107  );//第4排
    my_shiftOut(dataPin2, clockPin,  82 );//第3排
    my_shiftOut(dataPin2, clockPin,  170 );//第2排
    my_shiftOut(dataPin2, clockPin,  170 );//第1排
    my_gpio_set_level(latchPin2, 1); // 送完資料後要把 latchPin 設成高電位
    my_gpio_set_level(latchPin3, 0);  // 送資料前要先把 latchPin 設成低電位
    my_shiftOut(dataPin3, clockPin,  75  );//第50排
    my_shiftOut(dataPin3, clockPin,  75  ); //第49排
    my_shiftOut(dataPin3, clockPin,  86 );//第48排
    my_shiftOut(dataPin3, clockPin,  170 ); //第47排
    my_shiftOut(dataPin3, clockPin,  170 );//第46排
    my_shiftOut(dataPin3, clockPin,  91 );//第45排
    my_shiftOut(dataPin3, clockPin,  74  );//第44排
    my_shiftOut(dataPin3, clockPin,  84  );//第43排
    my_shiftOut(dataPin3, clockPin,  170  );//第42排
    my_shiftOut(dataPin3, clockPin,  170 );//第41排
    my_shiftOut(dataPin3, clockPin,  218 );//第40排
    my_shiftOut(dataPin3, clockPin,  90 ); //第39排
    my_shiftOut(dataPin3, clockPin,  213 );//第38排
    my_shiftOut(dataPin3, clockPin,  42  ); //第37排
    my_shiftOut(dataPin3, clockPin,  170  );//第36排
    my_shiftOut(dataPin3, clockPin,  218 );//第35排
    my_shiftOut(dataPin3, clockPin,  90 );//第34排
    my_shiftOut(dataPin3, clockPin,  149 );//第33排
    my_shiftOut(dataPin3, clockPin,  74  );//第32排
    my_shiftOut(dataPin3, clockPin,  170  );//第31排
    my_shiftOut(dataPin3, clockPin,  146 ); //第30排
    my_shiftOut(dataPin3, clockPin,  210 ); //第29排
    my_shiftOut(dataPin3, clockPin,  181 ); //第28排
    my_shiftOut(dataPin3, clockPin,  82 );//第27排
    my_shiftOut(dataPin3, clockPin,  173   );//第26排
    my_shiftOut(dataPin3, clockPin,  146 );//第25排
    my_shiftOut(dataPin3, clockPin,  214 );//第24排
    my_shiftOut(dataPin3, clockPin,  165 );//第23排
    my_shiftOut(dataPin3, clockPin,  85 );//第22排
    my_shiftOut(dataPin3, clockPin,  85  ); //第21排
    my_shiftOut(dataPin3, clockPin,  182 ); //第20排
    my_shiftOut(dataPin3, clockPin,  148 );//第19排
    my_shiftOut(dataPin3, clockPin,  173 );//第18排
    my_shiftOut(dataPin3, clockPin,  85 );//第17排
    my_shiftOut(dataPin3, clockPin,  85 );//第16排
    my_shiftOut(dataPin3, clockPin,  180 );//第15排
    my_shiftOut(dataPin3, clockPin,  181 );//第14排
    my_shiftOut(dataPin3, clockPin,  169 ); //第13排
    my_shiftOut(dataPin3, clockPin,  85 );//第12排
    my_shiftOut(dataPin3, clockPin,  85  );//第11排
    my_shiftOut(dataPin3, clockPin,  36 );//第10排
    my_shiftOut(dataPin3, clockPin,  181 );//第9排
    my_shiftOut(dataPin3, clockPin,  42 );//第8排
    my_shiftOut(dataPin3, clockPin,  85 );//第7排
    my_shiftOut(dataPin3, clockPin,  85  );//第6排
    my_shiftOut(dataPin3, clockPin,  37  );//第5排
    my_shiftOut(dataPin3, clockPin,  165 );//第4排
    my_shiftOut(dataPin3, clockPin,  106 );//第3排
    my_shiftOut(dataPin3, clockPin,  181 );//第2排
    my_shiftOut(dataPin3, clockPin,  84 );//第1排
    my_gpio_set_level(latchPin3, 1); // 送完資料後要把 latchPin 設成高電位
    timer_pause(TIMER_GROUP_0, 0);
    timer_get_counter_time_sec(TIMER_GROUP_0, 0, &duration);
    timer_set_counter_value(TIMER_GROUP_0, 0, 0x00000000ULL);
    printf("Running time: ");
    printf("%.2f" , duration*1000000);
    printf("us\n\n");    
}




/*
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO output gpio_num error", ESP_ERR_INVALID_ARG);
    gpio_hal_set_level(gpio_context.gpio_hal, gpio_num, level);
    return ESP_OK;
}
#define gpio_hal_set_level(hal, gpio_num, level) gpio_ll_set_level((hal)->dev, gpio_num, level)

static inline void gpio_ll_set_level(gpio_dev_t *hw, gpio_num_t gpio_num, uint32_t level)
{
    if (level) {
        if (gpio_num < 32) {
            hw->out_w1ts = (1 << gpio_num);
        } else {
            hw->out1_w1ts.data = (1 << (gpio_num - 32));
        }
    } else {
        if (gpio_num < 32) {
            hw->out_w1tc = (1 << gpio_num);
        } else {
            hw->out1_w1tc.data = (1 << (gpio_num - 32));
        }
    }
}
*/
