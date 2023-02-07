#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define dataPin    12
#define clockPin    13
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_ISR_CORE_ID_UNINIT    (3)
#define GPIO_HAL_GET_HW(num) GPIO_LL_GET_HW(num)
#define GPIO_LL_GET_HW(num) (((num) == 0) ? (&GPIO) : NULL)
void my_gpio_set_level(gpio_num_t gpio_num, uint32_t level);
void my_shiftOut(uint8_t DataPin, uint8_t ClockPin, uint8_t val);
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
void app_main(void)
{
    init_gpio_to_output();
    int cnt = 0;
    while(1) {
        printf("hello world \n");
        printf("cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);

        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        my_gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        printf("GPIO_OUT_DATA= %x\n",GPIO_OUT_DATA );
    }
}
inline void my_gpio_set_level(gpio_num_t gpio_num, uint32_t level){//gpio_num<32
    if(level)
     gpio_context.gpio_hal->dev->out_w1ts = (1<<gpio_num);
    else
     gpio_context.gpio_hal->dev->out_w1tc = (1<<gpio_num);
}
inline void my_shiftOut(uint8_t DataPin, uint8_t ClockPin, uint8_t val)
{
        //MSB
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
        my_gpio_set_level(DataPin, (val & 128) != 0);
        val <<= 1;
                
        my_gpio_set_level(ClockPin , 1);
        my_gpio_set_level(ClockPin , 0);
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
