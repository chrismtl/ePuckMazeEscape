#include "../stm32f4xx.h"
#include "../system_clock_config.h"

#define LED1     	GPIOD, 5
#define LED3     	GPIOD, 6
#define LED5     	GPIOD, 10
#define LED7     	GPIOD, 11

void _init(void) {}

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin) {
    port->OTYPER |= (1 << pin);
    port->ODR &= ~(1 << pin);
    port->PUPDR &= ~(3 << (pin * 2));
    port->OSPEEDR |= (3 << (pin * 2));
    port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
}

void gpio_set(GPIO_TypeDef *port, unsigned int pin) {
    port->BSRR = (1 << pin);
}

void gpio_clear(GPIO_TypeDef *port, unsigned int pin) {
    port->BSRR = (1 << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef *port, unsigned int pin) {
    if (port->ODR & (1<<pin)) {
        gpio_clear(port, pin);
    } else {
        gpio_set(port, pin);
    }
}

int main(void) {
    SystemClock_Config();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    gpio_config_output_opendrain(LED1);
    gpio_config_output_opendrain(LED3);
    gpio_config_output_opendrain(LED5);
    gpio_config_output_opendrain(LED7);

    while (1) {
        gpio_toggle(LED1);
        gpio_toggle(LED3);
        gpio_toggle(LED5);
        gpio_toggle(LED7);
        for(int i=0; i < 1680000; i++)
            asm("nop");
    }
    return 0;
}