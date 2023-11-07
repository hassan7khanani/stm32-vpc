#include "stm32f4xx.h" // Device header

void ControlPin(GPIO_TypeDef* port, uint32_t pinNumber, uint8_t pinStatus) {
    // Enable clock for the required port
    if (port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }

    // Set the pin as a general-purpose output
    port->MODER &= ~(0x3UL << (2 * pinNumber)); //reseting the value
    port->MODER |= (0x1UL << (2 * pinNumber)); //setting the pin as output

    // Set the output type as push-pull
    port->OTYPER &= ~(1 << pinNumber);

    // Set the pin either high or low
    if (pinStatus) {
        port->BSRR = (1 << pinNumber);
    } else {
        port->BSRR = (1 << (pinNumber + 16));
    }
}

int main(void) {

    ControlPin(GPIOA, 0, 1); // Set PA0 high
    ControlPin(GPIOB, 4, 1); // Set PB3 high
    ControlPin(GPIOC, 13, 0); // Set PC13 low
    return 0;
}
