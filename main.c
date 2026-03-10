
#include <eff.h>
#include <stdio.h>
#include "gnss_types.h"

int main() {
    int i = 0;

    eff_pinmux_set(PINMUX_11, PINMUX_GPIO);
    eff_gpio_dir_set(GPIO_11, GPIO_PIN_2, EFF_GPIO_OUT);

    while (1) {
        printf("Energy is everything! %i\r\n", i);

        printf("Try GNSS type library: %x\r\n", gnss_sat16(0x0000FFFF));

        if (i % 2)
            eff_gpio_set(GPIO_11, GPIO_PIN_2);
        else
            eff_gpio_clear(GPIO_11, GPIO_PIN_2);

        i++;
        sleep(1);
    }
}