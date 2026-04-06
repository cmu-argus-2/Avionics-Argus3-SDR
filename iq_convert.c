#include <stdio.h>
#include <stdint.h>

int main() {
    FILE *f = fopen("raw.iq","rb");

    for(int k=0;k<16;k++) {
        uint8_t i8,q8;

        fread(&i8,1,1,f);
        fread(&q8,1,1,f);

        int16_t i = ((int)i8 - 128) << 8;
        int16_t q = ((int)q8 - 128) << 8;

        printf("{%x,%x},\n", i, q);
    }
}