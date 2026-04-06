#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

int main(void) {
    FILE *fin = fopen("raw.iq", "rb");
    FILE *fout = fopen("parsed_iq.txt", "w");

    if (fin == NULL) {
        perror("fopen raw.iq");
        return 1;
    }
    if (fout == NULL) {
        perror("fopen parsed_iq.txt");
        fclose(fin);
        return 1;
    }

    for (int k = 0; k < 7500; k++) {
        uint8_t i8, q8;

        if (fread(&i8, 1, 1, fin) != 1 || fread(&q8, 1, 1, fin) != 1) {
            fprintf(stderr, "Reached EOF or read error at sample %d\n", k);
            break;
        }

        int16_t i = ((int)i8 - 128) << 8;
        int16_t q = ((int)q8 - 128) << 8;

        fprintf(fout, "{%d,%d},\n", i, q);
    }

    fclose(fin);
    fclose(fout);
    return 0;
}