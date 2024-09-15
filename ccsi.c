#include <stdio.h>
#include <stdint.h>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 



int main(int argc, char *argv[]){

    uint8_t data[] = {0xAA, 0x10, 0xAA, 0x10};

    uint8_t shift = 0;
    uint8_t mask_h = 0xFF;
    uint8_t mask_l1 = 0xFF;
    uint8_t mask_l2 = 0x00;
    uint8_t mask_valid = 0x80;

    uint8_t out[10];
    size_t j = 0;

    for(int i = 0; i < sizeof(data); i=i+2){
        out[j] |= data[i] >> shift & mask_h;
        out[j+1] = (data[i+1] >> shift) & mask_l1;
        out[j+2] = mask_valid | ((data[i+1] << shift ) & mask_l2);
        j=j+2;
        mask_h = mask_h >> 1;
        mask_l1 = mask_l1 >> 1;
        mask_l2 = mask_l2 << 1 | 0x01;
        mask_valid = mask_valid >> 1;
        shift++;

        //reset after 8
        if (shift > 7){
            shift = 0;
            mask_h = 0xFF;
            mask_l1 = 0xFF;
            mask_l2 = 0x00;
            mask_valid = 0x80;
        }

    }

    for(int i = 0; i < sizeof(data); i++){
        printf("%d = " BYTE_TO_BINARY_PATTERN "\n", i, BYTE_TO_BINARY(data[i])); 
 
    }

    printf("\n----new----\n");

    for(int i = 0; i < sizeof(out); i++){
        printf("%d = " BYTE_TO_BINARY_PATTERN "\n", i, BYTE_TO_BINARY(out[i])); 
 
    }



    return 0;
}