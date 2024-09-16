#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ccsi.h"

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

//SRAM
uint8_t frame_sample_a_in[] = {0xAA, 0x30, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF};
uint8_t frame_sample_a_out[] = {0xAA, 0x30, 0xBF, 0xFF, 0xDF, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF};

//something i made up
uint8_t frame_sample_b_in[] = {0xAA, 0x10, 0xAA, 0x10};
uint8_t frame_sample_b_out[] = {0xAA, 0x30, 0xD5, 0x08, 0x7F};

void print_buffer_formatting(uint8_t *buf, size_t len){
    for(int i = 0; i < len; i++){
        printf("%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X\n", i, BYTE_TO_BINARY(buf[i]), buf[i]); 
    }
}

int main(int argc, char *argv[]){
    size_t len = 0;
    uint8_t data[] = {0xAA, 0x10, 0xAA, 0x10};
    len = sizeof(data);

    size_t out_len = output_len_calc(sizeof(data));
    uint8_t out[out_len];
    ccsi_packet_gen(data, sizeof(data), out, out_len, 0);

    print_buffer_formatting(data, sizeof(data));
    printf("\n----new----\n");
    print_buffer_formatting(out, sizeof(out));


    size_t out_len2 = output_len_calc(sizeof(frame_sample_a_in));
    uint8_t out2[out_len2];
    ccsi_packet_gen(frame_sample_a_in, sizeof(frame_sample_a_in), out2, out_len2, 0);

    print_buffer_formatting(frame_sample_a_in, sizeof(frame_sample_a_in));
    printf("\n----new----\n");
    print_buffer_formatting(out2, sizeof(out2));

    return 0;
}