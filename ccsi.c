#include <stdio.h>
#include <stdint.h>

#define STOP_HOLD_BITLEN    18

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
const char frame_sample_a_in[] = {0xAA, 0x30, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF};
const char frame_sample_a_out[] = {0xAA, 0x30, 0xBF, 0xFF, 0xDF, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF};

//something i made up
const char frame_sample_b_in[] = {0xAA, 0x10, 0xAA, 0x10};
const char frame_sample_b_out[] = {0xAA, 0x30, 0xD5, 0x08, 0x7F};



/// @brief calculate the number of bytes for output buffer from input len.  There could be extra bits for stop beyond 18.
/// @param len 
/// @return 
size_t output_len_calc(size_t len){
    size_t words = len / 2;

    if(len % 2 != 0){
        printf("ERROR odd number of bytes...should be all 16bit words\n");
        return 0;
    }

    size_t bits = (words * 16) + words + STOP_HOLD_BITLEN;

    printf("number of bits = %d\n", bits);
    size_t array_len = (bits % 8) > 0 ? 1 : 0;
    printf("bytes_extra = %d\n", array_len);

    array_len += (bits / 8);
    printf("bytes = %d\n", array_len);

    return array_len;
}

/// @brief generate packet
/// @param in input data
/// @param len_in lengh of input data
/// @param out output buffer pre-allocated at length
/// @param len_out length of allocated buffer
/// @return error
int8_t ccsi_packet_gen(uint8_t *in, size_t len_in, uint8_t *out, size_t len_out){
    uint8_t shift = 0;
    uint8_t mask_h1 = 0xFF;
    uint8_t mask_h2 = 0x00;
    uint8_t mask_l1 = 0xFF;
    uint8_t mask_l2 = 0x00;
    uint8_t mask_valid = 0x80;

    // uint8_t out[len_out];
    size_t j = 0;

    memset(out, 0U, len_out);

    for(int i = 0; i < len_in; i=i+2){
        printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d\n", j, i, BYTE_TO_BINARY(out[j]), out[j], shift); 

        out[j] |= (in[i] >> shift) & mask_h1;
        printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d\n", j, i, BYTE_TO_BINARY(out[j]), out[j], shift); 
        out[j+1] = (in[i+1] >> shift) & mask_l1 | (in[i] << 8-shift);
        printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d, mask=%02X\n", j+1, i, BYTE_TO_BINARY(out[j+1]), out[j+1], shift, mask_valid); 
        out[j+2] = mask_valid | (in[i+1] << 8-shift );
        printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d, mask=%02X\n", j+2, i, BYTE_TO_BINARY(out[j+2]), out[j+2], shift, mask_valid); 
        j=j+2;
        mask_h1 = mask_h1 >> 1;
        mask_h2 = mask_h2 >>1 | 0x80;
        mask_l1 = mask_l1 >> 1;
        mask_l2 = mask_l2 << 1 | 0x01;
        mask_valid = mask_valid >> 1;
        shift++;

        //reset after 8
        if (shift > 7){
            shift = 0;
            mask_h1 = 0xFF;
            mask_h2 = 0x00;
            mask_l1 = 0xFF;
            mask_l2 = 0x00;
            mask_valid = 0x80;
        }

    }

    //add stop bits >= 18 high, i.e. two full bytes plus initial mask.
    out[j] |= 0xFF >> shift & mask_h1;
    out[j+1] = 0xFF;
    out[j+2] = 0xFF;

    return 0;
}

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
    ccsi_packet_gen(data, sizeof(data), out, out_len);

    print_buffer_formatting(data, sizeof(data));
    printf("\n----new----\n");
    print_buffer_formatting(out, sizeof(out));


    size_t out_len2 = output_len_calc(sizeof(frame_sample_a_in));
    uint8_t out2[out_len2];
    ccsi_packet_gen(frame_sample_a_in, sizeof(frame_sample_a_in), out2, out_len2);

    print_buffer_formatting(frame_sample_a_in, sizeof(frame_sample_a_in));
    printf("\n----new----\n");
    print_buffer_formatting(out2, sizeof(out2));

    return 0;
}