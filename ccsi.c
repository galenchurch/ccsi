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


const char frame_sample_a_in[] = {0xAA, 0x30, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF};
const char frame_sample_a_out[] = {0xAA, 0x30, 0xBF, 0xFF, 0xDF, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF};

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
int8_t ccsi_packet_gen(char *in, size_t len_in, char *out, size_t len_out){
    uint8_t shift = 0;
    uint8_t mask_h = 0xFF;
    uint8_t mask_l1 = 0xFF;
    uint8_t mask_l2 = 0x00;
    uint8_t mask_valid = 0x80;

    // uint8_t out[len_out];
    size_t j = 0;

    for(int i = 0; i < len_in; i=i+2){
        out[j] |= in[i] >> shift & mask_h;
        out[j+1] = (in[i+1] >> shift) & mask_l1;
        out[j+2] = mask_valid | ((in[i+1] << shift ) & mask_l2);
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

    //add stop bits >= 18 high, i.e. two full bytes plus initial mask.
    out[j] |= 0xFF >> shift & mask_h;
    out[j+1] = 0xFF;
    out[j+2] = 0xFF;

    return 0;
}

int main(int argc, char *argv[]){
    size_t len = 0;
    uint8_t data[] = {0xAA, 0x10, 0xAA, 0x10};
    len = sizeof(data);

    size_t out_len = output_len_calc(sizeof(data));
    uint8_t out[out_len];
    ccsi_packet_gen(data, sizeof(data), out, out_len);

    for(int i = 0; i < sizeof(data); i++){
        printf("%d = " BYTE_TO_BINARY_PATTERN "\n", i, BYTE_TO_BINARY(data[i])); 
 
    }

    printf("\n----new----\n");

    for(int i = 0; i < sizeof(out); i++){
        printf("%d = " BYTE_TO_BINARY_PATTERN "\n", i, BYTE_TO_BINARY(out[i])); 
 
    }


    size_t out_len2 = output_len_calc(sizeof(frame_sample_a_in));
    uint8_t out2[out_len2];
    ccsi_packet_gen(frame_sample_a_in, sizeof(frame_sample_a_in), out2, out_len2);

    for(int i = 0; i < sizeof(frame_sample_a_in); i++){
        printf("%d = " BYTE_TO_BINARY_PATTERN "\n", i, BYTE_TO_BINARY(frame_sample_a_in[i])); 
 
    }

    printf("\n----new----\n");

    for(int i = 0; i < sizeof(out2); i++){
        printf("%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X\n", i, BYTE_TO_BINARY(out2[i]), out2[i]); 
 
    }

    return 0;
}