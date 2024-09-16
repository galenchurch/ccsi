#ifdef USER_SPACE
#include <stdint.h>
#include <string.h>
#else 
#include <linux/string.h>
#include <linux/module.h>
// #include <linux/stdint.h>
#endif

#define STOP_HOLD_BITLEN    18

/// @brief calculate the number of bytes for output buffer from input len.  There could be extra bits for stop beyond 18.
/// @param len 
/// @return 
size_t output_len_calc(size_t len){
    size_t words = len / 2;

    if(len % 2 != 0){
        // printf("ERROR odd number of bytes...should be all 16bit words\n");
        return 0;
    }
    size_t bits = (words * 16) + words + STOP_HOLD_BITLEN; //num words * 16bits + words (valid bit) + 18 stop bits
    size_t array_len = (bits % 8) > 0 ? 1 : 0;
    array_len += (bits / 8);

    return array_len;
}


int8_t ccsi_packet_gen(uint8_t *in, size_t len_in, uint8_t *out, size_t len_out, uint8_t fix_valid){
    uint8_t shift = 0;
    uint8_t mask_h1 = 0xFF;
    uint8_t mask_h2 = 0x00;
    uint8_t mask_l1 = 0xFF;
    uint8_t mask_l2 = 0x00;
    uint8_t mask_valid = 0x80;

    // uint8_t out[len_out];
    size_t j = 0;

    memset(out, 0U, len_out);

    int i = 0;
    for(i = 0; i < len_in; i=i+2){
        // printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d\n", j, i, BYTE_TO_BINARY(out[j]), out[j], shift); 

        out[j] |= (in[i] >> shift) & mask_h1;
        // printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d\n", j, i, BYTE_TO_BINARY(out[j]), out[j], shift); 
        out[j+1] = (in[i+1] >> shift) & mask_l1 | (in[i] << (8-shift));
        // printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d, mask=%02X\n", j+1, i, BYTE_TO_BINARY(out[j+1]), out[j+1], shift, mask_valid); 
        if (fix_valid == 1){
            out[j+2] = mask_valid | (in[i+1] << 8-shift );
        } else {
            out[j+2] = (mask_valid & ~((in[i] >> 7) << 7-shift))  | (in[i+1] << (8-shift) );
        }
        
        // printf("%d in-%d = " BYTE_TO_BINARY_PATTERN " - 0x%02X - shift =%d, mask=%02X\n", j+2, i, BYTE_TO_BINARY(out[j+2]), out[j+2], shift, mask_valid); 
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

    if (j+2 > len_out){
        // there will be an index out of bounds with stop bits
        return -1;
    }
    //add stop bits >= 18 high, i.e. two full bytes plus initial mask.
    out[j] |= 0xFF >> shift & mask_h1;
    out[j+1] = 0xFF;
    out[j+2] = 0xFF;

    return 0;
}

#ifndef USER_SPACE
EXPORT_SYMBOL(output_len_calc);

EXPORT_SYMBOL(ccsi_packet_gen);

MODULE_AUTHOR("Galen Church");
MODULE_DESCRIPTION("i.MX6 SSI->CCSI Driver");
MODULE_LICENSE("GPL");
#endif