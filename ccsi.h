#ifdef USER_SPACE
#include <stdint.h>
#endif

/// @brief calculate the number of bytes for output buffer from input len.  There could be extra bits for stop beyond 18.
/// @param len 
/// @return 
size_t output_len_calc(size_t len);

/// @brief generate packet
/// @param in input data
/// @param len_in lengh of input data
/// @param out output buffer pre-allocated at length
/// @param len_out length of allocated buffer
/// @param fix_valid 0 = normal valid is !bit[16], 1 = valid is hardcoded as 1
/// @return error
int8_t ccsi_packet_gen(uint8_t *in, size_t len_in, uint8_t *out, size_t len_out, uint8_t fix_valid);