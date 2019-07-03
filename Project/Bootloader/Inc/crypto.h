#ifndef CRYPTO_H_
#define CRYPTO_H_

#include <stdbool.h>
#include <stddef.h>

bool check_hash(const uint8_t* sign, size_t signsize, const uint8_t* prg, size_t prgsize);

#endif /* CRYPTO_H_ */
