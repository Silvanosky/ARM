#ifndef CRYPTO_H_
#define CRYPTO_H_

#include <stdbool.h>
#include <stddef.h>

#include "flash.h"

bool check_hash(const uint8_t* sign, size_t signsize, const uint8_t* prg, size_t prgsize);
bool check_app(__IO app_t* app);

#endif /* CRYPTO_H_ */
