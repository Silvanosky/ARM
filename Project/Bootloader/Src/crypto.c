/*
 *  Public key-based signature verification program
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#define mbedtls_snprintf        snprintf
#define mbedtls_printf          printf
#define mbedtls_exit            exit
#define MBEDTLS_EXIT_SUCCESS    EXIT_SUCCESS
#define MBEDTLS_EXIT_FAILURE    EXIT_FAILURE
#endif /* MBEDTLS_PLATFORM_C */

#include "crypto.h"

#if !defined(MBEDTLS_BIGNUM_C) || !defined(MBEDTLS_MD_C) || \
    !defined(MBEDTLS_SHA256_C) || !defined(MBEDTLS_PK_PARSE_C) ||   \
    !defined(MBEDTLS_FS_IO)
bool check_hash(const uint8_t* sign, size_t signsize, const uint8_t* prg, size_t prgsize)
{
    mbedtls_printf("MBEDTLS_BIGNUM_C and/or MBEDTLS_MD_C and/or "
           "MBEDTLS_SHA256_C and/or MBEDTLS_PK_PARSE_C and/or "
           "MBEDTLS_FS_IO not defined.\n");
    return false;
}
#else

#include "mbedtls/error.h"
#include "mbedtls/md.h"
#include "mbedtls/pk.h"

#include <stdio.h>
#include <string.h>

const unsigned char key[2048];

/**
* Use saved in bootloader key to check input hash
*/
bool check_hash(const uint8_t* sign, size_t signsize, const uint8_t* prg, size_t prgsize)
{
    int ret = 1;
    int exit_code = MBEDTLS_EXIT_FAILURE;
    unsigned char hash[32];

    mbedtls_pk_context pk;
    mbedtls_pk_init( &pk );

    mbedtls_printf( "\n  . Reading public key from '%s'\n"); //TODO remove

    if( (ret = mbedtls_pk_parse_public_key(&pk, key, sizeof(key))) != 0 ) {
        mbedtls_printf( " failed\n  ! mbedtls_pk_parse_public_key returned -0x%04x\n", -ret );
        return false;
    }

    mbedtls_printf( "\n  . Verifying the SHA-256 signature\n" );

    if( ( ret = mbedtls_md(
                    mbedtls_md_info_from_type( MBEDTLS_MD_SHA256 ),
                    prg, prgsize, hash) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! Could not open or read %s\n\n", argv[2] );
        return false;
    }

    if( ( ret = mbedtls_pk_verify( &pk, MBEDTLS_MD_SHA256, hash, 0,
                           sign, signsize) ) != 0 )
    {
        mbedtls_printf( " failed\n  ! mbedtls_pk_verify returned -0x%04x\n", -ret );
        return false;
    }

    mbedtls_printf( "\n  . OK (the signature is valid)\n\n" );

    exit_code = MBEDTLS_EXIT_SUCCESS;

    mbedtls_pk_free( &pk );

    return true;
}
#endif /* MBEDTLS_BIGNUM_C && MBEDTLS_SHA256_C &&
          MBEDTLS_PK_PARSE_C && MBEDTLS_FS_IO */
