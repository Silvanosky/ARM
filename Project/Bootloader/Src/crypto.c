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

#include "crypto.h"

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include "mbedtls/platform.h"
#include "mbedtls/error.h"
#include "mbedtls/md.h"
#include "mbedtls/pk.h"
#include "crypto.h"
#include "uart.h"
#include "flash.h"

#include <stdio.h>
#include <string.h>


static const unsigned char * key = (unsigned char *)"-----BEGIN PUBLIC KEY-----\n"
"MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAE3tXUzyOI913cG5JQQpZf8emFB/Za\n"
"CXBAwpxJk2U/csWTwde/qVo3W0uoowjwpJgIeC/visyiR4AAxb1u138Fsw==\n"
"-----END PUBLIC KEY-----\n";

bool check_app(__IO app_t* app)
{
	//check_hash(&app->signature, app->sign_size, &app->app, app->app_size);
	return true;
}

/**
* Use saved in bootloader key to check input hash
*/


bool check_hash(const uint8_t* sign, size_t signsize, const uint8_t* prg, size_t prgsize)
{
    int ret = 1;
    int exit_code = MBEDTLS_EXIT_FAILURE;
    unsigned char hash[32];

	unsigned char memory_buf[30000];
	mbedtls_memory_buffer_alloc_init( memory_buf, sizeof(memory_buf) );

    mbedtls_pk_context pk;
    mbedtls_pk_init( &pk );

    uart_tx_str((uint8_t*)"\nReading public key from\n"); //TODO remove
    uart_tx_str((uint8_t*)key); //TODO remove

    if( (ret = mbedtls_pk_parse_public_key(&pk, key, 183)) != 0 ) {
		char buf[128];
		sprintf(buf, "Error mbedtls_pk_parse_public_keyfile returned -0x%04x\n", -ret);
        uart_tx_str((uint8_t*)buf);
        return false;
    }

    mbedtls_printf( "\n  . Verifying the SHA-256 signature\n" );

    if( ( ret = mbedtls_md(
                    mbedtls_md_info_from_type( MBEDTLS_MD_SHA256 ),
                    prg, prgsize, hash) ) != 0 )
    {
		uart_tx_str((uint8_t*)" failed\n  ! Could not open or read\n\n");
        return false;
    }

    if( ( ret = mbedtls_pk_verify( &pk, MBEDTLS_MD_SHA256, hash, 0,
                           sign, signsize) ) != 0 )
    {
        uart_tx_str((uint8_t*)" failed\n  ! mbedtls_pk_verify returned -0x\n");
        return false;
    }

    uart_tx_str((uint8_t*)"\n  . OK (the signature is valid)\n\n" );

    exit_code = MBEDTLS_EXIT_SUCCESS;

    mbedtls_pk_free( &pk );

    return true;
}
