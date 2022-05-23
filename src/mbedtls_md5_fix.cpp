#include <mbedtls/md5.h>
#include <mbedtls/error.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// fix for mismatch of mbedtls version and ESPAsyncWebServer/WebAuthentication.cpp

void mbedtls_md5_init( mbedtls_md5_context *ctx ) {}

int mbedtls_md5_starts_ret( mbedtls_md5_context *ctx ) 
{
    return( 0 );
}

int mbedtls_md5_update_ret( mbedtls_md5_context *ctx,
                            const unsigned char *input,
                            size_t ilen )
{
    return( 0 );
}

int mbedtls_md5_finish_ret( mbedtls_md5_context *ctx,
                            unsigned char output[16] )
{
    return( 0 );
}


#ifdef __cplusplus
}
#endif