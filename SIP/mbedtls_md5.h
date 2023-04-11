#pragma once


#include <iostream>

extern "C" {
#include "mbedtls/md5.h"
}

class MbedtlsMd5
{
public:
    MbedtlsMd5()
    {
        mbedtls_md5_init(&m_ctx);
    }

    ~MbedtlsMd5()
    {
        mbedtls_md5_free(&m_ctx);
    }

    void start()
    {
        mbedtls_md5_starts(&m_ctx);
    }

    void update(const std::string& input)
    {
        mbedtls_md5_update(&m_ctx, reinterpret_cast<const unsigned char*>(input.c_str()), input.size());
    }

    void finish(unsigned char hash[16])
    {
        mbedtls_md5_finish(&m_ctx, hash);
    }

private:
    mbedtls_md5_context m_ctx;
};
