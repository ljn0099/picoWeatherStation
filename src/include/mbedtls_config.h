#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

/* Workaround for some mbedtls source files using INT_MAX without including limits.h */
#include <limits.h>

#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_HARDWARE_ALT

#define MBEDTLS_SSL_OUT_CONTENT_LEN 2048

#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#define MBEDTLS_HAVE_TIME
#define MBEDTLS_HAVE_TIME_DATE
#define MBEDTLS_PLATFORM_MS_TIME_ALT
#define MBEDTLS_PLATFORM_TIME_ALT

#define MBEDTLS_HAVE_ASM

// Ciphers
#define MBEDTLS_CIPHER_MODE_CBC
#define MBEDTLS_CIPHER_MODE_CFB
#define MBEDTLS_CIPHER_MODE_CTR
#define MBEDTLS_CIPHER_MODE_OFB
#define MBEDTLS_CIPHER_MODE_XTS

// Support for specific padding modes in the cipher layer.
#define MBEDTLS_CIPHER_PADDING_PKCS7
#define MBEDTLS_CIPHER_PADDING_ONE_AND_ZEROS
#define MBEDTLS_CIPHER_PADDING_ZEROS_AND_LEN
#define MBEDTLS_CIPHER_PADDING_ZEROS

// Enable specific cuvers within the Elliptic Curve Module
#define MBEDTLS_ECP_DP_SECP192R1_ENABLED
#define MBEDTLS_ECP_DP_SECP224R1_ENABLED
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED
#define MBEDTLS_ECP_DP_SECP521R1_ENABLED
#define MBEDTLS_ECP_DP_SECP192K1_ENABLED
#define MBEDTLS_ECP_DP_SECP224K1_ENABLED
#define MBEDTLS_ECP_DP_SECP256K1_ENABLED
#define MBEDTLS_ECP_DP_BP256R1_ENABLED
#define MBEDTLS_ECP_DP_BP384R1_ENABLED
#define MBEDTLS_ECP_DP_BP512R1_ENABLED
// Montgomery curves (supporting ECP)
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED
#define MBEDTLS_ECP_DP_CURVE448_ENABLED

// Use nist optimizations for Elliptic Curve
#define MBEDTLS_ECP_NIST_OPTIM

// Enable deterministic ECDSA
#define MBEDTLS_ECDSA_DETERMINISTIC

// Enable the PSK based ciphersuite modes in SSL / TLS
#define MBEDTLS_KEY_EXCHANGE_PSK_ENABLED

// Enable the DHE-PSK based ciphersuite modes in SSL / TLS
#define MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED

// Enable the ECDHE-PSK based ciphersuite modes in SSL / TLS
#define MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED

// Enable the RSA-PSK based ciphersuite modes in SSL / TLS.
#define MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED

// Enable the RSA-only based ciphersuite modes in SSL / TLS
#define MBEDTLS_KEY_EXCHANGE_RSA_ENABLED

// Enable the DHE-RSA based ciphersuite modes in SSL / TLS
#define MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED

// Enable the ECDHE-RSA based ciphersuite modes in SSL / TLS.
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED

// Enable the ECDHE-ECDSA based ciphersuite modes in SSL / TLS.
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED

// Enable the ECDH-ECDSA based ciphersuite modes in SSL / TLS.
#define MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED

// Enable the ECDH-RSA based ciphersuite modes in SSL / TLS.
#define MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED

// Enhance support for reading EC keys using variants of SEC1 not allowed by RFC 5915 and RFC 5480.
#define MBEDTLS_PK_PARSE_EC_EXTENDED

// Enable the support for parsing public keys of type Short Weierstrass
// (MBEDTLS_ECP_DP_SECP_XXX and MBEDTLS_ECP_DP_BP_XXX) which are using the
// compressed point format. This parsing is done through ECP module's functions.
#define MBEDTLS_PK_PARSE_EC_COMPRESSED

// Enable a dummy error function to make use of mbedtls_strerror() in
// third party libraries easier when MBEDTLS_ERROR_C is disabled
#define MBEDTLS_ERROR_STRERROR_DUMMY

// Enable the prime-number generation code.
#define MBEDTLS_GENPRIME

// Support external private RSA keys (eg from a HSM) in the PK layer.
#define MBEDTLS_PK_RSA_ALT_SUPPORT

// Enable support for PKCS#1 v1.5 encoding.
#define MBEDTLS_PKCS1_V15

// Enable support for PKCS#1 v2.1 encoding.
#define MBEDTLS_PKCS1_V21

// Enable the checkup functions (*_self_test).
#define MBEDTLS_SELF_TEST

// Enable sending of alert messages in case of encountered errors as per RFC.
#define MBEDTLS_SSL_ALL_ALERT_MESSAGES

// Enable serialization of the TLS context structures, through use of the
// functions mbedtls_ssl_context_save() and mbedtls_ssl_context_load().
#define MBEDTLS_SSL_CONTEXT_SERIALIZATION

// This allows peers that both support it to use a more robust protection for
// ciphersuites using CBC, providing deep resistance against timing attacks
// on the padding or underlying cipher.
#define MBEDTLS_SSL_ENCRYPT_THEN_MAC

#define MBEDTLS_SSL_EXTENDED_MASTER_SECRET
#define MBEDTLS_SSL_KEEP_PEER_CERTIFICATE
#define MBEDTLS_SSL_RENEGOTIATION
#define MBEDTLS_SSL_MAX_FRAGMENT_LENGTH
#define MBEDTLS_SSL_PROTO_TLS1_3
#define MBEDTLS_SSL_TLS1_3_COMPATIBILITY_MODE
#define MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_ENABLED
#define MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_EPHEMERAL_ENABLED
#define MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_EPHEMERAL_ENABLED
#define MBEDTLS_SSL_ALPN
#define MBEDTLS_SSL_SERVER_NAME_INDICATION
#define MBEDTLS_VERSION_FEATURES
#define MBEDTLS_X509_RSASSA_PSS_SUPPORT
#define MBEDTLS_AESNI_C
#define MBEDTLS_AESCE_C
#define MBEDTLS_AES_C
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_BASE64_C
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_CAMELLIA_C
#define MBEDTLS_ARIA_C
#define MBEDTLS_CCM_C
#define MBEDTLS_CHACHA20_C
#define MBEDTLS_CHACHAPOLY_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CMAC_C
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_DEBUG_C
#define MBEDTLS_DES_C
#define MBEDTLS_DHM_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECJPAKE_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_ERROR_C
#define MBEDTLS_GCM_C
#define MBEDTLS_HKDF_C
#define MBEDTLS_HMAC_DRBG_C
#define MBEDTLS_LMS_C
#define MBEDTLS_NIST_KW_C
#define MBEDTLS_MD_C
#define MBEDTLS_MD5_C
#define MBEDTLS_OID_C
#define MBEDTLS_PADLOCK_C
#define MBEDTLS_PEM_PARSE_C
#define MBEDTLS_PEM_WRITE_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_PK_WRITE_C
#define MBEDTLS_PKCS5_C
#define MBEDTLS_PKCS7_C
#define MBEDTLS_PKCS12_C
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_POLY1305_C
#define MBEDTLS_PSA_CRYPTO_C
#define MBEDTLS_RIPEMD160_C
#define MBEDTLS_RSA_C
#define MBEDTLS_SHA1_C
#define MBEDTLS_SHA224_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA384_C
#define MBEDTLS_SHA512_C
#define MBEDTLS_SHA3_C
#define MBEDTLS_SSL_CACHE_C
#define MBEDTLS_SSL_COOKIE_C
#define MBEDTLS_SSL_TICKET_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_SRV_C
#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_VERSION_C
#define MBEDTLS_X509_USE_C
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_X509_CRL_PARSE_C
#define MBEDTLS_X509_CSR_PARSE_C
#define MBEDTLS_X509_CREATE_C
#define MBEDTLS_X509_CRT_WRITE_C
#define MBEDTLS_X509_CSR_WRITE_C

#endif
