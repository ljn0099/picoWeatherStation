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

// Enable specific cuvers within the Elliptic Curve Module
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED
#define MBEDTLS_ECP_DP_SECP521R1_ENABLED
// Montgomery curves (supporting ECP)
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED
#define MBEDTLS_ECP_DP_CURVE448_ENABLED

// Use nist optimizations for Elliptic Curve
#define MBEDTLS_ECP_NIST_OPTIM

// Enable deterministic ECDSA
#define MBEDTLS_ECDSA_DETERMINISTIC

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

// This option controls the availability of the API mbedtls_ssl_get_peer_cert()
// giving access to the peer's certificate after completion of the handshake.
#define MBEDTLS_SSL_KEEP_PEER_CERTIFICATE

// Enable support for RFC 6066 max_fragment_length extension in SSL.
#define MBEDTLS_SSL_MAX_FRAGMENT_LENGTH

// Enable support for TLS 1.3
#define MBEDTLS_SSL_PROTO_TLS1_3

// Enable TLS 1.3 middlebox compatibility mode.
#define MBEDTLS_SSL_TLS1_3_COMPATIBILITY_MODE

// Enable TLS 1.3 PSK key exchange mode.
#define MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_ENABLED

// Enable TLS 1.3 ephemeral key exchange mode.
#define MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_EPHEMERAL_ENABLED

// Enable TLS 1.3 PSK ephemeral key exchange mode.
#define MBEDTLS_SSL_TLS1_3_KEY_EXCHANGE_MODE_PSK_EPHEMERAL_ENABLED

// Enable support for ALPN.
#define MBEDTLS_SSL_ALPN

// Enable support for RFC 6066 server name indication (SNI) in SSL.
#define MBEDTLS_SSL_SERVER_NAME_INDICATION

// Allow run-time checking of compile-time enabled features.
#define MBEDTLS_VERSION_FEATURES

// Enable parsing and verification of X.509 certificates, CRLs and CSRS
// signed with RSASSA-PSS (aka PKCS#1 v2.1).
#define MBEDTLS_X509_RSASSA_PSS_SUPPORT

// Enable the AES block cipher.
#define MBEDTLS_AES_C

// Enable the generic ASN1 parser.
#define MBEDTLS_ASN1_PARSE_C

// Enable the generic ASN1 writer.
#define MBEDTLS_ASN1_WRITE_C

// This module is required for PEM support (required by X.509).
#define MBEDTLS_BASE64_C

// Enable the multi-precision integer library.
#define MBEDTLS_BIGNUM_C

// Enable the Camellia block cipher.
#define MBEDTLS_CAMELLIA_C

//  Enable the ARIA block cipher.
#define MBEDTLS_ARIA_C

// Enable the Counter with CBC-MAC (CCM) mode for 128-bit block cipher.
#define MBEDTLS_CCM_C

// Enable the ChaCha20 stream cipher.
#define MBEDTLS_CHACHA20_C

// Enable the ChaCha20-Poly1305 AEAD algorithm.
#define MBEDTLS_CHACHAPOLY_C

// Enable the generic cipher layer.
#define MBEDTLS_CIPHER_C

// Enable the CMAC (Cipher-based Message Authentication Code) mode for block
// ciphers.
#define MBEDTLS_CMAC_C

// Enable the CTR_DRBG AES-based random generator.
#define MBEDTLS_CTR_DRBG_C

// Enable the debug functions.
#define MBEDTLS_DEBUG_C

// Enable the DES block cipher.
#define MBEDTLS_DES_C

// Enable the Diffie-Hellman-Merkle module.
#define MBEDTLS_DHM_C

// Enable the elliptic curve Diffie-Hellman library.
#define MBEDTLS_ECDH_C

// Enable the elliptic curve DSA library.
#define MBEDTLS_ECDSA_C

// Enable the elliptic curve J-PAKE library.
#define MBEDTLS_ECJPAKE_C

// Enable the elliptic curve over GF(p) library
#define MBEDTLS_ECP_C

// Enable the platform-specific entropy code.
#define MBEDTLS_ENTROPY_C

// Enable error code to error string conversion.
#define MBEDTLS_ERROR_C

// Enable the Galois/Counter Mode (GCM).
#define MBEDTLS_GCM_C

// Enable the HKDF algorithm (RFC 5869).
#define MBEDTLS_HKDF_C

// Enable the HMAC_DRBG random generator.
#define MBEDTLS_HMAC_DRBG_C

// Enable the LMS stateful-hash asymmetric signature algorithm.
#define MBEDTLS_LMS_C

// Enable the Key Wrapping mode for 128-bit block ciphers
#define MBEDTLS_NIST_KW_C

// Enable the generic layer for message digest (hashing) and HMAC.
#define MBEDTLS_MD_C

// Enable the MD5 hash algorithm.
#define MBEDTLS_MD5_C

// Enable the OID database.
#define MBEDTLS_OID_C

// Enable PEM decoding / parsing.
#define MBEDTLS_PEM_PARSE_C

// Enable PEM encoding / writing.
#define MBEDTLS_PEM_WRITE_C

// Enable the generic public (asymmetric) key layer.
#define MBEDTLS_PK_C

// Enable the generic public (asymmetric) key parser.
#define MBEDTLS_PK_PARSE_C

// Enable the generic public (asymmetric) key writer.
#define MBEDTLS_PK_WRITE_C

// Enable PKCS#5 functions.
#define MBEDTLS_PKCS5_C

// Enable PKCS #7 core for using PKCS #7-formatted signatures.
#define MBEDTLS_PKCS7_C

// Enable PKCS#12 PBE functions.
#define MBEDTLS_PKCS12_C

// Enable the platform abstraction layer that allows you to re-assign
// functions like calloc(), free(), snprintf(), printf(), fprintf(), exit().
#define MBEDTLS_PLATFORM_C

// Enable the Poly1305 MAC algorithm.
#define MBEDTLS_POLY1305_C

// Enable the Platform Security Architecture cryptography API.
#define MBEDTLS_PSA_CRYPTO_C

// Enable the RIPEMD-160 hash algorithm.
#define MBEDTLS_RIPEMD160_C

// Enable the RSA public-key cryptosystem.
#define MBEDTLS_RSA_C

// Enable the SHA1 cryptographic hash algorithm.
#define MBEDTLS_SHA1_C

// Enable the SHA-224 cryptographic hash algorithm.
#define MBEDTLS_SHA224_C

// Enable the SHA-256 cryptographic hash algorithm.
#define MBEDTLS_SHA256_C

// Enable the SHA-384 cryptographic hash algorithm.
#define MBEDTLS_SHA384_C

// Enable SHA-512 cryptographic hash algorithms.
#define MBEDTLS_SHA512_C

// Enable the SHA3 cryptographic hash algorithm.
#define MBEDTLS_SHA3_C

// Enable simple SSL cache implementation.
#define MBEDTLS_SSL_CACHE_C

//  Enable an implementation of TLS server-side callbacks for session tickets.
#define MBEDTLS_SSL_TICKET_C

// Enable the SSL/TLS client code.
#define MBEDTLS_SSL_CLI_C

// Enable the generic SSL/TLS code.
#define MBEDTLS_SSL_TLS_C

// Enable run-time version information.
#define MBEDTLS_VERSION_C

// Enable X.509 core for using certificates.
#define MBEDTLS_X509_USE_C

// Enable X.509 certificate parsing.
#define MBEDTLS_X509_CRT_PARSE_C

// Enable X.509 CRL parsing.
#define MBEDTLS_X509_CRL_PARSE_C

// Enable X.509 Certificate Signing Request (CSR) parsing.
#define MBEDTLS_X509_CSR_PARSE_C

// Enable X.509 core for creating certificates.
#define MBEDTLS_X509_CREATE_C

// Enable creating X.509 certificates.
#define MBEDTLS_X509_CRT_WRITE_C

// Enable creating X.509 Certificate Signing Requests (CSR).
#define MBEDTLS_X509_CSR_WRITE_C

#endif
