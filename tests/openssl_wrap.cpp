// Fault injection for OpenSSL calls made by crypto.cpp / smp.cpp.
// test_fault_in is a countdown over wrapped calls: when it reaches 0 the
// current call fails, so a test can step through every error branch of an
// operation. -1 disables injection entirely.
#include <openssl/bio.h>
#include <openssl/bn.h>
#include <openssl/evp.h>
#include <openssl/kdf.h>
#include <openssl/pem.h>

extern "C" int test_fault_in = -1;

#define WRAPPED_FAILS() (test_fault_in > 0 && --test_fault_in == 0)

extern "C" {

BIO *__real_BIO_new(const BIO_METHOD *);
BIO *__wrap_BIO_new(const BIO_METHOD *type)
{
	return WRAPPED_FAILS() ? nullptr : __real_BIO_new(type);
}

BIO *__real_BIO_new_mem_buf(const void *, int);
BIO *__wrap_BIO_new_mem_buf(const void *buf, int len)
{
	return WRAPPED_FAILS() ? nullptr : __real_BIO_new_mem_buf(buf, len);
}

EVP_PKEY *__real_PEM_read_bio_PUBKEY(BIO *, EVP_PKEY **, pem_password_cb *, void *);
EVP_PKEY *__wrap_PEM_read_bio_PUBKEY(BIO *bp, EVP_PKEY **x, pem_password_cb *cb, void *u)
{
	return WRAPPED_FAILS() ? nullptr : __real_PEM_read_bio_PUBKEY(bp, x, cb, u);
}

EVP_PKEY *__real_PEM_read_bio_PrivateKey(BIO *, EVP_PKEY **, pem_password_cb *, void *);
EVP_PKEY *__wrap_PEM_read_bio_PrivateKey(BIO *bp, EVP_PKEY **x, pem_password_cb *cb, void *u)
{
	return WRAPPED_FAILS() ? nullptr : __real_PEM_read_bio_PrivateKey(bp, x, cb, u);
}

int __real_PEM_write_bio_PUBKEY(BIO *, EVP_PKEY *);
int __wrap_PEM_write_bio_PUBKEY(BIO *bp, EVP_PKEY *x)
{
	return WRAPPED_FAILS() ? 0 : __real_PEM_write_bio_PUBKEY(bp, x);
}

int __real_PEM_write_bio_PrivateKey(BIO *, EVP_PKEY *, const EVP_CIPHER *, const unsigned char *, int, pem_password_cb *, void *);
int __wrap_PEM_write_bio_PrivateKey(BIO *bp, EVP_PKEY *x, const EVP_CIPHER *enc, const unsigned char *kstr, int klen, pem_password_cb *cb, void *u)
{
	return WRAPPED_FAILS() ? 0 : __real_PEM_write_bio_PrivateKey(bp, x, enc, kstr, klen, cb, u);
}

EVP_PKEY_CTX *__real_EVP_PKEY_CTX_new(EVP_PKEY *, ENGINE *);
EVP_PKEY_CTX *__wrap_EVP_PKEY_CTX_new(EVP_PKEY *pkey, ENGINE *e)
{
	return WRAPPED_FAILS() ? nullptr : __real_EVP_PKEY_CTX_new(pkey, e);
}

EVP_PKEY_CTX *__real_EVP_PKEY_CTX_new_id(int, ENGINE *);
EVP_PKEY_CTX *__wrap_EVP_PKEY_CTX_new_id(int id, ENGINE *e)
{
	return WRAPPED_FAILS() ? nullptr : __real_EVP_PKEY_CTX_new_id(id, e);
}

int __real_EVP_PKEY_keygen_init(EVP_PKEY_CTX *);
int __wrap_EVP_PKEY_keygen_init(EVP_PKEY_CTX *ctx)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_keygen_init(ctx);
}

int __real_EVP_PKEY_CTX_set_rsa_keygen_bits(EVP_PKEY_CTX *, int);
int __wrap_EVP_PKEY_CTX_set_rsa_keygen_bits(EVP_PKEY_CTX *ctx, int bits)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_CTX_set_rsa_keygen_bits(ctx, bits);
}

int __real_EVP_PKEY_keygen(EVP_PKEY_CTX *, EVP_PKEY **);
int __wrap_EVP_PKEY_keygen(EVP_PKEY_CTX *ctx, EVP_PKEY **ppkey)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_keygen(ctx, ppkey);
}

int __real_EVP_PKEY_CTX_set_kem_op(EVP_PKEY_CTX *, const char *);
int __wrap_EVP_PKEY_CTX_set_kem_op(EVP_PKEY_CTX *ctx, const char *op)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_CTX_set_kem_op(ctx, op);
}

int __real_EVP_PKEY_derive_init(EVP_PKEY_CTX *);
int __wrap_EVP_PKEY_derive_init(EVP_PKEY_CTX *ctx)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_derive_init(ctx);
}

int __real_EVP_PKEY_derive_set_peer(EVP_PKEY_CTX *, EVP_PKEY *);
int __wrap_EVP_PKEY_derive_set_peer(EVP_PKEY_CTX *ctx, EVP_PKEY *peer)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_derive_set_peer(ctx, peer);
}

int __real_EVP_PKEY_derive(EVP_PKEY_CTX *, unsigned char *, size_t *);
int __wrap_EVP_PKEY_derive(EVP_PKEY_CTX *ctx, unsigned char *key, size_t *keylen)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_derive(ctx, key, keylen);
}

int __real_EVP_PKEY_encapsulate_init(EVP_PKEY_CTX *, const OSSL_PARAM[]);
int __wrap_EVP_PKEY_encapsulate_init(EVP_PKEY_CTX *ctx, const OSSL_PARAM params[])
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_encapsulate_init(ctx, params);
}

int __real_EVP_PKEY_encapsulate(EVP_PKEY_CTX *, unsigned char *, size_t *, unsigned char *, size_t *);
int __wrap_EVP_PKEY_encapsulate(EVP_PKEY_CTX *ctx, unsigned char *wrappedkey, size_t *wrappedkeylen, unsigned char *genkey, size_t *genkeylen)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_encapsulate(ctx, wrappedkey, wrappedkeylen, genkey, genkeylen);
}

int __real_EVP_PKEY_decapsulate_init(EVP_PKEY_CTX *, const OSSL_PARAM[]);
int __wrap_EVP_PKEY_decapsulate_init(EVP_PKEY_CTX *ctx, const OSSL_PARAM params[])
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_decapsulate_init(ctx, params);
}

int __real_EVP_PKEY_decapsulate(EVP_PKEY_CTX *, unsigned char *, size_t *, const unsigned char *, size_t);
int __wrap_EVP_PKEY_decapsulate(EVP_PKEY_CTX *ctx, unsigned char *unwrapped, size_t *unwrappedlen, const unsigned char *wrapped, size_t wrappedlen)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_PKEY_decapsulate(ctx, unwrapped, unwrappedlen, wrapped, wrappedlen);
}

EVP_CIPHER_CTX *__real_EVP_CIPHER_CTX_new(void);
EVP_CIPHER_CTX *__wrap_EVP_CIPHER_CTX_new(void)
{
	return WRAPPED_FAILS() ? nullptr : __real_EVP_CIPHER_CTX_new();
}

int __real_EVP_CipherInit_ex2(EVP_CIPHER_CTX *, const EVP_CIPHER *, const unsigned char *, const unsigned char *, int, const OSSL_PARAM[]);
int __wrap_EVP_CipherInit_ex2(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *cipher, const unsigned char *key, const unsigned char *iv, int enc, const OSSL_PARAM params[])
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_CipherInit_ex2(ctx, cipher, key, iv, enc, params);
}

int __real_EVP_EncryptInit_ex2(EVP_CIPHER_CTX *, const EVP_CIPHER *, const unsigned char *, const unsigned char *, const OSSL_PARAM[]);
int __wrap_EVP_EncryptInit_ex2(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *cipher, const unsigned char *key, const unsigned char *iv, const OSSL_PARAM params[])
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_EncryptInit_ex2(ctx, cipher, key, iv, params);
}

int __real_EVP_EncryptUpdate(EVP_CIPHER_CTX *, unsigned char *, int *, const unsigned char *, int);
int __wrap_EVP_EncryptUpdate(EVP_CIPHER_CTX *ctx, unsigned char *out, int *outl, const unsigned char *in, int inl)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_EncryptUpdate(ctx, out, outl, in, inl);
}

int __real_EVP_EncryptFinal_ex(EVP_CIPHER_CTX *, unsigned char *, int *);
int __wrap_EVP_EncryptFinal_ex(EVP_CIPHER_CTX *ctx, unsigned char *out, int *outl)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_EncryptFinal_ex(ctx, out, outl);
}

int __real_EVP_DecryptInit(EVP_CIPHER_CTX *, const EVP_CIPHER *, const unsigned char *, const unsigned char *);
int __wrap_EVP_DecryptInit(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *cipher, const unsigned char *key, const unsigned char *iv)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_DecryptInit(ctx, cipher, key, iv);
}

int __real_EVP_DecryptInit_ex2(EVP_CIPHER_CTX *, const EVP_CIPHER *, const unsigned char *, const unsigned char *, const OSSL_PARAM[]);
int __wrap_EVP_DecryptInit_ex2(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *cipher, const unsigned char *key, const unsigned char *iv, const OSSL_PARAM params[])
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_DecryptInit_ex2(ctx, cipher, key, iv, params);
}

int __real_EVP_DecryptUpdate(EVP_CIPHER_CTX *, unsigned char *, int *, const unsigned char *, int);
int __wrap_EVP_DecryptUpdate(EVP_CIPHER_CTX *ctx, unsigned char *out, int *outl, const unsigned char *in, int inl)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_DecryptUpdate(ctx, out, outl, in, inl);
}

int __real_EVP_DecryptFinal_ex(EVP_CIPHER_CTX *, unsigned char *, int *);
int __wrap_EVP_DecryptFinal_ex(EVP_CIPHER_CTX *ctx, unsigned char *outm, int *outl)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_DecryptFinal_ex(ctx, outm, outl);
}

EVP_KDF *__real_EVP_KDF_fetch(void *, const char *, const char *);
EVP_KDF *__wrap_EVP_KDF_fetch(void *ctx, const char *alg, const char *props)
{
	return WRAPPED_FAILS() ? nullptr : __real_EVP_KDF_fetch(ctx, alg, props);
}

EVP_KDF_CTX *__real_EVP_KDF_CTX_new(EVP_KDF *);
EVP_KDF_CTX *__wrap_EVP_KDF_CTX_new(EVP_KDF *kdf)
{
	return WRAPPED_FAILS() ? nullptr : __real_EVP_KDF_CTX_new(kdf);
}

int __real_EVP_KDF_derive(EVP_KDF_CTX *, unsigned char *, size_t, const OSSL_PARAM[]);
int __wrap_EVP_KDF_derive(EVP_KDF_CTX *ctx, unsigned char *key, size_t keylen, const OSSL_PARAM params[])
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_KDF_derive(ctx, key, keylen, params);
}

BIGNUM *__real_BN_new(void);
BIGNUM *__wrap_BN_new(void)
{
	return WRAPPED_FAILS() ? nullptr : __real_BN_new();
}

BN_CTX *__real_BN_CTX_new(void);
BN_CTX *__wrap_BN_CTX_new(void)
{
	return WRAPPED_FAILS() ? nullptr : __real_BN_CTX_new();
}

BIGNUM *__real_BN_dup(const BIGNUM *);
BIGNUM *__wrap_BN_dup(const BIGNUM *from)
{
	return WRAPPED_FAILS() ? nullptr : __real_BN_dup(from);
}

BIGNUM *__real_BN_mpi2bn(const unsigned char *, int, BIGNUM *);
BIGNUM *__wrap_BN_mpi2bn(const unsigned char *s, int len, BIGNUM *ret)
{
	return WRAPPED_FAILS() ? nullptr : __real_BN_mpi2bn(s, len, ret);
}

int __real_BN_bn2mpi(const BIGNUM *, unsigned char *);
int __wrap_BN_bn2mpi(const BIGNUM *a, unsigned char *to)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_bn2mpi(a, to);
}

int __real_BN_bn2bin(const BIGNUM *, unsigned char *);
int __wrap_BN_bn2bin(const BIGNUM *a, unsigned char *to)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_bn2bin(a, to);
}

char *__real_BN_bn2hex(const BIGNUM *);
char *__wrap_BN_bn2hex(const BIGNUM *a)
{
	return WRAPPED_FAILS() ? nullptr : __real_BN_bn2hex(a);
}

BIGNUM *__real_BN_bin2bn(const unsigned char *, int, BIGNUM *);
BIGNUM *__wrap_BN_bin2bn(const unsigned char *s, int len, BIGNUM *ret)
{
	return WRAPPED_FAILS() ? nullptr : __real_BN_bin2bn(s, len, ret);
}

int __real_BN_hex2bn(BIGNUM **, const char *);
int __wrap_BN_hex2bn(BIGNUM **a, const char *str)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_hex2bn(a, str);
}

int __real_BN_sub(BIGNUM *, const BIGNUM *, const BIGNUM *);
int __wrap_BN_sub(BIGNUM *r, const BIGNUM *a, const BIGNUM *b)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_sub(r, a, b);
}

int __real_BN_mod_exp(BIGNUM *, const BIGNUM *, const BIGNUM *, const BIGNUM *, BN_CTX *);
int __wrap_BN_mod_exp(BIGNUM *r, const BIGNUM *a, const BIGNUM *p, const BIGNUM *m, BN_CTX *ctx)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_mod_exp(r, a, p, m, ctx);
}

int __real_BN_mod_mul(BIGNUM *, const BIGNUM *, const BIGNUM *, const BIGNUM *, BN_CTX *);
int __wrap_BN_mod_mul(BIGNUM *r, const BIGNUM *a, const BIGNUM *b, const BIGNUM *m, BN_CTX *ctx)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_mod_mul(r, a, b, m, ctx);
}

int __real_BN_mod_sub(BIGNUM *, const BIGNUM *, const BIGNUM *, const BIGNUM *, BN_CTX *);
int __wrap_BN_mod_sub(BIGNUM *r, const BIGNUM *a, const BIGNUM *b, const BIGNUM *m, BN_CTX *ctx)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_mod_sub(r, a, b, m, ctx);
}

BIGNUM *__real_BN_mod_inverse(BIGNUM *, const BIGNUM *, const BIGNUM *, BN_CTX *);
BIGNUM *__wrap_BN_mod_inverse(BIGNUM *r, const BIGNUM *a, const BIGNUM *n, BN_CTX *ctx)
{
	return WRAPPED_FAILS() ? nullptr : __real_BN_mod_inverse(r, a, n, ctx);
}

int __real_BN_rand(BIGNUM *, int, int, int);
int __wrap_BN_rand(BIGNUM *rnd, int bits, int top, int bottom)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_rand(rnd, bits, top, bottom);
}

int __real_BN_set_word(BIGNUM *, unsigned long);
int __wrap_BN_set_word(BIGNUM *a, unsigned long w)
{
	return WRAPPED_FAILS() ? 0 : __real_BN_set_word(a, w);
}

EVP_MD_CTX *__real_EVP_MD_CTX_new(void);
EVP_MD_CTX *__wrap_EVP_MD_CTX_new(void)
{
	return WRAPPED_FAILS() ? nullptr : __real_EVP_MD_CTX_new();
}

int __real_EVP_DigestInit(EVP_MD_CTX *, const EVP_MD *);
int __wrap_EVP_DigestInit(EVP_MD_CTX *ctx, const EVP_MD *type)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_DigestInit(ctx, type);
}

int __real_EVP_DigestUpdate(EVP_MD_CTX *, const void *, size_t);
int __wrap_EVP_DigestUpdate(EVP_MD_CTX *ctx, const void *d, size_t cnt)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_DigestUpdate(ctx, d, cnt);
}

int __real_EVP_DigestFinal(EVP_MD_CTX *, unsigned char *, unsigned int *);
int __wrap_EVP_DigestFinal(EVP_MD_CTX *ctx, unsigned char *md, unsigned int *s)
{
	return WRAPPED_FAILS() ? 0 : __real_EVP_DigestFinal(ctx, md, s);
}
}
