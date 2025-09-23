#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

char* getenv(const char* name) { return NULL; }

int vsnprintf(char* s, size_t n, const char* format, va_list arg) {
  // Minimal stub: do nothing, return 0
  (void)s; (void)n; (void)format; (void)arg;
  return 0;
}

const unsigned short* __locale_ctype_ptr(void) { return NULL; }

unsigned long strtoul(const char* nptr, char** endptr, int base) { return 0; }

size_t fwrite(const void* ptr, size_t size, size_t nmemb, void* stream) {
  (void)ptr; (void)size; (void)stream;
  return nmemb;
}

void exit(int code) { (void)code; while(1); }

#ifdef __cplusplus
}
#endif

void _init(void) {}