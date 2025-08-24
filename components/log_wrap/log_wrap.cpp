// Pass-through wrappers for ESP-IDF logging.
// IDF 5.5 adds linker flags --wrap=esp_log_write{,v}. Some components (esp-matter
// logging glue, OpenThread) may directly reference the wrapper symbols.
// Provide them here so link succeeds even if we don't customize logging yet.

#include "esp_log.h"
#include <stdarg.h>

extern "C" {
void __real_esp_log_write(esp_log_level_t, const char *tag, const char *fmt, ...);
void __real_esp_log_writev(esp_log_level_t, const char *tag, const char *fmt, va_list);

void __wrap_esp_log_writev(esp_log_level_t level, const char *tag, const char *fmt, va_list args) {
    __real_esp_log_writev(level, tag, fmt, args);
}

void __wrap_esp_log_write(esp_log_level_t level, const char *tag, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    __wrap_esp_log_writev(level, tag, fmt, args);
    va_end(args);
}
}
