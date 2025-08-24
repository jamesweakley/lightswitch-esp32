// Lightweight pass-through wrappers for ESP-IDF log functions.
// IDF 5.5 link line adds -Wl,--wrap=esp_log_write and -Wl,--wrap=esp_log_writev
// but this project (or selected components) doesn't provide custom wrappers.
// Provide them here so any components (e.g. esp-matter / diagnostics) that
// directly reference __wrap_esp_log_write{,v} will link. Adjust if you later
// need to funnel logs elsewhere.

#include "esp_log.h"
#include <stdarg.h>

extern "C" {
// Forward declarations for the real functions (available because of --wrap)
void __real_esp_log_write(esp_log_level_t level, const char *tag, const char *format, ...);
void __real_esp_log_writev(esp_log_level_t level, const char *tag, const char *format, va_list args);

void __wrap_esp_log_writev(esp_log_level_t level, const char *tag, const char *format, va_list args) {
    // Simple passthrough for now; hook filtering or redaction here if desired.
    __real_esp_log_writev(level, tag, format, args);
}

void __wrap_esp_log_write(esp_log_level_t level, const char *tag, const char *format, ...) {
    va_list args;
    va_start(args, format);
    __wrap_esp_log_writev(level, tag, format, args);
    va_end(args);
}
}
