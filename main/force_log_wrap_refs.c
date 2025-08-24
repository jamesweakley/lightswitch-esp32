// Force early undefined references to __wrap_esp_log_write{,v} so that
// the log_wrap static library object providing them is pulled in when
// the archive is first seen by the linker. Without this, the wrappers
// are only referenced by later CHIP/OpenThread libraries that are inside
// a --start-group region; the earlier pass over log_wrap does not extract
// its object file and the group does not revisit outside archives, leading
// to unresolved symbols.
// This array is marked used to prevent linker GC.

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
void __wrap_esp_log_write(int level, const char *tag, const char *fmt, ...);
void __wrap_esp_log_writev(int level, const char *tag, const char *fmt, __builtin_va_list);
#ifdef __cplusplus
}
#endif

// Use a void * array so no code generation (calls) is emitted, only undefined refs.
__attribute__((used)) static void *s_force_log_wrap_refs[] = {
    (void *)&__wrap_esp_log_write,
    (void *)&__wrap_esp_log_writev,
};
