/******************************************************************************/
/* Mednafen Sega Saturn Emulation Module                                      */
/******************************************************************************/
/* jitdump.h - shared Linux perf jitdump writer for the DSP JITs
**  Copyright (C) 2026 pstef
*/

#ifndef __MDFN_SS_JITDUMP_H
#define __MDFN_SS_JITDUMP_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Single per-process jitdump stream shared by the SCU and SCSP DSP
 * JITs.  Without sharing, each compile unit would own its own
 * O_TRUNC'd /tmp/jit-<pid>.dump fd and clobber the other's header.
 * The implementation is in jitdump.c and is compiled only when
 * WANT_DSP_JIT_PERF_DUMP is set and the target is aarch64; on every
 * other configuration the stubs below collapse to no-ops at the call
 * site so callers don't need their own guards.
 */
#if defined(WANT_DSP_JIT_PERF_DUMP) && (defined(__aarch64__) || defined(__arm64__))

void SS_JitDump_Open(void);
void SS_JitDump_Emit(const char* name, const void* code_addr, size_t code_size);

#else

static inline void SS_JitDump_Open(void) {}
static inline void SS_JitDump_Emit(const char* name, const void* code_addr, size_t code_size)
{ (void)name; (void)code_addr; (void)code_size; }

#endif

#ifdef __cplusplus
}
#endif

#endif
