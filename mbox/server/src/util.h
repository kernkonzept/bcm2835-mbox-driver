/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 * Miscellaneous utility functions (pretty "readable" size, read time stamp
 * counter for logging.
 */

#pragma once

#include <string>

struct Util
{
  /// Return descriptive string like '5.6MiB' or similar.
  static std::string readable_size(l4_uint64_t size);

  /**
   * Determine if fine-grained clock available.
   *
   * The fine-grained clock is usually only used for tracing but certain drivers
   * might require it for other purposes. For example, the iproc SDHCI driver
   * uses it for delayed writing.
   *
   * \retval false Fine-grained clock not available.
   * \retval true Fine-grained clock available.
   */
  static bool tsc_available()
  {
    return tsc_init_success;
  }

  /**
   * Initialize fine-grained clock.
   */
  static void tsc_init();

  /**
   * Returns always 0 if no fine-grained clock available.
   *
   * This is no problem for tracing but if accurate time stamps are required for
   * polling, tsc_init() will fail early.
   */
  static l4_uint64_t read_tsc()
  {
#if defined(ARCH_arm)
    l4_uint64_t v;
    asm volatile("mrrc p15, 1, %Q0, %R0, c14" : "=r" (v));
    return v;
#elif defined(ARCH_arm64)
    l4_uint64_t v;
    asm volatile("mrs %0, CNTVCT_EL0" : "=r" (v));
    return v;
#elif defined(ARCH_x86) || defined(ARCH_amd64)
    l4_umword_t lo, hi;
    asm volatile ("rdtsc" : "=a"(lo), "=d"(hi));
    return (l4_uint64_t{hi} << 32) | lo;
#else
    return 0;
#endif
  }

  /// Returns 0 if no fine-grained clock available.
  static l4_uint64_t freq_tsc_hz()
  {
#if defined(ARCH_arm)
    return generic_timer_freq;
#elif defined(ARCH_arm64)
    return generic_timer_freq;
#elif defined(ARCH_x86) || defined(ARCH_amd64)
    return cpu_freq_khz * 1000;
#else
    return 0;
#endif
  }

  /// Returns 0 if no fine-grained clock available.
  static l4_uint64_t tsc_to_us(l4_uint64_t tsc)
  {
#if defined(ARCH_arm) || defined(ARCH_arm64)
    l4_uint64_t freq = freq_tsc_hz();
    return freq ? tsc * 1000000 / freq : 0;
#elif defined(ARCH_x86)
    l4_uint32_t dummy;
    l4_uint64_t us;
    asm ("movl  %%edx, %%ecx \n\t"
         "mull  %3           \n\t"
         "movl  %%ecx, %%eax \n\t"
         "movl  %%edx, %%ecx \n\t"
         "mull  %3           \n\t"
         "addl  %%ecx, %%eax \n\t"
         "adcl  $0, %%edx    \n\t"
         :"=A" (us), "=&c" (dummy)
         :"0" (tsc), "g" (scaler_tsc_to_us));
    return us;
#elif defined(ARCH_amd64)
    l4_uint64_t us, dummy;
    asm ("mulq %3; shrd $32, %%rdx, %%rax"
         :"=a"(us), "=d"(dummy)
         :"a"(tsc), "r"(static_cast<l4_uint64_t>(scaler_tsc_to_us)));
    return us;
#else
    static_cast<void>(tsc);
    return 0;
#endif
  }

  static l4_uint64_t tsc_to_ms(l4_uint64_t tsc)
  {
#if defined(ARCH_arm) || defined(ARCH_arm64)
    l4_uint64_t freq = freq_tsc_hz();
    return freq ? tsc * 1000 / freq : 0;
#elif defined(ARCH_x86) || defined(ARCH_amd64)
    return tsc_to_us(tsc) / 1000;
#else
    static_cast<void>(tsc);
    return 0;
#endif
  }

  static bool tsc_init_success;
#if defined(ARCH_x86) || defined(ARCH_amd64)
  static l4_uint32_t scaler_tsc_to_us;
  static l4_umword_t cpu_freq_khz;
#elif defined(ARCH_arm) || defined(ARCH_arm64)
  static l4_uint32_t generic_timer_freq;
#endif
};
