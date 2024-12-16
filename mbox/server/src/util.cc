/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 * Miscellaneous utility functions (pretty "readable" size, read time stamp
 * counter for logging (implementation).
 */

#include <string>
#include <l4/re/env>
#include "util.h"

#if defined(ARCH_x86) || defined(ARCH_amd64)
l4_uint32_t Util::scaler_tsc_to_us;
l4_umword_t Util::cpu_freq_khz;
#elif defined(ARCH_arm) || defined(ARCH_arm64)
l4_uint32_t Util::generic_timer_freq;
#endif
bool Util::tsc_init_success;

std::string
Util::readable_size(l4_uint64_t size)
{
  l4_uint32_t order = 1 << 30;
  for (unsigned i = 3;; --i, order >>= 10)
    {
      if (i == 1 || size >= order)
        {
          l4_uint64_t d2 = size / (order / 1024);
          l4_uint64_t d1 = d2 / (1 << 10);
          d2 = (d2 - d1 * 1024) * 10 / 1024;
          std::string s = std::to_string(d1);
          if (d2)
            s += "." + std::to_string(d2);
          return s + " KMG"[i] + "iB";
        }
    }
}

void
Util::tsc_init()
{
#if defined(ARCH_arm)

  // Read from virtual counter of ARM generic timer.
  l4_uint32_t v;
  asm volatile ("mrc p15, 0, %0, c14, c0, 0": "=r" (v));
  generic_timer_freq = v;
  tsc_init_success = true;

#elif defined(ARCH_arm64)

  // Read from virtual counter of ARM generic timer.
  l4_uint64_t v;
  asm volatile("mrs %0, CNTFRQ_EL0": "=r" (v));
  generic_timer_freq = v;
  tsc_init_success = true;

#elif defined(ARCH_x86) || defined(ARCH_amd64)

  auto muldiv = [](l4_uint32_t a, l4_uint32_t mul, l4_uint32_t div)
    {
      l4_uint32_t d;
      asm ("mull %4 ; divl %3\n\t"
           :"=a" (a), "=d" (d) :"a" (a), "c" (div), "d" (mul));
      return a;
    };

  l4_kernel_info_t const *kip = l4re_kip();
  if (!kip)
    return;
  if (!kip->frequency_cpu || kip->frequency_cpu >= 50'000'000U)
    return;

  cpu_freq_khz = kip->frequency_cpu;

  // See also l4re-core/l4util/l4_tsc_init().
  // scaler = (2^30 * 4'000) / (Hz / 1000) = (2^32 * 1'000'000) / Hz
  scaler_tsc_to_us = muldiv(1U << 30, 4000, cpu_freq_khz);

  tsc_init_success = true;

#else

  // n/a

#endif
}
