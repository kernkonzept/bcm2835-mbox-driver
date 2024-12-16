/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */
/**
 * \file
 * Mailbox basic bits.
 *
 * Used by clients as well as by the server.
 */

#pragma once

#include <cstring>
#include <functional>
#include <string>

#include <l4/cxx/bitfield>
#include <l4/cxx/minmax>
#include <l4/cxx/utils>
#include <l4/drivers/hw_mmio_register_block>
#include <l4/re/env>
#include <l4/re/error_helper>
#include <l4/re/rm>
#include <l4/re/util/debug>
#include <l4/sys/cache.h>

template <class IMPL>
class Bcm2835_mbox_base
{
public:
  // Device can only handle 32-bit DMA addresses.
  using Dma_addr = l4_uint32_t;

  // Mailbox register map
  enum
  {
    // Mailbox 0 for read
    Mbox0_read   = 0x00,
    Mbox0_peak   = 0x10,
    Mbox0_sender = 0x14,
    Mbox0_status = 0x18,
    Mbox0_config = 0x1c,
    // Mailbox 1 for write
    Mbox1_write  = 0x20,
    Mbox1_peek   = 0x30,
    Mbox1_sender = 0x34,
    Mbox1_status = 0x38,
    Mbox1_config = 0x3c,
  };

  // MboxX_status bits
  enum
  {
    Mbox_status_read_empty = (1U << 30),
    Mbox_status_write_full = (1U << 31),
  };

  enum { Channel_mask = 0xf };

  /**
   * bcm2835 SoC Mailbox channels.
   *
   * See https://github.com/raspberrypi/firmware/wiki/Mailboxes
   * So far we only support Property_vc.
   */
  enum class Chan : l4_uint32_t
  {
    Power_management = 0,
    Framebuffer = 1,
    Virtual_uart = 2,
    Vchiq = 3,
    Leds = 4,
    Buttons = 5,
    Touch_screen = 6,
    Property_vc = 8,
    Max = 15,
  };

  enum
  {
    Raspi_exp_gpio_bt = 0,              // active high
    Raspi_exp_gpio_wifi = 1,            // active low
    Raspi_exp_gpio_led_pwr = 2,         // active low
    Raspi_exp_gpio_vdd_sd_io_sel = 4,   // active high
    Raspi_exp_gpio_cam1 = 5,            // active high
    Raspi_exp_gpio_vcc_sd = 6,          // active high
  };

  /**
   * bcm2835 SoC revision number decoding.
   *
   * See https://github.com/raspberrypi/documentation/blob/develop/
   *      documentation/asciidoc/computers/raspberry-pi/revision-codes.adoc
   */
  struct Soc_rev
  {
    l4_uint32_t raw;
    /** Overvoltage allowed. */
    CXX_BITFIELD_MEMBER(31, 31, overvoltage, raw);
    /** OTP programming allowed. */
    CXX_BITFIELD_MEMBER(30, 30, otp_program, raw);
    /** OTP reading allowed. */
    CXX_BITFIELD_MEMBER(29, 29, otp_read, raw);
    /** Warranty has been voided by overclocking. */
    CXX_BITFIELD_MEMBER(25, 25, warranty, raw);
    /** New-style revision. */
    CXX_BITFIELD_MEMBER(23, 23, new_style, raw);
    /** Memory size. 0=256MB, 1=512MB, 2=1GB, 3=2GB, 4=4GB, 8=8GB. */
    CXX_BITFIELD_MEMBER(20, 22, memory_size, raw);
    /** Manufacturer: 0=Sony UK, 1=Egoman, 2=Embest, 3=Sony Japan, ... */
    CXX_BITFIELD_MEMBER(16, 19, manufacturer, raw);
    /** Processor: 0=BCM2835, 1=BCM2836, 2=BCM2837, 3=BCM2711, 4=BCM2712 */
    CXX_BITFIELD_MEMBER(12, 15, processor, raw);
    /** Type. */
    CXX_BITFIELD_MEMBER(4, 11, type, raw);
    /** Revision. */
    CXX_BITFIELD_MEMBER(0, 3, revision, raw);
  };

  /**
   * Known message tags. See
   * https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
   */
  struct Tag
  {
    enum class Type : l4_uint32_t
    {
      No_data = 0,
      Get_fw_version = 0x00000001,
      Get_board_model = 0x00010001,
      Get_board_rev = 0x00010002,
      Get_board_mac = 0x00010003,
      Get_board_serial = 0x00010004,
      Get_arm_memory = 0x00010005,
      Get_vc_memory = 0x00010006,
      Get_cmdline = 0x00050001,
      Get_dma_channels = 0x00060001,
      Get_clocks = 0x00010007,
      Get_power_state = 0x00020001,
      Set_power_state = 0x00028001,
      Get_timing = 0x00020002,
      Get_clock_state = 0x00030001,
      Set_clock_state = 0x00038001,
      Get_clock_rate = 0x00030002,
      Set_clock_rate = 0x00038002,
      Get_clock_rate_max = 0x00030004,
      Get_clock_rate_min = 0x00030007,
      Get_clock_rate_actual = 0x00030047,
      Get_throttled = 0x00030046,
      Get_domain_state = 0x00030030,
      Set_domain_state = 0x00038030,
      Get_gpio_state = 0x00030041,
      Set_gpio_state = 0x00038041,
      Get_gpio_config = 0x00030043,
      Set_gpio_config = 0x00038043,
      Get_periph_reg = 0x00030045,
      Set_periph_reg = 0x00038045,
      Notify_reboot = 0x00030048,
      Get_turbo = 0x00030009,
      Set_turbo = 0x00038009,
      Get_voltage = 0x00030003,
      Set_voltage = 0x00038003,
      Get_voltage_max = 0x00030005,
      Get_voltage_min = 0x00030008,
      Get_temp = 0x00030006,
      Get_temp_max = 0x0003000a,
      Allocate_memory = 0x0003000c,
      Lock_memory = 0x0003000d,
      Unlock_memory = 0x0003000e,
      Release_memory = 0x0003000f,
      Execute_code = 0x00030010,
      Get_dispmanx_res_mem_hdl = 0x00030014,
      Get_edid_block = 0x00030020,
    };
    Type tag;                   ///< firmware property tag: tag
    l4_uint32_t size_out;       ///< firmware property tag: value size
    l4_uint32_t size_in;        ///< firmware property tag: response size

    l4_uint32_t get_size_in() const { return size_in & ~(1U << 31); }
    std::string to_str() const;
  };

  /** Length in 32-bit words of messages for sending and receiving. */
  static constexpr unsigned tag_words(typename Tag::Type tag_type)
  {
    using Type = typename Tag::Type;
    switch (tag_type)
      {
      case Type::No_data: return 0;
      case Type::Get_fw_version: return 1;
      case Type::Get_board_model: return 1;
      case Type::Get_board_rev: return 1;
      case Type::Get_board_mac: return 2;
      case Type::Get_board_serial: return 2;
      case Type::Get_arm_memory: return 2;
      case Type::Get_vc_memory: return 2;
      case Type::Get_cmdline: return 0;
      case Type::Get_dma_channels: return 1;
      case Type::Get_clocks: return 0;
      case Type::Get_power_state: return 2;
      case Type::Set_power_state: return 2;
      case Type::Get_timing: return 2;
      case Type::Get_clock_state: return 2;
      case Type::Set_clock_state: return 2;
      case Type::Get_clock_rate: return 2;
      case Type::Set_clock_rate: return 3;
      case Type::Get_clock_rate_max: return 2;
      case Type::Get_clock_rate_min: return 2;
      case Type::Get_clock_rate_actual: return 2;
      case Type::Get_throttled: return 1;
      case Type::Get_domain_state: return 2;
      case Type::Set_domain_state: return 2;
      case Type::Get_gpio_state: return 2;
      case Type::Set_gpio_state: return 2;
      case Type::Get_gpio_config: return 5;
      case Type::Set_gpio_config: return 6;
      case Type::Get_periph_reg: return 2; // XXX
      case Type::Set_periph_reg: return 2; // XXX
      case Type::Notify_reboot: return 0;
      case Type::Get_turbo: return 2;
      case Type::Set_turbo: return 2;
      case Type::Get_voltage: return 2;
      case Type::Set_voltage: return 2;
      case Type::Get_voltage_max: return 2;
      case Type::Get_voltage_min: return 2;
      case Type::Get_temp: return 2;
      case Type::Get_temp_max: return 2;
      case Type::Allocate_memory: return 3;
      case Type::Lock_memory: return 1;
      case Type::Unlock_memory: return 1;
      case Type::Release_memory: return 1;
      case Type::Execute_code: return 7;
      case Type::Get_dispmanx_res_mem_hdl: return 2;
      case Type::Get_edid_block: return 34;
      default: return 0;
      }
  };

  /** Messages for sending and receiving. */
  template <typename Tag::Type tag_type = Tag::Type::No_data>
  struct alignas(16) Message
  {
    struct Hdr
    {
      /** Message request status. */
      enum class Status : l4_uint32_t
      {
        Request = 0U,           ///< Request phase.
        Success = 0x80000000U,  ///< Request successfully finished.
        Error   = 0x80000001U,  ///< Request failed.
        Mask    = 0x80000001U,
      };
      bool status_ok() const { return status == Status::Success; }

      l4_uint32_t bytes;
      Status status;

      std::string status_str() const
      {
        switch (status)
          {
          case Status::Request: return "RQ";
          case Status::Success: return "ok";
          case Status::Error: return "ERROR";
          default:
            {
              char s[16];
              snprintf(s, sizeof(s), "%x ", static_cast<l4_uint32_t>(status));
              return std::string(s);
            }
          }
      }
    };

    enum
    {
      Msg_data_words = tag_words(tag_type),
      Msg_data_bytes = 4 * Msg_data_words,
      Msg_nodata_bytes = sizeof(Hdr) + sizeof(Tag) + sizeof(l4_uint32_t),
      Msg_nodata_words = Msg_nodata_bytes / 4,
      Msg_all_bytes = Msg_nodata_bytes + Msg_data_bytes,
    };

    Hdr hdr = { Msg_all_bytes, Hdr::Status::Request };
    Tag tag = { tag_type, Msg_data_bytes, 0 };
    l4_uint32_t data[Msg_data_words + 1] = { 0, };

    Message() = default;

    Message(l4_uint32_t words)
    : hdr{4 * words, Hdr::Status::Request},
      tag{tag_type, words * 4 - Msg_nodata_bytes, 0}
    {
      if (words < Msg_nodata_words)
        L4Re::throw_error(-L4_EINVAL, "mbox message too short");
      // including terminator
      memset(data, 0, 4 * words - sizeof(Hdr) - sizeof(Tag));
    }

    l4_uint32_t const *raw() const { return &hdr.bytes; }
    l4_uint32_t *raw() { return &hdr.bytes; }

    /** Print up to two message words to string buffer (diagnostics). */
    void snprintf_msg_words(char *str, size_t str_size, size_t msg_size) const
    {
      // no I/O streams because of 'nofpu'
      if (msg_size >= 4)
        {
          char *s = str;
          s += snprintf(s, s - str + str_size, " (%08x",
                        static_cast<l4_uint32_t>(data[0]));
          if (msg_size >= 8)
            s += snprintf(s, s - str + str_size, ", %08x",
                          static_cast<l4_uint32_t>(data[1]));
          if (msg_size >= 12)
            s += snprintf(s, s - str + str_size, ", ...");
          snprintf(s, s - str + str_size, ")");
        }
    }

    /** Show mbox context (diagnostics). */
    void log(L4Re::Util::Dbg const &log, char const *text, Dma_addr phys)
    {
      l4_uint32_t bytes = hdr.bytes;

      std::string ostr;
      for (unsigned i = 0; i < bytes / 4; ++i)
        {
          // no string stream available with `nofpu`
          char s[16];
          snprintf(s, sizeof(s), "%08x ", raw()[i]);
          ostr += s;
        }
      log.printf("%s phys=%08x: %s\n", text, phys, ostr.c_str());
    }

    /**
     * Copy letter to memory where the device can fetch it.
     *
     * \param mbox_virt  Pointer to copy the message to.
     */
    void to_mbox_before_dma(void *mbox_virt) const
    {
      memcpy(mbox_virt, &hdr, hdr.bytes);
      l4_addr_t cache_beg = reinterpret_cast<l4_addr_t>(mbox_virt);
      l4_addr_t cache_end = cache_beg + hdr.bytes;
      l4_cache_flush_data(cache_beg, cache_end);
    }

    /**
     * Fetch letter from memory where device wrote it.
     *
     * \param mbox_virt      Pointer to fetch the modified message from.
     * \param max_data_size  Maximum data size. Usually taken from "out" phase.
     */
    void from_mbox_after_dma(void const *mbox_virt, l4_uint32_t max_data_size)
    {
      auto *mbox_msg = static_cast<Message<> const *>(mbox_virt);
      l4_addr_t cache_beg = reinterpret_cast<l4_addr_t>(mbox_msg);
      l4_addr_t cache_end = cache_beg + Msg_nodata_bytes + max_data_size;
      l4_cache_inv_data(cache_beg, cache_end);
      memcpy(&hdr.bytes, mbox_msg, sizeof(Hdr) + sizeof(Tag));
      memcpy(&data, mbox_msg->data, cxx::min(max_data_size, tag.get_size_in()));
    }

    /** Required number of 4-byte words for message including header and tag. */
    l4_uint32_t required_words() const
    { return (tag.get_size_in() + 3) / 4 + Msg_nodata_words; }

    /** Verify that this message has correct tag type at least required size. */
    bool is_valid() const
    { return hdr.bytes >= Msg_all_bytes && tag.tag == tag_type; }

    /** Cast this message to a message of another type (non-const). */
    template <typename Tag::Type to_tag_type>
    Message<to_tag_type> *to_msg()
    { return reinterpret_cast<Message<to_tag_type> *>(this); }

    /** Cast this message to a message of another type (const). */
    template <typename Tag::Type to_tag_type>
    Message<to_tag_type> const *to_msg() const
    { return reinterpret_cast<Message<to_tag_type> const *>(this); }

    /** Verify if this message is a valid message of the specified tag type. */
    template <typename Tag::Type to_tag_type>
    bool valid_msg() const
    { return to_msg<to_tag_type>()->is_valid(); }
  };

  inline void rmb()
  {
    if (_remote_service)
      {
#if defined(__ARM_ARCH) && __ARM_ARCH >= 8
        asm volatile ("dmb ld" ::: "memory");
#elif defined(__ARM_ARCH) && __ARM_ARCH == 7
        asm volatile ("dmb st" ::: "memory");
#elif defined(__amd64__) || defined(__i386__) || defined(__i686__)
        asm volatile ("lfence" ::: "memory");
#elif defined(__mips__)
        asm volatile ("sync" : : : "memory");
#elif defined(__riscv)
        asm volatile ("fence ir, ir" ::: "memory");
#else
#warning Missing proper memory read barrier
        asm volatile ("" ::: "memory");
#endif
      }
  }

  inline void wmb()
  {
    if (!_remote_service)
      {
#if defined(__ARM_ARCH) && __ARM_ARCH >= 7
        asm volatile ("dmb st":::"memory");
#elif defined(__amd64__) || defined(__i386__) || defined(__i686__)
        asm volatile ("sfence" ::: "memory");
#elif defined(__mips__)
        asm volatile ("sync" : : : "memory");
#elif defined(__riscv)
        asm volatile ("fence ow, ow" ::: "memory");
#else
#warning Missing proper memory write barrier
        asm volatile ("" ::: "memory");
#endif
      }
  }

  /** Submit request. */
  void send_mail(l4_uint32_t letter, Chan channel, L4Re::Util::Dbg &log)
  {
    if (letter & Channel_mask)
      L4Re::throw_error(-L4_EINVAL, "Send_mail: No place for channel in `letter`");
    if (channel > Chan::Max)
      L4Re::throw_error(-L4_EINVAL, "Send_mail: Wrong channel");

    // Timeout 20ms
    unsigned i;
    for (i = 0; (_regs[Mbox1_status] & Mbox_status_write_full) && i < 100; ++i)
      busy_wait_us(200);
    if (i >= 100)
      log.printf("Timeout exceeded waiting for mbox being ready for write!\n");

    l4_uint32_t value = letter | static_cast<l4_uint32_t>(channel);
    // Could also write DMA offset if not Mmio space.
    wmb();
    _regs[Mbox1_write] = value;
  }

  /** Read request result. */
  l4_uint32_t read_mail(Chan channel, L4Re::Util::Dbg &log)
  {
    if (channel > Chan::Max)
      return 0;

    l4_uint32_t letter;
    // Timeout 20ms * 100 = 2s
    unsigned i, j;
    for (i = 0; i < 100; ++i)
      {
        for (j = 0; (_regs[Mbox0_status] & Mbox_status_read_empty) && j < 100; ++j)
          busy_wait_us(200);
        if (j >= 100)
          log.printf("Timeout exceeded waiting for mbox being ready for read!\n");
        letter = _regs[Mbox0_read];
        if (static_cast<Chan>(letter & 0xf) == channel)
          break;
      }
    if (i >= 100)
      log.printf("Timeout exceeded waiting for mbox proving data to read!\n");

    rmb();

    // Need to poll once again to re-enable interrupt notification (if enabled)!
    volatile l4_uint32_t status = _regs[Mbox0_status];
    static_cast<void>(status);

    return letter & ~0xf;
  }

  void busy_wait_us(l4_uint32_t us)
  {
    auto const *kip = l4re_kip();
    l4_uint64_t until = l4_kip_clock(kip) + us;
    while (l4_kip_clock(kip) < until)
      l4_barrier();
  }


  // ==== Board-specific ====

  /** Retrieve the GPIO setting for a particular expander GPIO pin. */
  l4_uint32_t get_fw_gpio(unsigned offset)
  {
    Message<Bcm2835_mbox_base::Tag::Type::Get_gpio_state> msg;
    msg.data[0] = 128 + offset;
    static_cast<IMPL &>(*this).do_request(&msg);
    return msg.data[1];
  }

  /** Set the GPIO setting for a particular expander GPIO pin. */
  void set_fw_gpio(unsigned offset, l4_uint32_t value)
  {
    Message<Bcm2835_mbox_base::Tag::Type::Set_gpio_state> msg;
    msg.data[0] = 128 + offset;
    msg.data[1] = value;
    static_cast<IMPL &>(*this).do_request(&msg);
  }

  /** Get Bcm2835 SoC board revision. */
  Soc_rev get_board_rev()
  {
    Message<Bcm2835_mbox_base::Tag::Type::Get_board_rev> msg;
    static_cast<IMPL &>(*this).do_request(&msg);
    return Soc_rev{msg.data[0]};
  }

  std::string get_board_mac()
  {
    Message<Bcm2835_mbox_base::Tag::Type::Get_board_mac> msg;
    static_cast<IMPL &>(*this).do_request(&msg);
    char buf[24];
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             msg.data[0] & 0xff,
             (msg.data[0] & 0xff00) >> 8,
             (msg.data[0] & 0xff0000) >> 16,
             (msg.data[0] & 0xff000000) >> 24,
             msg.data[1] & 0xff,
             (msg.data[1] & 0xff00) >> 8);
    return buf;
  }

protected:
  // Convenience class for local MMIO access.
  struct Mmio_map_register_block_32 : L4drivers::Mmio_register_block<32>
  {
    explicit Mmio_map_register_block_32(L4::Cap<L4Re::Dataspace> iocap,
                                        l4_uint64_t phys_addr, l4_uint64_t size)
    {
      auto *e = L4Re::Env::env();
      l4_addr_t offset = phys_addr & ~L4_PAGEMASK;
      auto rm_flags = L4Re::Rm::F::Search_addr
                      | L4Re::Rm::F::Cache_uncached
                      | L4Re::Rm::F::RW
                      | L4Re::Rm::F::Eager_map;
      L4Re::chksys(e->rm()->attach(&vregion, size, rm_flags,
                                   L4::Ipc::make_cap_rw(iocap),
                                   phys_addr, L4_PAGESHIFT),
                   "Attach I/O memory");
      this->set_base(vregion.get() + offset);
    }
    L4Re::Rm::Unique_region<l4_addr_t> vregion;
  };

  // false=local (direct MMIO access), true=remote (Mmio_space access)
  bool _remote_service = false;

  // local/remote: either direct device access (local) or MMIO space (remote)
  L4drivers::Register_block<32> _regs;
};
