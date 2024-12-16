/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 */

#pragma once

#include <l4/cxx/ipc_stream>
#include <l4/cxx/ref_ptr>
#include <l4/re/mmio_space>
#include <l4/re/error_helper>
#include <l4/re/util/object_registry>
#include <l4/sys/cxx/ipc_epiface>
#include <l4/vbus/vbus>

#include <string>
#include <cstring>

#include "consts.h"
#include "debug.h"
#include "bcm2835_mbox.h"

class Client
: public L4::Epiface_t<Client,
                       L4::Kobject_2t<void, L4vbus::Vbus, L4Re::Mmio_space>>,
  public cxx::Ref_obj
{
  enum : l4vbus_device_handle_t
  {
    Id_icu = 0x12344321,
    Id_mbox = 0x12344322,
  };

  // number of letters processed simultaneously for this client
  enum { Num_letters = 16  };

  static constexpr char const *hidname_mbox = "BCM2835_mbox";
  static constexpr l4_uint64_t res_mbox_start = 0xfe00b880;
  static constexpr l4_uint64_t res_mbox_end = 0xfe00b8bf;
  static constexpr l4_size_t   log2_size_mbox_data = Log2_client_buffer_size;
  static constexpr l4_size_t   size_mbox_data = 1U << log2_size_mbox_data;
  static constexpr l4_uint64_t res_mbox_data_start = 0x00100000;
  static constexpr l4_uint64_t res_mbox_data_end = res_mbox_data_start
                                                 + size_mbox_data - 1;
  static constexpr char const *hidname_icu = "L40009";
  static constexpr unsigned res_mbox_irqnum = 32 + 0x21;

public:
  Client(L4Re::Util::Object_registry *registry, Bcm2835_mbox *phys_device,
         std::string const &profile,
         std::vector<l4_uint8_t> const &expgpio_pins,
         std::vector<l4_uint8_t> const &clocks);

  ~Client()
  {}


  // Mmio_space interface
  long op_mmio_read(L4::Typeid::Rights<L4Re::Mmio_space>,
                    L4Re::Mmio_space::Addr addr, char log2_size,
                    l4_uint64_t &value)
  {
    try
      {
        if (log2_size != 2)
          L4Re::throw_error(-L4_EINVAL, "Only 32-bit access allowed");
        return handle_mmio_read(addr, value);
      }
    catch (L4::Runtime_error const &e)
      {
        warn.printf("Mmio_space read: %s: %s\n", e.extra_str(), e.str());
        return e.err_no();
      }
  }
  long op_mmio_write(L4::Typeid::Rights<L4Re::Mmio_space>,
                     L4Re::Mmio_space::Addr addr, char log2_size,
                     l4_uint64_t value)
  {
    try
      {
        if (log2_size != 2)
          L4Re::throw_error(-L4_EINVAL, "Only 32-bit access allowed");
        return handle_mmio_write(addr, value);
      }
    catch (L4::Runtime_error const &e)
      {
        warn.printf("Mmio_space write: %s: %s\n", e.extra_str(), e.str());
        return e.err_no();
      }
  }

  // IO server dispatch interface
  l4_msgtag_t op_dispatch(l4_utcb_t *utcb, l4_msgtag_t tag, L4vbus::Vbus::Rights)
  {
    L4::Ipc::Iostream ios(utcb);
    ios.Istream::tag() = tag;

    l4vbus_device_handle_t devid;
    if (L4_UNLIKELY(!ios.get(devid)))
      return l4_msgtag(-L4_EMSGTOOSHORT, 0, 0, 0);

    l4_uint32_t func;
    if (L4_UNLIKELY(!ios.get(func)))
      return l4_msgtag(-L4_EMSGTOOSHORT, 0, 0, 0);

    long ret = handle_vbus_msg(devid, func, ios);

    return ios.prepare_ipc(ret);
  }

  // Dataspace for IO
  long op_allocate(L4Re::Dataspace::Rights, l4_addr_t, l4_size_t)
  { return -L4_ENOSYS; }
  long op_copy_in(L4Re::Dataspace::Rights, L4Re::Dataspace::Offset,
                  L4::Ipc::Snd_fpage const &, L4Re::Dataspace::Offset,
                  L4Re::Dataspace::Size)
  { return -L4_ENOSYS; }
  long op_info(L4Re::Dataspace::Rights, L4Re::Dataspace::Stats &)
  { return -L4_ENOSYS; }
  long op_clear(L4Re::Dataspace::Rights, L4Re::Dataspace::Offset,
                L4Re::Dataspace::Size)
  { return -L4_ENOSYS; }
  long op_map(L4Re::Dataspace::Rights, L4Re::Dataspace::Offset addr,
              L4Re::Dataspace::Map_addr, L4Re::Dataspace::Flags flags,
              L4::Ipc::Snd_fpage &snd_fp)
  {
    if (addr >= res_mbox_data_start && addr <= res_mbox_data_end)
      {
        using Snd_fpage = L4::Ipc::Snd_fpage;
        using Dataspace = L4Re::Dataspace;
        Snd_fpage::Cacheopt cache_opts;
        if ((flags & Dataspace::F::Caching_mask).raw == Dataspace::F::Cacheable)
          cache_opts = Snd_fpage::Cached;
        else
          cache_opts = Snd_fpage::Uncached;
        snd_fp = Snd_fpage(l4_fpage(reinterpret_cast<l4_addr_t>(_mbox_data_dma_virt),
                                    log2_size_mbox_data, L4_FPAGE_RW),
                           0, Snd_fpage::Map, cache_opts);
        return L4_EOK;
      }

    warn.printf("Cannot map physical memory %08llx!\n", addr);
    return -L4_ENOSYS;
  }

  long op_map_info(L4Re::Dataspace::Rights, l4_addr_t &, l4_addr_t &)
  {
    printf("\033[35mop_map_info\033[m\n");
    return -L4_ENOSYS;
  }


  // ICU for Vbus events -- dummy
  long op_bind(L4::Icu::Rights, l4_umword_t, L4::Ipc::Snd_fpage irq_cap_fp)
  { return bind_irq(irq_cap_fp); }
  int op_unbind(L4::Icu::Rights, l4_umword_t, L4::Ipc::Snd_fpage)
  { return unbind_irq(); }
  int op_info(L4::Icu::Rights, L4::Icu::_Info &)
  { return -L4_ENOSYS; }
  int op_msi_info(L4::Icu::Rights, l4_umword_t, l4_uint64_t, l4_icu_msi_info_t &)
  { return -L4_ENOSYS; }
  int op_mask(L4::Icu::Rights, l4_umword_t)
  { return -L4_ENOSYS; }
  int op_unmask(L4::Icu::Rights, l4_umword_t)
  { return -L4_ENOREPLY; }
  int op_set_mode(L4::Icu::Rights, l4_umword_t, l4_umword_t)
  {
    printf("\033[35mop_set_mode\033[m\n");
    return -L4_ENOSYS;
  }

  // Event server for Vbus events -- dummy
  long op_get_buffer(L4Re::Event::Rights, L4::Ipc::Cap<L4Re::Dataspace> &)
  { return -L4_ENOSYS; }
  long op_get_num_streams(L4Re::Event::Rights)
  { return -L4_ENOSYS; }
  long op_get_stream_info(L4Re::Event::Rights, int, L4Re::Event_stream_info)
  { return -L4_ENOSYS; }
  long op_get_stream_info_for_id(L4Re::Event::Rights, l4_umword_t,
                                 L4Re::Event_stream_info)
  { return -L4_ENOSYS; }
  long op_get_axis_info(L4Re::Event::Rights, l4_umword_t,
                        L4::Ipc::Array_in_buf<unsigned, unsigned long> const &,
                        L4::Ipc::Array_ref<L4Re::Event_absinfo, unsigned long> &)
  { return -L4_ENOSYS; }
  long op_get_stream_state_for_id(L4Re::Event::Rights, l4_umword_t,
                                  L4Re::Event_stream_state &)
  { return -L4_ENOSYS; }

  // Inhibitor -- dummy
  long op_acquire(L4Re::Inhibitor::Rights, l4_umword_t, L4::Ipc::String<>)
  { return L4_EOK; }
  long op_release(L4Re::Inhibitor::Rights, l4_umword_t)
  { return L4_EOK; }
  long op_next_lock_info(L4Re::Inhibitor::Rights, l4_mword_t &,
                         L4::Ipc::String<char> &)
  { return -L4_ENOSYS; }

  /** IRQ notification from device. */
  void handle_irq();

private:
  long handle_vbus_msg(l4vbus_device_handle_t devid, l4_int32_t func,
                       L4::Ipc::Iostream &ios);
  long handle_mmio_read(L4Re::Mmio_space::Addr addr, l4_uint64_t &value);
  long handle_mmio_write(L4Re::Mmio_space::Addr addr, l4_uint64_t value);
  long bind_irq(L4::Ipc::Snd_fpage irq_cap_fp);
  long unbind_irq();
  long get_mbox_device(L4::Ipc::Iostream &ios) const;
  long get_mbox_device_resource(int res_idx, L4::Ipc::Iostream &ios) const;
  long get_icu_device(L4::Ipc::Iostream &ios) const;

  static constexpr bool hid_eq_hidname(char const *hid, unsigned long sz,
                                       char const *hidname)
  { return sz == __builtin_strlen(hidname) + 1 && !strcmp(hid, hidname); }

  static constexpr l4_uint32_t str2id(char const *s)
  {
    l4_uint32_t id = 0;
    for (unsigned i = 0; i < 4 && s[i]; ++i)
      id |= s[i] << 8*i;

    return id;
  }

  static constexpr char const *devid2str(l4vbus_device_handle_t devid)
  {
    if (devid == Id_icu)
      return "Icu";
    else if (devid == Id_mbox)
      return "Mbox";
    else
      return "???";
  }

  /** Get letter from free list and add it to active list. */
  Bcm2835_mbox::Letter *new_letter();
  /** Get next letter from the active list not owned by the device. */
  Bcm2835_mbox::Letter *get_processed_letter() const;
  /** Remove letter from active list. */
  void drop_letter(Bcm2835_mbox::Letter *l);
  /**
   * Filter message according to client configuration.
   *
   * Filtering is done using the provided message content.
   * This function is able to change the message content, for example change the
   * status.
   *
   * \retval true No client restriction. Message content was not changed.
   * \retval false Client is not allowed to send this message to the service.
   *               The message content might have changed.
   */
  bool filter_message(Bcm2835_mbox::Message<> *msg);

  // mailbox data when performing internal mailbox messages
  L4Re::Util::Unique_cap<L4Re::Dataspace> _mbox_data_ds;
  // virtual address of internal mailbox data
  l4_uint8_t *_mbox_data_dma_virt;
  // physical address of internal mailbox data
  Bcm2835_mbox::Dma_addr _mbox_data_dma_phys;
  // client profile name
  std::string _profile;
  // pointer to physical device
  Bcm2835_mbox *_phys_device = nullptr;
  // list of enabled pins at the expander GPIO
  std::vector<l4_uint8_t> _expgpio_pins;
  // list of clocks
  std::vector<l4_uint8_t> _clocks;
  // client IRQ sink
  L4::Cap<L4::Irq> _irq_client;
  // client view of Mbox0_config because bit 0 stays always enabled at device
  l4_uint32_t _mbox0_config = 0;
  // True if IRQ notification is required. This flag is reset after an IRQ
  // notification and set after the client read all pending active letters
  // which are not "owned by device".
  bool _do_irq_notify = true;

  Bcm2835_mbox::Letter _letters[Num_letters];
  Bcm2835_mbox::Letter *_free_letters_head = nullptr;
  Bcm2835_mbox::Letter *_active_letters_head = nullptr;
  Bcm2835_mbox::Letter *_active_letters_tail = nullptr;

  Dbg warn;
  Dbg info;
  Dbg trace;
};
