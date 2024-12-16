/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 * Client interface.
 */

#include <algorithm>

#include <l4/vbus/vdevice-ops.h>

#include "client.h"

Client::Client(L4Re::Util::Object_registry *registry, Bcm2835_mbox *phys_device,
               std::string const &profile,
               std::vector<l4_uint8_t> const &expgpio_pins,
               std::vector<l4_uint8_t> const &clocks)
: _profile(profile),
  _phys_device(phys_device),
  _expgpio_pins(expgpio_pins),
  _clocks(clocks),
  warn(Dbg::Warn, "client", profile),
  info(Dbg::Info, "client", profile),
  trace(Dbg::Trace, "client", profile)
{
  auto env = L4Re::Env::env();

  _mbox_data_ds = L4Re::chkcap(L4Re::Util::make_unique_cap<L4Re::Dataspace>(),
                               "Allocate dataspace capability for DMA data");
  auto alloc_flags = L4Re::Mem_alloc::Continuous | L4Re::Mem_alloc::Pinned;
  L4Re::chksys(env->mem_alloc()->alloc(size_mbox_data, _mbox_data_ds.get(),
                                       alloc_flags),
               "Allocate mailbox data dataspace");
  phys_device->attach_and_dma_map(_mbox_data_ds.get(), log2_size_mbox_data,
                                  &_mbox_data_dma_virt, &_mbox_data_dma_phys);

  for (unsigned i = 0; i < Num_letters - 1; ++i)
    _letters[i].next = _letters + i + 1;
  _letters[Num_letters - 1].next = nullptr;
  _free_letters_head = _letters;

  L4Re::chkcap(registry->register_obj(this), "Register mailbox client");

  info.printf("Created client with profile '%s'.\n", profile.c_str());
}

long
Client::handle_vbus_msg(l4vbus_device_handle_t devid, l4_int32_t func,
                             L4::Ipc::Iostream &ios)
{
  trace.printf("handle_vbox_msg func = %08x\n", func);
  switch (func)
    {
    case L4vbus_vicu_get_cap:
      if (devid == Id_icu)
        ios << obj_cap();
      else
        return -L4_ENODEV;
      break;

    case L4vbus_vdevice_get_by_hid:
      {
        l4vbus_device_handle_t child;
        if (L4_UNLIKELY(!ios.get(child)))
          return -L4_EMSGTOOSHORT;

        int depth;
        if (L4_UNLIKELY(!ios.get(depth)))
          return -L4_EMSGTOOSHORT;

        unsigned long sz;
        char const *hid = 0;
        ios >> L4::Ipc::buf_in(hid, sz);

        if (!hid)
          return -L4_EINVAL;

        trace.printf("handle_vbox_msg: L4vbus_vdevice_get_by_hid: hid='%.*s'\n",
                     static_cast<int>(sz), hid);
        if (hid_eq_hidname(hid, sz, hidname_mbox))
          return get_mbox_device(ios);
        else if (hid_eq_hidname(hid, sz, hidname_icu))
          return get_icu_device(ios);
        else
          return -L4_EINVAL;
      }

    case L4vbus_vdevice_get_resource:
      {
        int res_idx;
        if (!ios.get(res_idx))
          return -L4_EMSGTOOSHORT;
        trace.printf("handle_vbox_msg: L4vbus_vdevice_get_resource: devid=%s res=%u\n",
                     devid2str(devid), res_idx);
        if (devid == Id_mbox)
          return get_mbox_device_resource(res_idx, ios);
        else
          return -L4_ENOENT;
      }

    default:
      warn.printf("Unknown vbus function: devid=0x%lx func=0x%x\n", devid, func);
      return -L4_ENOSYS;
    }

  return L4_EOK;
}

long
Client::handle_mmio_read(L4Re::Mmio_space::Addr addr, l4_uint64_t &value)
{
  if (addr < res_mbox_start || addr >= res_mbox_end - 4)
    L4Re::throw_error_fmt(-L4_EINVAL,
                          "Client read offset %llx/%08llx-%08llx\n",
                          addr, res_mbox_start, res_mbox_end);
  unsigned reg = addr - res_mbox_start;
  long ret = L4_EOK;

  switch (reg)
    {
    case Bcm2835_mbox::Mbox0_read:
      if (Bcm2835_mbox::Letter *l = get_processed_letter())
        {
          auto *msg = reinterpret_cast<Bcm2835_mbox::Message<> const *>(l->msg);
          if (!trace.is_active())
            {
              // Do this logging here to have a client relation.
              char str[32] = "";
              msg->snprintf_msg_words(str, sizeof(str), msg->tag.get_size_in());
              info.printf("get: %s %u bytes%s %s\n",
                          msg->tag.to_str().c_str(), msg->tag.get_size_in(),
                          str, msg->hdr.status_str().c_str());
            }
          l4_uint64_t dma_offs = _phys_device->get_dma_offset();
          // We return the shared data offset, not the physical address!
          value = (l->phys - dma_offs - _mbox_data_dma_phys) | l->channel;
          drop_letter(l);
        }
      else
        {
          value = 0;
          _do_irq_notify = true;
        }
      break;

    case Bcm2835_mbox::Mbox0_status:
      if (get_processed_letter())
        value = 0;
      else
        {
          value = Bcm2835_mbox::Mbox_status_read_empty;
          _do_irq_notify = true;
        }
      break;

    case Bcm2835_mbox::Mbox0_config:
      value = _mbox0_config;
      break;

    default:
      ret = _phys_device->mmio_read(reg, &value);
      break;
    }

  trace.printf("Mmio_space read offs=%02x => %08llx\n", reg, value);
  return ret;
}

long
Client::handle_mmio_write(L4Re::Mmio_space::Addr addr, l4_uint64_t value)
{
  if (addr < res_mbox_start || addr >= res_mbox_end - 4)
    L4Re::throw_error_fmt(-L4_EINVAL,
                          "Client write offset %llx/%08llx-%08llx\n",
                          addr, res_mbox_start, res_mbox_end);
  unsigned reg = addr - res_mbox_start;
  // We expect the client to send the shared data offset, not physical address!
  Bcm2835_mbox::Dma_addr const data_offs = value & ~Bcm2835_mbox::Channel_mask;
  trace.printf("Mmio_space write offs=%02x value=%08llx data_offs=%08x\n",
               reg, value, data_offs);
  Bcm2835_mbox::Letter *l = nullptr;
  if (reg == Bcm2835_mbox::Mbox1_write)
    {
      if (value >= static_cast<Bcm2835_mbox::Dma_addr>(~0))
        L4Re::throw_error_fmt(
          -L4_EINVAL,
          "Client letter address with huge DMA address (%llx)", value);
      if (data_offs >= size_mbox_data - 4)
        L4Re::throw_error_fmt(
          -L4_EINVAL,
          "Client letter address exceeding DMA region (%llx/%08zx)",
          value, size_mbox_data);
      // must not be 'const' in case we filter-out this message
      auto *msg =
        reinterpret_cast<Bcm2835_mbox::Message<> *>(_mbox_data_dma_virt + data_offs);
      l4_uint32_t msg_size = msg->hdr.bytes;
      if (msg_size > size_mbox_data || data_offs > size_mbox_data - msg_size)
        L4Re::throw_error_fmt(
          -L4_EINVAL,
          "Client letter size exceeding DMA region (%llx/%08zx)",
          value + msg_size, size_mbox_data);
      if (!trace.is_active())
        {
          // Do this logging here to have a client relation.
          char str[32] = "";
          msg->snprintf_msg_words(str, sizeof(str), msg->tag.size_out);
          info.printf("put: %s %u bytes%s\n",
                      msg->tag.to_str().c_str(), msg->tag.size_out, str);
        }
      l = new_letter();
      if (!filter_message(msg))
        {
          l->owned_by_device = false;
          handle_irq();
          return L4_EOK;
        }
    }
  else if (reg == Bcm2835_mbox::Mbox0_config)
    {
      _mbox0_config = value;
      info.printf("%sable interrupt notification\n", (value & 1) ? "En" : "Dis");
      // Never change the device register because the device IRQ remains enabled.
      return L4_EOK;
    }

  return _phys_device->mmio_write(reg, value, _mbox_data_dma_virt,
                                  _mbox_data_dma_phys, data_offs, l);
}

long
Client::get_mbox_device(L4::Ipc::Iostream &ios) const
{
  l4vbus_device_t mbox;
  snprintf(mbox.name, sizeof(mbox.name), "%s", hidname_mbox);
  mbox.flags = 0;
  mbox.num_resources = 3;
  mbox.type = 0;
  ios << l4vbus_device_handle_t(Id_mbox);
  ios.put(mbox);
  return L4_EOK;
}

long
Client::get_mbox_device_resource(int res_idx, L4::Ipc::Iostream &ios) const
{
  l4vbus_resource_t res;
  switch (res_idx)
    {
    case 0:
      res.start = res_mbox_start;
      res.end = res_mbox_end;
      res.type = L4VBUS_RESOURCE_MEM;
      res.flags = L4VBUS_RESOURCE_F_MEM_MMIO_READ
                | L4VBUS_RESOURCE_F_MEM_MMIO_WRITE;
      res.provider = 0;
      res.id = str2id("reg0");
      break;
    case 1:
      res.start = res_mbox_data_start;
      res.end = res_mbox_data_end;
      res.type = L4VBUS_RESOURCE_MEM;
      res.flags = L4VBUS_RESOURCE_F_MEM_R | L4VBUS_RESOURCE_F_MEM_W;
      res.provider = 0;
      res.id = str2id("reg1");
      break;
    case 2:
      res.start = res_mbox_irqnum;
      res.end = res_mbox_irqnum;
      res.type = L4VBUS_RESOURCE_IRQ;
      res.flags = L4_IRQ_F_EDGE | L4_IRQ_F_POS;
      res.provider = 0;
      res.id = str2id("irq0");
      break;
    default:
      return -L4_ENOENT;
    }
  ios.put(res);
  return L4_EOK;
}

long
Client::get_icu_device(L4::Ipc::Iostream &ios) const
{
  l4vbus_device_t icu;
  snprintf(icu.name, sizeof(icu.name), "%s", hidname_icu);
  icu.flags = 0;
  icu.num_resources = 0;
  icu.type = 0;
  ios << l4vbus_device_handle_t(Id_icu);
  ios.put(icu);
  return L4_EOK;
}

long
Client::bind_irq(L4::Ipc::Snd_fpage irq_cap_fp)
{
  if (!irq_cap_fp.cap_received())
    return -L4_EINVAL;
  L4::Cap<L4::Irq> irqc = server_iface()->rcv_cap<L4::Irq>(0);
  if (!irqc.is_valid())
    return -L4_EINVAL;
  l4_msgtag_t msg = L4Re::Env::env()->task()->cap_valid(irqc);
  if (msg.label() == 0)
    return -L4_EINVAL;
  if (_irq_client.is_valid())
    return -L4_EBUSY;
  _irq_client = irqc;
  server_iface()->realloc_rcv_cap(0);
  return L4_EOK;
}

long
Client::unbind_irq()
{
  _irq_client = L4::Cap<L4::Irq>::Invalid;
  return L4_EOK;
}

void
Client::handle_irq()
{
  if (_irq_client.is_valid() // Client bound to IRQ
      && (_mbox0_config & 1) // Client told device to generate IRQ
      && _do_irq_notify)     // IRQ emulation, see member description
    if (Bcm2835_mbox::Letter *l = get_processed_letter())
      {
        trace.printf("handle_irq: found letter %08x\n", l->phys);
        if (long ret = l4_ipc_error(_irq_client->trigger(), l4_utcb()))
          {
            trace.printf("handle_irq: IPC error %s during trigger -- detaching!\n",
                         l4sys_errtostr(ret));
            printf("NOTIFY IRQ\n");
            _irq_client = L4::Cap<L4::Irq>::Invalid;
          }
        _do_irq_notify = false;
      }
}

Bcm2835_mbox::Letter *
Client::new_letter()
{
  if (!_free_letters_head)
    L4Re::throw_error(-L4_EINVAL, "No free letter context available");

  Bcm2835_mbox::Letter *l = _free_letters_head;

  // remove from free list
  _free_letters_head = l->next;

  // append to active list
  l->next = nullptr;
  if (!_active_letters_head)
    _active_letters_head = l;
  l->prev = _active_letters_tail;
  if (_active_letters_tail)
    _active_letters_tail->next = l;
  _active_letters_tail = l;

  l->client = this;

  return l;
}

void
Client::drop_letter(Bcm2835_mbox::Letter *l)
{
  // remove from active list
  (l->next ? l->next->prev : _active_letters_tail) = l->prev;
  (l->prev ? l->prev->next : _active_letters_head) = l->next;

  // insert at the beginning of free list
  l->next = _free_letters_head;
  _free_letters_head = l;
}

Bcm2835_mbox::Letter *
Client::get_processed_letter() const
{
  Bcm2835_mbox::Letter *l = _active_letters_head;
  while (l && l->owned_by_device)
    l = l->next;
  return l;
}

bool
Client::filter_message(Bcm2835_mbox::Message<> *msg)
{
  using Type = Bcm2835_mbox::Tag::Type;

  // Set_gpio_{state,config}: Client allowed to set pin state/configuration.
  if (   msg->valid_msg<Type::Set_gpio_state>()
      || msg->valid_msg<Type::Set_gpio_config>())
    {
      l4_uint8_t pin = msg->data[0];
      if (std::find(_expgpio_pins.begin(), _expgpio_pins.end(), pin)
          == _expgpio_pins.end())
        {
          // Report error to client.
          // XXX Maybe just ignore a change?
          warn.printf(
            "\033[31mClient not allowed to set%s expander GPIO pin %u!\033[m\n",
            msg->valid_msg<Type::Set_gpio_config>() ? " config of" : "", pin);
          msg->hdr.status = Bcm2835_mbox::Message<>::Hdr::Status::Error;
          return false;
        }
    }

  // Set_clock_{rate,state}: Client allowed to set clock rate/state.
  if (   msg->valid_msg<Type::Set_clock_state>()
      || msg->valid_msg<Type::Set_clock_rate>())
    {
      l4_uint8_t clock = msg->data[0];
      if (std::find(_clocks.begin(), _clocks.end(), clock) == _clocks.end())
        {
          // Report error to client.
          // XXX Maybe just ignore a clock change? Or test if the clock really
          //     changes and report success if it doesn't?
          warn.printf(
            "\033[31mClient not allowed to change clock %s of clock %u!\033[m\n",
            msg->valid_msg<Type::Set_clock_state>() ? "state" : "rate", clock);
          msg->hdr.status = Bcm2835_mbox::Message<>::Hdr::Status::Error;
          return false;
        }
    }

  return true;
}
