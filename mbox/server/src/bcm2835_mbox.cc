/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 * Driver for the physical bcm2835 mbox device.
 */

#include <iomanip>
#include <map>

#include <l4/re/env>
#include <l4/re/error_helper>
#include <l4/re/rm>
#include <l4/re/util/unique_cap>

#include "bcm2835_mbox.h"
#include "consts.h"
#include "util.h"

static L4Re::Env const *e = L4Re::Env::env();

Bcm2835_mbox::Bcm2835_mbox(char const *cap_name, l4_uint8_t log2_dma_buf_size,
                           L4Re::Util::Object_registry *registry)
: warn(Dbg::Warn, "dev"),
  info(Dbg::Info, "dev"),
  trace(Dbg::Trace, "dev"),
  trace2(Dbg::Trace2, "dev")
{
  auto vbus = L4Re::chkcap(e->get_cap<L4vbus::Vbus>("vbus"),
                           "Get 'vbus' capability", -L4_ENOENT);
  auto root = vbus->root();
  L4vbus::Device dev;
  l4vbus_device_t di;
  l4_addr_t mmio_addr = 0;
  l4_size_t mmio_size = 0;

  auto _icu = L4Re::chkcap(L4Re::Util::cap_alloc.alloc<L4::Icu>(),
                          "Allocate ICU capability");
  L4vbus::Icu icudev;
  L4Re::chksys(vbus->root().device_by_hid(&icudev, "L40009"),
               "Look for ICU device");
  L4Re::chksys(icudev.vicu(_icu), "Request ICU capability");

  unsigned long dma_domain = -1UL;
  unsigned long irq_num = 0;
  L4_irq_mode irq_mode = L4_IRQ_F_LEVEL_HIGH;

  while (root.next_device(&dev, L4VBUS_MAX_DEPTH, &di) == L4_EOK)
    {
      trace.printf("Scanning child 0x%lx (%s).\n", dev.dev_handle(), di.name);
      if (dev.is_compatible("brcm,bcm2835-mbox") < 1)
        continue;

      for (unsigned i = 0; i < di.num_resources; ++i)
        {
          l4vbus_resource_t res;
          L4Re::chksys(dev.get_resource(i, &res));
          if (res.type == L4VBUS_RESOURCE_MEM)
            {
              mmio_addr = res.start;
              mmio_size = res.end - res.start + 1;
            }
          else if (res.type == L4VBUS_RESOURCE_DMA_DOMAIN)
            dma_domain = res.start;
          else if (res.type == L4VBUS_RESOURCE_IRQ)
            {
              // Create IRQ server object and register at registry.
              irq_num = res.start;
              irq_mode = L4_irq_mode(res.flags);
            }
          else if (res.type == L4VBUS_RESOURCE_IRQ)
            {
              // Create IRQ server object and register at registry.
              irq_num = res.start;
              irq_mode = L4_irq_mode(res.flags);
            }
        }

      if (!mmio_addr)
        L4Re::throw_error(-L4_EINVAL, "Device has MMIO resource.\n");

      if (dma_domain == -1UL)
        info.printf("Using Vbus global DMA domain.\n");
      else
        trace.printf("Using device's DMA domain %lu.\n", dma_domain);

      break;
    }

  if (!mmio_addr)
    L4Re::throw_error(-L4_EINVAL, "No device found.\n");

  auto offset = mmio_addr & ~L4_PAGEMASK;
  auto rm_flags = L4Re::Rm::F::Search_addr | L4Re::Rm::F::Cache_uncached
                  | L4Re::Rm::F::RW | L4Re::Rm::F::Eager_map;
  L4Re::chksys(e->rm()->attach(&_regs_region, mmio_size, rm_flags,
                               L4::Ipc::make_cap_rw(vbus), mmio_addr,
                               L4_PAGESHIFT),
               "Attach vbus device memory");
  _regs = new L4drivers::Mmio_register_block<32>(_regs_region.get() + offset);

  _dma = create_dma_space(vbus, dma_domain);
  attach_dma_buffer(cap_name, log2_dma_buf_size);

  _irq = L4Re::chkcap(registry->register_irq_obj(this),
                      "Register IRQ server object");
  _irq_num = irq_num;
  L4Re::chksys(_icu->set_mode(irq_num, irq_mode), "Set IRQ mode");
  int ret = L4Re::chksys(_icu->bind(irq_num, _irq), "Bind interrupt to ICU");
  _irq_unmask_at_icu = ret == 1;

  info.printf("Device @ %08lx-%08lx IRQ %lx (%s-triggered).\n",
              mmio_addr, mmio_addr + mmio_size - 1, irq_num,
              irq_mode == L4_IRQ_F_LEVEL_HIGH ? "level-high" : "edge");

  // enable interrupt generation when data ready to read
  _regs[Mbox0_config] = 1;
  unmask_irq();

  show_info();
}

Bcm2835_mbox::~Bcm2835_mbox()
{}

void
Bcm2835_mbox::attach_dma_buffer(char const *cap_name, l4_size_t log2_size)
{
  l4_size_t size = 1UL << log2_size;
  if (cap_name)
    {
      auto ds = e->get_cap<L4Re::Dataspace>(cap_name);
      if (ds.is_valid() && ds->size() == size)
        return attach_and_dma_map(ds, log2_size, &_local_dma_virt, &_local_dma_phys);

      if (ds.is_valid())
        warn.printf(
          "Capability '%s' has wrong size (expecting %s) -- allocating buffer.\n",
          cap_name, Util::readable_size(size).c_str());
      else
        info.printf(
          "Capability '%s' not found -- allocating buffer.\n", cap_name);
    }

  _local_dma_ds = L4Re::chkcap(L4Re::Util::make_unique_cap<L4Re::Dataspace>(),
                               "Allocate dataspace capability for DMA memory.");
  auto alloc_flags = L4Re::Mem_alloc::Continuous | L4Re::Mem_alloc::Pinned;
  L4Re::chksys(e->mem_alloc()->alloc(size, _local_dma_ds.get(), alloc_flags),
               "Allocate pinned DMA memory.");

  attach_and_dma_map(_local_dma_ds.get(), log2_size, &_local_dma_virt, &_local_dma_phys);
}

void
Bcm2835_mbox::attach_and_dma_map(L4::Cap<L4Re::Dataspace> ds, l4_uint8_t log2_size,
                                 l4_uint8_t **dma_virt, Dma_addr *dma_phys)
{
  _mapped_dma_spaces.emplace_back(log2_size, ds, _dma.get(), _dma_limit);
  *dma_virt = _mapped_dma_spaces.back().region.get();
  *dma_phys = _mapped_dma_spaces.back().phys;
}

L4Re::Util::Shared_cap<L4Re::Dma_space>
Bcm2835_mbox::create_dma_space(L4::Cap<L4vbus::Vbus> bus, long unsigned id)
{
  static std::map<long unsigned, L4Re::Util::Shared_cap<L4Re::Dma_space>> spaces;
  auto ires = spaces.find(id);
  if (ires != spaces.end())
    return ires->second;

  auto dma = L4Re::chkcap(L4Re::Util::make_shared_cap<L4Re::Dma_space>(),
                          "Allocate capability for DMA space.");
  L4Re::chksys(e->user_factory()->create(dma.get()), "Create DMA space.");
  auto const assign_flags = L4VBUS_DMAD_BIND | L4VBUS_DMAD_L4RE_DMA_SPACE;
  L4Re::chksys(bus->assign_dma_domain(id, assign_flags, dma.get()),
               "Assignment of DMA domain.");

  spaces[id] = dma;
  return dma;
}

void
Bcm2835_mbox::show_info()
{
  using Type = Bcm2835_mbox::Tag::Type;

    {
      Soc_rev board_rev = get_board_rev();
      _dma_offset = 0xc0000000UL; // default: assume old revision
      _dma_limit = 0x3fffffffULL;
      if (board_rev.new_style())
        {
          // Try to detect the C0 stepping
          switch (board_rev.type())
            {
            case 0x11: // 4B
              if (board_rev.revision() <= 2)
                break;
              _dma_offset = 0UL; // new revision
              _dma_limit = 0xffffffffULL; // XXX is this correct?
              break;
            case 0x13: // 400
              _dma_offset = 0UL; // new revision
              _dma_limit = 0xffffffffULL; // XXX is this correct?
              break;
            }
        }

      // from now on, apply the DMA offset
      _local_dma_phys += _dma_offset;

      l4_uint64_t memsize;
      switch (board_rev.memory_size())
        {
        case 0: memsize = 256 << 20; break;
        case 1: memsize = 512 << 20; break;
        case 2: memsize = 1ULL << 30; break;
        case 3: memsize = 2ULL << 30; break;
        case 4: memsize = 4ULL << 30; break;
        case 5: memsize = 8ULL << 30; break;
        default: memsize = 0; break;
        }

      printf("RAM: %s, Revision: %08x => \033[31;1mDMA offset = %08llx\033[m.\n",
             memsize ? Util::readable_size(memsize).c_str() : "unknown",
             board_rev.raw, _dma_offset);
    }

  if constexpr (0)
    {
      std::string mac = get_board_mac();
      info.printf("Got MAC %s.\n", mac.c_str());
    }

  if constexpr (0)
    {
      // example for message with unknown size
      l4_uint32_t words;
        {
          Bcm2835_mbox::Message<Type::Get_cmdline> msg;
          do_request(&msg);
          words = msg.required_words();
        }
        {
          auto *mem = new l4_uint32_t[words];
          auto *msg = new (mem) Bcm2835_mbox::Message<Type::Get_cmdline>(words);
          do_request(msg);
          printf("Got %s\n", reinterpret_cast<char *>(msg->data));
          delete[] mem;
        }
    }
}

template <typename Bcm2835_mbox::Tag::Type tag_type>
void
Bcm2835_mbox::do_request(Message<tag_type> *msg)
{
  // Paranoia. We are single-threaded.
  if (busy)
    L4Re::throw_error(-L4_EBUSY, "bcm2835 mbox busy");
  busy = true;

  l4_uint32_t bytes = msg->hdr.bytes;
  if (bytes < msg->Msg_nodata_bytes)
    L4Re::throw_error(-L4_EINVAL, "bcm2835 mbox message too short");

  if (trace2.is_active())
    msg->log(trace2, "Send", _local_dma_phys);

  l4_uint32_t size_out = msg->tag.size_out;

  msg->to_mbox_before_dma(_local_dma_virt);
  send_mail(_local_dma_phys, Chan::Property_vc, trace);
  unmask_irq();
  read_mail(Chan::Property_vc, trace);
  msg->from_mbox_after_dma(_local_dma_virt, size_out);

  busy = false;

  if constexpr (false) // debugging
    msg->log(warn, "Recv", _local_dma_phys);

  if (msg->hdr.status != Message<tag_type>::Hdr::Status::Success)
    {
      msg->log(warn, "Recv", _local_dma_phys);
      L4Re::throw_error(-L4_EINVAL, "Mailbox receive failed");
    }
}

long
Bcm2835_mbox::mmio_read(unsigned reg_offs, l4_uint64_t *value)
{
  switch (reg_offs)
    {
    case Mbox1_status:
      *value = _regs[reg_offs];
      break;

    default:
      warn.printf("mmio_read: ignoring read from offset %02x\n", reg_offs);
      *value = 0;
      return -L4_EINVAL;
    }

  return L4_EOK;
}

long
Bcm2835_mbox::mmio_write(unsigned reg_offs, l4_uint64_t value,
                         l4_uint8_t *msg_virt, Dma_addr msg_phys,
                         unsigned data_offs, Letter *letter)
{
  switch (reg_offs)
    {
    case Mbox1_write:
      {
        auto *msg = reinterpret_cast<Message<> *>(msg_virt + data_offs);
        l4_uint32_t bytes = msg->hdr.bytes;
        if (bytes < msg->Msg_nodata_bytes)
          L4Re::throw_error_fmt(-L4_EINVAL,
                                "bcm2835 mbox message too short (%u/%u bytes)",
                                bytes, msg->Msg_nodata_bytes);
        l4_uint32_t channel = value & Channel_mask;
        if (Chan{channel} != Chan::Property_vc)
          L4Re::throw_error_fmt(-L4_EINVAL,
                                "bcm2835 mbox unexpected channel (%u/%u)",
                                channel, static_cast<unsigned>(Chan::Property_vc));
        msg_virt += data_offs;
        msg_phys += data_offs + _dma_offset;
        if (trace2.is_active())
          msg->log(trace2, "Send", msg_phys);
        else if (trace.is_active())
          {
            char str[32] = "";
            msg->snprintf_msg_words(str, sizeof(str), msg->tag.size_out);
            info.printf("send_mail: %s <= %s size=%u%s\n",
                        msg->tag.to_str().c_str(), msg->hdr.status_str().c_str(),
                        msg->tag.size_out, str);
          }

        // insert letter into AVL tree so we find it during mmio_read()
        insert_letter_into_active(letter, msg_virt, msg_phys, channel);

        l4_cache_flush_data(reinterpret_cast<l4_addr_t>(msg),
                            reinterpret_cast<l4_addr_t>(msg) + bytes);
        send_mail(msg_phys, Chan{channel}, trace);
        unmask_irq();
        break;
      }

    default:
      warn.printf("mmio_write: ignoring write %08llx to offset %02x\n",
                  value, reg_offs);
      return -L4_EINVAL;
    }

  return L4_EOK;
}

/**
 * Device triggered IRQ notifying us that there are more finished letters which
 * can be fetched by reading Mbox0_read.
 *
 * We mark the letter as "no longer owned by device" and for all clients check
 * if a client has got a new letter.
 */
void
Bcm2835_mbox::handle_irq()
{
  while (!(_regs[Mbox0_status] & Mbox_status_read_empty))
    {
      l4_uint32_t letter = _regs[Mbox0_read];
      trace.printf("handle_irq: read letter phys=%08x\n", letter);

      l4_uint32_t value_phys = letter & ~Channel_mask;
      l4_uint32_t value_chan = letter & Channel_mask;
      Letter *l = remove_letter_from_active(value_phys);
      if (!l)
        L4Re::throw_error_fmt(
          -L4_EINVAL, "Couldn't find letter for address %08x", value_phys);
      if (value_chan != l->channel)
        L4Re::throw_error_fmt(-L4_EINVAL, "Channel mismatch (%u/%u)",
                              value_chan, l->channel);
      l4_uint32_t const phys_max = l->phys + (1U << Log2_client_buffer_size);
      // to determine the size, only consider the first 4 bytes of the message
      if (value_phys < l->phys || value_phys > l->phys + 4)
        L4Re::throw_error_fmt(
          -L4_EINVAL,
          "Device returned physical address outside of buffer (%08x/%08x-%08x)",
          value_phys, l->phys, phys_max);
      auto *msg = reinterpret_cast<Message<> *>(l->msg);
      l4_uint32_t bytes = msg->hdr.bytes;
      // now check with known message size
      if (value_phys + bytes > phys_max)
        L4Re::throw_error_fmt(
          -L4_EINVAL,
          "Device returned physical address outside of buffer (%08x-%08x/%08x-%08x)",
          value_phys, value_phys + bytes, l->phys, phys_max);
      l4_cache_inv_data(reinterpret_cast<l4_addr_t>(msg),
                        reinterpret_cast<l4_addr_t>(msg) + bytes);
      if (trace2.is_active())
        msg->log(trace2, "Recv", l->phys);
      else if (trace.is_active())
        {
          char str[32] = "";
          if (msg->hdr.status_ok())
            msg->snprintf_msg_words(str, sizeof(str), msg->tag.get_size_in());
          info.printf("handle_irq: %s => %s size=%u%s\n",
                      msg->tag.to_str().c_str(), msg->hdr.status_str().c_str(),
                      msg->tag.get_size_in(), str);
        }

      l->owned_by_device = false;
    }

  if (_handle_irq_at_clients)
    _handle_irq_at_clients();
}

void
Bcm2835_mbox::unmask_irq()
{
  if (_irq_unmask_at_icu)
    _icu->unmask(_irq_num);
  else
    obj_cap()->unmask();
}

/** For diagnostics. */
template <class IMPL>
std::string
Bcm2835_mbox_base<IMPL>::Tag::to_str() const
{
#define CASE_LABEL_TO_STR(l) case Type::l: return #l;
  switch (tag)
    {
    CASE_LABEL_TO_STR(Get_fw_version);
    CASE_LABEL_TO_STR(Get_board_model);
    CASE_LABEL_TO_STR(Get_board_rev);
    CASE_LABEL_TO_STR(Get_board_mac);
    CASE_LABEL_TO_STR(Get_board_serial);
    CASE_LABEL_TO_STR(Get_arm_memory);
    CASE_LABEL_TO_STR(Get_vc_memory);
    CASE_LABEL_TO_STR(Get_cmdline);
    CASE_LABEL_TO_STR(Get_dma_channels);
    CASE_LABEL_TO_STR(Get_clocks);
    CASE_LABEL_TO_STR(Get_power_state);
    CASE_LABEL_TO_STR(Set_power_state);
    CASE_LABEL_TO_STR(Get_timing);
    CASE_LABEL_TO_STR(Get_clock_state);
    CASE_LABEL_TO_STR(Set_clock_state);
    CASE_LABEL_TO_STR(Get_clock_rate);
    CASE_LABEL_TO_STR(Set_clock_rate);
    CASE_LABEL_TO_STR(Get_clock_rate_max);
    CASE_LABEL_TO_STR(Get_clock_rate_min);
    CASE_LABEL_TO_STR(Get_clock_rate_actual);
    CASE_LABEL_TO_STR(Get_throttled);
    CASE_LABEL_TO_STR(Get_domain_state);
    CASE_LABEL_TO_STR(Set_domain_state);
    CASE_LABEL_TO_STR(Get_gpio_state);
    CASE_LABEL_TO_STR(Set_gpio_state);
    CASE_LABEL_TO_STR(Get_gpio_config);
    CASE_LABEL_TO_STR(Set_gpio_config);
    CASE_LABEL_TO_STR(Get_periph_reg);
    CASE_LABEL_TO_STR(Set_periph_reg);
    CASE_LABEL_TO_STR(Notify_reboot);
    CASE_LABEL_TO_STR(Get_turbo);
    CASE_LABEL_TO_STR(Set_turbo);
    CASE_LABEL_TO_STR(Get_voltage);
    CASE_LABEL_TO_STR(Set_voltage);
    CASE_LABEL_TO_STR(Get_voltage_max);
    CASE_LABEL_TO_STR(Get_voltage_min);
    CASE_LABEL_TO_STR(Get_temp);
    CASE_LABEL_TO_STR(Get_temp_max);
    CASE_LABEL_TO_STR(Allocate_memory);
    CASE_LABEL_TO_STR(Lock_memory);
    CASE_LABEL_TO_STR(Unlock_memory);
    CASE_LABEL_TO_STR(Release_memory);
    CASE_LABEL_TO_STR(Execute_code);
    CASE_LABEL_TO_STR(Get_dispmanx_res_mem_hdl);
    CASE_LABEL_TO_STR(Get_edid_block);
    default:
      {
        std::stringstream ss;
        ss << std::hex << static_cast<l4_uint32_t>(tag);
        return ss.str();
      }
    }
#undef CASE_LABEL_TO_STR
}

void
Bcm2835_mbox::insert_letter_into_active(Letter *l, l4_uint8_t *msg,
                                        Dma_addr phys, l4_uint32_t channel)
{
  l->owned_by_device = true;
  l->msg = msg;
  l->phys = phys;
  l->channel = channel;
  trace.printf("set_letter_for_addr phys=%08x letter=%p\n", l->phys, l);
  _letter_tree.insert(l);
}

Bcm2835_mbox::Letter *
Bcm2835_mbox::remove_letter_from_active(Dma_addr phys)
{
  Letter *l = _letter_tree.remove(phys);
  trace.printf("get_letter_for_addr phys=%08x => %p\n", phys, l);
  return l;
}
