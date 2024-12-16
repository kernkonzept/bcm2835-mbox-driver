/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */
/**
 * \file
 * Mailbox client access.
 *
 * The client either performs direct MMIO access on the device or it uses the
 * mbox service as proxy.
 */

#pragma once

#include <cstring>
#include <mutex>
#include <string>

#include <l4/cxx/list_alloc>
#include <l4/cxx/ref_ptr>
#include <l4/mbox/mbox_base.h>
#include <l4/re/env>
#include <l4/re/error_helper>
#include <l4/re/mmio_space>
#include <l4/re/rm>
#include <l4/re/util/debug>
#include <l4/re/util/shared_cap>
#include <l4/re/util/unique_cap>
#include <l4/vbus/vbus>

class Bcm2835_mbox : public Bcm2835_mbox_base<Bcm2835_mbox>
{
private:
  using Dbg = L4Re::Util::Dbg;

  /** An MMIO block with 32 bit registers and little endian byte order. */
  class Mmio_space_register_block_base
  {
  public:
    explicit Mmio_space_register_block_base(L4::Cap<L4Re::Mmio_space> mmio_space,
                                            l4_uint64_t phys, l4_uint64_t)
    : _mmio_space(mmio_space), _phys(phys) {}

    template< typename T >
    T read(l4_addr_t reg) const
    {
      l4_uint64_t value;
      L4Re::chksys(_mmio_space->mmio_read(_phys + reg, log2_size(T{0}), &value),
                   "Read mbox Mmio_space register");
      return value;
    }

    template< typename T >
    void write(T value, l4_addr_t reg) const
    {
      L4Re::chksys(_mmio_space->mmio_write(_phys + reg, log2_size(T{0}), value),
                   "Write mbox MMio_space register");
    }

  private:
    static constexpr char log2_size(l4_uint32_t) { return 2; } // 32-bit

    L4::Cap<L4Re::Mmio_space> _mmio_space;
    l4_uint64_t _phys;
  };

  struct Mmio_space_register_block_32
  : L4drivers::Register_block_impl<Mmio_space_register_block_32, 32>,
    Mmio_space_register_block_base
  {
    explicit Mmio_space_register_block_32(L4::Cap<L4Re::Mmio_space> cap,
                                          l4_uint64_t base, l4_uint64_t size)
    : Mmio_space_register_block_base(cap, base, size) {}
  };

  /** Buffer usable for DMA transfers. */
  class Inout_buffer : public cxx::Ref_obj
  {
  public:
    Inout_buffer(char const *cap_name, l4_uint8_t log2_size,
                 L4Re::Util::Shared_cap<L4Re::Dma_space> const &dma,
                 L4Re::Dma_space::Direction dir,
                 L4Re::Rm::Flags flags = L4Re::Rm::Flags(0))
    : _size(1UL << log2_size), _dma(dma), _paddr(0), _dir(dir)
    {
      auto *e = L4Re::Env::env();
      l4_size_t size = 1UL << log2_size;

      if (cap_name)
        {
          // use provided capability instead of calling the allocator
          auto ds = e->get_cap<L4Re::Dataspace>(cap_name);
          if (ds.is_valid() && ds->size() == size)
            {
              attach_and_dma_map(log2_size, dir, ds, flags);
              return;
            }

          printf("Capability '%s' not found -- allocating buffer.\n", cap_name);
        }

      _ds = L4Re::chkcap(L4Re::Util::make_unique_cap<L4Re::Dataspace>(),
                         "Allocate dataspace capability for IO memory.");

      auto alloc_flags = L4Re::Mem_alloc::Continuous | L4Re::Mem_alloc::Pinned;
      L4Re::chksys(e->mem_alloc()->alloc(size, _ds.get(), alloc_flags),
                   "Allocate pinned memory.");

      attach_and_dma_map(log2_size, dir, _ds.get(), flags);
    }

    Inout_buffer(Inout_buffer const &) = delete;
    Inout_buffer(Inout_buffer &&) = delete;
    Inout_buffer &operator=(Inout_buffer &&rhs) = delete;

    ~Inout_buffer()
    {
      if (_paddr)
        {
          auto attr = L4Re::Dma_space::Attributes::None;
          L4Re::chksys(_dma->unmap(_paddr, _size, attr, _dir),
                       "Unmap region from DMA.");
          _paddr = 0;
        }
    }

    void attach_and_dma_map(l4_uint8_t log2_size, L4Re::Dma_space::Direction dir,
                            L4::Cap<L4Re::Dataspace> ds, L4Re::Rm::Flags flags)
    {
      auto *e = L4Re::Env::env();
      auto rm_flags = L4Re::Rm::F::Search_addr | L4Re::Rm::F::RW | flags;
      L4Re::chksys(e->rm()->attach(&_region, 1UL << log2_size, rm_flags,
                                   L4::Ipc::make_cap_rw(ds), 0, log2_size),
                   "Attach IO memory");

      l4_size_t out_size = 1UL << log2_size;
      L4Re::chksys(_dma->map(L4::Ipc::make_cap_rw(ds), 0, &out_size,
                             L4Re::Dma_space::Attributes::None, dir, &_paddr),
                   "Lock memory region for DMA");
      if (out_size < (1UL << log2_size))
        L4Re::throw_error(-L4_ENOMEM, "Mapping whole region into DMA space");

      if (_paddr >= 0x1'0000'0000ULL)
        printf("\033[31;1mInout buffer phys at %08llx\033[m\n", _paddr);
    }

    template <class T>
    T *get(unsigned offset = 0) const
    { return reinterpret_cast<T *>(_region.get() + offset); }

    l4_uint64_t pget(unsigned offset = 0) const
    { return _paddr + offset; }

    l4_size_t size() const
    { return _size; }

  private:
    l4_size_t _size;
    L4Re::Util::Unique_cap<L4Re::Dataspace> _ds;
    L4Re::Util::Shared_cap<L4Re::Dma_space> _dma;
    L4Re::Rm::Unique_region<char *> _region;
    L4Re::Dma_space::Dma_addr _paddr;
    L4Re::Dma_space::Direction _dir;
  };

public:
  /** Attach to remote or local mbox device. */
  Bcm2835_mbox(L4::Cap<L4vbus::Vbus> vbus, L4Re::Util::Dbg &log,
               L4Re::Util::Shared_cap<L4Re::Dma_space> const &dma)
  : _log(log)
  {
    connect_vbus_device(vbus);

    if (!_remote_service)
      {
        // Local: Create local buffer for mbox messages and attach to DMA.
        _local_data = new Inout_buffer("mbox_memory", L4_LOG2_PAGESIZE, dma,
                                       L4Re::Dma_space::Direction::Bidirectional,
                                       L4Re::Rm::F::Cache_normal);
        if (   _local_data->pget() > 0x3fffffff
            || _local_data->pget() + _local_data->size() > 0x40000000)
          L4Re::throw_error_fmt(-L4_EINVAL,
                                "mbox DMA memory at %08llx-%08llx beyond 1GB",
                                _local_data->pget(),
                                _local_data->pget() + _local_data->size());
        if (_local_data->pget() & 0xf)
          L4Re::throw_error(-L4_ENOMEM, "mbox DMA memory not aligned");

        _data_size = _local_data->size();
        _data_virt = _local_data->get<l4_uint8_t>();
        _data_phys = _local_data->pget(); // no DMA offset!
      }
    // otherwise the buffer was already allocated in connect_vbus_device().

    _data_alloc.free(_data_virt, _data_size, true);
  }

  /** Attach to remote mbox device. */
  Bcm2835_mbox(L4::Cap<L4vbus::Vbus> vbus, L4Re::Util::Dbg &log)
  : _log(log)
  {
    connect_vbus_device(vbus);

    if (!_remote_service)
      L4Re::throw_error(-L4_EINVAL,
                        "No DMA space provided"); // use other constructor!

    _data_alloc.free(_data_virt, _data_size, true);
  }

  virtual ~Bcm2835_mbox()
  {
    if (_local_data)
      {
        delete _local_data;
        _local_data = nullptr;
      }
  }

  /**
   * Send message to mailbox channel `Chan::Property_vc` and fetch response
   * performing busy waiting.
   *
   * \param[in,out] msg  The message to send. The first word contains the size.
   */
  template <Tag::Type tag_type = Tag::Type::No_data>
  void do_request(Message<tag_type> *msg)
  {
    l4_uint32_t bytes = msg->hdr.bytes;
    if (bytes < msg->Msg_nodata_bytes)
      L4Re::throw_error(-L4_EINVAL, "mbox message too short");

    if constexpr (false)
      msg->log(_log, "Send", 0);

    l4_uint32_t size_out = msg->tag.size_out;

    size_t shdata_offs = alloc_shared_data(bytes);
    msg->to_mbox_before_dma(_data_virt + shdata_offs);
    send_mail(_data_phys + shdata_offs, Chan::Property_vc, _log);
    read_mail(Chan::Property_vc, _log);
    msg->from_mbox_after_dma(_data_virt + shdata_offs, size_out);
    free_shared_data(shdata_offs, bytes);

    if constexpr (false)
      msg->log(_log, "Recv", 0);

    if (msg->hdr.status != Message<tag_type>::Hdr::Status::Success)
      {
        msg->log(_log, "Recv", 0);
        L4Re::throw_error(-L4_EINVAL, "Mailbox receive failed");
      }
  }

private:
  void connect_vbus_device(L4::Cap<L4vbus::Vbus> vbus)
  {
    l4vbus_device_t devinfo;
    L4Re::chksys(vbus->root().device_by_hid(&_mbox, "BCM2835_mbox",
                                            L4VBUS_MAX_DEPTH, &devinfo),
                 "Locate BCM2835_mbox device on vbus");
    _mbox_num_resources = devinfo.num_resources;
    auto mmio_space = L4::cap_dynamic_cast<L4Re::Mmio_space>(_mbox.bus_cap());
    if (mmio_space)
      _remote_service = true;

    unsigned num_mem = 0;
    for (unsigned i = 0; i < _mbox_num_resources; ++i)
      {
        l4vbus_resource_t res;
        L4Re::chksys(_mbox.get_resource(i, &res), "Get mbox device info.");
        if (res.type == L4VBUS_RESOURCE_MEM)
          {
            l4_uint64_t addr = res.start;
            l4_uint64_t size = res.end - res.start + 1;
            if (num_mem == 0)
              {
                if (_remote_service)
                  _regs = new Mmio_space_register_block_32(
                                mmio_space, addr, size);
                else
                  _regs = new Mmio_map_register_block_32(
                                _mbox.bus_cap(), addr, size);
              }
            else if (num_mem == 1)
              {
                // size must be log2
                l4_size_t log2_size = L4_LOG2_PAGESIZE;
                while ((1UL << log2_size) < size)
                  ++log2_size;
                if (size != (1UL << log2_size))
                  L4Re::throw_error(
                    -L4_EINVAL, "Mbox shared region not log2-sized!");

                // "Special" region implemented by the Mbox service.
                auto rm_flags = L4Re::Rm::F::Search_addr
                                | L4Re::Rm::F::Cache_normal
                                | L4Re::Rm::F::RW
                                | L4Re::Rm::F::Eager_map;
                auto *e = L4Re::Env::env();
                L4Re::chksys(e->rm()->attach(&_srv_data_region, size, rm_flags,
                                             _mbox.bus_cap(), addr, log2_size),
                             "Attach mbox service device data.");
                _data_size = size;
              }
            ++num_mem;
          }
      }

    if (num_mem == 0)
      L4Re::throw_error(-L4_ENOENT, "Invalid resources for mbox device.");
    if (mmio_space && num_mem != 2)
      L4Re::throw_error(-L4_ENOENT, "MMIO space access requires 2 memory regions");

    if (_remote_service)
      {
        _data_virt = _srv_data_region.get();
        _data_phys = 0; // offset
      }
  }

protected:
  /**
   * Allocate 16-byte aligned buffer on _srv_data_region.
   *
   * \param size  Size for entire message in bytes.
   * \returns Offset into _data_virt region.
   *
   * We allocate a dedicated buffer per request to allow multiple concurrent
   * requests from different threads / guest vCPUs.
   */
  size_t alloc_shared_data(size_t size)
  {
    std::lock_guard<std::mutex> lock(_data_alloc_mtx);

    if (size > _data_size)
      L4Re::throw_error_fmt(
        -L4_ENOMEM, "Allocating %zu bytes for message exceeds region", size);

    auto *block = static_cast<l4_uint8_t *>(_data_alloc.alloc(size, 16));
    if (!block)
      {
        _data_alloc.dump_free_list(_log);
        L4Re::throw_error_fmt(
          -L4_ENOMEM, "Cannot allocate %zu bytes for message", size);
      }

    // paranoia
    if (block < _data_virt || block > _data_virt + _data_size - size)
      L4Re::throw_error_fmt(
        -L4_ENOMEM, "Allocated block outsize region (%p/%zu / %p-%p)\n",
        block, size, _data_virt, _data_virt + _data_size);

    return block - _data_virt;
  }

  /**
   * Free buffer on _drv_data_region which was allocated by alloc_shared_data().
   */
  void free_shared_data(size_t offs, size_t size)
  {
    // paranoia
    if (offs > _data_size || size > _data_size || offs > _data_size - size)
      L4Re::throw_error_fmt(
        -L4_ENOMEM, "Cannot free block outside region (%p/%zu / %p-%p)\n",
        _data_virt + offs, size, _data_virt, _data_virt + _data_size);

    std::lock_guard<std::mutex> lock(_data_alloc_mtx);
    _data_alloc.free(_data_virt + offs, size);
  }

private:
  // Logging
  Dbg _log;
  // remote: Shared memory with mbox service: address
  L4Re::Rm::Unique_region<l4_uint8_t *> _srv_data_region;

protected:
  // vbus device handle
  L4vbus::Device _mbox;
  unsigned _mbox_num_resources;
  // local: Mbox data for messages from this application.
  Inout_buffer *_local_data = nullptr;
  // local/remote: Virtual address of Mbox data.
  l4_uint8_t *_data_virt = nullptr;
  // local/remote: Physical address of Mbox data (local) or offset (remote)
  l4_addr_t _data_phys = 0;
  // local/remote: Size of Mbox data region.
  l4_addr_t _data_size = 0;
  // allocator on _srv_data_region
  // XXX Using the list allocator we have to trust the mbox service because the
  //     free list is shared with the service!
  cxx::List_alloc _data_alloc;
  // protect List_alloc
  std::mutex _data_alloc_mtx;
};
