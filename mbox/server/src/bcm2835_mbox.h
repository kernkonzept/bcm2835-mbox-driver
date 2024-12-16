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

#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include <l4/cxx/avl_map>
#include <l4/cxx/minmax>
#include <l4/cxx/bitfield>
#include <l4/cxx/unique_ptr>
#include <l4/cxx/utils>
#include <l4/drivers/hw_mmio_register_block>
#include <l4/mbox/mbox_base.h>
#include <l4/re/error_helper>
#include <l4/re/dataspace>
#include <l4/re/dma_space>
#include <l4/re/env>
#include <l4/re/rm>
#include <l4/re/util/shared_cap>
#include <l4/re/util/unique_cap>
#include <l4/re/util/object_registry>
#include <l4/vbus/vbus>
#include <l4/sys/cxx/ipc_epiface>

#include "debug.h"

class Client;

// See https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
class Bcm2835_mbox : public Bcm2835_mbox_base<Bcm2835_mbox>,
                     public L4::Irqep_t<Bcm2835_mbox>
{
  using Handle_irq = std::function<void()>;

  // RAM shared with the device DMA space. On destruction, unmap this region
  // from the DMA space (and also from the virtual address space).
  struct Dma_region
  {
    using Dma_addr = L4Re::Dma_space::Dma_addr;
    Dma_region(l4_uint8_t log2_size, L4::Cap<L4Re::Dataspace> ds,
               L4::Cap<L4Re::Dma_space> dma, Dma_addr dma_limit)
    : size(1UL << log2_size), dma(dma)
    {
      l4_size_t size = 1UL << log2_size;
      auto const *e = L4Re::Env::env();
      auto rm_flags = L4Re::Rm::F::Search_addr | L4Re::Rm::F::RW
                      | L4Re::Rm::F::Eager_map | L4Re::Rm::F::Cache_normal;
      L4Re::chksys(e->rm()->attach(&region, size, rm_flags,
                                   L4::Ipc::make_cap_rw(ds), 0, log2_size),
                   "Attach DMA memory");
      l4_size_t out_size = size;
      Dma_addr dma_phys;
      L4Re::chksys(dma->map(L4::Ipc::make_cap_rw(ds), 0,
                            &out_size, L4Re::Dma_space::Attributes::None,
                            L4Re::Dma_space::Direction::Bidirectional,
                            &dma_phys),
                   "Lock memory region for DMA");
      phys = dma_phys;
      if (out_size < size)
        L4Re::throw_error(-L4_ENOMEM, "Mapping whole region into DMA space");
      // This check works only for the client DMA buffers but not for our
      // _local_dma_phys because _dma_limit is set after allocating this buffer
      if (dma_phys >= dma_limit)
        printf("\033[31;1mDMA buffer phys at %08llx beyond limit %08llx\033[m\n",
               dma_phys, dma_limit);
      if (dma_phys >= 0x1'0000'0000ULL)
        printf("\033[31;1mDMA buffer phys at %08llx beyond 4GB\033[m\n",
               dma_phys);
    }

    ~Dma_region()
    {
      if (phys)
        L4Re::chksys(dma->unmap(phys, size, L4Re::Dma_space::Attributes::None,
                                L4Re::Dma_space::Direction::Bidirectional),
                     "Unmap region from DMA.");
    }

    Dma_region(const Dma_region &) = delete;
    Dma_region operator=(Dma_region &) = delete;
    Dma_region(Dma_region &&other)
    : phys(other.phys), size(other.size), dma(other.dma)
    {
      region = std::move(other.region);
      other.region.reset();
      other.phys = 0;
    }
    Dma_region &operator=(Dma_region&& other)
    {
      phys = other.phys;
      other.phys = 0;
      size = other.size;
      dma = other.dma;
      region = std::move(other.region);
      other.region.reset();
      return *this;
    }

    Dma_addr phys = 0;
    l4_uint32_t size = 0;
    L4::Cap<L4Re::Dma_space> dma;
    L4Re::Rm::Unique_region<l4_uint8_t *> region;
  };

public:
  /** Structure for storing messages active at the device. */
  class Letter : public cxx::Avl_tree_node
  {
  public:
    /** Device can only handle 32-bit DMA addresses. */
    using Mbox_dma_addr = l4_uint32_t;

    /** if in active list: false: owned by driver, true: owned by device */
    bool owned_by_device;
    /** letter channel; encoded in the lower 4 bits of the letter address */
    l4_uint8_t channel;
    /** virtual address of the letter; only valid if in active list */
    l4_uint8_t *msg;
    /** physical address of the letter; only valid if in active list */
    Mbox_dma_addr phys;
    /** submitting client */
    Client *client;
    /** pointer to previous letter in active list; nullptr if first element */
    Letter *prev;
    /**
     * pointer to next letter in free list or in active list; nullptr if last
     * element in list
     */
    Letter *next;
  };

  explicit Bcm2835_mbox(char const *dma_buffer, l4_uint8_t log2_dma_buf_size,
                        L4Re::Util::Object_registry *registry);
  ~Bcm2835_mbox();

  /** Client notification: How to notify clients when an IRQ was triggered. */
  void set_handle_irq_at_clients(Handle_irq handle_irq_at_clients)
  { _handle_irq_at_clients = handle_irq_at_clients; }

  /** Client interface: DMA-attach a DMA buffer. */
  void attach_and_dma_map(L4::Cap<L4Re::Dataspace> ds, l4_uint8_t log2_size,
                          l4_uint8_t **dma_virt, Dma_addr *dma_phys);
  /**
   * Client interface: Read MMIO register.
   * \param[in]  reg_offs  Register offset.
   * \param[out] value     Read value returned to client.
   *
   * The most important Mbox0_read and Mbox0_status registers are emulated at
   * the client side.
   */

  long mmio_read(unsigned reg_offs, l4_uint64_t *value);
  /**
   * Client interface: Write MMIO register.
   * \param[in] reg_offs   Register offset.
   * \param[in] value      Value to be written to the register.
   * \param[in] msg_virt   Virtual address of shared message buffer.
   * \param[in] msg_phys   Physical address of shared message buffer (logging).
   * \param[in] data_offs  Offset of data within shared message buffer.
   *                       Only relevant for reg_offs=Mbox1_write.
   * \param[in] letter     Letter (message context).
   *                       Only set for reg_offs=Mbox1_write, otherwise nullptr.
   */
  long mmio_write(unsigned reg_offs, l4_uint64_t value, l4_uint8_t *msg_virt,
                  Dma_addr msg_phys, unsigned data_offs, Letter *letter);

  /** IRQ endpoint handler. */
  void handle_irq();

  l4_uint64_t get_dma_offset() const
  { return _dma_offset; }

  /** Send + receive mail for internal usage. */
  template <typename Tag::Type tag_type = Tag::Type::No_data>
  void do_request(Message<tag_type> *msg);

private:
  void attach_dma_buffer(char const *dma_buffer, l4_size_t size);
  L4Re::Util::Shared_cap<L4Re::Dma_space>
    create_dma_space(L4::Cap<L4vbus::Vbus> bus, long unsigned id);

  void unmask_irq();

  /** Tracing. */
  void show_info();

  /** Put letter into AVL tree sorted by DMA address. */
  void insert_letter_into_active(Letter *letter, l4_uint8_t *msg, Dma_addr phys,
                                 l4_uint32_t channel);
  /** Get letter from AVL tree according to DMA address. */
  Letter *remove_letter_from_active(Dma_addr phys);
  /** AVL tree to associate physical address of Mbox letter with Letter. */
  struct Letter_get_key
  {
    typedef Dma_addr Key_type;
    static Key_type const &key_of(Letter const *l)
    { return l->phys; }
  };
  struct Letter_key_compare
  {
    bool operator () (Dma_addr const &l, Dma_addr const &r) const
    { return l < r; }
  };
  cxx::Avl_tree<Letter, Letter_get_key, Letter_key_compare> _letter_tree;

  // device registers
  L4Re::Rm::Unique_region<l4_addr_t> _regs_region;

  // Local DMA region for mbox letters.
  L4Re::Util::Shared_cap<L4Re::Dma_space> _dma;
  L4Re::Util::Unique_cap<L4Re::Dataspace> _local_dma_ds;
  l4_uint8_t *_local_dma_virt;
  Dma_addr _local_dma_phys;
  // All DMA-attached DMA regions.
  std::vector<Dma_region> _mapped_dma_spaces;

  // The ICU the hardware interrupt is triggered on.
  L4::Cap<L4::Icu> _icu;
  // The IRQ object for the hardware interrupt.
  L4::Cap<L4::Irq> _irq;
  // The IRQ number of the IRQ at the ICU.
  int _irq_num;
  // True, if the IRQ has to be unmasked at the ICU.
  bool _irq_unmask_at_icu;
  // Function to call when the IRQ was triggered and at least one client wants
  // to receive IRQs.
  Handle_irq _handle_irq_at_clients;

  l4_uint64_t _dma_offset = 0ULL;
  l4_uint64_t _dma_limit = ~0ULL;

  bool busy = false;

  Dbg warn;
  Dbg info;
  Dbg trace;
  Dbg trace2;
};
