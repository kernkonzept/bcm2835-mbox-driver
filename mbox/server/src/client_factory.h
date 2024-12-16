/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 */

#include <vector>
#include <string>

#include <l4/cxx/ref_ptr>
#include <l4/sys/factory>
#include <l4/re/util/object_registry>

#include "debug.h"
#include "client_collection.h"

class Bcm2835_mbox;
class Client;
struct lua_State;

class Client_factory
: public L4::Epiface_t<Client_factory, L4::Factory>,
  public Client_collection
{
public:
  Client_factory(L4Re::Util::Object_registry *registry, lua_State *ls,
                 Bcm2835_mbox *phys_device);

  long op_create(L4::Factory::Rights, L4::Ipc::Cap<void> &res,
                 l4_umword_t type, L4::Ipc::Varg_list_ref valist);

private:
  template <typename T>
  bool load_profile_table(lua_State *ls, char const *name, std::vector<T> &a);

  L4Re::Util::Object_registry *_registry;
  lua_State *_ls;
  Bcm2835_mbox *_phys_device;

  Dbg warn;
  Dbg info;
  Dbg trace;
};
