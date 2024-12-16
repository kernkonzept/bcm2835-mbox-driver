/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 */

#include <string>
#include <lua.h>

#include <l4/re/error_helper>
#include <l4/sys/cxx/ipc_epiface>

#include "client_factory.h"
#include "client.h"

static bool
parse_string_param(L4::Ipc::Varg const &param, std::string const prefix,
                   std::string *out)
{
  l4_size_t prefixlen = prefix.length();
  if (param.length() < prefixlen)
    return false;

  std::string const pstr(param.value<char const *>());
  if (pstr.compare(0, prefixlen, prefix) != 0)
    return false;

  *out = pstr.substr(prefixlen);
  return true;
}

Client_factory::Client_factory(L4Re::Util::Object_registry *registry,
                               lua_State *ls, Bcm2835_mbox *phys_device)
: _registry(registry), _ls(ls), _phys_device(phys_device),
  warn(Dbg::Warn, "factory"),
  info(Dbg::Info, "factory"),
  trace(Dbg::Info, "factory")
{
  L4Re::chkcap(registry->register_obj(this, "svr"),
               "Register factory server");
}

long
Client_factory::op_create(L4::Factory::Rights, L4::Ipc::Cap<void> &res,
                          l4_umword_t type, L4::Ipc::Varg_list_ref valist)
{
  if (type != 0)
    {
      warn.printf("factory create: Invalid object type.\n");
      return -L4_EINVAL;
    }

  std::string profile;
  for (L4::Ipc::Varg const p: valist)
    {
      if (!p.is_of<char const *>())
        {
          warn.printf("factory create: String parameter expected.\n");
          return -L4_EINVAL;
        }

      if (parse_string_param(p, "profile=", &profile))
        continue;
    }

  if (profile.empty())
    {
      warn.printf("factory create: Required parameter 'profile=' not found.\n");
      return -L4_EINVAL;
    }

  std::vector<l4_uint8_t> expgpio_pins;
  std::vector<l4_uint8_t> clocks;
  lua_getglobal(_ls, profile.c_str());
  if (lua_isnil(_ls, -1) || !lua_istable(_ls, -1))
    {
      warn.printf("op_create: profile '%s' not found\n", profile.c_str());
      lua_pop(_ls, 1);
      return -L4_EINVAL;
    }

  load_profile_table(_ls, "expgpio_pins", expgpio_pins);
  for (auto i: expgpio_pins)
    trace.printf("op_create(%s): expgpio pin %u\n", profile.c_str(), i);

  load_profile_table(_ls, "clocks", clocks);
  for (auto i: clocks)
    trace.printf("op_create(%s): clock %u\n", profile.c_str(), i);

  try
    {
      auto client = cxx::make_ref_obj<Client>(
                           _registry, _phys_device, profile,
                           expgpio_pins, clocks);
      _clients.push_back(client);
      res = L4::Ipc::make_cap(client->obj_cap(), L4_CAP_FPAGE_RWSD);
    }
  catch (L4::Runtime_error const &e)
    {
      warn.printf("%s: %s\n", e.extra_str(), e.str());
      return e.err_no();
    }

  return L4_EOK;
}


template <typename T>
bool
Client_factory::load_profile_table(lua_State *ls, char const *name,
                                        std::vector<T> &a)
{
  lua_pushstring(ls, name);
  lua_gettable(ls, -2);
  if (lua_isnil(ls, -1) || !lua_istable(ls, -1))
    {
      lua_pop(ls, 1);
      return false;
    }

  lua_pushnil(ls);
  while (lua_next(ls, -2) != 0)
    {
      if (lua_isnumber(ls, -1))
        {
          lua_tointeger(ls, -2);
          int v = lua_tointeger(ls, -1);
          a.push_back(v);
        }
      lua_pop(ls, 1);
    }
  lua_pop(ls, 1);
  return true;
}
