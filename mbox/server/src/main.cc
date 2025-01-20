/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 */

#include <getopt.h>
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include <l4/re/util/br_manager>
#include <terminate_handler-l4>

#include "debug.h"
#include "bcm2835_mbox.h"
#include "client_factory.h"
#include "util.h"

extern "C" char const _binary_mbox_lua_start[];
extern "C" char const _binary_mbox_lua_end[];

namespace {

static Dbg warn(Dbg::Warn, "main");
static Dbg info(Dbg::Info, "main");
static Dbg trace(Dbg::Trace, "main");

static int
parse_args(int argc, char *const *argv)
{
  int debug_level = 1;

  static struct option const loptions[] =
  {
    { "verbose", no_argument, NULL, 'v' },
    { "quite",   no_argument, NULL, 'q' },
    { 0,         0,           NULL, 0   },
  };

  static char const *usage_str =
  "Usage: %s [-vq]\n"
  "\n"
  "Options:\n"
  " -v      Verbose mode\n"
  " -q      Be quiet\n";

  for (;;)
    {
      int opt = getopt_long(argc, argv, "vq", loptions, NULL);
      if (opt == -1)
        break;

      switch (opt)
        {
        case 'v':
          debug_level <<= 1;
          ++debug_level;
          break;
        case 'q':
          debug_level = 0;
          break;
        default:
          warn.printf(usage_str, argv[0]);
          return -1;
        }
    }

  Dbg::set_level(debug_level);
  return optind;
}

} // namespace

int main(int argc, char **argv)
{
  Dbg::set_level(3);

  int arg_idx = parse_args(argc, argv);
  if (arg_idx < 0)
    return arg_idx;

  lua_State *ls = luaL_newstate();
  if (luaL_loadbuffer(ls, _binary_mbox_lua_start,
                      _binary_mbox_lua_end - _binary_mbox_lua_start,
                      "@mbox.lua"))
    {
      fprintf(stderr, "lua error: %s.\n", lua_tostring(ls, -1));
      lua_pop(ls, lua_gettop(ls));
      return 1;
    }

  if (lua_pcall(ls, 0, 1, 0))
    {
      fprintf(stderr, "lua error: %s.\n", lua_tostring(ls, -1));
      lua_pop(ls, lua_gettop(ls));
      return 2;
    }

  lua_pop(ls, lua_gettop(ls));

  if (arg_idx >= argc)
    {
      fprintf(stderr, "No Lua config script passed!\n");
      return 3;
    }

  if (luaL_loadfile(ls, argv[arg_idx]) || lua_pcall(ls, 0, 0, 0))
    {
      fprintf(stderr, "Cannot parse config file.\n");
      return 4;
    }

  info.printf("Mbox service says hello.\n");

  Util::tsc_init();

  L4Re::Util::Registry_server<L4Re::Util::Br_manager_hooks> registry_server;

  Bcm2835_mbox bcm2835_mbox("dma_buffer", L4_LOG2_PAGESIZE,
                            registry_server.registry());
  Client_factory factory(registry_server.registry(), ls, &bcm2835_mbox);
  bcm2835_mbox.set_handle_irq_at_clients([&factory]{ factory.handle_irq(); });

  registry_server.loop();

  return 0;
}
