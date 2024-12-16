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

#include <l4/cxx/avl_map>
#include <l4/cxx/std_ops>

#include "client.h"

class Client_collection
{
public:
  void handle_irq();

  std::vector<cxx::Ref_ptr<Client>> _clients;
};
