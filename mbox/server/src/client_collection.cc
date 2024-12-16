/*
 * Copyright (C) 2024 Kernkonzept GmbH.
 * Author(s): Frank Mehnert <frank.mehnert@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

/**
 * \file
 */

#include <l4/cxx/ref_ptr>

#include "client_collection.h"
#include "client.h"

void
Client_collection::handle_irq()
{
  for (auto const &c : _clients)
    c->handle_irq();
}
