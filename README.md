# bcm2835-mbox driver

Driver for the bcm2835 mailbox which can be found on Raspberry Pi 4.

The driver implements the IO server vbus protocol to mimic a special bcm2835
mailbox device containing 3 resources:
 - An MMIO region accessible using the `Mmio_space` protocol implementing the
   MMIO device registers.
   See [here](https://github.com/raspberrypi/firmware/wiki/Mailboxes) for the
   layout.
 - A mappable MMIO region used to share data between the client and the mbox
   service. Doing so it is not necessary that the client shares its address
   space with the service. Physical addresses for this region are actually
   relative offsets to this shared data region.
 - An interrupt resource which is used to notify the client when there is
   something to read. Interrupt notification is required by the Linux driver.


## Interrupt notification and client multiplexing

Linux requires interrupt notification to find out when the firmware has finished
processing a request and some message result is ready to read. The interrupt
handler reads `Mbox0_status` and in case bit 30 (1: nothing to read) is _not_
set, it reads `Mbox0_read` to find out which request is ready for reading. The
value read from the register contains the message channel (bits 0..3) and the
message address (16-byte aligned).

The service implements per-client handling like this:
 - The device interrupt for Mbox0 is always enabled. Any write to `Mbox0_config`
   is ignored.
 - On writing to `Mbox1_write`, a new "letter" context is created and inserted
   into an AVL tree and into a list of "active requests".
 - On interrupt, the corresponding letter context is looked up from the AVL
   tree and the flag "owned-by-device" is cleared. Then, we iterate through all
   clients bound to its respective client IRQ and notify a client if there is at
   least one letter with "owned by device" clear available for that client.
 - `Mbox0_read` and `Mbox0_status` are fully emulated. For both registers we
   check the "active requests" list and check if there is a letter not owned by
   the device. In case of `Mbox0_status`, we return the corresponding value that
   such a letter is available. For `Mbox0_read`, we return the physical address
   of that letter and destroy the letter context (of course also remove from
   active list and from the AVL tree).
 - Interrupt notification is disabled after the client was notified. It gets
   re-enabled when the client recognizes from `Mbox0_read` or `Mbox0_status`
   that no more letters are available.


## Client filtering

The bcm2835 mailbox driver is the gate for performing firmware requests on
Raspberry Pi 4. At the moment, there is only support for channel 8, the property
interface for communication with the VideoCore (VC).

Many of these functions are to retrieve information, but a few functions change
hardware properties:

 - **`Set_power_state`** (0x00028001)
   Enable or disable power (old firmware interface).
   *Linux 6.12 defines:*
   - 3: `RPI_OLD_POWER_DOMAIN_USB`
   - 10: `RPI_OLD_POWER_DOMAIN_V3D`

 - **`Set_domain_state`** (0x00038030)
   Enable or disable power on a specific domain.
   See `RPI_POWER_DOMAIN_xxx` in `dt-bindings/power/raspberrypi-power.h`.
   *There are about 23 domains declared in Linux 6.12.*

 - **`Set_clock_state`** (0x00038001)
   *Not used by Linux 6.12.*

 - **`Set_clock_rate`** (0x00038002)
   Set clock frequency.
   *Linux 6.12 only considers a few clocks and ignores the others:*
   - "arm" (3)
   - "core" (4)
   - "v3d" (5)
   - "pixel" (9)
   - "hevc" (11)
   - "m2mc" (13)
   - "pixel-bvb" (14)
   - "vec" (15)
   Note that it would be possible to change the clocks directly but the firmware
   ultimately takes care of mitigating overheating/undervoltage situations and
   we would be changing frequencies behind its back.

 - **`Set_gpio_state`** (0x00038041)
   Set state of specific expander GPIO pin:
   - pin 128+0: Bluetooth 0=off/1=on
   - pin 128+1: WIFI reset 0=on/1=off
   - pin 128+2: power LED 0=on/1=off
   - pin 128+3: global reset
   - pin 128+4: 0=3.3V/1=1.8V VDD for SD card
   - pin 128+5: Camera power regulator 0=off/1=on
   - pin 128+6: SD card power 0=off/1=on
   *Filter per-pin together with `Set_gpio_config`.*
   Maybe also hide GPIO pins from `Get_gpio_config` and `Get_gpio_state`?
   Hopefully that's not required.

 - `Set_gpio_config` (0x00038043)
   Configure a particular expander GPIO pin:
    - direction
    - termination enabled
    - termination using pull-up
    - polarity
    *Filter per-pin together with `Set_gpio_state`.*

 - **`Set_periph_reg`** (0x00038045)
   *Not used by Linux 6.12.*

 - **`Set_turbo`** (0x00038009)
   Set GPU clock to maximum (1) or minimum (1).
   *Not used by Linux 6.12.*

 - **`Set_voltage`** (0x00038003)
   Set voltage of a particular ID.
   *Not used by Linux 6.12.*
