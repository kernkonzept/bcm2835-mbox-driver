-- vi:ft=lua

-- Expander GPIO. Pins start at 128, see Linux RPI_EXP_GPIO_BASE!
Expgpio_pins =
{
  Bluetooth = 128,      -- Bluetooth on
  Wifi = 129,           -- WIFI on
  Power_led_off = 130,  -- Power LED off
  Global_reset = 131,   -- Global reset
  Sd_io_1v8 = 132,      -- SD card 1.8V (0:3.3V, 1:1.8V)
  Cam1 = 133,           -- Camera
  Sd_vcc = 134,         -- SD card power on
};

-- Firmware clocks.
Clocks =
{
  Emmc = 1,
  Uart = 2,
  Arm = 3,
  Core = 4,
  V3d = 5,
  H264 = 6,
  Isp = 7,
  Sdram = 8,
  Pixel = 9,
  Pwm = 10,
  Hevc = 11,
  Emmc2 = 12,
  M2mc = 13,
  Pixel_bvb = 14,
  Vec = 15,
};
