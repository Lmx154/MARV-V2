#ifndef MARV_USB_CONFIG_TUSB_CONFIG_H
#define MARV_USB_CONFIG_TUSB_CONFIG_H

// The TinyUSB configuration of the composite flight image (ground-segment unit 12a, decision 0053): the Pico SDK's own
// tusb_config.h (pico_stdio_usb, found next in the include path) with two CDC instances in place of one. Instance 0 is the
// console (the stdio driver, unchanged), instance 1 the link byte transport (marv/hal_target/usb.hpp). Everything else,
// the FIFO and endpoint buffer sizes included, is the SDK's. Not used by the HIL images (fw/hal/target/CMakeLists.txt).
#include_next <tusb_config.h>

#undef CFG_TUD_CDC
#define CFG_TUD_CDC (2)

#endif
