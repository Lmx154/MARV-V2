// USB descriptors for a single CDC ACM interface (enumerates as /dev/ttyACM* on Linux).
#include "pico/unique_id.h"
#include "tusb.h"

enum {
    kStrLang = 0,
    kStrManufacturer,
    kStrProduct,
    kStrSerial,
    kStrCdc,
    kStrCount,
};

#define ITF_CDC 0
#define ITF_COUNT 2
#define EP_CDC_NOTIF 0x81
#define EP_CDC_OUT 0x02
#define EP_CDC_IN 0x82
#define CONFIG_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x2E8A,   // Raspberry Pi
    .idProduct = 0x000A,  // Pico SDK CDC
    .bcdDevice = 0x0100,
    .iManufacturer = kStrManufacturer,
    .iProduct = kStrProduct,
    .iSerialNumber = kStrSerial,
    .bNumConfigurations = 1,
};

static const uint8_t desc_config[CONFIG_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC, kStrCdc, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

static const char* const desc_str[kStrCount] = {
    [kStrLang] = "",
    [kStrManufacturer] = "MARV",
    [kStrProduct] = "MARV flight controller",
    [kStrSerial] = serial_str,
    [kStrCdc] = "MARV link",
};

const uint8_t* tud_descriptor_device_cb(void) { return (const uint8_t*)&desc_device; }

const uint8_t* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_config;
}

const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t out[32];
    uint8_t len;
    if (index == kStrLang) {
        out[1] = 0x0409;  // English
        len = 1;
    } else {
        if (index >= kStrCount) return NULL;
        if (index == kStrSerial && !serial_str[0]) pico_get_unique_board_id_string(serial_str, sizeof(serial_str));
        const char* s = desc_str[index];
        for (len = 0; len < 31 && s[len]; ++len) out[1 + len] = (uint16_t)s[len];
    }
    out[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return out;
}
