/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif

LOG_MODULE_REGISTER(dongle_esb, CONFIG_ESB_PRX_APP_LOG_LEVEL);

// ============================================================================
// HID Report Descriptors (ZMK Compatible)
// ============================================================================

#define HID_REPORT_ID_KEYBOARD    0x01
#define HID_REPORT_ID_CONSUMER    0x02
#define HID_REPORT_ID_MOUSE       0x03

// ZMK-compatible HID descriptor
static const uint8_t hid_report_desc[] = {
    // Keyboard Report
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xa1, 0x01,        // Collection (Application)
    0x85, HID_REPORT_ID_KEYBOARD,  // Report ID (1)
    
    // Modifier keys
    0x05, 0x07,        // Usage Page (Key Codes)
    0x19, 0xe0,        // Usage Minimum (224)
    0x29, 0xe7,        // Usage Maximum (231)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x08,        // Report Count (8)
    0x81, 0x02,        // Input (Data,Var,Abs)
    
    // Reserved byte
    0x95, 0x01,        // Report Count (1)
    0x75, 0x08,        // Report Size (8)
    0x81, 0x01,        // Input (Cnst,Var,Abs)
    
    // Key array (6 keys)
    0x95, 0x06,        // Report Count (6)
    0x75, 0x08,        // Report Size (8)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x65,        // Logical Maximum (101)
    0x05, 0x07,        // Usage Page (Key Codes)
    0x19, 0x00,        // Usage Minimum (0)
    0x29, 0x65,        // Usage Maximum (101)
    0x81, 0x00,        // Input (Data,Ary,Abs)
    0xc0,              // End Collection

    // Consumer Report  
    0x05, 0x0c,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xa1, 0x01,        // Collection (Application)
    0x85, HID_REPORT_ID_CONSUMER,  // Report ID (2)
    0x05, 0x0c,        // Usage Page (Consumer)
    0x15, 0x00,        // Logical Minimum (0)
    0x26, 0xff, 0x0f,  // Logical Maximum (4095)
    0x19, 0x00,        // Usage Minimum (0)
    0x2a, 0xff, 0x0f,  // Usage Maximum (4095)
    0x75, 0x10,        // Report Size (16)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x00,        // Input (Data,Ary,Abs)
    0xc0,              // End Collection

    // Mouse Report
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xa1, 0x01,        // Collection (Application)
    0x85, HID_REPORT_ID_MOUSE,     // Report ID (3)
    0x09, 0x01,        // Usage (Pointer)
    0xa1, 0x00,        // Collection (Physical)
    
    // Mouse buttons
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        // Usage Minimum (1)
    0x29, 0x05,        // Usage Maximum (5)
    0x15, 0x00,        // Logical Minimum (0)
    0x25, 0x01,        // Logical Maximum (1)
    0x75, 0x01,        // Report Size (1)
    0x95, 0x05,        // Report Count (5)
    0x81, 0x02,        // Input (Data,Var,Abs)
    
    // Padding
    0x95, 0x03,        // Report Count (3)
    0x81, 0x01,        // Input (Cnst,Var,Abs)
    
    // Mouse X,Y
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x30,        // Usage (X)
    0x09, 0x31,        // Usage (Y)
    0x16, 0x01, 0x80,  // Logical Minimum (-32767)
    0x26, 0xff, 0x7f,  // Logical Maximum (32767)
    0x75, 0x10,        // Report Size (16)
    0x95, 0x02,        // Report Count (2)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    // Mouse scroll wheel
    0x09, 0x38,        // Usage (Wheel)
    0x15, 0x81,        // Logical Minimum (-127)
    0x25, 0x7f,        // Logical Maximum (127)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x01,        // Report Count (1)
    0x81, 0x06,        // Input (Data,Var,Rel)
    
    0xc0,              // End Collection
    0xc0,              // End Collection
};

// ============================================================================
// ESB ↔ USB HID Protocol Definition
// ============================================================================

/**
 * ESB Packet Format from BLESB:
 * 
 * [header: 2 bytes][HID report data: variable length]
 * 
 * Total ESB payload size = header + HID data (max 32 bytes)
 * This format matches exactly what your BLESB ESB firmware sends
 */

// Packet header structure (exactly matches BLESB implementation)
struct hid_packet_header {
    uint8_t type;      // HID report type: 1=keyboard, 2=consumer, 3=mouse
    uint8_t length;    // Length of HID report data following this header
} __packed;

// HID packet types (must match ZMK HID report IDs)
#define HID_PACKET_TYPE_KEYBOARD  1   // Standard keyboard report (8 bytes)
#define HID_PACKET_TYPE_CONSUMER  2   // Media/consumer report (2 bytes)  
#define HID_PACKET_TYPE_MOUSE     3   // Mouse report (6 bytes)

// ZMK HID Report Structures (what gets forwarded to USB)
struct zmk_keyboard_report {
    uint8_t modifiers;    // Modifier keys (Ctrl, Alt, Shift, etc.)
    uint8_t reserved;     // Reserved byte (always 0)
    uint8_t keys[6];      // Up to 6 simultaneous key presses (6KRO)
} __packed;

struct zmk_consumer_report {
    uint16_t usage;       // Consumer usage code (volume, play/pause, etc.)
} __packed;

struct zmk_mouse_report {
    uint8_t buttons;      // Mouse button state (5 buttons + 3 padding bits)
    int16_t x;            // X axis movement (relative)
    int16_t y;            // Y axis movement (relative)
    int8_t wheel;         // Scroll wheel movement
} __packed;

// Expected report sizes for validation
#define KEYBOARD_REPORT_SIZE  sizeof(struct zmk_keyboard_report)  // 8 bytes
#define CONSUMER_REPORT_SIZE  sizeof(struct zmk_consumer_report)  // 2 bytes
#define MOUSE_REPORT_SIZE     sizeof(struct zmk_mouse_report)     // 6 bytes

// ESB constraints
#define ESB_MAX_PAYLOAD_SIZE        32
#define MAX_HID_REPORT_SIZE         (ESB_MAX_PAYLOAD_SIZE - sizeof(struct hid_packet_header))

// ============================================================================
// USB HID Device Management
// ============================================================================

static const struct device *hid_dev;
static bool hid_configured = false;
static K_SEM_DEFINE(hid_sem, 1, 1);

static void hid_int_ep_ready(const struct device *dev) {
    k_sem_give(&hid_sem);
}

static int hid_get_report(const struct device *dev, struct usb_setup_packet *setup,
                         int32_t *len, uint8_t **data) {
    LOG_DBG("Get report: type=%d, id=%d", 
            (setup->wValue >> 8) & 0xFF, setup->wValue & 0xFF);
    return -ENOTSUP;
}

static int hid_set_report(const struct device *dev, struct usb_setup_packet *setup,
                         int32_t *len, uint8_t **data) {
    LOG_DBG("Set report: type=%d, id=%d", 
            (setup->wValue >> 8) & 0xFF, setup->wValue & 0xFF);
    return -ENOTSUP;
}

static void hid_status_cb(enum usb_dc_status_code status, const uint8_t *param) {
    switch (status) {
    case USB_DC_CONFIGURED:
        hid_configured = true;
        LOG_INF("USB HID configured");
        break;
    case USB_DC_DISCONNECTED:
        hid_configured = false;
        LOG_INF("USB HID disconnected");
        break;
    default:
        break;
    }
}

static const struct hid_ops hid_callbacks = {
    .int_in_ready = hid_int_ep_ready,
    .get_report = hid_get_report,
    .set_report = hid_set_report,
};

// ============================================================================
// USB HID Report Forwarding Functions
// ============================================================================

/**
 * @brief Send HID report over USB
 * @param report_id USB HID report ID (1=keyboard, 2=consumer, 3=mouse)
 * @param data Raw HID report data (without report ID)
 * @param len Length of HID report data
 * @return 0 on success, negative error code on failure
 */
static int send_hid_report(uint8_t report_id, const uint8_t *data, size_t len) {
    if (!hid_configured) {
        LOG_WRN("USB HID not configured, dropping report ID %d", report_id);
        return -ENODEV;
    }

    // Take semaphore with timeout to prevent blocking
    if (k_sem_take(&hid_sem, K_MSEC(10)) != 0) {
        LOG_WRN("HID semaphore timeout for report ID %d", report_id);
        return -ETIMEDOUT;
    }

    // USB HID requires report ID as first byte, followed by report data
    uint8_t report_buffer[64];  // Max HID report size
    if (len + 1 > sizeof(report_buffer)) {
        LOG_ERR("HID report too large: %d bytes", len);
        k_sem_give(&hid_sem);
        return -EINVAL;
    }
    
    report_buffer[0] = report_id;
    memcpy(&report_buffer[1], data, len);

    int ret = hid_int_ep_write(hid_dev, report_buffer, len + 1, NULL);
    if (ret != 0) {
        k_sem_give(&hid_sem);  // Release semaphore on failure
        LOG_ERR("Failed to send HID report ID %d: %d", report_id, ret);
    }
    // Note: semaphore released in hid_int_ep_ready() callback on success
    
    return ret;
}

/**
 * @brief Forward keyboard report to USB HID
 * @param data Keyboard report data (8 bytes: modifiers + reserved + 6 keys)
 * @param len Length of keyboard report (should be 8)
 */
static void forward_keyboard_report(const uint8_t *data, size_t len) {
    if (len != KEYBOARD_REPORT_SIZE) {
        LOG_ERR("Invalid keyboard report size: %d (expected %d)", len, KEYBOARD_REPORT_SIZE);
        return;
    }

    // Cast to structure for easier debugging
    const struct zmk_keyboard_report *kbd = (const struct zmk_keyboard_report*)data;
    
    LOG_DBG("Keyboard report: mod=0x%02x, keys=[0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x]",
            kbd->modifiers, kbd->keys[0], kbd->keys[1], kbd->keys[2], 
            kbd->keys[3], kbd->keys[4], kbd->keys[5]);

    int ret = send_hid_report(HID_REPORT_ID_KEYBOARD, data, len);
    if (ret != 0) {
        LOG_ERR("Failed to forward keyboard report: %d", ret);
    }
}

/**
 * @brief Forward consumer control report to USB HID
 * @param data Consumer report data (2 bytes: usage code)
 * @param len Length of consumer report (should be 2)
 */
static void forward_consumer_report(const uint8_t *data, size_t len) {
    if (len != CONSUMER_REPORT_SIZE) {
        LOG_ERR("Invalid consumer report size: %d (expected %d)", len, CONSUMER_REPORT_SIZE);
        return;
    }

    // Cast to structure for easier debugging
    const struct zmk_consumer_report *consumer = (const struct zmk_consumer_report*)data;
    
    LOG_DBG("Consumer report: usage=0x%04x", consumer->usage);

    int ret = send_hid_report(HID_REPORT_ID_CONSUMER, data, len);
    if (ret != 0) {
        LOG_ERR("Failed to forward consumer report: %d", ret);
    }
}

/**
 * @brief Forward mouse report to USB HID
 * @param data Mouse report data (6 bytes: buttons + x + y + wheel)
 * @param len Length of mouse report (should be 6)
 */
static void forward_mouse_report(const uint8_t *data, size_t len) {
    if (len != MOUSE_REPORT_SIZE) {
        LOG_ERR("Invalid mouse report size: %d (expected %d)", len, MOUSE_REPORT_SIZE);
        return;
    }

    // Cast to structure for easier debugging
    const struct zmk_mouse_report *mouse = (const struct zmk_mouse_report*)data;
    
    LOG_DBG("Mouse report: buttons=0x%02x, x=%d, y=%d, wheel=%d",
            mouse->buttons, mouse->x, mouse->y, mouse->wheel);

    int ret = send_hid_report(HID_REPORT_ID_MOUSE, data, len);
    if (ret != 0) {
        LOG_ERR("Failed to forward mouse report: %d", ret);
    }
}

// ============================================================================
// ESB Event Handling & Packet Processing
// ============================================================================

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);

/**
 * @brief Validate received HID packet
 * @param header Packet header
 * @param total_payload_size Total ESB payload size
 * @return true if packet is valid
 */
static bool validate_hid_packet(const struct hid_packet_header *header, size_t total_payload_size) {
    // Check minimum packet size (header + at least 1 byte data)
    if (total_payload_size < sizeof(struct hid_packet_header) + 1) {
        LOG_WRN("Packet too small: %d bytes", total_payload_size);
        return false;
    }

    // Validate packet type
    if (header->type < HID_PACKET_TYPE_KEYBOARD || header->type > HID_PACKET_TYPE_MOUSE) {
        LOG_WRN("Invalid packet type: %d", header->type);
        return false;
    }

    // Check data length consistency
    size_t expected_header_plus_data = sizeof(struct hid_packet_header) + header->length;
    if (expected_header_plus_data > total_payload_size) {
        LOG_WRN("Invalid packet: header claims %d data bytes, but only %d total bytes received", 
                header->length, total_payload_size - sizeof(struct hid_packet_header));
        return false;
    }

    // Validate report size for each type
    bool size_valid = false;
    switch (header->type) {
        case HID_PACKET_TYPE_KEYBOARD:
            size_valid = (header->length == KEYBOARD_REPORT_SIZE);
            break;
        case HID_PACKET_TYPE_CONSUMER:
            size_valid = (header->length == CONSUMER_REPORT_SIZE);
            break;
        case HID_PACKET_TYPE_MOUSE:
            size_valid = (header->length == MOUSE_REPORT_SIZE);
            break;
    }

    if (!size_valid) {
        LOG_WRN("Invalid report size for type %d: expected %d, got %d", 
                header->type, 
                (header->type == HID_PACKET_TYPE_KEYBOARD) ? KEYBOARD_REPORT_SIZE :
                (header->type == HID_PACKET_TYPE_CONSUMER) ? CONSUMER_REPORT_SIZE : MOUSE_REPORT_SIZE,
                header->length);
        return false;
    }

    return true;
}

/**
 * @brief ESB event handler - processes incoming packets and forwards to USB HID
 */
void event_handler(struct esb_evt const *event) {
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        LOG_DBG("ESB TX success");
        break;
        
    case ESB_EVENT_TX_FAILED:
        LOG_DBG("ESB TX failed");
        break;
        
    case ESB_EVENT_RX_RECEIVED:
        // Read the ESB packet
        if (esb_read_rx_payload(&rx_payload) != 0) {
            LOG_ERR("Failed to read ESB payload");
            break;
        }

        LOG_DBG("ESB packet received: length=%d, rssi=%d", rx_payload.length, rx_payload.rssi);

        // Parse and validate packet
        struct hid_packet_header *header = (struct hid_packet_header*)rx_payload.data;
        
        if (!validate_hid_packet(header, rx_payload.length)) {
            LOG_WRN("Dropping invalid packet");
            break;
        }

        // Extract HID report data (everything after the header)
        uint8_t *hid_data = rx_payload.data + sizeof(struct hid_packet_header);
        size_t hid_data_len = header->length;

        LOG_DBG("Valid HID packet: type=%d, len=%d", header->type, header->length);

        // Forward to appropriate USB HID interface
        switch (header->type) {
        case HID_PACKET_TYPE_KEYBOARD:
            LOG_DBG("Forwarding keyboard report");
            forward_keyboard_report(hid_data, hid_data_len);
            break;
            
        case HID_PACKET_TYPE_CONSUMER:
            LOG_DBG("Forwarding consumer report");
            forward_consumer_report(hid_data, hid_data_len);
            break;
            
        case HID_PACKET_TYPE_MOUSE:
            LOG_DBG("Forwarding mouse report");
            forward_mouse_report(hid_data, hid_data_len);
            break;
            
        default:
            // This should never happen due to validation above
            LOG_ERR("Unexpected packet type: %d", header->type);
            break;
        }
        break;
        
    default:
        LOG_WRN("Unknown ESB event: %d", event->evt_id);
        break;
    }
}

// ============================================================================
// Clock Control (from original sample)
// ============================================================================

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void) {
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

    LOG_DBG("HF clock started");
    return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

int clocks_start(void) {
    int err;
    int res;
    const struct device *radio_clk_dev =
        DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
    struct onoff_client radio_cli;

    nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

    sys_notify_init_spinwait(&radio_cli.notify);

    err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

    do {
        err = sys_notify_fetch_result(&radio_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err == -EAGAIN);

    nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
    nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

    LOG_DBG("HF clock started");
    return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif

// ============================================================================
// ESB Initialization & Configuration
// ============================================================================

/**
 * @brief Initialize ESB in Primary Receiver (PRX) mode
 * 
 * CRITICAL: These addresses and settings must EXACTLY match your BLESB configuration!
 * Any mismatch will prevent communication between BLESB and DONGLE.
 */
int esb_initialize(void) {
    int err;
    
    // ESB addresses - MUST match BLESB exactly
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};  // Base address for pipe 0
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};  // Base address for pipes 1-7
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

    struct esb_config config = ESB_DEFAULT_CONFIG;

    // Protocol settings - MUST match BLESB
    config.protocol = ESB_PROTOCOL_ESB_DPL;    // Dynamic Payload Length
    config.bitrate = ESB_BITRATE_2MBPS;        // 2 Mbps for low latency
    config.mode = ESB_MODE_PRX;                // Primary Receiver (DONGLE receives)
    config.event_handler = event_handler;       // Our packet handler function
    config.selective_auto_ack = true;          // Enable automatic acknowledgments
    
    // Performance optimizations
    if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
        config.use_fast_ramp_up = true;        // Faster radio switching
    }
    
    // Additional performance settings for low latency
    config.retransmit_delay = 600;             // 600μs retry delay
    config.retransmit_count = 3;               // Max 3 retry attempts

    LOG_INF("Initializing ESB PRX with 2Mbps bitrate");

    err = esb_init(&config);
    if (err) {
        LOG_ERR("ESB init failed: %d", err);
        return err;
    }

    err = esb_set_base_address_0(base_addr_0);
    if (err) {
        LOG_ERR("ESB set base address 0 failed: %d", err);
        return err;
    }

    err = esb_set_base_address_1(base_addr_1);
    if (err) {
        LOG_ERR("ESB set base address 1 failed: %d", err);
        return err;
    }

    err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
    if (err) {
        LOG_ERR("ESB set prefixes failed: %d", err);
        return err;
    }

    LOG_INF("ESB initialized successfully");
    LOG_INF("Base addresses: 0=%02x%02x%02x%02x, 1=%02x%02x%02x%02x", 
            base_addr_0[0], base_addr_0[1], base_addr_0[2], base_addr_0[3],
            base_addr_1[0], base_addr_1[1], base_addr_1[2], base_addr_1[3]);

    return 0;
}

// ============================================================================
// USB HID Initialization
// ============================================================================

static int hid_device_init(void) {
    int err;

    hid_dev = device_get_binding("HID_0");
    if (!hid_dev) {
        LOG_ERR("Cannot get USB HID device binding");
        return -ENODEV;
    }

    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), 
                           &hid_callbacks);

    usb_hid_init(hid_dev);

    err = usb_enable(hid_status_cb);
    if (err != 0) {
        LOG_ERR("Failed to enable USB: %d", err);
        return err;
    }

    LOG_INF("USB HID initialized");
    return 0;
}

// ============================================================================
// Main Function
// ============================================================================

int main(void) {
    int err;

    LOG_INF("Enhanced ShockBurst HID Receiver starting...");

    // Initialize clocks
    err = clocks_start();
    if (err) {
        LOG_ERR("Clock start failed: %d", err);
        return 0;
    }

    // Initialize USB HID
    err = hid_device_init();
    if (err) {
        LOG_ERR("USB HID initialization failed: %d", err);
        return 0;
    }

    // Initialize ESB
    err = esb_initialize();
    if (err) {
        LOG_ERR("ESB initialization failed: %d", err);
        return 0;
    }

    LOG_INF("Initialization complete");

    // Write initial TX payload (for any bidirectional communication)
    err = esb_write_payload(&tx_payload);
    if (err) {
        LOG_ERR("Write payload failed: %d", err);
        return 0;
    }

    LOG_INF("Starting ESB reception...");

    // Start ESB reception
    err = esb_start_rx();
    if (err) {
        LOG_ERR("RX setup failed: %d", err);
        return 0;
    }

    return 0;
}