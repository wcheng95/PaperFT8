#define DEBUG_LOG 1

#include <cstdio>
#include <cmath>
#include "esp_log.h"
#include "esp_spiffs.h"
extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "ft8/message.h"
  #include "ft8/encode.h"
  #include "ft8/debug.h"
  #include "common/monitor.h"
  }

#include "ui.h"
#include "TouchKeyboard.h"
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "autoseq.h"
#include <M5Unified.h>
#include <sstream>
#include <iterator>
#include <cstdio>
#include <string>
#include <cstdint>
#include <vector>
#include <cstring>
#include <unordered_map>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <memory>
#include "driver/usb_serial_jtag.h"
#include "hal/uart_ll.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_random.h"
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "stream_uac.h"

#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

static const char* STATION_FILE = "/spiffs/Station.ini";
static sdmmc_card_t* g_sd_card = NULL;
static bool g_sd_mounted = false;
static bool g_ble_enabled = true;
static bool g_ble_qso_pick_mode = false;
static bool g_ble_dump_in_progress = false;

#define ENABLE_BLE 1

#if ENABLE_BLE
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "esp_bt.h"
#include "esp_mac.h"

#endif
#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif

#define BLE_UI_SVC_UUID   0xFFE0
#define BLE_UI_RX_UUID    0xFFE1
#define BLE_UI_TX_UUID    0xFFE2

#if ENABLE_BLE
static const ble_uuid16_t ble_ui_svc_uuid = BLE_UUID16_INIT(BLE_UI_SVC_UUID);
static const ble_uuid16_t ble_ui_rx_uuid = BLE_UUID16_INIT(BLE_UI_RX_UUID);
static const ble_uuid16_t ble_ui_tx_uuid = BLE_UUID16_INIT(BLE_UI_TX_UUID);
#endif

static constexpr size_t BLE_UI_INPUT_MAX = 160;
struct BleUiInput {
    uint16_t len = 0;
    char data[BLE_UI_INPUT_MAX] = {};
};

#if ENABLE_BLE

static QueueHandle_t ble_cmd_queue = nullptr;
static uint16_t gatt_tx_handle = 0;
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool g_ble_synced = false;
static bool g_ble_stack_active = false;
static bool g_ble_nvs_ready = false;
static bool g_ble_force_send = false;
static std::string g_ble_adv_name;
static std::string g_ble_last_payload;
static int64_t g_ble_last_tick_slot = -1;
static int g_ble_last_tick_sec = -1;
static bool g_ble_text_mode = false;
static uint8_t g_own_addr_type = 0;
static const char* BT_TAG = "BLE_INIT";

enum class BleDumpTxMode : uint8_t {
    Notify = 0,
    Indicate = 1,
};

struct BleDumpTransferState {
    bool active = false;
    BleDumpTxMode mode = BleDumpTxMode::Notify;
    int file_lines = 0;
    int retries = 0;
    int failed_lines = 0;
    int notify_pace_ms = 8;
    uint16_t mtu = 23;
};

static BleDumpTransferState g_ble_dump_xfer{};
static volatile bool g_ble_tx_notify_enabled = false;
static volatile bool g_ble_tx_indicate_enabled = false;
static volatile bool g_ble_indicate_waiting = false;
static volatile int g_ble_indicate_status = 0;
static volatile uint16_t g_ble_att_mtu = 23;

static constexpr int kBleDumpIndicateAckTimeoutMs = 1500;
static constexpr int kBleDumpIndicateMaxRetries = 3;
static constexpr int kBleDumpNotifyMaxRetries = 4;
static constexpr int kBleDumpNotifyBackoffMs[kBleDumpNotifyMaxRetries] = {5, 10, 20, 40};
static constexpr int kBleDumpNotifyPaceMinMs = 8;
static constexpr int kBleDumpNotifyPaceMaxMs = 20;

static int gap_cb(struct ble_gap_event *event, void *arg);
static void nimble_host_task(void *param);
static void ble_on_sync(void);
static void ble_app_advertise(void);
static void ble_update_name_from_station(bool restart_adv);
static bool ble_stack_start(void);
static bool ble_stack_stop(void);
static bool ble_ensure_nvs_ready(void);
static void ble_reset_runtime_state(void);
static void ble_delete_cmd_queue(void);
static void ble_countdown_tick();
static int ble_send_payload_raw(const std::string& payload, bool indicate);
static bool ble_wait_for_indicate_ack(int timeout_ms);
static void ble_dump_reset_transfer_state(bool use_indicate);
static bool ble_dump_send_line(const std::string& raw);

static std::string ble_trim_trailing_crlf(const char* data, uint16_t len)
{
    if (!data || len == 0) return std::string();
    size_t used = len;
    while (used > 0 && (data[used - 1] == '\r' || data[used - 1] == '\n')) used--;
    return std::string(data, data + used);
}

static char ble_parse_ui_command(const char* data, uint16_t len)
{
    const std::string payload = ble_trim_trailing_crlf(data, len);
    if (payload.size() != 1) return 0;  // command mode is single-character only
    char c = payload[0];
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - ('a' - 'A'));

    if (c >= '1' && c <= '6') return c;
    switch (c) {
      case 'S':
      case 'R':
      case 'T':
      case 'M':
      case 'Q':
      case 'B':
      case 'F':
      case 'D':
      case 'N':
      case 'O':
        return c;
      case 'U':  // page up
        return ';';
      case 'V':  // page down
        return '.';
      case 'Z':  // left (Zuo)
        return ',';
      case 'X':  // right (You)
        return '/';
      case 'E':  // ESC / TX cancel
        return '`';
      default:
        return 0;
    }
}

static int ble_ui_rx_cb(uint16_t conn_handle,
                        uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt,
                        void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (!ble_cmd_queue || !ctxt || !ctxt->om) return 0;
    BleUiInput input{};
    size_t copy_len = ctxt->om->om_len;
    if (copy_len > BLE_UI_INPUT_MAX) copy_len = BLE_UI_INPUT_MAX;
    input.len = static_cast<uint16_t>(copy_len);
    if (copy_len > 0) {
      std::memcpy(input.data, ctxt->om->om_data, copy_len);
    }
    xQueueSend(ble_cmd_queue, &input, 0);
    return 0;  // ignore unsupported input silently
}

static int ble_ui_tx_cb(uint16_t conn_handle,
                        uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt,
                        void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)ctxt;
    (void)arg;
    return 0;
}

#include "esp_nimble_hci.h"

// C++-safe static characteristics table (fully initialized for -Werror).
static const struct ble_gatt_chr_def gatt_uart_chrs[] = {
    {
        &ble_ui_rx_uuid.u,                       // uuid
        ble_ui_rx_cb,                            // access_cb
        nullptr,                                 // arg
        nullptr,                                 // descriptors
        BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP, // flags
        0,                                       // min_key_size
        nullptr,                                 // val_handle
        nullptr,                                 // cpfd
    },
    {
        &ble_ui_tx_uuid.u,                       // uuid
        ble_ui_tx_cb,                            // access_cb
        nullptr,                                 // arg
        nullptr,                                 // descriptors
        BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE, // flags
        0,                                       // min_key_size
        &gatt_tx_handle,                         // val_handle
        nullptr,                                 // cpfd
    },
    {
        nullptr,                                 // uuid terminator
        nullptr,                                 // access_cb
        nullptr,                                 // arg
        nullptr,                                 // descriptors
        0,                                       // flags
        0,                                       // min_key_size
        nullptr,                                 // val_handle
        nullptr,                                 // cpfd
    },
};

// C++-safe service table (fully initialized for -Werror).
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        BLE_GATT_SVC_TYPE_PRIMARY,               // type
        &ble_ui_svc_uuid.u,                      // uuid
        nullptr,                                 // includes
        gatt_uart_chrs,                          // characteristics
    },
    {
        0,                                       // type terminator
        nullptr,                                 // uuid
        nullptr,                                 // includes
        nullptr,                                 // characteristics
    }
};


static void ble_delete_cmd_queue(void) {
    if (!ble_cmd_queue) return;
    vQueueDelete(ble_cmd_queue);
    ble_cmd_queue = nullptr;
}

static void ble_reset_runtime_state(void) {
    gatt_tx_handle = 0;
    g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    g_ble_synced = false;
    g_ble_force_send = false;
    g_ble_last_payload.clear();
    g_ble_last_tick_slot = -1;
    g_ble_last_tick_sec = -1;
    g_ble_text_mode = false;
    g_ble_qso_pick_mode = false;
    g_ble_dump_in_progress = false;
    g_own_addr_type = 0;
    g_ble_tx_notify_enabled = false;
    g_ble_tx_indicate_enabled = false;
    g_ble_indicate_waiting = false;
    g_ble_indicate_status = 0;
    g_ble_att_mtu = 23;
    g_ble_dump_xfer = BleDumpTransferState{};
}

static bool ble_ensure_nvs_ready(void) {
    if (g_ble_nvs_ready) return true;

    esp_err_t nvrc = nvs_flash_init();
    if (nvrc == ESP_ERR_NVS_NO_FREE_PAGES || nvrc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvrc = nvs_flash_init();
    }
    if (nvrc != ESP_OK) {
        ESP_LOGE(BT_TAG, "nvs_flash_init failed: %s", esp_err_to_name(nvrc));
        return false;
    }
    g_ble_nvs_ready = true;
    return true;
}

static bool ble_stack_start(void) {
    if (g_ble_stack_active) return true;
    ESP_LOGI(BT_TAG, "BLE stack start");

    if (!ble_ensure_nvs_ready()) return false;

    ble_delete_cmd_queue();
    ble_reset_runtime_state();

    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "nimble_port_init failed: %d", rc);
        return false;
    }
    ESP_LOGI(BT_TAG, "nimble_port_init OK");

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_LOGI(BT_TAG, "GAP/GATT init done");

    g_ble_stack_active = true;
    ble_cmd_queue = xQueueCreate(32, sizeof(BleUiInput));
    if (!ble_cmd_queue) {
        ESP_LOGE(BT_TAG, "xQueueCreate failed");
        g_ble_stack_active = false;
        nimble_port_deinit();
        return false;
    }
    ble_update_name_from_station(false);

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "ble_gatts_count_cfg failed: %d", rc);
        ble_delete_cmd_queue();
        g_ble_stack_active = false;
        nimble_port_deinit();
        return false;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "ble_gatts_add_svcs failed: %d", rc);
        ble_delete_cmd_queue();
        g_ble_stack_active = false;
        nimble_port_deinit();
        return false;
    }
    ESP_LOGI(BT_TAG, "Services added");

    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(nimble_host_task);
    g_ble_force_send = true;
    ESP_LOGI(BT_TAG, "Host task started");
    return true;
}

static bool ble_stack_stop(void) {
    if (!g_ble_stack_active) {
        ble_delete_cmd_queue();
        ble_reset_runtime_state();
        return true;
    }

    ESP_LOGI(BT_TAG, "BLE stack stop");

    if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        int rc = ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGW(BT_TAG, "ble_gap_terminate rc=%d", rc);
        }
    }
    if (g_ble_synced) {
        int rc = ble_gap_adv_stop();
        if (rc != 0) {
            ESP_LOGW(BT_TAG, "ble_gap_adv_stop rc=%d", rc);
        }
    }

    int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "nimble_port_stop failed: %d", rc);
        return false;
    }

    esp_err_t deinit_rc = nimble_port_deinit();
    if (deinit_rc != ESP_OK) {
        ESP_LOGE(BT_TAG, "nimble_port_deinit failed: %s", esp_err_to_name(deinit_rc));
        return false;
    }

    ble_delete_cmd_queue();
    ble_reset_runtime_state();
    g_ble_stack_active = false;
    ESP_LOGI(BT_TAG, "BLE stack stopped");
    return true;
}

static int gap_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            if (!g_ble_enabled) {
              ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
              g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
              return 0;
            }
            g_ble_force_send = true;
            g_ble_last_tick_slot = -1;
            g_ble_last_tick_sec = -1;
            g_ble_text_mode = false;
            g_ble_tx_notify_enabled = false;
            g_ble_tx_indicate_enabled = false;
            g_ble_indicate_waiting = false;
            g_ble_indicate_status = 0;
            g_ble_att_mtu = 23;
            ESP_LOGI(BT_TAG, "Connected, handle=%u", g_conn_handle);
        } else {
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(BT_TAG, "Connect failed; restarting adv");
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_ble_last_payload.clear();
        g_ble_last_tick_slot = -1;
        g_ble_last_tick_sec = -1;
        g_ble_text_mode = false;
        g_ble_tx_notify_enabled = false;
        g_ble_tx_indicate_enabled = false;
        g_ble_indicate_waiting = false;
        g_ble_indicate_status = 0;
        g_ble_att_mtu = 23;
        if (ble_cmd_queue) xQueueReset(ble_cmd_queue);
        ESP_LOGW(BT_TAG, "Disconnected; restarting adv");
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.conn_handle == g_conn_handle &&
            event->subscribe.attr_handle == gatt_tx_handle) {
            g_ble_tx_notify_enabled = event->subscribe.cur_notify != 0;
            g_ble_tx_indicate_enabled = event->subscribe.cur_indicate != 0;
            ESP_LOGI(BT_TAG, "TX subscribe: notify=%d indicate=%d",
                     g_ble_tx_notify_enabled ? 1 : 0,
                     g_ble_tx_indicate_enabled ? 1 : 0);
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.conn_handle == g_conn_handle &&
            event->notify_tx.attr_handle == gatt_tx_handle &&
            event->notify_tx.indication &&
            g_ble_indicate_waiting) {
            const int st = event->notify_tx.status;
            if (st != 0) {
                g_ble_indicate_status = st;
                g_ble_indicate_waiting = false;
            }
        }
        break;

    case BLE_GAP_EVENT_MTU:
        if (event->mtu.conn_handle == g_conn_handle && event->mtu.value > 0) {
            g_ble_att_mtu = event->mtu.value;
            ESP_LOGI(BT_TAG, "ATT MTU=%u", (unsigned)g_ble_att_mtu);
        }
        break;

    default:
        break;
    }
    return 0;
}


static bool ble_pop_input(BleUiInput& out) {
    if (!ble_cmd_queue) return false;
    return xQueueReceive(ble_cmd_queue, &out, 0) == pdTRUE;
}
#endif // ENABLE_BLE

int64_t rtc_now_ms();
static esp_err_t copy_file_overwrite(const char* src_path, const char* dst_path);

static void debug_log_line(const std::string& msg);
//exported symbol (linkable from other .cpp)
void debug_log_line_public(const std::string& msg) {
  debug_log_line(msg);
}

//static const char *TAG = "sdtest";

struct SdSpiPins {
    gpio_num_t sclk;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t cs;
};

static SdSpiPins resolve_sd_spi_pins() {
    // Fallback for M5PaperS3 in case board detection pin table is unavailable.
    SdSpiPins pins = {
        GPIO_NUM_39, // SCLK
        GPIO_NUM_38, // MOSI (COPI)
        GPIO_NUM_40, // MISO (CIPO)
        GPIO_NUM_47, // CS
    };

    const int sclk = M5.getPin(m5::pin_name_t::sd_spi_sclk);
    const int mosi = M5.getPin(m5::pin_name_t::sd_spi_mosi);
    const int miso = M5.getPin(m5::pin_name_t::sd_spi_miso);
    const int cs   = M5.getPin(m5::pin_name_t::sd_spi_cs);

    if (sclk >= 0) pins.sclk = static_cast<gpio_num_t>(sclk);
    if (mosi >= 0) pins.mosi = static_cast<gpio_num_t>(mosi);
    if (miso >= 0) pins.miso = static_cast<gpio_num_t>(miso);
    if (cs >= 0)   pins.cs   = static_cast<gpio_num_t>(cs);

    return pins;
}

void mount_sd_spi(void)
{
    static const char* SD_TAG = "SD";
    esp_err_t ret;
    const char mount_point[] = "/sdcard";
    const SdSpiPins pins = resolve_sd_spi_pins();
    ESP_LOGI(SD_TAG, "SD SPI pins: sclk=%d mosi=%d miso=%d cs=%d",
             (int)pins.sclk, (int)pins.mosi, (int)pins.miso, (int)pins.cs);

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = pins.mosi;
    bus_cfg.miso_io_num = pins.miso;
    bus_cfg.sclk_io_num = pins.sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(SD_TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = pins.cs;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &g_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGW(SD_TAG, "esp_vfs_fat_sdspi_mount failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        g_sd_card = NULL;
        g_sd_mounted = false;
        return;
    }

    g_sd_mounted = true;
}

void unmount_sd_spi(const char *mount_point)
{
    if (g_sd_mounted && g_sd_card) {
        esp_vfs_fat_sdcard_unmount(mount_point, g_sd_card);
        g_sd_card = NULL;
        g_sd_mounted = false;
    }
    spi_bus_free(SPI2_HOST);
}

// ---------- Log copy/delete helpers ----------
static bool sdcard_is_mounted() {
  struct stat st;
  return (stat("/sdcard", &st) == 0) && S_ISDIR(st.st_mode);
}

static esp_err_t ensure_sdcard_mounted() {
  if (sdcard_is_mounted()) return ESP_OK;
  mount_sd_spi();
  if (sdcard_is_mounted()) return ESP_OK;
  return ESP_FAIL;
}

static void build_rxtx_log_path(char* path, size_t path_sz) {
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);

  // RT[YYMMDD].log
  snprintf(path, path_sz, "/spiffs/RT%02d%02d%02d.log",
           (t.tm_year + 1900) % 100,
           (t.tm_mon + 1) % 100,
           t.tm_mday % 100);
}

static bool file_exists(const char* path) {
  struct stat st;
  return (stat(path, &st) == 0) && S_ISREG(st.st_mode);
}

static void sync_station_ini_from_sd_to_spiffs() {
  static const char* TAG = "FT8";

  if (ensure_sdcard_mounted() != ESP_OK) {
    ESP_LOGI(TAG, "SD not mounted, using SPIFFS Station.ini");
    return;
  }

  const char* sd_path = "/sdcard/Station.ini";
  const char* spiffs_path = "/spiffs/Station.ini";

  if (!file_exists(sd_path)) {
    ESP_LOGI(TAG, "No Station.ini on SD, using SPIFFS Station.ini");
    unmount_sd_spi("/sdcard");
    return;
  }

  if (copy_file_overwrite(sd_path, spiffs_path) == ESP_OK) {
    ESP_LOGI(TAG, "Copied Station.ini from SD to SPIFFS");
  } else {
    ESP_LOGW(TAG, "Failed to copy Station.ini from SD, using SPIFFS Station.ini");
  }

  unmount_sd_spi("/sdcard");
}


static esp_err_t copy_file_overwrite(const char* src_path, const char* dst_path) {
  FILE* fs = fopen(src_path, "rb");
  if (!fs) return ESP_FAIL;

  FILE* fd = fopen(dst_path, "wb");  // overwrite
  if (!fd) { fclose(fs); return ESP_FAIL; }

  uint8_t buf[4096];
  size_t r = 0;

  while ((r = fread(buf, 1, sizeof(buf), fs)) > 0) {
    if (fwrite(buf, 1, r, fd) != r) {
      fclose(fd);
      fclose(fs);
      return ESP_FAIL;
    }
  }

  // Detect read error (not just EOF)
  if (ferror(fs)) {
    fclose(fd);
    fclose(fs);
    return ESP_FAIL;
  }

  // Ensure SD gets the bytes
  fflush(fd);
  fsync(fileno(fd));

  fclose(fd);
  fclose(fs);
  return ESP_OK;
}

// Copy all regular files from SPIFFS -> SD card, overwriting destination.
static esp_err_t copy_logs_spiffs_to_sd_overwrite() {
  esp_err_t mret = ensure_sdcard_mounted();
  if (mret != ESP_OK) return mret;
  vTaskDelay(pdMS_TO_TICKS(100));

  DIR* d = opendir("/spiffs");
  if (!d) {
    unmount_sd_spi("/sdcard");
    return ESP_FAIL;
  }

  esp_err_t last_err = ESP_OK;
  struct dirent* ent;

  while ((ent = readdir(d)) != nullptr) {
    const char* name = ent->d_name;
    if (!name || name[0] == '.') continue;

    std::string src = std::string("/spiffs/") + name;

    struct stat st;
    bool stat_ok = false;
    for (int i = 0; i < 5; ++i) {
      if (stat(src.c_str(), &st) == 0) { stat_ok = true; break; }
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!stat_ok) { last_err = ESP_FAIL; continue; }
    if (!S_ISREG(st.st_mode)) continue;

    std::string dst = std::string("/sdcard/") + name;
    esp_err_t err = copy_file_overwrite(src.c_str(), dst.c_str());
    if (err != ESP_OK) last_err = err;
  }

  closedir(d);
  unmount_sd_spi("/sdcard");
  return last_err;
}
// Delete all regular files on SPIFFS, except Station.ini.
static esp_err_t delete_logs_on_spiffs_keep_stationdata() {
  DIR* d = opendir("/spiffs");
  if (!d) return ESP_FAIL;

  struct dirent* ent;
  while ((ent = readdir(d)) != nullptr) {
    const char* name = ent->d_name;
    if (!name || name[0] == '.') continue;
    if (strcmp(name, "Station.ini") == 0) continue;
    std::string path = std::string("/spiffs/") + name;
    struct stat st;
    if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
    unlink(path.c_str());  // ignore missing/err
  }
  closedir(d);
  return ESP_OK;
}

#define CALLSIGN_HASHTABLE_SIZE 256

static struct
{
    char callsign[12]; /// Up to 11 symbols of callsign + trailing zero
    uint32_t hash;     /// 8 MSBs = age, 22 LSBs = hash value
} callsign_hashtable[CALLSIGN_HASHTABLE_SIZE];

static int callsign_hashtable_size;

void hashtable_init(void)
{
    callsign_hashtable_size = 0;
    memset(callsign_hashtable, 0, sizeof(callsign_hashtable));
}

// Increment age for all existing entries (saturate at 255). Call once per slot.
static void hashtable_age_all(void)
{
    for (int i = 0; i < CALLSIGN_HASHTABLE_SIZE; ++i)
    {
        if (callsign_hashtable[i].callsign[0] != '\0')
        {
            uint8_t age = (uint8_t)(callsign_hashtable[i].hash >> 24);
            if (age < 255)
            {
                age++;
                callsign_hashtable[i].hash =
                    ((uint32_t)age << 24) | (callsign_hashtable[i].hash & 0x003FFFFFu);
            }
        }
    }
}

// Trim the hash table if it grows too large by evicting the oldest entries
void hashtable_trim_size(int max_size)
{
    while (callsign_hashtable_size > max_size)
    {
        int oldest_idx = -1;
        uint8_t oldest_age = 0;

        for (int i = 0; i < CALLSIGN_HASHTABLE_SIZE; ++i)
        {
            if (callsign_hashtable[i].callsign[0] == '\0')
                continue;

            uint8_t age = (uint8_t)(callsign_hashtable[i].hash >> 24);
            if (oldest_idx < 0 || age > oldest_age)
            {
                oldest_idx = i;
                oldest_age = age;
            }
        }

        if (oldest_idx < 0)
            break;

        LOG(LOG_INFO, "Hashtable trim: removing oldest [%s], age=%u\n",
            callsign_hashtable[oldest_idx].callsign, (unsigned)oldest_age);

        callsign_hashtable[oldest_idx].callsign[0] = '\0';
        callsign_hashtable[oldest_idx].hash = 0;
        callsign_hashtable_size--;
    }
}

void hashtable_add(const char* callsign, uint32_t hash)
{
    if (!callsign || !callsign[0])
        return;

    uint32_t hash_payload = hash & 0x003FFFFFu;   // 22-bit value
    uint16_t hash10 = (hash_payload >> 12) & 0x03FFu;
    int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    int start_idx = idx;

    while (callsign_hashtable_size >= CALLSIGN_HASHTABLE_SIZE)
    {
        hashtable_trim_size(CALLSIGN_HASHTABLE_SIZE - 50);
        if (callsign_hashtable_size >= CALLSIGN_HASHTABLE_SIZE)
        {
            LOG(LOG_INFO, "Hash table full; ignoring new callsign [%s]\n", callsign);
            return;
        }
    }

    // Linear probing: must match lookup logic
    while (callsign_hashtable[idx].callsign[0] != '\0')
    {
        uint32_t existing_hash = callsign_hashtable[idx].hash & 0x003FFFFFu;

        if ((existing_hash == hash_payload) &&
            (strcmp(callsign_hashtable[idx].callsign, callsign) == 0))
        {
            // Refresh age to 0, keep same callsign/hash
            callsign_hashtable[idx].hash = hash_payload;
            LOG(LOG_DEBUG, "Found duplicate [%s], refreshed age\n", callsign);
            return;
        }

        if (existing_hash == hash_payload)
        {
            // Same 22-bit hash but different callsign: replace old one
            LOG(LOG_INFO, "Replacing [%s] with [%s] on same hash\n",
                callsign_hashtable[idx].callsign, callsign);

            strncpy(callsign_hashtable[idx].callsign, callsign, 11);
            callsign_hashtable[idx].callsign[11] = '\0';
            callsign_hashtable[idx].hash = hash_payload;
            return;
        }

        idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
        if (idx == start_idx)
        {
            LOG(LOG_INFO, "Hash table probe wrapped; abort insert for [%s]\n", callsign);
            return;
        }
    }

    strncpy(callsign_hashtable[idx].callsign, callsign, 11);
    callsign_hashtable[idx].callsign[11] = '\0';
    callsign_hashtable[idx].hash = hash_payload;  // age=0
    callsign_hashtable_size++;
}

bool hashtable_lookup(ftx_callsign_hash_type_t hash_type, uint32_t hash, char* callsign)
{
    if (!callsign)
        return false;

    uint8_t hash_shift =
        (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? 12 :
        (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? 10 : 0;

    // Derive the same start bucket from the top 10 bits of the 22-bit hash.
    // For 10-bit lookup: hash is already the top 10 bits.
    // For 12-bit lookup: top 10 bits are hash >> 2.
    // For 22-bit lookup: top 10 bits are hash >> 12.
    uint16_t hash10 =
        (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? (hash & 0x03FFu) :
        (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? ((hash >> 2) & 0x03FFu) :
                                                   ((hash >> 12) & 0x03FFu);

    int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    // Important: entries can be deleted by hashtable_trim_size(), which creates
    // empty holes in probe chains. Stopping at the first empty slot can miss
    // valid entries that were inserted later in that chain. Scan the full table.
    for (int probe = 0; probe < CALLSIGN_HASHTABLE_SIZE; ++probe)
    {
        int scan_idx = (idx + probe) % CALLSIGN_HASHTABLE_SIZE;
        if (callsign_hashtable[scan_idx].callsign[0] == '\0')
            continue;

        uint32_t existing_hash = callsign_hashtable[scan_idx].hash & 0x003FFFFFu;

        if ((existing_hash >> hash_shift) == hash)
        {
            strcpy(callsign, callsign_hashtable[scan_idx].callsign);

            // Reset age to 0 on successful hit, preserve 22-bit payload.
            callsign_hashtable[scan_idx].hash = existing_hash;
            return true;
        }
    }

    callsign[0] = '\0';
    return false;
}

ftx_callsign_hash_interface_t hash_if = {
    .lookup_hash = hashtable_lookup,
    .save_hash = hashtable_add
};

static std::string normalize_call_token(std::string s) {
  // trim <> wrappers used for hashed nonstd calls
  if (!s.empty() && s.front() == '<') s.erase(s.begin());
  if (!s.empty() && s.back()  == '>') s.pop_back();

  for (auto& ch : s) ch = (char)toupper((unsigned char)ch);
  return s;
}

static bool rewrite_dxpedition_for_mycall(const std::string& raw_text,
                                          const std::string& mycall_up,
                                          std::string& rewritten_text) {
  std::istringstream iss(raw_text);
  std::string call1, rr73_tok, call2, foxcall, rpt;
  if (!(iss >> call1 >> rr73_tok >> call2 >> foxcall >> rpt)) return false;

  std::string trailing;
  if (iss >> trailing) return false;
  if (rr73_tok != "RR73;") return false;

  std::string call1_up = normalize_call_token(call1);
  std::string call2_up = normalize_call_token(call2);
  if (call1_up.empty() || call2_up.empty() || mycall_up.empty()) return false;

  if (call1_up == mycall_up) {
    rewritten_text = call1 + " " + foxcall + " RR73";
    return true;
  }
  if (call2_up == mycall_up) {
    rewritten_text = call2 + " " + foxcall + " " + rpt;
    return true;
  }
  return false;
}

static const char* TAG = "FT8";
enum class UIMode { RX, TX, BAND, MENU, CONTROL, DEBUG, STATUS, QSO };
static UIMode ui_mode = UIMode::RX;
static int tx_page = 0;
static std::vector<UiRxLine> g_rx_lines;
static volatile bool g_tx_view_dirty = false;  // Set when autoseq state changes
int64_t g_decode_slot_idx = -1; // set at decode trigger to tag RX lines with slot parity

// State machine variables (matching reference project architecture)
// TX is scheduled by setting these flags; actual TX starts at slot boundary
static volatile bool g_qso_xmit = false;        // TX is pending
static volatile int g_target_slot_parity = 0;   // 0=even, 1=odd - parity of slot to TX on
static volatile bool g_was_txing = false;       // We were transmitting (for tick timing)
volatile bool g_decode_in_progress = false; // Block TX trigger while decoding
static int g_last_slot_parity = -1;             // For slot boundary detection (just parity, like reference)

//enum class BeaconMode { OFF = 0, EVEN, EVEN2, ODD, ODD2 };
enum class BeaconMode { OFF = 0, EVEN, ODD };
struct BandItem {
  const char* name;
  int freq;
};
static std::vector<BandItem> g_bands = {
    {"160m", 1840},   {"80m", 3573},   {"60m", 5357},   {"40m", 7074},
    {"30m", 10136},   {"20m", 14074},  {"17m", 18100},  {"15m", 21074},
    {"12m", 24915},   {"10m", 28074},  {"6m", 50313},   {"2m", 144174},
};
static std::string g_active_band_text = "80 40 20 17 15 12 10";
static std::vector<int> g_active_band_indices;
static int band_page = 0;
static int band_edit_idx = -1;       // absolute index into g_bands
static std::string band_edit_buffer; // text while editing
static void update_autoseq_cq_type();
static void update_autoseq_cq_type();
static BeaconMode g_beacon = BeaconMode::OFF;
static int g_offset_hz = 1500;
static int g_band_sel = 1; // default 80m
static bool g_tune = false;
static BeaconMode g_status_beacon_temp = BeaconMode::OFF;
[[maybe_unused]] static bool g_cat_toggle_high = false;
static std::string g_date = "2025-12-11";
static std::string g_time = "10:10:00";
static int status_edit_idx = -1;     // 0-5
static std::string status_edit_buffer;
static int status_cursor_pos = -1;
static std::vector<std::string> g_debug_lines;
static int debug_page = 0;
static const size_t DEBUG_MAX_LINES = 18; // 3 pages
static UIMode g_ble_qso_return_mode = UIMode::RX;

static void host_handle_line(const std::string& line);
static void save_station_data();
// TX entry for display and scheduling (populated by autoseq)
static AutoseqTxEntry g_pending_tx;
static bool g_pending_tx_valid = false;
static volatile bool g_tx_cancel_requested = false;
static void host_process_bytes(const uint8_t* buf, size_t len);
static void poll_host_uart();
static bool ble_pop_input(BleUiInput& out);
static void ble_update_name_from_station(bool restart_adv);
static void ble_mirror_tick();
static void ble_countdown_tick();
struct GestureEvent;
static bool poll_gesture(GestureEvent& out);
static UIMode swipe_next_mode(UIMode current, int dir);
static void enter_mode(UIMode new_mode);
static void apply_ble_enabled_policy(bool runtime_apply);
static std::string menu_sleep_batt_line();
static bool g_rx_dirty = false;
#if ENABLE_BLE
static void ble_start_qso_pick_mode();
static void ble_cancel_qso_pick_mode();
static void ble_try_dump_qso_file_by_key(char key);
#endif



static std::vector<std::string> g_ctrl_lines = {
    "C MODE: USB serial",
    "Commands:",
    "WRITEBIN <file> <size> <crc32_hex>",
    "WRITE/APPEND",
    "READ/DELETE",
    "DATE [YYYY-MM-DD]",
    "TIME [HH:MM:SS]",
    "SLEEP - deep sleep",
    "LIST/INFO/HELP",
    "EXIT to leave"
};

static std::vector<std::string> g_startup_lines = {
    "PaperFT8 V1.4.3",
    "S: Status(Operate)",
    "R: Rx page",
    "T: Tx page",
    "M: Menu(Setting)",
    "Other: Q/C/B/N/O/D"
};

// Runtime latch: when true, we keep showing the startup screen until any key is pressed.
static bool g_startup_active = true;

enum class GestureAction {
  None,
  TapMode,
  TapLine,
  TapCommand,
  SwipeLeft,
  SwipeRight,
  SwipeUp,
  SwipeDown,
};

struct GestureEvent {
  GestureAction action = GestureAction::None;
  int x = 0;
  int y = 0;
  int line_idx = -1;
  int command_idx = -1;
};
static std::vector<std::string> g_q_lines;
static std::vector<std::string> g_q_files;
static bool g_q_show_entries = false;
static int q_page = 0;
static std::string g_q_current_file;
static std::string host_input;
static const char* HOST_PROMPT = "MINIFT8> ";
static bool usb_ready = false;
static QueueHandle_t s_key_inject_queue = nullptr;
static bool host_bin_active = false;
static size_t host_bin_remaining = 0;
static FILE* host_bin_fp = nullptr;
static uint32_t host_bin_crc = 0;
static uint32_t host_bin_expected_crc = 0;
static size_t host_bin_received = 0;
static std::vector<uint8_t> host_bin_buf;
static const size_t HOST_BIN_CHUNK = 512;
static size_t host_bin_chunk_expect = 0; // payload bytes this chunk (excludes CRC trailer)
static uint8_t host_bin_first8[8] = {0};
static uint8_t host_bin_last8[8] = {0};
static size_t host_bin_first_filled = 0;
static std::string host_bin_path;

// Software RTC
static time_t rtc_epoch_base = 0;
static int64_t rtc_ms_start = 0;
static int64_t rtc_last_update = 0;
static bool rtc_valid = false;

// RTC deep sleep compensation
// rtc_sleep_epoch: epoch time when entering deep sleep (for calculating elapsed time)
// rtc_comp is not user-configurable in PaperFT8 (battery-backed RTC).
static time_t g_rtc_sleep_epoch = 0;
static int g_rtc_comp = 0;

enum class CqType { CQ, CQSOTA, CQPOTA, CQQRP, CQFD, CQFREETEXT };
enum class OffsetSrc { RANDOM, CURSOR, RX };
enum class RadioType { NONE, TRUSDX, QMX };
static CqType g_cq_type = CqType::CQ;
static std::string g_cq_freetext = "FreeText";
static bool g_skip_tx1 = false;
static int g_autoseq_max_retry = AUTOSEQ_MAX_RETRY;
static std::string g_free_text = "TNX 73";
static std::string g_call = "YOURCALL";
static std::string g_grid = "CM97";
bool g_decode_enabled = true;
static OffsetSrc g_offset_src = OffsetSrc::RANDOM;
static RadioType g_radio = RadioType::QMX;
static constexpr size_t kIgnorePrefixTextMaxLen = 64;
static std::string g_comment1 = "MiniFT8 /Radio";
static std::string g_ignore_prefix_text;
static std::vector<std::string> g_ignore_prefixes;
static bool g_rxtx_log = true;
// Single-threaded TX state machine (replaces separate tx_send_task)
// TX runs in main loop via tx_tick(), one tone at a time
static bool g_tx_active = false;           // TX state machine is running
static int g_tx_tone_idx = 0;              // Current tone index (0-78)
static int64_t g_tx_next_tone_time = 0;    // When to send next tone (ms)
static int64_t g_tx_slot_start_ms = 0;     // Slot boundary time for tone alignment
static uint8_t g_tx_tones[79];             // Encoded tones
static int g_tx_base_hz = 0;               // Base frequency for TA commands
static int64_t g_tx_slot_idx = 0;          // Slot index for autoseq_mark_sent
static bool g_tx_cat_ok = false;           // CAT available for this TX
static int g_tx_last_ta_int = -1;          // For TA command deduplication
static int g_tx_last_ta_frac = -1;

static SemaphoreHandle_t log_mutex = NULL;                       // Protects log_rxtx_line file access
static int menu_page = 0;
static int menu_edit_idx = -1;
static std::string menu_edit_buf;
static bool menu_long_edit = false;
static enum { LONG_NONE, LONG_FT, LONG_COMMENT, LONG_ACTIVE, LONG_IGNORE } menu_long_kind = LONG_NONE;
static std::string menu_long_buf;
static std::string menu_long_backup;
static int menu_flash_idx = -1;          // absolute index to flash highlight
static int64_t menu_flash_deadline = 0;  // ms timestamp when flash ends
static bool menu_delete_confirm = false;  // confirmation state for Delete Logs
static int rx_flash_idx = -1;
static int64_t rx_flash_deadline = 0;
enum class TouchKbdEditTarget { None, MenuLong, MenuEdit, BandFreq, StatusDate, StatusTime };
static TouchKbdEditTarget g_touchkbd_target = TouchKbdEditTarget::None;
static std::string g_touchkbd_header;
static char g_touchkbd_buffer[256] = {0};
static bool g_touchkbd_needs_display = false;
bool g_streaming = false;
static void draw_menu_view();
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging);
static std::string digits_only_limited(const std::string& in, size_t max_len) {
  std::string out;
  out.reserve(max_len);
  for (unsigned char ch : in) {
    if (std::isdigit(ch) != 0) {
      out.push_back(static_cast<char>(ch));
      if (out.size() >= max_len) break;
    }
  }
  return out;
}

static std::string format_date_digits_preview(const std::string& digits) {
  static constexpr int kPos[8] = {0, 1, 2, 3, 5, 6, 8, 9};
  std::string out = "____-__-__";
  const size_t n = std::min<size_t>(digits.size(), 8);
  for (size_t i = 0; i < n; ++i) out[kPos[i]] = digits[i];
  return out;
}

static std::string format_time_digits_preview(const std::string& digits) {
  static constexpr int kPos[6] = {0, 1, 3, 4, 6, 7};
  std::string out = "__:__:__";
  const size_t n = std::min<size_t>(digits.size(), 6);
  for (size_t i = 0; i < n; ++i) out[kPos[i]] = digits[i];
  return out;
}

static bool format_date_digits_exact(const std::string& digits, std::string& out) {
  if (digits.size() != 8) return false;
  out.assign("____-__-__");
  static constexpr int kPos[8] = {0, 1, 2, 3, 5, 6, 8, 9};
  for (size_t i = 0; i < 8; ++i) out[kPos[i]] = digits[i];
  return true;
}

static bool format_time_digits_exact(const std::string& digits, std::string& out) {
  if (digits.size() != 6) return false;
  out.assign("__:__:__");
  static constexpr int kPos[6] = {0, 1, 3, 4, 6, 7};
  for (size_t i = 0; i < 6; ++i) out[kPos[i]] = digits[i];
  return true;
}

static void draw_status_view();
static void draw_status_line(int idx, const std::string& text, bool highlight);
static void draw_touch_keyboard_overlay(const std::string& header);
static bool touch_keyboard_active();
static void touch_keyboard_process_touch();
static void touch_keyboard_end_session();
static const char* menu_edit_label(int idx);
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
static void update_countdown();
static void menu_flash_tick();
static void rx_flash_tick();
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static std::string g_last_reply_text;
static void rebuild_active_bands();
static void schedule_tx_if_idle();
static int64_t s_last_tx_slot_idx = -1000;  // Track last TX slot for retry scheduling
[[maybe_unused]] static bool g_sync_pending = false;
[[maybe_unused]] static int g_sync_delta_ms = 0;
static void enqueue_beacon_cq();
static void qso_load_file_list();
static void qso_load_fetch_file_list();
static void qso_load_entries(const std::string& path);

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter = -1);
static void log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd);
#if !MIC_PROBE_APP
void log_heap(const char* tag) {
  size_t free_sz = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  ESP_LOGI(tag, "HEAP: free=%u min=%u largest=%u", (unsigned)free_sz, (unsigned)min_free, (unsigned)largest);
}
static std::string fd_trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
  while (b > a && (s[b-1] == ' ' || s[b-1] == '\t' || s[b-1] == '\r' || s[b-1] == '\n')) --b;
  return s.substr(a, b - a);
}

static std::string fd_strip_R(const std::string& s) {
  std::string t = fd_trim(s);
  if (t.size() >= 2 && t[0] == 'R' && t[1] == ' ') return fd_trim(t.substr(2));
  return t;
}

static std::string fd_get_section_from_exchange(const std::string& ex) {
  // ex: "1B SCV" (or "R 1B SCV")
  std::string t = fd_strip_R(ex);
  size_t sp = t.find(' ');
  if (sp == std::string::npos) return "DX";
  return fd_trim(t.substr(sp + 1));
}

static void cabrillo_fd_ensure_header(const char* path, const std::string& mycall, const std::string& location) {
  struct stat st;
  if (stat(path, &st) == 0) return;

  FILE* f = fopen(path, "w");
  if (!f) return;

  fprintf(f, "START-OF-LOG: 3.0\n");
  fprintf(f, "CREATED-BY: PaperFT8\n");
  fprintf(f, "CONTEST: ARRL-FIELD-DAY\n");
  fprintf(f, "CALLSIGN: %s\n", mycall.c_str());
  fprintf(f, "CATEGORY-OPERATOR: SINGLE-OP\n");
  fprintf(f, "CATEGORY-TRANSMITTER: ONE\n");
  fprintf(f, "CATEGORY-ASSISTED: NON-ASSISTED\n");
  fprintf(f, "CATEGORY-BAND: ALL\n");
  fprintf(f, "CATEGORY-MODE: MIXED\n");
  fprintf(f, "CATEGORY-POWER: LOW\n");
  fprintf(f, "CATEGORY-STATION: PORTABLE\n");
  fprintf(f, "LOCATION: %s\n", location.c_str());
  fprintf(f, "OPERATORS: %s\n", mycall.c_str());
  fprintf(f, "END-OF-LOG:\n");
  fclose(f);
}

static bool cabrillo_fd_truncate_end_marker(FILE* f) {
  if (!f) return false;

  if (fseek(f, 0, SEEK_END) != 0) return false;
  long file_end = ftell(f);
  if (file_end <= 0) return false;

  const long kMaxTail = 256;
  long tail_start = (file_end > kMaxTail) ? (file_end - kMaxTail) : 0;

  if (fseek(f, tail_start, SEEK_SET) != 0) return false;

  std::string tail;
  tail.resize((size_t)(file_end - tail_start));
  size_t n = fread(tail.data(), 1, tail.size(), f);
  tail.resize(n);

  // Find end of last non-empty line
  size_t line_end = tail.size();
  while (line_end > 0 && (tail[line_end - 1] == '\n' || tail[line_end - 1] == '\r')) {
    line_end--;
  }
  if (line_end == 0) return false;

  size_t line_start = tail.rfind('\n', line_end - 1);
  line_start = (line_start == std::string::npos) ? 0 : (line_start + 1);

  std::string last = tail.substr(line_start, line_end - line_start);
  if (last != "END-OF-LOG:") return false;

  long truncate_at = tail_start + (long)line_start;
  int fd = fileno(f);
  if (fd < 0) return false;
  if (ftruncate(fd, truncate_at) != 0) return false;

  // Seek to new end
  fseek(f, 0, SEEK_END);
  return true;
}

static void cabrillo_fd_append_qso_with_end(const char* path, const std::string& qso_line) {
  FILE* f = fopen(path, "r+");
  if (!f) {
    f = fopen(path, "a+");
    if (!f) return;
  }

  // Remove trailing END-OF-LOG if present
  cabrillo_fd_truncate_end_marker(f);

  // Append QSO and END-OF-LOG
  fseek(f, 0, SEEK_END);

  // Ensure newline separation
  long end = ftell(f);
  if (end > 0) {
    if (fseek(f, -1, SEEK_END) == 0) {
      int c = fgetc(f);
      fseek(f, 0, SEEK_END);
      if (c != '\n') fputc('\n', f);
    } else {
      fseek(f, 0, SEEK_END);
    }
  }

  fprintf(f, "%s\n", qso_line.c_str());
  fprintf(f, "END-OF-LOG:\n");
  fclose(f);
}

// Called by autoseq when an FD QSO completes. We derive freq/time from current radio state
// and use FreeText as our FD exchange (e.g. "1B SCV").
static void log_cabrillo_fd_entry(const std::string& dxcall, const std::string& their_fd_exchange) {
  if (g_cq_type != CqType::CQFD) return;

  const std::string my_fd = fd_strip_R(g_free_text);
  const std::string their_fd = fd_strip_R(their_fd_exchange);

  if (my_fd.empty() || their_fd.empty() || dxcall.empty()) return;

  // Time (UTC assumed as RTC timebase, same as ADIF writer)
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);

  char date_ymd[16];
  snprintf(date_ymd, sizeof(date_ymd), "%04d-%02d-%02d",
           (t.tm_year + 1900) % 10000, (t.tm_mon + 1) % 100, t.tm_mday % 100);

  char time_hhmm[8];
  snprintf(time_hhmm, sizeof(time_hhmm), "%02d%02d", t.tm_hour % 100, t.tm_min % 100);

  // Frequency: use selected band dial frequency (kHz)
  int freq_khz = (int)g_bands[g_band_sel].freq;

  const char* path = "/spiffs/fieldday.log";

  std::string location = fd_get_section_from_exchange(my_fd);
  cabrillo_fd_ensure_header(path, g_call, location);

  char qso_line[128];
  snprintf(qso_line, sizeof(qso_line), "QSO: %d DG %s %s %s %s %s %s",
           freq_khz,
           date_ymd,
           time_hhmm,
           g_call.c_str(),
           my_fd.c_str(),
           dxcall.c_str(),
           their_fd.c_str());

  cabrillo_fd_append_qso_with_end(path, qso_line);
}

#else
static inline void log_heap(const char*) {}
#endif

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter) {
  if (!g_rxtx_log) return;
  if (!log_mutex) return;  // Not initialized yet

  // Prepare log line outside mutex
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char ts[32];
  snprintf(ts, sizeof(ts), "%04d%02d%02d %02d%02d%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  double freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;

  char log_path[64];
  build_rxtx_log_path(log_path, sizeof(log_path));

  // Take mutex for file access
  if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGW(TAG, "RxTxLog mutex timeout");
    return;
  }

  FILE* f = fopen(log_path, "a");
  if (!f) {
    ESP_LOGW(TAG, "RxTxLog open failed: %s", log_path);
    xSemaphoreGive(log_mutex);
    return;
  }

  // For TX, omit SNR and repeat; for RX keep SNR.
  if (dir == 'T') {
    fprintf(f, "%c [%s][%.3f] %s %d\n",
            dir, ts, freq_mhz, text.c_str(), offset_hz);
  } else {
    fprintf(f, "%c [%s][%.3f] %s %d %d\n",
            dir, ts, freq_mhz, text.c_str(), snr, offset_hz);
  }

  fclose(f);
  xSemaphoreGive(log_mutex);
}

static void qso_load_file_list() {
  g_q_files.clear();
  g_q_lines.clear();
  DIR* dir = opendir("/spiffs");
  if (!dir) {
    g_q_lines.push_back("No ADIF logs");
    return;
  }
  struct dirent* ent;
  while ((ent = readdir(dir)) != nullptr) {
    const char* name = ent->d_name;
    size_t len = strlen(name);
    if (len >= 4 && strcasecmp(name + len - 4, ".adi") == 0) {
      g_q_files.emplace_back(name);
    }
  }
  closedir(dir);
  std::sort(g_q_files.begin(), g_q_files.end(), std::greater<std::string>());
  if (g_q_files.empty()) {
    g_q_lines.push_back("No ADIF logs");
    return;
  }
  for (size_t i = 0; i < g_q_files.size(); ++i) {
    g_q_lines.push_back(g_q_files[i]);
  }
}

static void qso_load_fetch_file_list() {
  g_q_files.clear();
  g_q_lines.clear();
  DIR* dir = opendir("/spiffs");
  if (!dir) {
    g_q_lines.push_back("No SPIFFS files");
    return;
  }
  struct dirent* ent;
  while ((ent = readdir(dir)) != nullptr) {
    const char* name = ent->d_name;
    if (!name || name[0] == '.') continue;
    std::string path = std::string("/spiffs/") + name;
    struct stat st;
    if (stat(path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
    g_q_files.emplace_back(name);
  }
  closedir(dir);
  std::sort(g_q_files.begin(), g_q_files.end(), std::greater<std::string>());
  if (g_q_files.empty()) {
    g_q_lines.push_back("No SPIFFS files");
    return;
  }
  for (size_t i = 0; i < g_q_files.size(); ++i) {
    g_q_lines.push_back(g_q_files[i]);
  }
}

static void qso_load_entries(const std::string& path) {
  g_q_lines.clear();
  std::string full = std::string("/spiffs/") + path;
  FILE* f = fopen(full.c_str(), "r");
  if (!f) {
    g_q_lines.push_back("Open fail");
    return;
  }
  char line[256];
  while (fgets(line, sizeof(line), f)) {
    std::string s(line);
    std::string s_lower = s;
    std::transform(s_lower.begin(), s_lower.end(), s_lower.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (s_lower.find("<call:") == std::string::npos) continue;
    auto get_field = [&](const std::string& tag)->std::string {
      size_t p = s_lower.find("<" + tag);
      if (p == std::string::npos) return "";
      size_t gt = s.find('>', p);
      if (gt == std::string::npos) return "";
      size_t end_space = s.find(' ', gt + 1);
      size_t end_tag = s.find('<', gt + 1);
      size_t end = s.size();
      if (end_space != std::string::npos && end_space < end) end = end_space;
      if (end_tag != std::string::npos && end_tag < end) end = end_tag;
      return s.substr(gt + 1, end - gt - 1);
    };
    auto format_report = [](const std::string& raw, char prefix)->std::string {
      if (raw.empty()) return std::string(1, prefix) + "---";
      char* end = nullptr;
      long v = strtol(raw.c_str(), &end, 10);
      if (!end || end == raw.c_str() || *end != '\0') {
        return std::string(1, prefix) + "---";
      }
      if (v < -99) v = -99;
      if (v > 99) v = 99;
      char out[8];
      std::snprintf(out, sizeof(out), "%c%+03ld", prefix, v);
      return out;
    };
    auto trim_head = [](const std::string& in, size_t max_len)->std::string {
      if (in.size() <= max_len) return in;
      if (max_len == 0) return "";
      if (max_len == 1) return ">";
      return in.substr(0, max_len - 1) + ">";
    };
    std::string call = get_field("call:");
    std::string time_on = get_field("time_on:");
    std::string freq = get_field("freq:");
    std::string rst_rcvd = get_field("rst_rcvd:");
    std::string rst_sent = get_field("rst_sent:");
    std::string band = freq;
    if (!freq.empty()) {
      // crude map: take MHz and map to band name from our band list
      double mhz = atof(freq.c_str());
      for (const auto& b : g_bands) {
        double bm = b.freq * 0.001;
        if (fabs(bm - mhz) < 0.1) { band = b.name; break; }
      }
    }
    if (time_on.size() >= 4) {
      time_on = time_on.substr(0,4);
      time_on.insert(2, ":");
    }
    if (time_on.size() != 5) time_on = "??:??";
    if (call.empty()) call = "?";
    if (band.empty()) band = freq.empty() ? "?" : freq;

    const std::string call_disp = trim_head(call, 11);
    const std::string band_disp = trim_head(band, 6);
    const std::string rcvd_disp = format_report(rst_rcvd, 'R');
    const std::string sent_disp = format_report(rst_sent, 'S');
    std::string call_field = call_disp;
    if (call_field.size() < 11) {
      call_field.append(11 - call_field.size(), ' ');
    }

    g_q_lines.push_back(time_on + " " + band_disp + " " + call_field + " " + rcvd_disp + " " + sent_disp);
  }
  fclose(f);
  if (g_q_lines.empty()) g_q_lines.push_back("No QSOs");
}

static void log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd) {
  // Protect ADIF file access with the same mutex used for RxTxLog.
  // log_qso_if_needed can be called from the UAC streaming task (core 1)
  // via generate_response, so concurrent writes must be serialized.
  if (!log_mutex) return;
  if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    ESP_LOGW(TAG, "ADIF mutex timeout");
    return;
  }

  // Build file name based on current date
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char date[16];
  int year = t.tm_year + 1900;
  int month = t.tm_mon + 1;
  int day = t.tm_mday;
  snprintf(date, sizeof(date), "%04d%02d%02d", year % 10000, month % 100, day % 100);
  char path[64];
  snprintf(path, sizeof(path), "/spiffs/%s.adi", date);

  bool need_header = false;
  struct stat st;
  if (stat(path, &st) != 0) need_header = true;

  FILE* f = fopen(path, "a");
  if (!f) {
    ESP_LOGW(TAG, "ADIF open failed");
    xSemaphoreGive(log_mutex);
    return;
  }
  if (need_header) {
    fprintf(f, "ADIF EXPORT\n<eoh>\n");
  }

  char time_on[16];
  int hour = t.tm_hour;
  int min = t.tm_min;
  int sec = t.tm_sec;
  snprintf(time_on, sizeof(time_on), "%02d%02d%02d", hour % 100, min % 100, sec % 100);
  double freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;
  char freq_str[16];
  snprintf(freq_str, sizeof(freq_str), "%.3f", freq_mhz);

  std::string comment_expanded = g_comment1;
  auto repl = [](std::string& s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
  };
  // Expand placeholders using current radio string
  auto radio_name_local = [](RadioType r) {
    switch (r) {
      case RadioType::TRUSDX: return "QMX";
      case RadioType::QMX: return "QMX";
      default: return "None";
    }
  };
  repl(comment_expanded, "/Radio", radio_name_local(g_radio));
  // Build rst_sent/rst_rcvd fragments — omit when -99 (no data),
  // matching DXFT8 reference behavior (ADIF.c omits when value is 0).
  char rst_sent_buf[32] = "";
  char rst_rcvd_buf[32] = "";
  if (rst_sent != -99) {
    snprintf(rst_sent_buf, sizeof(rst_sent_buf), "<rst_sent:%d>%d ",
             (int)snprintf(nullptr, 0, "%d", rst_sent), rst_sent);
  }
  if (rst_rcvd != -99) {
    snprintf(rst_rcvd_buf, sizeof(rst_rcvd_buf), "<rst_rcvd:%d>%d ",
             (int)snprintf(nullptr, 0, "%d", rst_rcvd), rst_rcvd);
  }
  fprintf(f, "<call:%zu>%s <gridsquare:%zu>%s <mode:3>FT8<qso_date:8>%s <time_on:6>%s <freq:%zu>%s <station_callsign:%zu>%s <my_gridsquare:%zu>%s %s%s<comment:%zu>%s <eor>\n",
          dxcall.size(), dxcall.c_str(),
          dxgrid.size(), dxgrid.c_str(),
          date, time_on,
          strlen(freq_str), freq_str,
          g_call.size(), g_call.c_str(),
          g_grid.size(), g_grid.c_str(),
          rst_sent_buf, rst_rcvd_buf,
          comment_expanded.size(), comment_expanded.c_str());
  fclose(f);
  xSemaphoreGive(log_mutex);
}


static void ensure_usb() {
  if (usb_ready) return;
  usb_serial_jtag_driver_config_t cfg = {
    .tx_buffer_size = 1024,
    .rx_buffer_size = 4096,
  };
  if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
    usb_ready = true;
  }
}

static bool uart0_last_was_cr = false;

static void poll_uart0_keys() {
  if (!s_key_inject_queue) return;
  // Read directly from UART0 hardware FIFO — no driver needed.
  // The console (sdkconfig) already has UART0 configured on TX=GPIO1, RX=GPIO2.
  uart_dev_t *hw = UART_LL_GET_HW(0);
  while (true) {
    uint32_t avail = uart_ll_get_rxfifo_len(hw);
    if (avail == 0) break;
    if (avail > 64) avail = 64;
    uint8_t buf[64];
    uart_ll_read_rxfifo(hw, buf, avail);
    for (uint32_t i = 0; i < avail; i++) {
      char ch = (char)buf[i];
      // CR/LF handling: \r -> Enter, \n after \r -> skip (avoid double Enter)
      if (ch == '\r') {
        char enter = '\n';
        xQueueSend(s_key_inject_queue, &enter, 0);
        uart0_last_was_cr = true;
      } else if (ch == '\n' && uart0_last_was_cr) {
        uart0_last_was_cr = false;  // skip LF after CR
      } else {
        uart0_last_was_cr = false;
        xQueueSend(s_key_inject_queue, &ch, 0);
      }
    }
  }
}

static void host_write_str(const std::string& s) {
  ensure_usb();
  if (usb_ready) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(s.data());
    size_t remaining = s.size();
    while (remaining > 0) {
      size_t chunk = remaining;
      if (chunk > 256) chunk = 256;
      int written = usb_serial_jtag_write_bytes(p, chunk, portMAX_DELAY);
      if (written <= 0) break;
      p += written;
      remaining -= written;
    }
  }
}

struct WAVHeader {
  char riff[4];
  uint32_t file_size;
  char wave[4];
  char fmt[4];
  uint32_t fmt_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data[4];
  uint32_t data_size;
};

[[maybe_unused]] static esp_err_t decode_wav(const char* path) {
  ESP_LOGI(TAG, "Decoding %s", path);
  FILE* f = fopen(path, "rb");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s", path);
    return ESP_FAIL;
  }

  WAVHeader hdr;
  if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
    ESP_LOGE(TAG, "Failed to read WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (memcmp(hdr.riff, "RIFF", 4) != 0 || memcmp(hdr.wave, "WAVE", 4) != 0) {
    ESP_LOGE(TAG, "Invalid WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (hdr.sample_rate != FT8_SAMPLE_RATE || hdr.num_channels != 1) {
    ESP_LOGE(TAG, "WAV must be mono %d Hz (got %u Hz, %u ch)", FT8_SAMPLE_RATE, hdr.sample_rate, hdr.num_channels);
    fclose(f);
    return ESP_FAIL;
  }

  const int bytes_per_sample = hdr.bits_per_sample / 8;

  monitor_config_t mon_cfg;
  mon_cfg.f_min = 200.0f;
  mon_cfg.f_max = 3000.0f;
  mon_cfg.sample_rate = FT8_SAMPLE_RATE;
  mon_cfg.time_osr = 2;
  mon_cfg.freq_osr = 2;
  mon_cfg.protocol = FTX_PROTOCOL_FT8;

  monitor_t mon;
  monitor_init(&mon, &mon_cfg);
  monitor_reset(&mon);

  float* chunk = (float*)malloc(sizeof(float) * mon.block_size);
  if (!chunk) {
    ESP_LOGE(TAG, "Chunk alloc failed");
    fclose(f);
    monitor_free(&mon);
    return ESP_ERR_NO_MEM;
  }

  while (!feof(f)) {
    int read_samples = 0;
    while (read_samples < mon.block_size && !feof(f)) {
      float sample_value = 0.0f;
      if (bytes_per_sample == 1) {
        int s = fgetc(f);
        if (s == EOF) break;
        sample_value = ((float)s - 128.0f) / 128.0f;
      } else if (bytes_per_sample == 2) {
        int low = fgetc(f);
        int high = fgetc(f);
        if (low == EOF || high == EOF) break;
        int16_t s = (int16_t)((high << 8) | low);
        sample_value = (float)s / 32768.0f;
      }
      chunk[read_samples++] = sample_value;
    }
    if (read_samples == 0) break;
    for (int i = read_samples; i < mon.block_size; ++i) {
      chunk[i] = 0.0f;
    }

    // Simple per-block AGC to ~0.1 target level
    double acc = 0.0;
    for (int i = 0; i < mon.block_size; ++i) acc += fabsf(chunk[i]);
    float level = (float)(acc / mon.block_size);
    float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
    if (gain < 0.1f) gain = 0.1f;
    if (gain > 10.0f) gain = 10.0f;
    for (int i = 0; i < mon.block_size; ++i) {
      chunk[i] *= gain;
    }

    monitor_process(&mon, chunk);
  }

  free(chunk);
  fclose(f);

  if (mon.wf.num_blocks == 0) {
    ESP_LOGW(TAG, "No audio blocks processed");
    monitor_free(&mon);
    return ESP_FAIL;
  }
  decode_monitor_results(&mon, &mon_cfg, false); // defer UI to main loop on core1
  monitor_free(&mon);

  return ESP_OK;
}

static void redraw_tx_view() {
  // Get QSO states from autoseq for display
  std::vector<std::string> qtext;
  autoseq_get_qso_states(qtext);

  std::vector<bool> marks(qtext.size(), false);  // No delete marks with autoseq
  std::vector<int> slots;

  // Slot color for pending TX
  slots.push_back(g_pending_tx_valid ? (g_pending_tx.slot_id & 1) : 0);
  // All QSO entries use their context's slot
  for (size_t i = 0; i < qtext.size(); ++i) {
    slots.push_back(0);  // Default to even; autoseq manages internally
  }

  std::string next_line;
  if (g_pending_tx_valid && !g_pending_tx.text.empty()) {
    // Use scheduled TX text if available
    next_line = g_pending_tx.text;
  } else {
    // Fall back to autoseq's next TX (for display when TX not yet scheduled)
    autoseq_get_next_tx(next_line);
  }

  ui_draw_tx(next_line, qtext, tx_page, -1, marks, slots);
}

static void draw_band_view() {
  if (band_edit_idx >= 0 && band_edit_idx < (int)g_bands.size()) {
    draw_touch_keyboard_overlay(std::string("Band ") + g_bands[band_edit_idx].name + ": " + band_edit_buffer);
    return;
  }

  std::vector<std::string> lines;
  lines.reserve(g_bands.size());
  for (size_t i = 0; i < g_bands.size(); ++i) {
    lines.push_back(std::string(g_bands[i].name) + ": " + std::to_string(g_bands[i].freq));
  }
  ui_draw_list(lines, band_page, band_edit_idx);
}

static const char* beacon_name(BeaconMode m) {
  switch (m) {
    case BeaconMode::OFF: return "OFF";
    case BeaconMode::EVEN: return "EVEN";
    //case BeaconMode::EVEN2: return "EVEN2";
    case BeaconMode::ODD: return "ODD";
    //case BeaconMode::ODD2: return "ODD2";
  }
  return "OFF";
}

static const char* cq_type_name(CqType t) {
  switch (t) {
    case CqType::CQ: return "CQ";
    case CqType::CQSOTA: return "CQ SOTA";
    case CqType::CQPOTA: return "CQ POTA";
    case CqType::CQQRP: return "CQ QRP";
    case CqType::CQFD: return "CQ FD";
    case CqType::CQFREETEXT: return "FreeText";
  }
  return "CQ";
}

static const char* offset_name(OffsetSrc o) {
  switch (o) {
    case OffsetSrc::RANDOM: return "Random";
    case OffsetSrc::CURSOR: return "Fixed";
    case OffsetSrc::RX: return "RX";
  }
  return "Random";
}

static const char* radio_name(RadioType r) {
  switch (r) {
    case RadioType::NONE: return "QMX";
    case RadioType::TRUSDX: return "QMX";
    case RadioType::QMX: return "QMX";
  }
  return "None";
}

static std::string expand_comment1() {
  std::string out = g_comment1;
  auto repl = [](std::string& s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
  };
  repl(out, "/Radio", radio_name(g_radio));
  return out;
}

static void rebuild_ignore_prefixes() {
  g_ignore_prefixes.clear();
  std::istringstream iss(g_ignore_prefix_text);
  std::string tok;
  while (iss >> tok) {
    std::string norm = normalize_call_token(tok);
    if (norm.empty()) continue;
    bool duplicate = false;
    for (const auto& existing : g_ignore_prefixes) {
      if (existing == norm) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) g_ignore_prefixes.push_back(norm);
  }
}

static bool ignorelist_matches_normalized_dxcall(const std::string& dxcall_norm) {
  if (dxcall_norm.empty()) return false;
  for (const auto& prefix : g_ignore_prefixes) {
    if (!prefix.empty() && dxcall_norm.rfind(prefix, 0) == 0) return true;
  }
  return false;
}

static std::string clamp_ignore_prefix_text(const std::string& s) {
  if (s.size() <= kIgnorePrefixTextMaxLen) return s;
  return s.substr(0, kIgnorePrefixTextMaxLen);
}

static std::string menu_sleep_batt_line() {
  int level = (int)M5.Power.getBatteryLevel();
  if (level < 0 || level > 100) level = 0;
  char buf[32];
  snprintf(buf, sizeof(buf), "Sleep/Batt %d%%", level);
  return buf;
}

static std::string elide_right(const std::string& s, size_t max_len = 22) {
  if (s.size() <= max_len) return s;
  if (max_len <= 3) return s.substr(s.size() - max_len);
  return std::string("...") + s.substr(s.size() - (max_len - 3));
}

static std::string head_trim(const std::string& s, size_t max_len = 35) {
  if (s.size() <= max_len) return s;
  if (max_len == 0) return "";
  if (max_len == 1) return ">";
  return s.substr(0, max_len - 1) + ">";
}

static std::string highlight_pos(const std::string& s, int pos) {
  if (pos < 0 || pos >= (int)s.size()) return s;
  std::string out;
  out.reserve(s.size() + 2);
  out.append(s, 0, pos);
  out.push_back('[');
  out.push_back(s[pos]);
  out.push_back(']');
  out.append(s, pos + 1, std::string::npos);
  return out;
}

static bool rtc_set_from_strings() {
  int y, M, d, h, m, s;
  if (sscanf(g_date.c_str(), "%d-%d-%d", &y, &M, &d) != 3) return false;
  if (sscanf(g_time.c_str(), "%d:%d:%d", &h, &m, &s) != 3) return false;
  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = M - 1;
  t.tm_mday = d;
  t.tm_hour = h;
  t.tm_min = m;
  t.tm_sec = s;
  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) return false;
  rtc_epoch_base = epoch;
  rtc_ms_start = esp_timer_get_time() / 1000;
  rtc_last_update = rtc_ms_start;
  rtc_valid = true;
  return true;
}

// Initialize soft RTC from hardware RTC (persists through deep sleep)
// Applies compensation if we have valid sleep epoch data
static bool rtc_init_from_hw() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL) != 0) return false;

  // Check if hardware RTC has valid time (year > 2020)
  struct tm t;
  localtime_r(&tv.tv_sec, &t);
  if (t.tm_year + 1900 < 2020) return false;

  time_t compensated_now = tv.tv_sec;

  // Apply compensation if we have valid sleep data
  if (g_rtc_sleep_epoch > 0 && tv.tv_sec > g_rtc_sleep_epoch) {
    int64_t raw_elapsed = tv.tv_sec - g_rtc_sleep_epoch;
    int64_t actual_elapsed = raw_elapsed;

    // Apply compensation: actual = raw * 10000 / (10000 + comp)
    if (g_rtc_comp != 0) {
      actual_elapsed = raw_elapsed * 10000 / (10000 + g_rtc_comp);
    }

    // Fixed 1s boot delay: deep sleep entry → wake → gettimeofday
    static constexpr int64_t BOOT_DELAY_SEC = 1;
    compensated_now = g_rtc_sleep_epoch + actual_elapsed + BOOT_DELAY_SEC;

    ESP_LOGI(TAG, "RTC wake: raw_elapsed=%lld, actual_elapsed=%lld, comp=%d, boot_adj=%lld",
             (long long)raw_elapsed, (long long)actual_elapsed, g_rtc_comp,
             (long long)BOOT_DELAY_SEC);

    // Clear sleep epoch after use (one-time compensation)
    g_rtc_sleep_epoch = 0;
  }

  rtc_epoch_base = compensated_now;
  // Account for sub-second offset: tv.tv_usec tells us how far past the
  // whole second we are, so rewind rtc_ms_start by that amount.
  rtc_ms_start = esp_timer_get_time() / 1000 - tv.tv_usec / 1000;
  rtc_last_update = rtc_ms_start;
  rtc_valid = true;

  // Update g_date/g_time strings from compensated time
  localtime_r(&compensated_now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;

  ESP_LOGI(TAG, "RTC initialized: %s %s (compensated=%s)",
           g_date.c_str(), g_time.c_str(),
           (g_rtc_comp != 0) ? "yes" : "no");
  return true;
}

// Sync hardware RTC from soft RTC (call after FT8 time sync)
static void rtc_sync_to_hw() {
  if (!rtc_valid) return;

  int64_t elapsed_ms = esp_timer_get_time() / 1000 - rtc_ms_start;
  time_t now = rtc_epoch_base + (elapsed_ms / 1000);
  int64_t rem_ms = elapsed_ms % 1000;
  if (rem_ms < 0) {
    rem_ms += 1000;
    --now;
  }
  struct timeval tv = { .tv_sec = now, .tv_usec = static_cast<suseconds_t>(rem_ms * 1000) };
  settimeofday(&tv, NULL);

  // Also push system time into external RTC (BM8563/PCF8563-compatible backend).
  if (M5.Rtc.isEnabled()) {
    // External RTC is second-resolution; round instead of floor to avoid steady lag.
    time_t rtc_sec = now + ((tv.tv_usec >= 500000) ? 1 : 0);
    struct tm t_utc;
    gmtime_r(&rtc_sec, &t_utc);
    M5.Rtc.setDateTime(&t_utc);
  }

  ESP_LOGI(TAG, "Hardware RTC synced from soft RTC");
}

static void rtc_update_strings() {
  if (!rtc_valid) return;
  struct tm t;
  time_t now = rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
  localtime_r(&now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;
}

int64_t rtc_now_ms() {
  if (!rtc_valid) {
    return esp_timer_get_time() / 1000;
  }
  return (int64_t)rtc_epoch_base * 1000 + (esp_timer_get_time() / 1000 - rtc_ms_start);
}

static void rtc_tick() {
  if (!rtc_valid) {
    rtc_set_from_strings();
    if (!rtc_valid) return;
  }
  int64_t now_ms = esp_timer_get_time() / 1000;
  if (now_ms - rtc_last_update >= 1000) {
    rtc_last_update += 1000; // Increment by interval to prevent drift accumulation
    if (status_edit_idx != 5) { // keep time ticking unless editing time
      std::string old_date = g_date;
      std::string old_time = g_time;
      rtc_update_strings();
      if (ui_mode == UIMode::STATUS && status_edit_idx == -1) {
        if (old_date != g_date) {
          draw_status_line(4, std::string("Date: ") + g_date, false);
        }
        if (old_time != g_time) {
          draw_status_line(5, std::string("Time: ") + g_time, false);
        }
      }
    }
  }
}

static void update_countdown() {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / 15000;
  int64_t slot_ms = now_ms % 15000;
  static int64_t last_slot_idx = -1;
  static int last_sec = -1;
  int sec = (int)(slot_ms / 1000);
  if (slot_idx != last_slot_idx || sec != last_sec) {
    float frac = (float)slot_ms / 15000.0f;
    bool even = (slot_idx % 2) == 0;
    ui_draw_countdown(frac, even, g_offset_hz);
    last_slot_idx = slot_idx;
    last_sec = sec;
  }
}

// Forward declarations for single-threaded TX state machine
static void tx_start(int skip_tones);
static void tx_tick();

// Slot boundary check - called from main loop
// Matches reference project: tick after TX slot ends, TX trigger at slot start
static void check_slot_boundary() {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / 15000;
  int slot_ms = (int)(now_ms % 15000);
  int slot_parity = (int)(slot_idx & 1);

  // Detect slot boundary (parity change)
  if (slot_parity != g_last_slot_parity) {
    g_last_slot_parity = slot_parity;
  }

  // Call tick AFTER TX has completed (not while TX is still active)
  // This ensures autoseq_tick() operates on the correct completed TX entry
  if (g_was_txing && !g_tx_active) {
    ESP_LOGI(TAG, "TX completed, calling tick (slot %lld, parity %d)",
             (long long)slot_idx, slot_parity);
    autoseq_tick(slot_idx, slot_parity, 0);
    g_was_txing = false;
    g_tx_view_dirty = true;
  }

  // TX trigger: check if we should start TX in this slot
  // Conditions: qso_xmit flag set, correct parity, early enough in slot, not already TXing,
  // and decode must be complete (TX is always triggered by decode results)
  if (g_qso_xmit &&
      g_target_slot_parity == slot_parity &&
      slot_ms < 4000 &&
      !g_tx_active &&
      !g_decode_in_progress) {

    ESP_LOGI(TAG, "TX trigger: starting TX in slot %lld (parity %d)",
             (long long)slot_idx, slot_parity);

    // Calculate skip_tones for partial slot
    int skip_tones = slot_ms / 160;
    if (skip_tones < 79) {
      // Only proceed if we have a valid pending TX
      // NOTE: Don't clear g_qso_xmit until we're sure g_pending_tx is valid.
      // This avoids a race condition where decode_monitor_results is still
      // writing g_pending_tx on core 1 while we read it on core 0.
      if (g_pending_tx_valid && !g_pending_tx.text.empty()) {
        g_qso_xmit = false;  // Clear flag only AFTER validation succeeds
        g_was_txing = true;  // Set IMMEDIATELY when TX starts (prevents decode_monitor_results from re-setting flags)

        // Compute actual TX offset now (before logging) based on offset_src setting
        int actual_offset;
        if (g_offset_src == OffsetSrc::CURSOR) {
          actual_offset = g_offset_hz;
        } else if (g_offset_src == OffsetSrc::RX &&
                   g_pending_tx.offset_hz > 0 &&
                   g_pending_tx.text.rfind("CQ ", 0) != 0) {
          actual_offset = g_pending_tx.offset_hz;
        } else {
          // RANDOM mode or CQ in RX mode: generate random offset
          actual_offset = 500 + (int)(esp_random() % 2001);
        }
        g_pending_tx.offset_hz = actual_offset;  // Store for tx_start to use
        log_rxtx_line('T', 0, actual_offset, g_pending_tx.text, g_pending_tx.repeat_counter);
        tx_start(skip_tones);
      }
    }
  }
}

  static void menu_flash_tick() {
    if (menu_flash_idx < 0) return;
    int64_t now = rtc_now_ms();
    if (now >= menu_flash_deadline) {
      menu_flash_idx = -1;
      if (ui_mode == UIMode::MENU && !menu_long_edit && menu_edit_idx < 0) {
        draw_menu_view();
      }
  }
}

static void rx_flash_tick() {
  if (rx_flash_idx < 0) return;
  int64_t now = rtc_now_ms();
  if (now >= rx_flash_deadline) {
    const int flash_idx = rx_flash_idx;
    rx_flash_idx = -1;
    rx_flash_deadline = 0;
    if (ui_mode == UIMode::RX && flash_idx >= 0) {
      ui_flash_rx_line(flash_idx, false);
    }
  }
}

static UIMode swipe_next_mode(UIMode current, int dir) {
  // Order: R -> T -> Q -> S (MENU)
  static const UIMode order[] = {UIMode::RX, UIMode::TX, UIMode::QSO, UIMode::MENU};
  int idx = 0;
  for (int i = 0; i < 4; ++i) {
    if (order[i] == current) { idx = i; break; }
  }
  int next = (idx + dir + 4) % 4;
  return order[next];
}

static char touch_command_to_key(int command_idx) {
  switch (command_idx) {
    case 0: {  // "12"
      ui_toggle_countdown_enabled();
      debug_log_line(ui_is_countdown_enabled() ? "Countdown ON" : "Countdown OFF");
      return 0;
    }
    case 1: return '`';  // ESC
    case 2: return 'S';
    case 3: return 'R';
    case 4: return 'T';
    case 5: return 'Q';
    case 6: return 'M';
    case 7: return 'B';
    case 8: return 'C';
    case 9: return 'D';
    case 10: return ';'; // prev/up
    case 11: return '.'; // next/down
    default: return 0;
  }
}

static bool is_startup_direct_mode_key(char c) {
  const char k = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  switch (k) {
    case 'S':
    case 'R':
    case 'T':
    case 'Q':
    case 'M':
    case 'B':
    case 'F':
    case 'C':
    case 'D':
      return true;
    default:
      return false;
  }
}

static bool control_mode_blocked_by_uac() {
  const uac_stream_state_t st = uac_get_state();
  return st == UAC_STATE_CONNECTED || st == UAC_STATE_STREAMING;
}

static bool poll_gesture(GestureEvent& out) {
  static bool tracking = false;
  static int start_x = 0;
  static int start_y = 0;
  static int last_x = 0;
  static int last_y = 0;

  out = GestureEvent{};

  auto d = M5.Touch.getDetail();
  if (d.wasPressed()) {
    tracking = true;
    start_x = d.x;
    start_y = d.y;
    last_x = d.x;
    last_y = d.y;
    return false;
  }
  if (tracking && d.isPressed()) {
    last_x = d.x;
    last_y = d.y;
    return false;
  }
  if (tracking && d.wasReleased()) {
    tracking = false;
    int dx = last_x - start_x;
    int dy = last_y - start_y;
    const int tap_thresh = 8;
    const int swipe_thresh = 24;
    if (abs(dx) < tap_thresh && abs(dy) < tap_thresh) {
      int cmd = ui_command_hit_test(last_x, last_y);
      if (cmd >= 0) {
        out.action = GestureAction::TapCommand;
        out.command_idx = cmd;
      } else {
        int line = ui_text_line_hit_test(last_x, last_y);
        if (line >= 0) {
          out.action = GestureAction::TapLine;
          out.line_idx = line;
        } else {
          const UiLayout& lay = ui_layout();
          if (lay.mode_box.contains(last_x, last_y)) {
            out.action = GestureAction::TapMode;
          }
        }
      }
      out.x = last_x;
      out.y = last_y;
      return out.action != GestureAction::None;
    }
    if (abs(dx) >= abs(dy) && abs(dx) > swipe_thresh) {
      out.action = (dx > 0) ? GestureAction::SwipeRight : GestureAction::SwipeLeft;
      return true;
    }
    if (abs(dy) > swipe_thresh) {
      out.action = (dy > 0) ? GestureAction::SwipeDown : GestureAction::SwipeUp;
      return true;
    }
  }
  return false;
}

static void apply_pending_sync() {}

static int band_number_from_name(const std::string& name) {
  int num = 0;
  for (char c : name) {
    if (c >= '0' && c <= '9') {
      num = num * 10 + (c - '0');
    } else {
      break;
    }
  }
  return num;
}

static void rebuild_active_bands() {
  std::string cleaned = g_active_band_text;
  for (char& c : cleaned) {
    if (c == ',' || c == '/' || c == '\\' || c == ';') c = ' ';
    if (c == 'm' || c == 'M') c = ' ';
  }
  std::istringstream iss(cleaned);
  std::vector<int> bands;
  int v;
  while (iss >> v) {
    if (v <= 0) continue;
    // match to g_bands by number prefix
    for (size_t i = 0; i < g_bands.size(); ++i) {
      if (band_number_from_name(g_bands[i].name) == v) {
        if (std::find(bands.begin(), bands.end(), (int)i) == bands.end()) {
          bands.push_back((int)i);
        }
        break;
      }
    }
  }
  if (bands.empty()) {
    bands.resize(g_bands.size());
    for (size_t i = 0; i < g_bands.size(); ++i) bands[i] = (int)i;
  }
  g_active_band_indices = bands;
  if (std::find(g_active_band_indices.begin(), g_active_band_indices.end(), g_band_sel) == g_active_band_indices.end()) {
    g_band_sel = g_active_band_indices[0];
  }
  // normalize text
  std::ostringstream oss;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (i) oss << ' ';
    oss << band_number_from_name(g_bands[g_active_band_indices[i]].name);
  }
  g_active_band_text = oss.str();
}

static void update_autoseq_cq_type() {
  AutoseqCqType t = AutoseqCqType::CQ;
  switch (g_cq_type) {
    case CqType::CQSOTA: t = AutoseqCqType::SOTA; break;
    case CqType::CQPOTA: t = AutoseqCqType::POTA; break;
    case CqType::CQQRP:  t = AutoseqCqType::QRP;  break;
    case CqType::CQFD:   t = AutoseqCqType::FD;   break;
    case CqType::CQFREETEXT: t = AutoseqCqType::FREETEXT; break;
    default: t = AutoseqCqType::CQ; break;
  }
  const std::string& ft =
    (g_cq_type == CqType::CQFREETEXT || g_cq_type == CqType::CQFD) ? g_free_text : g_cq_freetext;
  autoseq_set_cq_type(t, ft);
}

static void advance_active_band(int delta) {
  if (g_active_band_indices.empty()) rebuild_active_bands();
  if (g_active_band_indices.empty()) return;
  int pos = 0;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (g_active_band_indices[i] == g_band_sel) { pos = (int)i; break; }
  }
  int n = (int)g_active_band_indices.size();
  pos = (pos + delta + n) % n;
  g_band_sel = g_active_band_indices[pos];
}

static void fft_waterfall_tx_tone(uint8_t tone) {
  // Map tone 0-7 to screen width and push a bright bin
  std::array<uint8_t, 896> row{};
  int pos = (int)((tone * row.size()) / 8);
  if (pos < 0) pos = 0;
  if (pos >= (int)row.size()) pos = (int)row.size() - 1;
  row[pos] = 200;
  ui_push_waterfall_row(row.data(), (int)row.size());
}

[[maybe_unused]] static bool is_grid4(const std::string& s) {
  if (s.size() != 4) return false;
  auto is_letter = [](char c){ return c >= 'A' && c <= 'R'; };
  auto is_digitc = [](char c){ return c >= '0' && c <= '9'; };
  return is_letter(toupper((unsigned char)s[0])) &&
         is_letter(toupper((unsigned char)s[1])) &&
         is_digitc(s[2]) &&
         is_digitc(s[3]);
}

[[maybe_unused]] static int parse_report_snr(const std::string& f3) {
  if (f3.empty()) return -99;
  std::string s = f3;
  if (!s.empty() && (s[0] == 'R' || s[0] == 'r')) {
    s = s.substr(1);
  }
  if (s.empty()) return -99;
  bool neg = false;
  size_t idx = 0;
  if (s[0] == '+' || s[0] == '-') {
    neg = (s[0] == '-');
    idx = 1;
  }
  int val = 0;
  bool found = false;
  for (; idx < s.size(); ++idx) {
    char c = s[idx];
    if (c < '0' || c > '9') break;
    val = val * 10 + (c - '0');
    found = true;
    if (val > 99) break;
  }
  if (!found) return -99;
  if (neg) val = -val;
  return val;
}

void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui) {
  const int max_cand = 50;
  static ftx_candidate_t candidates[max_cand];
  int num_candidates = ftx_find_candidates(&mon->wf, max_cand, candidates, 5);
  ESP_LOGI(TAG, "Candidates found: %d", num_candidates);


  // ---- slot index + once-per-slot hashtable maintenance ----
  int64_t slot_idx = -1;
  if (g_decode_slot_idx >= 0) {
    slot_idx = g_decode_slot_idx;
  } else {
    slot_idx = rtc_now_ms() / 15000LL;
  }
  int slot_id = (int)(slot_idx & 1);

  // Age callsign hash table once per slot
  static int64_t s_last_aged_slot = -1;
  if (slot_idx != s_last_aged_slot) {
    s_last_aged_slot = slot_idx;
    hashtable_age_all();
  }

  // ---- estimate noise floor ----
  float noise_db = -120.0f;
  if (mon->wf.mag && mon->wf.num_blocks > 0) {
    const size_t total = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
    static uint32_t hist[256];
    memset(hist, 0, sizeof(hist));               // <-- FIX

    for (size_t i = 0; i < total; ++i) hist[mon->wf.mag[i]]++;

    uint64_t target = total * 25 / 100;          // <-- use lower percentile than median
    uint64_t accum = 0;
    int noise_scaled = 0;
    for (int v = 0; v < 256; ++v) {
      accum += hist[v];
      if (accum >= target) { noise_scaled = v; break; }
    }
    noise_db = 0.5f * ((float)noise_scaled - 240.0f);
  }

  //static float snr_to_2500 = 0.0f;
  //static bool snr_to_2500_init = false;

  //if (!snr_to_2500_init) {
  //  float bw_eff = 1.5f / (mon->symbol_period * cfg->freq_osr); // Hz
  //  snr_to_2500 = 10.0f * log10f(bw_eff / 2500.0f);
  //  snr_to_2500_init = true;
  //}

  auto to_upper = [](std::string s) {
    for (auto& ch : s) ch = (char)toupper((unsigned char)ch);
    return s;
  };
  std::string mycall_up = to_upper(g_call);

  auto fill_fields_from_text = [&](UiRxLine& line) {
    // Split tokens
    std::vector<std::string> toks;
    {
      std::istringstream iss(line.text);
      std::string tok;
      while (iss >> tok) toks.push_back(tok);
    }

    auto is_digits = [](const std::string& s) {
      return !s.empty() && std::all_of(s.begin(), s.end(),
        [](char c){ return c >= '0' && c <= '9'; });
    };
    auto is_alpha = [](const std::string& s) {
      return !s.empty() && std::all_of(s.begin(), s.end(),
        [](char c){ return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); });
    };

    line.field1.clear(); line.field2.clear(); line.field3.clear();

    // Heuristic: CQ <num/word> CALL GRID  (e.g. CQ DX W1XYZ FN31)
    if (!toks.empty() && toks[0] == "CQ" && toks.size() >= 2) {
      bool short_token = (toks[1].size() <= 3 && is_digits(toks[1])) ||
                         (toks[1].size() <= 4 && is_alpha(toks[1]));
      if (short_token) {
        line.field1 = toks[1];
        if (toks.size() > 2) line.field2 = toks[2];

        // field3 = remainder starting at toks[3]
        if (toks.size() > 3) {
          line.field3.clear();
          for (size_t i = 3; i < toks.size(); ++i) {
            if (i > 3) line.field3.push_back(' ');
            line.field3 += toks[i];
          }
        }
        return;
      }
    }

    // Default: first 2 tokens + remainder as field3
    if (!toks.empty()) line.field1 = toks[0];
    if (toks.size() > 1) line.field2 = toks[1];
    if (toks.size() > 2) {
      line.field3.clear();
      for (size_t i = 2; i < toks.size(); ++i) {
        if (i > 2) line.field3.push_back(' ');
        line.field3 += toks[i];
      }
    }
    
  };

  // ---- local message de-duplication like reference decode() ----
  // Dedupe based on message.hash and payload bytes.
  const int kMaxDecoded = 50; // keep <= max_cand
  static ftx_message_t decoded[kMaxDecoded];
  static ftx_message_t* decoded_hashtable[kMaxDecoded];
  for (int i = 0; i < kMaxDecoded; ++i) decoded_hashtable[i] = nullptr;
  int num_decoded = 0;

  std::vector<UiRxLine> ui_lines;
  ui_lines.reserve(32);
  std::vector<float> time_offsets;
  time_offsets.reserve(32);

  if (num_candidates <= 0) {
    ESP_LOGW(TAG, "No candidates found");
    g_rx_lines.clear();
    if (update_ui) { ui_set_rx_list(g_rx_lines); ui_draw_rx(); }
    else g_rx_dirty = true;
    g_decode_in_progress = false;  // Clear flag before early return
    return;
  }

  int decodedCount = 0;
  std::unordered_map<std::string, int> seen_idx; // displayed-text -> ui_lines index

  for (int i = 0; i < num_candidates; ++i) {
    ftx_message_t message;
    ftx_decode_status_t status;
    memset(&message, 0, sizeof(message));
    memset(&status, 0, sizeof(status));

    if (!ftx_decode_candidate(&mon->wf, &candidates[i], 25, &message, &status)) {
      continue;
    }

    // --- payload/hash dedupe (open addressing) ---
    int idx_hash = (int)(message.hash % kMaxDecoded);
    bool found_empty = false;
    bool found_dup = false;
    for (int probe = 0; probe < kMaxDecoded; ++probe) {
      ftx_message_t* p = decoded_hashtable[idx_hash];
      if (p == nullptr) { found_empty = true; break; }
      if (p->hash == message.hash &&
          0 == memcmp(p->payload, message.payload, sizeof(message.payload))) {
        found_dup = true;
        break;
      }
      idx_hash = (idx_hash + 1) % kMaxDecoded;
    }
    if (found_dup) continue;
    if (!found_empty) continue; // table full; drop extras

    // store unique
    memcpy(&decoded[idx_hash], &message, sizeof(message));
    decoded_hashtable[idx_hash] = &decoded[idx_hash];
    ++num_decoded;

    // --- decode to human text using ftx_message_decode ONLY ---
    char text[FTX_MAX_MESSAGE_LENGTH] = {0};
    ftx_message_offsets_t offsets;
    ftx_message_rc_t urc = ftx_message_decode(&message, &hash_if, text, &offsets);
    if (urc != FTX_MESSAGE_RC_OK || text[0] == '\0') {
      continue;
    }

    // freq/time/SNR like your current code
    float freq_hz = (mon->min_bin + candidates[i].freq_offset +
                    candidates[i].freq_sub / (float)cfg->freq_osr) / mon->symbol_period;
    float time_s = (candidates[i].time_offset +
                   candidates[i].time_sub / (float)cfg->time_osr) * mon->symbol_period;

    float cand_db = noise_db;
    {
      // Canonical waterfall indexing:
      // [time_block][time_sub][freq_sub][freq_bin]
      // Clamp time to valid range (1A) to avoid fallback-to-noise zero SNR on edge candidates.
      int t_index = candidates[i].time_offset * mon->wf.time_osr + candidates[i].time_sub;
      const int t_count = mon->wf.num_blocks * mon->wf.time_osr;
      if (t_count > 0) {
        if (t_index < 0) t_index = 0;
        if (t_index >= t_count) t_index = t_count - 1;
      } else {
        t_index = 0;
      }

      int f_index = candidates[i].freq_sub * mon->wf.num_bins + candidates[i].freq_offset;
      const int f_count = mon->wf.freq_osr * mon->wf.num_bins;
      if (f_count > 0) {
        if (f_index < 0) f_index = 0;
        if (f_index >= f_count) f_index = f_count - 1;
      } else {
        f_index = 0;
      }

      size_t offset2 = (size_t)t_index * (size_t)f_count + (size_t)f_index;
      size_t total2 = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
      if (mon->wf.mag && offset2 < total2) {
        int scaled = mon->wf.mag[offset2];
        cand_db = 0.5f * ((float)scaled - 240.0f);
      }
    }
    
    //float snr_db = (cand_db - noise_db) + snr_to_2500;
    float snr_db = (cand_db - noise_db);

    int snr_q = (int)lrintf(snr_db);
    if (snr_q < -30) snr_q = -30;
    if (snr_q >  99) snr_q = 99;

    ESP_LOGI(TAG, "Decoded[%d] t=%.2fs f=%.1fHz snr=%d : %s",
             decodedCount, time_s, freq_hz, snr_q, text);

    // UI de-dupe by displayed text (keep highest SNR)
    std::string raw_text_str(text);
    std::string text_str = raw_text_str;
    if (rewrite_dxpedition_for_mycall(raw_text_str, mycall_up, text_str)) {
      ESP_LOGI(TAG, "DXpedition raw match: %s", raw_text_str.c_str());
    }
    auto it = seen_idx.find(text_str);
    if (it != seen_idx.end()) {
      int idx_ui = it->second;
      if (snr_q > ui_lines[idx_ui].snr) {
        ui_lines[idx_ui].snr = snr_q;
        ui_lines[idx_ui].offset_hz = (int)lrintf(freq_hz);
        ui_lines[idx_ui].slot_id = slot_id;
        // keep original text, but refresh parsed fields based on it
        fill_fields_from_text(ui_lines[idx_ui]);
      }
      continue;
    }

    UiRxLine line;
    line.text = text_str;
    line.snr = snr_q;
    line.offset_hz = (int)lrintf(freq_hz);
    line.slot_id = slot_id;

    time_offsets.push_back(time_s);

    fill_fields_from_text(line);

    // CQ detection (now works because text contains CQ again)
    if (line.text.rfind("CQ ", 0) == 0 || line.text == "CQ") line.is_cq = true;

    // to-me detection (same behavior as your old code)
    //std::string f1_up = to_upper(line.field1);
    std::string f1_up = normalize_call_token(line.field1);
    if (!mycall_up.empty() && f1_up == mycall_up) line.is_to_me = true;

    ui_lines.push_back(line);
    seen_idx[text_str] = (int)ui_lines.size() - 1;

    log_rxtx_line('R', snr_q, (int)lrintf(freq_hz), text_str, -1);

    decodedCount++;
    if (decodedCount >= 32) break;
  }

  if (decodedCount == 0) {
    ESP_LOGW(TAG, "Candidates present but no messages decoded");
  }

  // ---- Auto sync RTC (your existing logic) ----
  if (time_offsets.size() > 3) {
    std::vector<float> tmp = time_offsets;
    std::sort(tmp.begin(), tmp.end());
    float median = tmp[tmp.size() / 2];
    if (std::fabs(median) > 0.3f) {
      int delta_ms = (int)lrintf(-median * 1000.0f);
      if (delta_ms > 320) delta_ms = 320;
      if (delta_ms < -320) delta_ms = -320;
      rtc_ms_start -= delta_ms;
      rtc_last_update -= delta_ms;
      rtc_update_strings();
      rtc_sync_to_hw();
      ESP_LOGI("SYNC", "Applied RTC sync: median=%.2fs delta=%dms", median, delta_ms);
    } else {
      ESP_LOGD("SYNC", "Median=%.2fs within threshold; no sync", median);
    }
  }

  // ---- group: to-me, CQ, other ----
  std::vector<UiRxLine> to_me, cqs, others;
  std::string mycall = to_upper(g_call);
  for (auto& l : ui_lines) {
    //std::string f1 = to_upper(l.field1);
    std::string f1 = normalize_call_token(l.field1);
    if (!mycall.empty() && !f1.empty() && f1 == mycall) {
      l.is_to_me = true;
      to_me.push_back(l);
    } else if (l.is_cq) {
      cqs.push_back(l);
    } else {
      others.push_back(l);
    }
  }

  // ---- autoseq trigger logic (unchanged idea) ----
  if (!g_was_txing) {
    std::vector<UiRxLine> to_me_auto;
    to_me_auto.reserve(to_me.size());
    for (const auto& msg : to_me) {
      const std::string dxcall_norm = normalize_call_token(msg.field2);
      if (ignorelist_matches_normalized_dxcall(dxcall_norm)) {
        ESP_LOGI(TAG, "IgnoreList: skip auto reply to %s", dxcall_norm.c_str());
        continue;
      }
      to_me_auto.push_back(msg);
    }

    if (!to_me_auto.empty()) {
      autoseq_on_decodes(to_me_auto);
      g_tx_view_dirty = true;
      g_last_reply_text = to_me_auto.front().text;
    }

    AutoseqTxEntry pending;
    if (autoseq_fetch_pending_tx(pending)) {
      g_qso_xmit = true;
      g_target_slot_parity = pending.slot_id & 1;
      g_pending_tx = pending;
      g_pending_tx_valid = true;
      ESP_LOGI(TAG, "TX ready: %s parity=%d", pending.text.c_str(), g_target_slot_parity);
    } else if (g_beacon != BeaconMode::OFF) {
      enqueue_beacon_cq();
      if (autoseq_fetch_pending_tx(pending)) {
        g_qso_xmit = true;
        g_target_slot_parity = pending.slot_id & 1;
        g_pending_tx = pending;
        g_pending_tx_valid = true;
        ESP_LOGI(TAG, "Beacon CQ ready: %s parity=%d", pending.text.c_str(), g_target_slot_parity);
      }
    }
  }

  std::vector<UiRxLine> merged;
  merged.reserve(to_me.size() + cqs.size() + others.size());
  merged.insert(merged.end(), to_me.begin(), to_me.end());
  merged.insert(merged.end(), cqs.begin(), cqs.end());
  merged.insert(merged.end(), others.begin(), others.end());

  g_rx_lines = merged;

  if (update_ui) {
    ui_set_rx_list(g_rx_lines);
    ui_draw_rx();
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  } else {
    g_rx_dirty = true;
  }

#ifdef DEBUG_LOG
    //char buf[32];
    //snprintf(buf, sizeof(buf), "HashTableSize %d", callsign_hashtable_size);
    //debug_log_line_public(buf);
#endif

  g_decode_in_progress = false;  // Allow TX trigger now that decode is complete
}

[[maybe_unused]] static void draw_menu_long_edit() {
  std::vector<std::string> lines(6, "");
  std::string text = menu_long_buf;
  size_t idx = 0;
  int line = 0;
  while (idx < text.size() && line < 6) {
    size_t chunk = std::min<size_t>(18, text.size() - idx);
    lines[line] = text.substr(idx, chunk);
    idx += chunk;
    line++;
  }
  // cursor indicator on the last line
  if (line == 0) {
    lines[0] = "_";
  } else {
    if (lines[line - 1].size() < 20) lines[line - 1].push_back('_');
    else if (line < 6) lines[line] = "_";
  }
  ui_draw_list(lines, 0, -1);
}

static bool touch_keyboard_active() {
  if (ui_mode == UIMode::STATUS) {
    return (status_edit_idx == 4 || status_edit_idx == 5);
  }
  if (ui_mode == UIMode::MENU) {
    return menu_long_edit || (menu_edit_idx >= 0);
  }
  if (ui_mode == UIMode::BAND) {
    return (band_edit_idx >= 0);
  }
  return false;
}

static void touch_keyboard_apply_buffer_shadow();
static void touch_keyboard_commit_current();
static void touch_keyboard_cancel_current();
static void touch_keyboard_draw_edit_window();

class TouchKeyboardHost final : public touchkbd::IHost {
 public:
  uint32_t nowMs() const override {
    return static_cast<uint32_t>(rtc_now_ms() & 0xFFFFFFFFu);
  }

  void onBufferChanged(const touchkbd::BufferState&) override {
    touch_keyboard_apply_buffer_shadow();
    touch_keyboard_draw_edit_window();
  }

  void onCommit(const touchkbd::BufferState&) override {
    touch_keyboard_commit_current();
  }

  void onCancel(const touchkbd::BufferState&) override {
    touch_keyboard_cancel_current();
  }
};

class TouchKeyboardRenderer final : public touchkbd::IRenderer {
 public:
  void drawKeyboardBackground(const touchkbd::Rect& bounds) override {
    M5.Display.fillRect(bounds.x, bounds.y, bounds.w, bounds.h, TFT_WHITE);

    const int row_h = std::max<int16_t>(1, bounds.h / 4);
    for (int row = 0; row < 4; ++row) {
      const int y = bounds.y + row * row_h;
      M5.Display.drawFastHLine(bounds.x, y, bounds.w, TFT_BLACK);
      M5.Display.drawFastHLine(bounds.x, y + row_h - 1, bounds.w, TFT_BLACK);
    }

    touch_keyboard_draw_edit_window();
    g_touchkbd_needs_display = true;
  }

  void drawKey(const touchkbd::KeyDef& key, const touchkbd::KeyVisualState& state) override {
    constexpr int16_t kPadX = 3;
    constexpr int16_t kPadY = 3;

    int16_t x = key.rect.x + kPadX;
    int16_t y = key.rect.y + kPadY;
    int16_t w = key.rect.w - 2 * kPadX;
    int16_t h = key.rect.h - 2 * kPadY;
    if (w <= 0 || h <= 0) return;

    const uint16_t bg = state.pressed ? TFT_BLACK : TFT_WHITE;
    const uint16_t fg = state.pressed ? TFT_WHITE : TFT_BLACK;

    M5.Display.fillRect(x, y, w, h, bg);
    M5.Display.drawRect(x, y, w, h, TFT_BLACK);
    M5.Display.setTextColor(fg, bg);
    M5.Display.setTextDatum(middle_center);

    const size_t label_len = (key.label != nullptr) ? strlen(key.label) : 0;
    int text_size = (label_len >= 4) ? 2 : 3;
    if (label_len >= 6) text_size = 1;
    M5.Display.setTextSize(text_size);
    M5.Display.drawString((key.label != nullptr) ? key.label : "", x + (w / 2), y + (h / 2));

    g_touchkbd_needs_display = true;
  }
};

static TouchKeyboardHost& touch_keyboard_host() {
  static TouchKeyboardHost host;
  return host;
}

static TouchKeyboardRenderer& touch_keyboard_renderer() {
  static TouchKeyboardRenderer renderer;
  return renderer;
}

static touchkbd::TouchKeyboard& touch_keyboard_instance() {
  static touchkbd::Config cfg = [] { touchkbd::Config c; c.viewportCols = 38; return c; }();
  static touchkbd::TouchKeyboard keyboard(touch_keyboard_host(), &touch_keyboard_renderer(), cfg);
  return keyboard;
}

static void touch_keyboard_set_bounds() {
  const UiLayout& lay = ui_layout();
  touch_keyboard_instance().setBounds(
      static_cast<int16_t>(lay.text_area.x),
      static_cast<int16_t>(lay.text_area.y + (2 * lay.line_h)),
      static_cast<int16_t>(lay.text_area.w),
      static_cast<int16_t>(4 * lay.line_h));
}

static void touch_keyboard_apply_buffer_shadow() {
  const std::string text(g_touchkbd_buffer);

  switch (g_touchkbd_target) {
    case TouchKbdEditTarget::MenuLong:
      if (menu_long_kind == LONG_IGNORE) {
        menu_long_buf = clamp_ignore_prefix_text(text);
      } else {
        menu_long_buf = text;
      }
      break;
    case TouchKbdEditTarget::MenuEdit:
      menu_edit_buf = text;
      break;
    case TouchKbdEditTarget::BandFreq:
      band_edit_buffer = text;
      break;
    case TouchKbdEditTarget::StatusDate:
      status_edit_buffer = digits_only_limited(text, 8);
      break;
    case TouchKbdEditTarget::StatusTime:
      status_edit_buffer = digits_only_limited(text, 6);
      break;
    case TouchKbdEditTarget::None:
      break;
  }
}

static void touch_keyboard_end_session() {
  touch_keyboard_instance().detachBuffer();
  g_touchkbd_target = TouchKbdEditTarget::None;
  g_touchkbd_header.clear();
  g_touchkbd_buffer[0] = '\0';
  g_touchkbd_needs_display = false;
}

static void touch_keyboard_draw_edit_window() {
  const UiLayout& lay = ui_layout();
  const int row_h = lay.line_h;
  const int x = lay.text_area.x;
  const int y = lay.text_area.y;

  M5.Display.fillRect(x, y, lay.text_area.w, row_h * 2, TFT_WHITE);

  for (int row = 0; row < 2; ++row) {
    const int row_y = y + row * row_h;
    M5.Display.drawFastHLine(x, row_y, lay.text_area.w, TFT_BLACK);
    M5.Display.drawFastHLine(x, row_y + row_h - 1, lay.text_area.w, TFT_BLACK);
  }

  const touchkbd::VisibleLines visible = touch_keyboard_instance().getVisibleLines();
  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.setTextDatum(middle_left);
  M5.Display.setTextSize(4);
  M5.Display.drawString(visible.lines[0].data(), x + 24, y + (row_h / 2));
  M5.Display.drawString(visible.lines[1].data(), x + 24, y + row_h + (row_h / 2));

  if (visible.cursorVisible && visible.cursorLine < 2) {
    const int text_left = x + 24;
    const int text_right = x + lay.text_area.w - 24;
    const int fallback_char_w = std::max(1, static_cast<int>(M5.Display.textWidth("0")));
    int advance_px = 0;
    char glyph[2] = {0, 0};
    const size_t col_count = std::min(visible.cursorCol, visible.cols);
    for (size_t i = 0; i < col_count; ++i) {
      glyph[0] = visible.lines[visible.cursorLine][i];
      int glyph_w = static_cast<int>(M5.Display.textWidth(glyph));
      if (glyph_w <= 0 && glyph[0] == ' ') glyph_w = fallback_char_w;
      if (glyph_w <= 0) glyph_w = fallback_char_w;
      advance_px += glyph_w;
    }
    int cursor_x = text_left + advance_px;
    if (cursor_x > (text_right - 1)) cursor_x = text_right - 1;
    const int cursor_y = y + static_cast<int>(visible.cursorLine) * row_h;
    M5.Display.drawFastVLine(cursor_x, cursor_y + 10, row_h - 20, TFT_BLACK);
  }

  g_touchkbd_needs_display = true;
}

static void touch_keyboard_begin_session(TouchKbdEditTarget target,
                                         const std::string& initial_text,
                                         const std::string& header) {
  if (g_touchkbd_target == target && touch_keyboard_instance().isEditing()) {
    g_touchkbd_header = header;
    return;
  }

  touch_keyboard_end_session();

  size_t copy_len = std::min(initial_text.size(), sizeof(g_touchkbd_buffer) - 1);
  memcpy(g_touchkbd_buffer, initial_text.data(), copy_len);
  g_touchkbd_buffer[copy_len] = '\0';

  g_touchkbd_target = target;
  g_touchkbd_header = header;

  touch_keyboard_set_bounds();
  if (!touch_keyboard_instance().attachBuffer(g_touchkbd_buffer, sizeof(g_touchkbd_buffer))) {
    g_touchkbd_target = TouchKbdEditTarget::None;
    return;
  }

  touch_keyboard_apply_buffer_shadow();
}

static void touch_keyboard_commit_current() {
  touch_keyboard_apply_buffer_shadow();

  switch (g_touchkbd_target) {
    case TouchKbdEditTarget::MenuLong: {
      if (menu_long_kind == LONG_FT) {
        g_free_text = menu_long_buf;
        if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
        update_autoseq_cq_type();
      } else if (menu_long_kind == LONG_COMMENT) {
        g_comment1 = menu_long_buf;
      } else if (menu_long_kind == LONG_ACTIVE) {
        g_active_band_text = menu_long_buf;
        rebuild_active_bands();
      } else if (menu_long_kind == LONG_IGNORE) {
        g_ignore_prefix_text = clamp_ignore_prefix_text(menu_long_buf);
        rebuild_ignore_prefixes();
      }
      save_station_data();
      menu_long_edit = false;
      menu_long_kind = LONG_NONE;
      menu_long_buf.clear();
      menu_long_backup.clear();
      touch_keyboard_end_session();
      draw_menu_view();
      break;
    }
    case TouchKbdEditTarget::MenuEdit: {
      if (menu_edit_idx == 3) {
        g_call = menu_edit_buf;
        autoseq_set_station(g_call, g_grid);
        ble_update_name_from_station(true);
      } else if (menu_edit_idx == 4) {
        g_grid = menu_edit_buf;
        autoseq_set_station(g_call, g_grid);
      } else if (menu_edit_idx == 7) {
        g_offset_hz = atoi(menu_edit_buf.c_str());
      } else if (menu_edit_idx == 10) {
        g_comment1 = menu_edit_buf;
      } else if (menu_edit_idx == 15) {
        int v = atoi(menu_edit_buf.c_str());
        if (v < 0) v = 0;
        g_autoseq_max_retry = v;
        autoseq_set_max_retry(g_autoseq_max_retry);
      }
      save_station_data();
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      touch_keyboard_end_session();
      draw_menu_view();
      break;
    }
    case TouchKbdEditTarget::BandFreq: {
      bool applied = false;
      if (band_edit_idx >= 0 && band_edit_idx < (int)g_bands.size()) {
        if (!band_edit_buffer.empty() &&
            std::all_of(band_edit_buffer.begin(), band_edit_buffer.end(),
                        [](unsigned char ch) { return std::isdigit(ch) != 0; })) {
          int val = std::atoi(band_edit_buffer.c_str());
          g_bands[band_edit_idx].freq = val;
          save_station_data();
          applied = true;
        }
      }
      band_edit_idx = -1;
      band_edit_buffer.clear();
      touch_keyboard_end_session();
      draw_band_view();
      if (!applied) {
        debug_log_line("Band freq unchanged");
      }
      break;
    }
    case TouchKbdEditTarget::StatusDate:
    case TouchKbdEditTarget::StatusTime: {
      const bool is_date = (g_touchkbd_target == TouchKbdEditTarget::StatusDate);
      const std::string prev_date = g_date;
      const std::string prev_time = g_time;
      std::string formatted;
      bool ok = is_date
          ? format_date_digits_exact(status_edit_buffer, formatted)
          : format_time_digits_exact(status_edit_buffer, formatted);

      if (ok) {
        if (is_date) g_date = formatted;
        else g_time = formatted;

        if (rtc_set_from_strings()) {
          save_station_data();
          rtc_sync_to_hw();
        } else {
          ok = false;
        }
      }

      if (!ok) {
        g_date = prev_date;
        g_time = prev_time;
        debug_log_line(is_date ? "Invalid date; unchanged" : "Invalid time; unchanged");
      }

      status_edit_idx = -1;
      status_cursor_pos = -1;
      status_edit_buffer.clear();
      touch_keyboard_end_session();
      draw_status_view();
      break;
    }
    case TouchKbdEditTarget::None:
      break;
  }
}

static void touch_keyboard_cancel_current() {
  switch (g_touchkbd_target) {
    case TouchKbdEditTarget::MenuLong:
      menu_long_edit = false;
      menu_long_kind = LONG_NONE;
      menu_long_buf.clear();
      menu_long_backup.clear();
      touch_keyboard_end_session();
      draw_menu_view();
      break;
    case TouchKbdEditTarget::MenuEdit:
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      touch_keyboard_end_session();
      draw_menu_view();
      break;
    case TouchKbdEditTarget::BandFreq:
      band_edit_idx = -1;
      band_edit_buffer.clear();
      touch_keyboard_end_session();
      draw_band_view();
      break;
    case TouchKbdEditTarget::StatusDate:
    case TouchKbdEditTarget::StatusTime:
      status_edit_idx = -1;
      status_cursor_pos = -1;
      status_edit_buffer.clear();
      touch_keyboard_end_session();
      draw_status_view();
      break;
    case TouchKbdEditTarget::None:
      break;
  }
}

static void draw_touch_keyboard_overlay(const std::string& header) {
  TouchKbdEditTarget target = TouchKbdEditTarget::None;
  std::string initial_text;

  if (ui_mode == UIMode::MENU) {
    if (menu_long_edit) {
      target = TouchKbdEditTarget::MenuLong;
      initial_text = menu_long_buf;
    } else if (menu_edit_idx >= 0) {
      target = TouchKbdEditTarget::MenuEdit;
      initial_text = menu_edit_buf;
    }
  } else if (ui_mode == UIMode::BAND) {
    if (band_edit_idx >= 0 && band_edit_idx < (int)g_bands.size()) {
      target = TouchKbdEditTarget::BandFreq;
      initial_text = band_edit_buffer;
    }
  } else if (ui_mode == UIMode::STATUS) {
    if (status_edit_idx == 4) {
      target = TouchKbdEditTarget::StatusDate;
      initial_text = status_edit_buffer;
    } else if (status_edit_idx == 5) {
      target = TouchKbdEditTarget::StatusTime;
      initial_text = status_edit_buffer;
    }
  }

  if (target == TouchKbdEditTarget::None) {
    touch_keyboard_end_session();
    return;
  }

  touch_keyboard_begin_session(target, initial_text, header);
  touch_keyboard_set_bounds();

  g_touchkbd_header = header;
  std::string mode_line = std::string("K: ") + head_trim(g_touchkbd_header, 26);
  ui_draw_mode_box(mode_line.c_str());

  touch_keyboard_instance().drawAll();
  touch_keyboard_draw_edit_window();
  if (g_touchkbd_needs_display) {
    M5.Display.display();
    g_touchkbd_needs_display = false;
  }
}

static void touch_keyboard_process_touch() {
  if (!touch_keyboard_active()) return;
  if (g_touchkbd_target == TouchKbdEditTarget::None) return;
  if (!touch_keyboard_instance().isEditing()) return;

  auto d = M5.Touch.getDetail();
  if (d.wasPressed()) {
    touch_keyboard_instance().onTouchDown(d.x, d.y);
  } else if (d.isPressed()) {
    touch_keyboard_instance().onTouchMove(d.x, d.y);
  }
  if (d.wasReleased()) {
    touch_keyboard_instance().onTouchUp(d.x, d.y);
    if (!touch_keyboard_active() ||
        g_touchkbd_target == TouchKbdEditTarget::None ||
        !touch_keyboard_instance().isEditing()) {
      return;
    }
  }

  touch_keyboard_instance().tick();
  if (!touch_keyboard_active() ||
      g_touchkbd_target == TouchKbdEditTarget::None ||
      !touch_keyboard_instance().isEditing()) {
    return;
  }
  touch_keyboard_instance().redrawDirty();

  if (g_touchkbd_needs_display) {
    M5.Display.display();
    g_touchkbd_needs_display = false;
  }
}

static const char* menu_edit_label(int idx) {
  switch (idx) {
    case 3:  return "Call";
    case 4:  return "Grid";
    case 7:  return "Cursor";
    case 9:  return "IgnoreList";
    case 10: return "Comment";
    case 15: return "Max Retry";
    default: return "Edit";
  }
}

static void log_tones(const uint8_t* tones, size_t n) {
  std::string line;
  for (size_t i = 0; i < n; ++i) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", (unsigned)tones[i]);
    line += buf;
    if ((i + 1) % 20 == 0 || i + 1 == n) {
      debug_log_line(line);
      line.clear();
    }
  }
}

static void encode_and_log_pending_tx() {
  if (!g_pending_tx_valid || g_pending_tx.text.empty()) {
    debug_log_line("No pending TX to encode");
    return;
  }
  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, &hash_if, g_pending_tx.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    debug_log_line("Encode failed");
    return;
  }
  uint8_t tones[79] = {0};
  ft8_encode(msg.payload, tones);
  debug_log_line(std::string("Tones for '") + g_pending_tx.text + "'");
  log_tones(tones, 79);
}

[[maybe_unused]] static bool looks_like_grid(const std::string& s) {
  if (s.size() != 4) return false;
  return std::isalpha((unsigned char)s[0]) && std::isalpha((unsigned char)s[1]) &&
         std::isdigit((unsigned char)s[2]) && std::isdigit((unsigned char)s[3]);
}

[[maybe_unused]] static bool looks_like_report(const std::string& s, int& out) {
  if (s.empty()) return false;
  int sign = 1;
  size_t idx = 0;
  if (s[0] == '-') { sign = -1; idx = 1; }
  else if (s[0] == '+') { idx = 1; }
  if (idx >= s.size()) return false;
  int val = 0;
  for (; idx < s.size(); ++idx) {
    if (!std::isdigit((unsigned char)s[idx])) return false;
    val = val * 10 + (s[idx] - '0');
  }
  out = sign * val;
  return true;
}

// Enqueue a beacon CQ. Parity is determined by beacon mode.
// Duplicate prevention is handled by autoseq_start_cq().
// TX trigger happens at slot boundary via check_slot_boundary().
static void enqueue_beacon_cq() {
  int target_parity = (g_beacon == BeaconMode::EVEN) ? 0 : 1;
  autoseq_start_cq(target_parity);
  g_tx_view_dirty = true;
}

static bool autoseq_has_pending_tx() {
  AutoseqTxEntry tmp;
  return autoseq_fetch_pending_tx(tmp);
}

// Schedule a one-off pending TX (e.g., manual FreeText) without touching autoseq state.
// Returns false if TX is already active or if scheduling failed.
// Uses the single-threaded state machine - TX will trigger at next matching slot boundary.
static bool schedule_manual_pending_tx(const AutoseqTxEntry& pending) {
  // Already transmitting or TX pending?
  if (g_tx_active || g_qso_xmit) {
    return false;
  }

  int target_parity = pending.slot_id & 1;

  // Set up pending TX
  g_pending_tx = pending;
  g_pending_tx_valid = true;

  // Set flags for check_slot_boundary() to trigger TX
  g_qso_xmit = true;
  g_target_slot_parity = target_parity;

  ESP_LOGI(TAG, "schedule_manual_pending_tx: queued TX=%s for parity=%d",
           pending.text.c_str(), target_parity);
  return true;
}

// NOTE: This function is now mostly superseded by the state machine approach.
// TX scheduling is done via g_qso_xmit and g_target_slot_parity flags,
// and check_slot_boundary() triggers TX at the right time.
// Keeping this as a no-op for now in case any code still calls it.
[[maybe_unused]] static void schedule_tx_if_idle() {
  // No-op: TX scheduling is now handled by decode_monitor_results setting
  // g_qso_xmit and check_slot_boundary triggering TX at slot start.
}

// Helper to send TA command (deduplicated)
static void tx_send_ta(float tone_hz) {
  int ta_int = (int)lrintf(tone_hz);
  float frac = tone_hz - (float)ta_int;
  int ta_frac = (int)lrintf(frac * 100.0f);
  if (ta_int == g_tx_last_ta_int && ta_frac == g_tx_last_ta_frac) return;
  char ta[16];
  snprintf(ta, sizeof(ta), "TA%04d.%02d;", ta_int, ta_frac);
  cat_cdc_send(reinterpret_cast<const uint8_t*>(ta), strlen(ta), 10);
  g_tx_last_ta_int = ta_int;
  g_tx_last_ta_frac = ta_frac;
}

// Start TX (single-threaded state machine initialization)
// Called from check_slot_boundary at the right time
// Uses g_pending_tx which was prepared by check_slot_boundary with correct offset
static void tx_start(int skip_tones) {
  // Already transmitting?
  if (g_tx_active) {
    return;
  }

  // Use g_pending_tx which was prepared by check_slot_boundary
  if (!g_pending_tx_valid || g_pending_tx.text.empty()) {
    ESP_LOGW(TAG, "tx_start: no pending TX");
    return;
  }

  // Get current slot info
  int64_t now_ms = rtc_now_ms();
  g_tx_slot_idx = now_ms / 15000;

  ESP_LOGI(TAG, "tx_start: TX=%s offset=%d skip=%d slot=%lld",
           g_pending_tx.text.c_str(), g_pending_tx.offset_hz, skip_tones, (long long)g_tx_slot_idx);

  // Encode message to tones
  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, &hash_if, g_pending_tx.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    ESP_LOGE(TAG, "Encode failed for TX");
    return;
  }
  ft8_encode(msg.payload, g_tx_tones);

  // Set up TX state machine
  // IMPORTANT: Tone timing must be based on slot boundary, not TX start time.
  // This ensures TX ends at the correct time even if TX started late,
  // allowing RX to start cleanly at the next slot boundary.
  g_tx_base_hz = g_pending_tx.offset_hz;
  g_tx_slot_start_ms = (now_ms / 15000) * 15000;  // Slot boundary time
  g_tx_tone_idx = (skip_tones >= 79) ? 79 : skip_tones;
  // Next tone time = slot_start + tone_idx * 160ms
  // This aligns all tones to the slot boundary, not to when TX started
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * 160;
  g_tx_last_ta_int = -1;
  g_tx_last_ta_frac = -1;

  ESP_LOGI(TAG, "TX base_hz=%d (from pre-computed offset, text=%s)", g_tx_base_hz, g_pending_tx.text.c_str());

  // Send CAT setup commands
  g_tx_cat_ok = cat_cdc_ready();
  if (g_tx_cat_ok) {
    const char* md = "MD6;";
    const char* tx = "TX;";
    cat_cdc_send(reinterpret_cast<const uint8_t*>(md), strlen(md), 200);
    cat_cdc_send(reinterpret_cast<const uint8_t*>(tx), strlen(tx), 200);
  }

  if (skip_tones > 0) {
    ESP_LOGI("TXTONE", "Skipping first %d tones due to late start", skip_tones);
  }

  // Send first tone TA if CAT is ready
  if (g_tx_cat_ok && g_tx_tone_idx < 79) {
    float tone_hz = g_tx_base_hz + 6.25f * g_tx_tones[g_tx_tone_idx];
    tx_send_ta(tone_hz);
  }

  // Mark TX as active
  g_tx_active = true;
  ui_set_tx_indicator(true);
}

// TX state machine tick - called from main loop
// Sends one tone at a time, non-blocking
static void tx_tick() {
  if (!g_tx_active) {
    return;
  }

  int64_t now_ms = rtc_now_ms();

  // Check for cancel request
  if (g_tx_cancel_requested) {
    ESP_LOGI(TAG, "tx_tick: TX cancelled at tone %d", g_tx_tone_idx);
    if (g_tx_cat_ok) {
      const char* rx = "RX;";
      cat_cdc_send(reinterpret_cast<const uint8_t*>(rx), strlen(rx), 200);
    }
    g_tx_active = false;
    ui_set_tx_indicator(false);
    g_pending_tx_valid = false;
    g_tx_cancel_requested = false;
    g_was_txing = false;  // TX was cancelled - don't call tick at slot boundary
    g_tx_view_dirty = true;
    return;
  }

  // Time to send next tone?
  if (now_ms < g_tx_next_tone_time) {
    return;  // Not yet
  }

  // All tones sent?
  if (g_tx_tone_idx >= 79) {
    ESP_LOGI(TAG, "tx_tick: TX complete, all 79 tones sent");
    if (g_tx_cat_ok) {
      const char* rx = "RX;";
      cat_cdc_send(reinterpret_cast<const uint8_t*>(rx), strlen(rx), 200);
    }
    // Record slot index for spacing and notify autoseq
    s_last_tx_slot_idx = g_tx_slot_idx;
    autoseq_mark_sent(g_tx_slot_idx);
    // g_was_txing stays true - tick will be called at slot boundary

    g_tx_active = false;
    ui_set_tx_indicator(false);
    g_pending_tx_valid = false;
    g_tx_cancel_requested = false;
    g_tx_view_dirty = true;
    return;
  }

  // Send current tone
  ESP_LOGD("TXTONE", "%02d %u", g_tx_tone_idx, (unsigned)g_tx_tones[g_tx_tone_idx]);
  fft_waterfall_tx_tone(g_tx_tones[g_tx_tone_idx]);
  if (g_tx_cat_ok) {
    float tone_hz = g_tx_base_hz + 6.25f * g_tx_tones[g_tx_tone_idx];
    tx_send_ta(tone_hz);
  }

  // Advance to next tone
  g_tx_tone_idx++;
  // Calculate next tone time from slot boundary to ensure TX ends at correct time
  // This guarantees RX can start cleanly at the next slot boundary
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * 160;
}

static void draw_menu_view() {
  // Approximate visible capacity per list row on PaperS3 with text size 4:
  // 39 chars total, with "N " index prefix drawn by ui_draw_list().
  static constexpr size_t kMenuContentChars = 37;
  static constexpr size_t kFreeTextMaxChars = kMenuContentChars - 2;    // "F:"
  static constexpr size_t kCommentMaxChars = kMenuContentChars - 2;     // "C:"
  static constexpr size_t kIgnoreListMaxChars = kMenuContentChars - 11; // "IgnoreList:"
  static constexpr size_t kActiveBandMaxChars = kMenuContentChars - 11; // "ActiveBand:"

  if (menu_long_edit) {
    draw_touch_keyboard_overlay(std::string("Edit: ") + menu_long_buf + "_");
    return;
  }
  if (menu_edit_idx >= 0) {
    draw_touch_keyboard_overlay(std::string(menu_edit_label(menu_edit_idx)) + ": " + menu_edit_buf);
    return;
  }
  std::vector<std::string> lines;
  lines.reserve(12);

  std::string cq_line = std::string("CQ Type:");
  if (g_cq_type == CqType::CQFREETEXT) cq_line += g_cq_freetext;
  else cq_line += cq_type_name(g_cq_type);
  lines.push_back(cq_line);
  lines.push_back("Send FreeText");
  lines.push_back(std::string("F:") + head_trim(g_free_text, kFreeTextMaxChars));
  lines.push_back(std::string("Call:") + elide_right(menu_edit_idx == 3 ? menu_edit_buf : g_call));
  lines.push_back(std::string("Grid:") + elide_right(menu_edit_idx == 4 ? menu_edit_buf : g_grid));
  lines.push_back(menu_sleep_batt_line());

  lines.push_back(std::string("Offset:") + offset_name(g_offset_src));
  if (menu_edit_idx == 7) {
    lines.push_back(std::string("Fixed:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("Fixed:") + std::to_string(g_offset_hz));
  }
  lines.push_back(std::string("Radio:") + radio_name(g_radio));
  lines.push_back(std::string("IgnoreList:") + head_trim(g_ignore_prefix_text, kIgnoreListMaxChars));
  lines.push_back(std::string("C:") + head_trim(expand_comment1(), kCommentMaxChars));
  lines.push_back(std::string("BLE ") + (g_ble_enabled ? "ON" : "OFF"));

  // Page 2 content (index 12+)
  lines.push_back(std::string("RxTxLog:") + (g_rxtx_log ? "ON" : "OFF"));
  lines.push_back(std::string("SkipTX1:") + (g_skip_tx1 ? "ON" : "OFF"));
  lines.push_back(std::string("ActiveBand:") + head_trim(g_active_band_text, kActiveBandMaxChars));
  if (menu_edit_idx == 15) {
    lines.push_back(std::string("Max Retry:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("Max Retry:") + std::to_string(g_autoseq_max_retry));
  }
  lines.push_back("Copy Logs to SD");
  lines.push_back(menu_delete_confirm ? "Delete Logs? ^:Y v:N" : "Delete Logs");

  int highlight_abs = -1;
  int64_t now = rtc_now_ms();
  if (menu_edit_idx >= 0) {
    highlight_abs = menu_edit_idx;
  } else if (menu_flash_idx >= 0 && now < menu_flash_deadline) {
    highlight_abs = menu_flash_idx;
  } else {
    menu_flash_idx = -1;
  }
  // Auto-clear flash after timeout
  if (menu_flash_idx >= 0 && now >= menu_flash_deadline) {
    menu_flash_idx = -1;
  }
  ui_draw_list(lines, menu_page, highlight_abs);
  // Draw battery icon on visible battery line
  int battery_abs_idx = 5;
  if (menu_page == (battery_abs_idx / 6)) {
    int line_on_page = battery_abs_idx % 6;
    const int line_h = 19;
    const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
    (void)line_on_page;
    (void)line_h;
    (void)start_y;
    //int y = start_y + line_on_page * line_h + 3;
    //int level = (int)M5.Power.getBatteryLevel();
    //bool charging = M5.Power.isCharging();
    //draw_battery_icon(190, y, 24, 12, level, charging);
  }
}

static void draw_status_view() {
  if (status_edit_idx == 4 || status_edit_idx == 5) {
    const bool is_date = (status_edit_idx == 4);
    const std::string masked = is_date
        ? format_date_digits_preview(status_edit_buffer)
        : format_time_digits_preview(status_edit_buffer);
    const char* label = is_date ? "Date: " : "Time: ";
    draw_touch_keyboard_overlay(std::string(label) + masked);
    return;
  }

  std::string lines[6];
  BeaconMode disp_beacon = (ui_mode == UIMode::STATUS) ? g_status_beacon_temp : g_beacon;
  lines[0] = std::string("Beacon: ") + beacon_name(disp_beacon);
  if (uac_is_streaming()) {
    lines[1] = std::string("Sync to ") + radio_name(g_radio);
  } else {
    lines[1] = std::string("Connect to ") + radio_name(g_radio);
  }
  lines[2] = std::string("Band: ") +
             std::string(g_bands[g_band_sel].name) + " " +
             std::to_string(g_bands[g_band_sel].freq);
  lines[3] = std::string("Tune: ") + (g_tune ? "ON" : "OFF");
  if (status_edit_idx == 4 && !status_edit_buffer.empty()) {
    lines[4] = std::string("Date: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[4] = std::string("Date: ") + g_date;
  }
  if (status_edit_idx == 5 && !status_edit_buffer.empty()) {
    lines[5] = std::string("Time: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[5] = std::string("Time: ") + g_time;
  }
  for (int i = 0; i < 6; ++i) {
    bool hl = (status_edit_idx == i);
    draw_status_line(i, lines[i], hl);
  }
  M5.Display.display();
}

static void debug_log_line(const std::string& msg) {
  if (g_debug_lines.size() >= DEBUG_MAX_LINES) {
    g_debug_lines.erase(g_debug_lines.begin());
  }
  g_debug_lines.push_back(msg);
  debug_page = (int)((g_debug_lines.size() - 1) / 6);
  if (ui_mode == UIMode::DEBUG) {
    ui_draw_debug(g_debug_lines, debug_page);
  }
}

#if ENABLE_BLE

static int page_count(int items, int page_size) {
  if (page_size <= 0) return 1;
  if (items <= 0) return 1;
  return (items + page_size - 1) / page_size;
}

static int ble_send_payload_raw(const std::string& payload, bool indicate) {
  if (!g_ble_stack_active) return BLE_HS_EINVAL;
  if (!g_ble_enabled) return BLE_HS_EINVAL;
  if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return BLE_HS_ENOTCONN;
  if (!gatt_tx_handle) return BLE_HS_EINVAL;
  struct os_mbuf* om = ble_hs_mbuf_from_flat(payload.data(), payload.size());
  if (!om) return BLE_HS_ENOMEM;
  int rc = indicate
             ? ble_gatts_indicate_custom(g_conn_handle, gatt_tx_handle, om)
             : ble_gatts_notify_custom(g_conn_handle, gatt_tx_handle, om);
  if (rc != 0) {
    ESP_LOGD(BT_TAG, "%s failed rc=%d", indicate ? "indicate" : "notify", rc);
  }
  return rc;
}

static void ble_notify_payload(const std::string& payload) {
  (void)ble_send_payload_raw(payload, false);
}

static bool ble_wait_for_indicate_ack(int timeout_ms) {
  const int step_ms = 10;
  int waited = 0;
  while (g_ble_indicate_waiting && waited < timeout_ms) {
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
      g_ble_indicate_status = BLE_HS_ENOTCONN;
      g_ble_indicate_waiting = false;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(step_ms));
    waited += step_ms;
  }
  if (g_ble_indicate_waiting) {
    g_ble_indicate_status = BLE_HS_ETIMEOUT;
    g_ble_indicate_waiting = false;
  }
  return g_ble_indicate_status == BLE_HS_EDONE;
}

static void ble_dump_reset_transfer_state(bool use_indicate) {
  g_ble_dump_xfer = BleDumpTransferState{};
  g_ble_dump_xfer.active = true;
  g_ble_dump_xfer.mode = use_indicate ? BleDumpTxMode::Indicate : BleDumpTxMode::Notify;
  g_ble_dump_xfer.notify_pace_ms = kBleDumpNotifyPaceMinMs;
  g_ble_dump_xfer.mtu = g_ble_att_mtu;
  g_ble_indicate_waiting = false;
  g_ble_indicate_status = 0;
}

static bool ble_dump_send_line(const std::string& raw) {
  std::string line = raw;
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }
  std::string payload = line + "\n";

  if (g_ble_dump_xfer.mode == BleDumpTxMode::Indicate) {
    for (int attempt = 0; attempt <= kBleDumpIndicateMaxRetries; ++attempt) {
      g_ble_indicate_status = 0;
      g_ble_indicate_waiting = true;
      const int rc = ble_send_payload_raw(payload, true);
      if (rc == 0) {
        if (ble_wait_for_indicate_ack(kBleDumpIndicateAckTimeoutMs)) {
          return true;
        }
      } else {
        g_ble_indicate_waiting = false;
        g_ble_indicate_status = rc;
      }
      if (attempt < kBleDumpIndicateMaxRetries) {
        g_ble_dump_xfer.retries++;
        const int bi = attempt < kBleDumpNotifyMaxRetries ? attempt : (kBleDumpNotifyMaxRetries - 1);
        vTaskDelay(pdMS_TO_TICKS(kBleDumpNotifyBackoffMs[bi]));
      }
    }
    g_ble_dump_xfer.failed_lines++;
    return false;
  }

  int attempt = 0;
  for (; attempt <= kBleDumpNotifyMaxRetries; ++attempt) {
    const int rc = ble_send_payload_raw(payload, false);
    if (rc == 0) break;
    if (attempt < kBleDumpNotifyMaxRetries) {
      g_ble_dump_xfer.retries++;
      vTaskDelay(pdMS_TO_TICKS(kBleDumpNotifyBackoffMs[attempt]));
    }
  }
  if (attempt > kBleDumpNotifyMaxRetries) {
    g_ble_dump_xfer.failed_lines++;
    return false;
  }

  if (attempt > 0) {
    int next_pace = g_ble_dump_xfer.notify_pace_ms + (attempt * 2);
    if (next_pace > kBleDumpNotifyPaceMaxMs) next_pace = kBleDumpNotifyPaceMaxMs;
    g_ble_dump_xfer.notify_pace_ms = next_pace;
  } else if (g_ble_dump_xfer.notify_pace_ms > kBleDumpNotifyPaceMinMs) {
    g_ble_dump_xfer.notify_pace_ms--;
  }
  vTaskDelay(pdMS_TO_TICKS(g_ble_dump_xfer.notify_pace_ms));
  return true;
}

static void apply_ble_enabled_policy(bool runtime_apply) {
  if (!runtime_apply) return;

  if (!g_ble_enabled) {
    if (!ble_stack_stop()) {
      ESP_LOGE(BT_TAG, "BLE OFF rejected: stop failed");
      g_ble_enabled = true;
    }
    return;
  }

  if (!g_ble_stack_active) {
    if (!ble_stack_start()) {
      ESP_LOGE(BT_TAG, "BLE ON rejected: start failed");
      g_ble_enabled = false;
      return;
    }
  }

  g_ble_force_send = true;
  g_ble_last_tick_slot = -1;
  g_ble_last_tick_sec = -1;
  if (g_ble_stack_active && g_ble_synced && g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
    ble_app_advertise();
  }
}

static std::string ble_mac_suffix() {
  uint8_t mac[6] = {};
  if (esp_efuse_mac_get_default(mac) != ESP_OK) {
    return "0000";
  }
  char out[5];
  std::snprintf(out, sizeof(out), "%02X%02X", mac[4], mac[5]);
  return std::string(out);
}

static std::string ble_sanitize_callsign(const std::string& call) {
  std::string out;
  out.reserve(call.size());
  for (unsigned char ch : call) {
    if (std::isalnum(ch)) {
      out.push_back(static_cast<char>(std::toupper(ch)));
    } else if (ch == '/' || ch == '_' || ch == '-') {
      out.push_back('_');
    }
    if (out.size() >= 12) break;
  }
  while (!out.empty() && out.back() == '_') out.pop_back();
  return out;
}

static void ble_update_name_from_station(bool restart_adv) {
  std::string suffix = ble_sanitize_callsign(g_call);
  if (suffix.empty()) suffix = ble_mac_suffix();
  std::string desired = std::string("PaperFT8-") + suffix;
  if (desired.size() > 24) desired.resize(24);
  if (desired.empty()) desired = "PaperFT8";

  g_ble_adv_name = desired;
  if (!g_ble_stack_active) return;
  int rc = ble_svc_gap_device_name_set(g_ble_adv_name.c_str());
  if (rc != 0) {
    ESP_LOGW(BT_TAG, "ble_svc_gap_device_name_set rc=%d", rc);
  }

  if (restart_adv && g_ble_stack_active && g_ble_enabled && g_ble_synced && g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
    ble_app_advertise();
  }
}

static const char* ble_page_label(UIMode mode) {
  switch (mode) {
    case UIMode::RX: return "RX";
    case UIMode::TX: return "TX";
    case UIMode::BAND: return "BAND";
    case UIMode::MENU: return "MENU";
    case UIMode::CONTROL: return "CONTROL";
    case UIMode::DEBUG: return "DEBUG";
    case UIMode::STATUS: return "STATUS";
    case UIMode::QSO: return "QSO";
  }
  return "PAGE";
}

static void ble_page_meta(int& cur, int& total) {
  cur = 1;
  total = 1;
  switch (ui_mode) {
    case UIMode::RX:
      ui_get_rx_page_info(cur, total);
      break;
    case UIMode::TX:
      total = page_count(autoseq_queue_size(), 5);
      cur = tx_page + 1;
      break;
    case UIMode::BAND:
      total = page_count((int)g_bands.size(), 6);
      cur = band_page + 1;
      break;
    case UIMode::MENU:
      total = 3;
      cur = menu_page + 1;
      break;
    case UIMode::DEBUG:
      total = page_count((int)g_debug_lines.size(), 6);
      cur = debug_page + 1;
      break;
    case UIMode::QSO:
      total = page_count((int)g_q_lines.size(), 6);
      cur = q_page + 1;
      break;
    default:
      break;
  }
  if (total < 1) total = 1;
  if (cur < 1) cur = 1;
  if (cur > total) cur = total;
}

static std::string ble_meta_line() {
  int cur = 1;
  int total = 1;
  ble_page_meta(cur, total);

  char meta[96];
  const char up = (cur > 1) ? 'u' : '-';
  const char down = (cur < total) ? 'v' : '-';
  std::snprintf(meta, sizeof(meta), "[%s %c%c]", ble_page_label(ui_mode), up, down);
  return std::string(meta);
}

static std::string ble_menu_long_edit_label() {
  switch (menu_long_kind) {
    case LONG_FT:
      return "F";
    case LONG_COMMENT:
      return "C";
    case LONG_ACTIVE:
      return "ActiveBand";
    case LONG_IGNORE:
      return "IgnoreList";
    case LONG_NONE:
    default:
      return "Edit";
  }
}

static std::string ble_text_mode_line7() {
  std::string item = "Edit";

  if (menu_delete_confirm) {
    item = "Delete Logs";
  } else if (menu_long_edit) {
    item = ble_menu_long_edit_label();
  } else if (menu_edit_idx >= 0) {
    item = menu_edit_label(menu_edit_idx);
  } else if (band_edit_idx >= 0 && band_edit_idx < (int)g_bands.size()) {
    item = std::string("Band ") + g_bands[band_edit_idx].name;
  } else if (status_edit_idx == 4) {
    item = "Date";
  } else if (status_edit_idx == 5) {
    item = "Time";
  }

  return std::string("[Edit ") + item + "]";
}

static void ble_slot_second_now(int64_t& slot_idx, int& sec, bool& even_slot) {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_ms = now_ms % 15000;
  if (slot_ms < 0) slot_ms += 15000;
  slot_idx = (now_ms - slot_ms) / 15000;
  sec = (int)(slot_ms / 1000);
  if (sec < 0) sec = 0;
  if (sec > 14) sec = 14;
  even_slot = ((slot_idx & 1) == 0);
}

static std::string ble_timing_token(int sec, bool even_slot, bool txing) {
  if (sec == 0) return "|";
  if (sec == 4) return "4";
  if (sec == 8) return "8";
  if (sec == 12) return "12";
  if (txing && (sec == 2 || sec == 6 || sec == 10 || sec == 14)) return "o";
  return even_slot ? ":" : ".";
}

static void ble_notify_line(const std::string& raw) {
  std::string line = raw;
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }
  ble_notify_payload(line + "\n");
}

static void ble_start_qso_pick_mode() {
  if (g_ble_qso_pick_mode) return;
  g_ble_qso_return_mode = ui_mode;
  g_ble_qso_pick_mode = true;
  g_ble_dump_in_progress = false;
  g_q_show_entries = false;
  q_page = 0;
  enter_mode(UIMode::QSO);
  ui_draw_list(g_q_lines, q_page, -1);
  g_ble_force_send = true;
}

static void ble_cancel_qso_pick_mode() {
  if (!g_ble_qso_pick_mode) return;
  g_ble_qso_pick_mode = false;
  g_ble_dump_in_progress = false;
  if (ble_cmd_queue) xQueueReset(ble_cmd_queue);
  enter_mode(g_ble_qso_return_mode);
  g_ble_force_send = true;
}

static void ble_dump_qso_file(const std::string& file_name) {
  g_ble_dump_in_progress = true;
  const bool use_indicate = g_ble_tx_indicate_enabled;
  ble_dump_reset_transfer_state(use_indicate);
  std::string full_path = std::string("/spiffs/") + file_name;
  if (!use_indicate) {
    (void)ble_dump_send_line("fallback notify mode (best effort)");
  }
  (void)ble_dump_send_line(std::string("\n--- <") + file_name + "> ---");

  FILE* f = fopen(full_path.c_str(), "r");
  if (!f) {
    (void)ble_dump_send_line("Open fail");
  } else {
    char line[256];
    while (fgets(line, sizeof(line), f)) {
      g_ble_dump_xfer.file_lines++;
      (void)ble_dump_send_line(line);
    }
    fclose(f);
  }
  (void)ble_dump_send_line("--- EOF ---");
  {
    char summary[160];
    std::snprintf(summary, sizeof(summary),
                  "F summary mode=%s mtu=%u lines=%d retries=%d failed=%d",
                  use_indicate ? "INDICATE" : "NOTIFY",
                  (unsigned)g_ble_dump_xfer.mtu,
                  g_ble_dump_xfer.file_lines,
                  g_ble_dump_xfer.retries,
                  g_ble_dump_xfer.failed_lines);
    (void)ble_dump_send_line(summary);
  }

  g_ble_dump_in_progress = false;
  g_ble_indicate_waiting = false;
  g_ble_dump_xfer.active = false;
  if (ui_mode == UIMode::QSO) {
    ui_draw_list(g_q_lines, q_page, -1);
  }
  g_ble_force_send = true;
}

static void ble_try_dump_qso_file_by_key(char key) {
  if (key < '1' || key > '6') return;
  int idx = q_page * 6 + (key - '1');
  if (idx < 0 || idx >= (int)g_q_files.size()) return;
  ble_dump_qso_file(g_q_files[idx]);
}

static void ble_mirror_tick() {
  if (!g_ble_enabled) return;
  if (g_ble_dump_in_progress) return;
  if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

  std::vector<std::string> lines;
  ui_get_visible_text_lines(lines);
  while ((int)lines.size() < 6) lines.push_back("");

  const std::string line7 = g_ble_text_mode ? ble_text_mode_line7() : ble_meta_line();

  std::string screen_key;
  screen_key.reserve(256);
  for (int i = 0; i < 6; ++i) {
    screen_key += lines[i];
    screen_key.push_back('\n');
  }
  screen_key += line7;

  if (g_ble_force_send || screen_key != g_ble_last_payload) {
    g_ble_force_send = false;
    g_ble_last_payload = screen_key;
    if (g_ble_last_tick_slot < 0 || g_ble_last_tick_sec < 0) {
      int64_t slot_idx = 0;
      int sec = 0;
      bool even_slot = true;
      ble_slot_second_now(slot_idx, sec, even_slot);
      (void)even_slot;
      g_ble_last_tick_slot = slot_idx;
      g_ble_last_tick_sec = sec;
    }
    std::string out;
    out.reserve(screen_key.size() + 40);
    out += "\n==========================\n";
    out += screen_key;
    ble_notify_payload(out);
  }
}

static void ble_countdown_tick() {
  if (!g_ble_enabled) return;
  if (g_ble_dump_in_progress) return;
  if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

  int64_t slot_idx = 0;
  int sec = 0;
  bool even_slot = true;
  ble_slot_second_now(slot_idx, sec, even_slot);

  if (g_ble_last_tick_slot == slot_idx && g_ble_last_tick_sec == sec) return;
  if (g_ble_last_tick_slot < 0 || g_ble_last_tick_sec < 0) {
    g_ble_last_tick_slot = slot_idx;
    g_ble_last_tick_sec = sec;
    return;
  }

  g_ble_last_tick_slot = slot_idx;
  g_ble_last_tick_sec = sec;
  ble_notify_payload(ble_timing_token(sec, even_slot, g_tx_active));
}

static void ble_on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "ensure addr failed: %d", rc);
    return;
  }
  rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "infer auto addr failed: %d", rc);
    return;
  }
  g_ble_synced = true;
  ble_update_name_from_station(false);
  ble_app_advertise();
}

static void nimble_host_task(void* param) {
  (void)param;
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void ble_app_advertise(void) {
  if (!g_ble_stack_active) return;
  if (!g_ble_enabled) return;
  if (!g_ble_synced) return;
  if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) return;

  struct ble_gap_adv_params adv{};
  adv.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

  struct ble_hs_adv_fields fields{};
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  const std::string name = g_ble_adv_name.empty() ? std::string("PaperFT8") : g_ble_adv_name;
  fields.name = (uint8_t*)name.c_str();
  fields.name_len = name.size();
  fields.name_is_complete = 1;

  ble_gap_adv_stop();
  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "adv_set_fields failed: %d", rc);
    return;
  }
  rc = ble_gap_adv_start(g_own_addr_type, nullptr, BLE_HS_FOREVER, &adv, gap_cb, nullptr);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "adv_start failed: %d", rc);
  } else {
    ESP_LOGI(BT_TAG, "Advertising as %s", name.c_str());
  }
}

#else  // ENABLE_BLE
static bool ble_pop_input(BleUiInput& out) { (void)out; return false; }
static void ble_update_name_from_station(bool restart_adv) { (void)restart_adv; }
static void apply_ble_enabled_policy(bool runtime_apply) { (void)runtime_apply; }
static void ble_mirror_tick() {}
static void ble_countdown_tick() {}
#endif // ENABLE_BLE

static std::string trim_copy(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && isspace((unsigned char)s[b])) ++b;
  while (e > b && isspace((unsigned char)s[e - 1])) --e;
  return s.substr(b, e - b);
}

static uint32_t parse_crc_hex(const std::string& hex) {
  if (hex.empty()) return 0;
  char* end = nullptr;
  unsigned long v = strtoul(hex.c_str(), &end, 16);
  if (end == hex.c_str() || *end != '\0') return 0;
  return (uint32_t)v;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = crc ^ 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static void host_debug_hex8(const char* prefix, const uint8_t* b) {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%s ", prefix);
  for (int i = 0; i < 8 && n + 3 < (int)sizeof(buf); ++i) {
    n += snprintf(buf + n, sizeof(buf) - n, "%02X ", b[i]);
  }
  if (n > 0 && buf[n - 1] == ' ') buf[n - 1] = 0;
  host_write_str(std::string(buf) + "\r\n");
}

static void host_handle_line(const std::string& line_in) {
  bool send_prompt = true;
  std::string line = trim_copy(line_in);
  if (line.empty()) { /* host_write_str(HOST_PROMPT);*/ return; }
  debug_log_line(std::string("[HOST RX] ") + line);
  //std::string echo = std::string("ECHO: ") + line + "\r\n";
  //host_write_str(echo);

  auto to_upper = [](std::string s) {
    for (auto& c : s) c = toupper((unsigned char)c);
    return s;
  };
  std::istringstream iss(line);
  std::string cmd;
  iss >> cmd;
  std::string cmd_up = to_upper(cmd);
  std::string rest;
  std::getline(iss, rest);
  rest = trim_copy(rest);

  auto send = [](const std::string& msg) { host_write_str(msg + "\r\n"); };

  if (cmd_up == "WRITE" || cmd_up == "APPEND") {
    std::istringstream rs(rest);
    std::string fname;
    rs >> fname;
    std::string content;
    std::getline(rs, content);
    content = trim_copy(content);
    if (fname.empty()) {
      send("ERROR: filename required");
    } else {
      std::string path = std::string("/spiffs/") + fname;
      const char* mode = (cmd_up == "WRITE") ? "w" : "a";
      FILE* f = fopen(path.c_str(), mode);
      if (!f) send("ERROR: open failed");
      else { fwrite(content.data(), 1, content.size(), f); fclose(f); send("OK"); }
    }
  } else if (cmd_up == "READ") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      std::string path = std::string("/spiffs/") + rest;
      FILE* f = fopen(path.c_str(), "r");
      if (!f) send("ERROR: open failed");
      else {
        char buf[128];
        while (fgets(buf, sizeof(buf), f)) host_write_str(std::string(buf));
        fclose(f);
        //send("OK");
        send_prompt = false;
      }
    }
  } else if (cmd_up == "DELETE") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      std::string path = std::string("/spiffs/") + rest;
      if (unlink(path.c_str()) == 0) send("OK"); else send("ERROR: delete failed");
    }
  } else if (cmd_up == "LIST") {
    DIR* d = opendir("/spiffs");
    if (!d) send("ERROR: opendir failed");
    else {
      struct dirent* ent;
      while ((ent = readdir(d)) != nullptr) {
        send(ent->d_name);
      }
      closedir(d);
      send("OK");
    }
  } else if (cmd_up == "WRITEBIN") {
    std::istringstream rs(rest);
    std::string fname;
    size_t size = 0;
    std::string crc_hex;
    rs >> fname >> size >> crc_hex;
    uint32_t crc_exp = parse_crc_hex(crc_hex);
    if (fname.empty() || size == 0 || crc_hex.empty()) {
      send("ERROR: filename, size, crc32_hex required");
    } else if (host_bin_active) {
      send("ERROR: binary upload in progress");
    } else {
      std::string path = std::string("/spiffs/") + fname;
      FILE* f = fopen(path.c_str(), "wb");
      if (!f) {
        send("ERROR: open failed");
      } else {
        host_bin_path = path;
        host_bin_active = true;
        host_bin_remaining = size;
        host_bin_fp = f;
        host_bin_crc = 0;
        host_bin_expected_crc = crc_exp;
        host_bin_received = 0;
        host_bin_buf.clear();
        host_bin_buf.reserve(HOST_BIN_CHUNK);
        host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        host_bin_first_filled = 0;
        memset(host_bin_first8, 0, sizeof(host_bin_first8));
        memset(host_bin_last8, 0, sizeof(host_bin_last8));
        host_write_str("OK: send " + std::to_string(size) + " bytes, chunk " + std::to_string(HOST_BIN_CHUNK) + " +4crc\r\n");
        send_prompt = false; // prompt after binary upload completes
      }
    }
  } else if (cmd_up == "DATE") {
    if (rest.empty()) {
      send("DATE " + g_date);
    } else {
      int y, M, d;
      if (sscanf(rest.c_str(), "%d-%d-%d", &y, &M, &d) != 3 ||
          y < 2024 || y > 2099 || M < 1 || M > 12 || d < 1 || d > 31) {
        send("ERROR: use DATE YYYY-MM-DD");
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d", y, M, d);
        g_date = buf;
        if (rtc_set_from_strings()) { rtc_sync_to_hw(); save_station_data(); send("OK"); }
        else send("ERROR: invalid date");
      }
    }
  } else if (cmd_up == "TIME") {
    if (rest.empty()) {
      send("TIME " + g_time);
    } else {
      int h, m, s;
      if (sscanf(rest.c_str(), "%d:%d:%d", &h, &m, &s) != 3 ||
          h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
        send("ERROR: use TIME HH:MM:SS");
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s);
        g_time = buf;
        if (rtc_set_from_strings()) { rtc_sync_to_hw(); save_station_data(); send("OK"); }
        else send("ERROR: invalid time");
      }
    }
  } else if (cmd_up == "SLEEP") {
    if (rtc_valid) {
      // Compute current time in milliseconds, round up to next second boundary
      int64_t elapsed_ms = esp_timer_get_time() / 1000 - rtc_ms_start;
      int64_t now_ms = (time_t)rtc_epoch_base * 1000LL + elapsed_ms;
      int64_t frac = now_ms % 1000;
      int64_t wait_ms = (frac > 0) ? (1000 - frac) : 0;
      time_t sleep_epoch = (time_t)((now_ms + 999) / 1000);  // ceil to next second

      g_rtc_sleep_epoch = sleep_epoch;
      save_station_data();

      // Wait until the second boundary, then set HW RTC and sleep
      if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));
      struct timeval tv = { .tv_sec = sleep_epoch, .tv_usec = 0 };
      settimeofday(&tv, NULL);
      if (M5.Rtc.isEnabled()) {
        struct tm t_utc;
        gmtime_r(&sleep_epoch, &t_utc);
        M5.Rtc.setDateTime(&t_utc);
      }
    }
    send("OK: entering deep sleep");
    M5.Display.sleep();
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_deep_sleep_start();
  } else if (cmd_up == "INFO") {
    send("Heap: " + std::to_string(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)));
    send("OK");
  } else if (cmd_up == "HELP") {
    for (auto& l : g_ctrl_lines) send(l);
  } else if (cmd_up == "EXIT") {
    send("OK: exit host");
    enter_mode(UIMode::RX);
    return;
  } else {
    send("ERROR: Unknown command. Type HELP.");
  }

  if (send_prompt) host_write_str(std::string(HOST_PROMPT));
}

static void host_process_bytes(const uint8_t* buf, size_t len) {
  ESP_LOGD(TAG, "host_process_bytes len=%u", (unsigned)len);
  for (size_t i = 0; i < len; ) {
    if (host_bin_active) {
      // Skip any stray CR/LF before first payload byte
      if (host_bin_received == 0 && host_bin_buf.empty() && (buf[i] == '\r' || buf[i] == '\n')) {
        ++i;
        continue;
      }
      size_t payload_need = host_bin_chunk_expect;
      size_t total_need = payload_need + 4; // payload + crc32 trailer
      size_t avail = len - i;
      size_t copy = total_need - host_bin_buf.size();
      if (copy > avail) copy = avail;
      host_bin_buf.insert(host_bin_buf.end(), buf + i, buf + i + copy);
      i += copy;

      if (host_bin_buf.size() >= total_need) {
        size_t payload_len = payload_need;
        uint32_t recv_crc = (uint32_t(host_bin_buf[payload_len])) |
                            (uint32_t(host_bin_buf[payload_len + 1]) << 8) |
                            (uint32_t(host_bin_buf[payload_len + 2]) << 16) |
                            (uint32_t(host_bin_buf[payload_len + 3]) << 24);
        uint32_t calc_crc = crc32_update(0, host_bin_buf.data(), payload_len);
        if (calc_crc != recv_crc) {
          char dbg[128];
          snprintf(dbg, sizeof(dbg), "ERROR: chunk crc off=%u len=%u calc=%08X recv=%08X\r\n",
                   (unsigned)(host_bin_received + payload_len), (unsigned)payload_len,
                   (unsigned)calc_crc, (unsigned)recv_crc);
          host_write_str(std::string(dbg));
          // Send first/last bytes of the chunk to compare
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK FIRST8", host_bin_buf.data());
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK LAST8", host_bin_buf.data() + payload_len - 8);
          if (payload_len < 8) host_debug_hex8("DBG CHUNK PART", host_bin_buf.data());
          // Also report the CRC trailer bytes as seen
          uint8_t crc_bytes[4] = {
            host_bin_buf[payload_len],
            host_bin_buf[payload_len + 1],
            host_bin_buf[payload_len + 2],
            host_bin_buf[payload_len + 3]
          };
          host_debug_hex8("DBG CRC BYTES", crc_bytes);
          fclose(host_bin_fp);
          host_bin_fp = nullptr;
          host_bin_active = false;
          host_bin_remaining = 0;
          host_bin_buf.clear();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }

        // Capture first/last bytes for debugging
        if (host_bin_first_filled < 8) {
          size_t need = 8 - host_bin_first_filled;
          if (need > payload_len) need = payload_len;
          memcpy(host_bin_first8 + host_bin_first_filled, host_bin_buf.data(), need);
          host_bin_first_filled += need;
        }
        // update last8 buffer
        if (payload_len >= 8) {
          memcpy(host_bin_last8, host_bin_buf.data() + payload_len - 8, 8);
        } else {
          // shift existing and append
          size_t shift = (payload_len + 8 > 8) ? (payload_len) : payload_len;
          if (shift > 0) {
            memmove(host_bin_last8, host_bin_last8 + shift, 8 - shift);
            memcpy(host_bin_last8 + (8 - payload_len), host_bin_buf.data(), payload_len);
          }
        }

        size_t written = fwrite(host_bin_buf.data(), 1, payload_len, host_bin_fp);
        if (written != payload_len) {
          host_write_str("ERROR: write failed\r\n");
          fclose(host_bin_fp);
          host_bin_fp = nullptr;
          host_bin_active = false;
          host_bin_remaining = 0;
          host_bin_buf.clear();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }
        host_bin_crc = crc32_update(host_bin_crc, host_bin_buf.data(), payload_len);
        host_bin_remaining -= payload_len;
        host_bin_received += payload_len;
        host_bin_buf.clear();
        host_write_str("ACK " + std::to_string(host_bin_received) + "\r\n");

        if (host_bin_remaining == 0) {
          fclose(host_bin_fp);
          host_bin_fp = nullptr;
          host_bin_active = false;
          uint32_t crc_final = host_bin_crc;
          // Reopen file to send first/last 8 bytes for debugging
          host_debug_hex8("DBG FIRST8", host_bin_first8);
          host_debug_hex8("DBG LAST8", host_bin_last8);
          char crc_line[64];
          snprintf(crc_line, sizeof(crc_line), "DBG CRC %08X EXPECT %08X\r\n",
                   (unsigned)crc_final, (unsigned)host_bin_expected_crc);
          host_write_str(std::string(crc_line));
          if (crc_final != host_bin_expected_crc) {
            host_write_str("ERROR: crc mismatch\r\n");
          } else {
            host_write_str("OK crc " + std::to_string(crc_final) + "\r\n");
          }
          host_write_str(std::string(HOST_PROMPT));
        } else {
          host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        }
      }
      continue;
    }
    char ch = (char)buf[i++];
    if (ch == '\r' || ch == '\n') {
      if (!host_input.empty()) {
    //ESP_LOGI(TAG, "HOST line: %s", host_input.c_str());
        host_handle_line(host_input);
        host_input.clear();
      } else {
        //host_write_str(std::string(HOST_PROMPT));
      }
    } else if (ch == 0x08 || ch == 0x7f) {
      if (!host_input.empty()) host_input.pop_back();
    } else if (ch >= 32 && ch < 127) {
      host_input.push_back(ch);
    }
  }
}

static void poll_host_uart() {
  ensure_usb();
  if (!usb_ready) return;
  uint8_t buf[512];
  while (true) {
    int r = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    if (r <= 0) break;
    host_process_bytes(buf, (size_t)r);
  }
}

static void load_station_data() {
  // If Station.ini exists on SD, prefer it by copying onto SPIFFS first.
  // If mount/copy fails, fall back to the on-device SPIFFS Station.ini.
  sync_station_ini_from_sd_to_spiffs();

  g_rtc_comp = 0;
  g_autoseq_max_retry = AUTOSEQ_MAX_RETRY;
  g_ble_enabled = true;

  FILE* f = fopen(STATION_FILE, "r");
  if (!f) {
    autoseq_set_max_retry(g_autoseq_max_retry);
    apply_ble_enabled_policy(false);
    return;
  }
  char line[128];
  while (fgets(line, sizeof(line), f)) {
    int idx = -1;
    int val = 0;
    if (sscanf(line, "band%d=%d", &idx, &val) == 2) {
      if (idx >= 0 && idx < (int)g_bands.size()) {
        g_bands[idx].freq = val;
      }
    } else if (sscanf(line, "beacon=%d", &val) == 1) {
      // beacon persists OFF only; ignore saved value
    } else if (sscanf(line, "offset=%d", &val) == 1) {
      g_offset_hz = val;
    } else if (sscanf(line, "band_sel=%d", &val) == 1) {
      if (val >= 0 && val < (int)g_bands.size()) g_band_sel = val;
    } else if (strncmp(line, "date=", 5) == 0) {
      g_date = trim_copy(line + 5);
    } else if (strncmp(line, "time=", 5) == 0) {
      g_time = trim_copy(line + 5);
    } else if (sscanf(line, "cq_type=%d", &val) == 1) {
      if (val >= 0 && val <= 5) g_cq_type = (CqType)val;
    } else if (sscanf(line, "offset_src=%d", &val) == 1) {
      if (val >= 0 && val <= 2) g_offset_src = (OffsetSrc)val;
    } else if (sscanf(line, "radio=%d", &val) == 1) {
      if (val >= 0 && val <= 2) g_radio = (RadioType)val;
    } else if (strncmp(line, "cq_ft=", 6) == 0) {
      g_cq_freetext = trim_copy(line + 6);
    } else if (strncmp(line, "free_text=", 10) == 0) {
      g_free_text = trim_copy(line + 10);
    } else if (strncmp(line, "call=", 5) == 0) {
      g_call = trim_copy(line + 5);
    } else if (strncmp(line, "grid=", 5) == 0) {
      g_grid = trim_copy(line + 5);
    } else if (strncmp(line, "comment1=", 9) == 0) {
      g_comment1 = trim_copy(line + 9);
    } else if (strncmp(line, "ignore_prefixes=", 16) == 0) {
      g_ignore_prefix_text = clamp_ignore_prefix_text(trim_copy(line + 16));
    } else if (sscanf(line, "rxtx_log=%d", &val) == 1) {
      g_rxtx_log = (val != 0);
    } else if (sscanf(line, "skiptx1=%d", &val) == 1) {
      g_skip_tx1 = (val != 0); autoseq_set_skip_tx1(g_skip_tx1);
    } else if (sscanf(line, "active_band=%d", &val) == 1) { // legacy single value
      g_active_band_text = std::to_string(val);
    } else if (strncmp(line, "active_bands=", 13) == 0) {
      g_active_band_text = trim_copy(line + 13);
    } else if (sscanf(line, "autoseq_max_retry=%d", &val) == 1) {
      if (val >= 0) g_autoseq_max_retry = val;
    } else if (sscanf(line, "ble_enabled=%d", &val) == 1) {
      g_ble_enabled = (val != 0);
    } else if (sscanf(line, "rtc_comp=%d", &val) == 1) {
      // Legacy key kept for file compatibility; ignored in PaperFT8.
    } else {
      long long epoch_tmp = 0;
      if (sscanf(line, "rtc_sleep_epoch=%lld", &epoch_tmp) == 1) {
        g_rtc_sleep_epoch = (time_t)epoch_tmp;
      }
    }
  }
  fclose(f);
  autoseq_set_max_retry(g_autoseq_max_retry);
  // Try hardware RTC first (persists through deep sleep), fall back to saved strings
  if (!rtc_init_from_hw()) {
    ESP_LOGI(TAG, "Hardware RTC not valid, using saved time strings");
    rtc_set_from_strings();
  }
  rebuild_active_bands();
  rebuild_ignore_prefixes();
  g_beacon = BeaconMode::OFF; // force off on load
  apply_ble_enabled_policy(false);
}

static void save_station_data() {
  FILE* f = fopen(STATION_FILE, "w");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s for write", STATION_FILE);
    return;
  }
  for (size_t i = 0; i < g_bands.size(); ++i) {
    fprintf(f, "band%u=%d\n", (unsigned)i, g_bands[i].freq);
  }
  // Beacon is not persisted (stays OFF on reload)
  fprintf(f, "offset=%d\n", g_offset_hz);
  fprintf(f, "band_sel=%d\n", g_band_sel);
  fprintf(f, "date=%s\n", g_date.c_str());
  fprintf(f, "time=%s\n", g_time.c_str());
  fprintf(f, "cq_type=%d\n", (int)g_cq_type);
  fprintf(f, "cq_ft=%s\n", g_cq_freetext.c_str());
  fprintf(f, "skiptx1=%d\n", g_skip_tx1 ? 1 : 0);
  fprintf(f, "free_text=%s\n", g_free_text.c_str());
  fprintf(f, "call=%s\n", g_call.c_str());
  fprintf(f, "grid=%s\n", g_grid.c_str());
  fprintf(f, "offset_src=%d\n", (int)g_offset_src);
  fprintf(f, "radio=%d\n", (int)g_radio);
  fprintf(f, "comment1=%s\n", g_comment1.c_str());
  fprintf(f, "ignore_prefixes=%s\n", g_ignore_prefix_text.c_str());
  fprintf(f, "rxtx_log=%d\n", g_rxtx_log ? 1 : 0);
  fprintf(f, "active_bands=%s\n", g_active_band_text.c_str());
  fprintf(f, "rtc_sleep_epoch=%lld\n", (long long)g_rtc_sleep_epoch);
  fprintf(f, "autoseq_max_retry=%d\n", g_autoseq_max_retry);
  fprintf(f, "ble_enabled=%d\n", g_ble_enabled ? 1 : 0);
  fclose(f);
}

static void enter_mode(UIMode new_mode) {
  // No special handling needed when leaving TX mode - autoseq manages queue internally
  if (ui_mode == UIMode::STATUS && new_mode != UIMode::STATUS) {
    if (g_beacon != g_status_beacon_temp) {
      bool was_off = (g_beacon == BeaconMode::OFF);
      g_beacon = g_status_beacon_temp;
      save_station_data();
      // No need to clear autoseq when beacon is turned off.
      // Any CQ in queue will transmit once, then tick moves CALLING→IDLE.
      g_tx_view_dirty = true;

      // If beacon was just enabled, enqueue CQ and set TX flag
      // TX will trigger at next slot boundary via check_slot_boundary()
      if (was_off && g_beacon != BeaconMode::OFF) {
        enqueue_beacon_cq();
        AutoseqTxEntry pending;
        if (autoseq_fetch_pending_tx(pending)) {
          g_qso_xmit = true;
          g_target_slot_parity = pending.slot_id & 1;
          g_pending_tx = pending;
          g_pending_tx_valid = true;
        }
      }
    }
    status_edit_idx = -1;
    status_edit_buffer.clear();
  }
  if (new_mode != UIMode::QSO) {
    g_ble_qso_pick_mode = false;
    g_ble_dump_in_progress = false;
  }
  ui_mode = new_mode;
  rx_flash_idx = -1;
  rx_flash_deadline = 0;
  switch (ui_mode) {
    case UIMode::RX:
      ui_set_active_mode_button('R');
      ui_draw_mode_box("R");
      // Force RX list redraw
      ui_force_redraw_rx();
      ui_draw_rx();
      break;
    case UIMode::TX:
      ui_set_active_mode_button('T');
      ui_draw_mode_box("T");
      tx_page = 0;
      redraw_tx_view();
      break;
    case UIMode::BAND:
      ui_set_active_mode_button('B');
      band_page = 0;
      band_edit_idx = -1;
      draw_band_view();
      break;
    case UIMode::MENU:
      ui_set_active_mode_button('M');
      ui_draw_mode_box("S");
      menu_page = 0;
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      menu_delete_confirm = false;
      draw_menu_view();
      break;
    case UIMode::DEBUG:
      ui_set_active_mode_button('D');
      debug_page = (int)((g_debug_lines.size() - 1) / 6);
      ui_draw_debug(g_debug_lines, debug_page);
      break;
    case UIMode::CONTROL:
      ui_set_active_mode_button('C');
      ui_draw_mode_box("S");
      ui_draw_list(g_ctrl_lines, 0, -1);
      host_input.clear();
      ensure_usb();
      if (usb_ready) {
        host_write_str("READY\r\n");
        host_write_str(std::string(HOST_PROMPT));
      } else {
        debug_log_line("USB serial not ready");
      }
      break;
    case UIMode::QSO:
      ui_set_active_mode_button('Q');
      ui_draw_mode_box("Q");
      g_q_show_entries = false;
      q_page = 0;
      if (g_ble_qso_pick_mode) {
        qso_load_fetch_file_list();
      } else {
        qso_load_file_list();
      }
      ui_draw_list(g_q_lines, q_page, -1);
      break;
    case UIMode::STATUS:
      ui_set_active_mode_button('S');
      ui_draw_mode_box("S");
      g_status_beacon_temp = g_beacon;
      status_edit_idx = -1;
      status_cursor_pos = -1;
      draw_status_view();
      break;
  }
}

#if ENABLE_BLE
static void ble_enter_text_mode() {
  g_ble_text_mode = true;
}

static void ble_exit_text_mode() {
  g_ble_text_mode = false;
}

static bool ble_text_target_active() {
  return menu_delete_confirm ||
         menu_long_edit ||
         menu_edit_idx >= 0 ||
         band_edit_idx >= 0 ||
         status_edit_idx == 4 ||
         status_edit_idx == 5;
}

static void ble_commit_text_input(const BleUiInput& input) {
  std::string value = ble_trim_trailing_crlf(input.data, input.len);

  if (menu_delete_confirm) {
    const char ans = value.empty() ? '\0' : value[0];
    if (ans == 'Y' || ans == 'y') {
      esp_err_t err = delete_logs_on_spiffs_keep_stationdata();
      menu_delete_confirm = false;
      menu_flash_idx = 17; // abs index of line 6 on page 2
      menu_flash_deadline = rtc_now_ms() + 500;
      debug_log_line(err == ESP_OK ? "Logs deleted" : "Delete failed");
      draw_menu_view();
    } else {
      menu_delete_confirm = false;
      draw_menu_view();
    }
    ble_exit_text_mode();
    return;
  }

  if (menu_long_edit) {
    const size_t kTouchTextMax = sizeof(g_touchkbd_buffer) - 1;
    if (value.size() > kTouchTextMax) value.resize(kTouchTextMax);
    if (menu_long_kind == LONG_IGNORE && value.size() > kIgnorePrefixTextMaxLen) {
      value.resize(kIgnorePrefixTextMaxLen);
    }
    menu_long_buf = value;
    if (menu_long_kind == LONG_FT) {
      g_free_text = menu_long_buf;
      if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
      update_autoseq_cq_type();
    } else if (menu_long_kind == LONG_COMMENT) {
      g_comment1 = menu_long_buf;
    } else if (menu_long_kind == LONG_ACTIVE) {
      g_active_band_text = menu_long_buf;
      rebuild_active_bands();
    } else if (menu_long_kind == LONG_IGNORE) {
      g_ignore_prefix_text = clamp_ignore_prefix_text(menu_long_buf);
      rebuild_ignore_prefixes();
    }
    save_station_data();
    menu_long_edit = false;
    menu_long_kind = LONG_NONE;
    menu_long_buf.clear();
    menu_long_backup.clear();
    draw_menu_view();
    ble_exit_text_mode();
    return;
  }

  if (menu_edit_idx >= 0) {
    const size_t max_len = (menu_edit_idx == 7 || menu_edit_idx == 15) ? 10 : (sizeof(g_touchkbd_buffer) - 1);
    if (value.size() > max_len) value.resize(max_len);
    menu_edit_buf = value;

    // Absolute indices across pages.
    if (menu_edit_idx == 3) {
      g_call = menu_edit_buf;
      autoseq_set_station(g_call, g_grid);
      ble_update_name_from_station(true);
    } else if (menu_edit_idx == 4) {
      g_grid = menu_edit_buf;
      autoseq_set_station(g_call, g_grid);
    } else if (menu_edit_idx == 7) {
      g_offset_hz = atoi(menu_edit_buf.c_str());
    } else if (menu_edit_idx == 10) {
      g_comment1 = menu_edit_buf;
    } else if (menu_edit_idx == 15) {
      int v = atoi(menu_edit_buf.c_str());
      if (v < 0) v = 0;
      g_autoseq_max_retry = v;
      autoseq_set_max_retry(g_autoseq_max_retry);
    }
    save_station_data();
    menu_edit_idx = -1;
    menu_edit_buf.clear();
    draw_menu_view();
    ble_exit_text_mode();
    return;
  }

  if (band_edit_idx >= 0) {
    if (value.size() > 10) value.resize(10);
    band_edit_buffer = value;
    if (!band_edit_buffer.empty()) {
      char* end = nullptr;
      long v = std::strtol(band_edit_buffer.c_str(), &end, 10);
      if (end != band_edit_buffer.c_str() && *end == '\0') {
        g_bands[band_edit_idx].freq = (int)v;
        save_station_data();
      }
    }
    band_edit_idx = -1;
    band_edit_buffer.clear();
    draw_band_view();
    ble_exit_text_mode();
    return;
  }

  if (status_edit_idx == 4 || status_edit_idx == 5) {
    const bool is_date = (status_edit_idx == 4);
    const size_t max_len = is_date ? 8 : 6;
    if (value.size() > max_len) value.resize(max_len);
    status_edit_buffer = value;

    const std::string prev_date = g_date;
    const std::string prev_time = g_time;
    std::string formatted;
    bool ok = is_date
        ? format_date_digits_exact(status_edit_buffer, formatted)
        : format_time_digits_exact(status_edit_buffer, formatted);

    if (ok) {
      if (is_date) g_date = formatted;
      else g_time = formatted;
      if (rtc_set_from_strings()) {
        save_station_data();
        rtc_sync_to_hw();
      } else {
        ok = false;
      }
    }

    if (!ok) {
      g_date = prev_date;
      g_time = prev_time;
      debug_log_line(is_date ? "Invalid date; unchanged" : "Invalid time; unchanged");
    }

    status_edit_idx = -1;
    status_cursor_pos = -1;
    status_edit_buffer.clear();
    draw_status_view();
    ble_exit_text_mode();
    return;
  }

  ble_exit_text_mode();
}
#endif

static void app_task_core0(void* /*param*/) {
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true
  };
  ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

  // Initialize mutexes for thread-safe operations
  log_mutex = xSemaphoreCreateMutex();

  ui_init();
  hashtable_init();

  // Initialize autoseq engine
  autoseq_init();
  
// Cabrillo Field Day log callback (implemented in autoseq.cpp; declared here to avoid header churn)
using CabrilloFdLogCallback = void (*)(const std::string& dxcall, const std::string& their_fd_exchange);
extern void autoseq_set_cabrillo_fd_callback(CabrilloFdLogCallback cb);

autoseq_set_adif_callback(log_adif_entry);
autoseq_set_cabrillo_fd_callback(log_cabrillo_fd_entry);


  ui_mode = UIMode::RX;
  load_station_data();
  apply_ble_enabled_policy(true);
  update_autoseq_cq_type();

  // Update autoseq with station info after loading
  autoseq_set_station(g_call, g_grid);

  // Prepare RX list (but don't draw yet - startup screen may be shown)
  std::vector<UiRxLine> empty;
  ui_set_rx_list(empty);
  ui_set_active_mode_button('R');

  if (g_startup_active) {
    ui_draw_list(g_startup_lines, 0, -1);
  } else {
    ui_force_redraw_rx();
    ui_draw_rx();
  }

  ESP_LOGI(TAG, "Free heap: %u, internal: %u, 8bit: %u",
           heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_8BIT));
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  }
  log_heap("BOOT");

  // Key injection queue for UART0 RX testing
  s_key_inject_queue = xQueueCreate(32, sizeof(char));

  // UI loop
  char last_key = 0;
  while (true) {
    M5.update();
    char c = 0;
    bool c_from_ble = false;
    if (!touch_keyboard_active() && g_touchkbd_target != TouchKbdEditTarget::None) {
      touch_keyboard_end_session();
    }

    GestureEvent ge{};
    if (touch_keyboard_active()) {
      touch_keyboard_process_touch();
    } else if (poll_gesture(ge)) {
      if (ge.action == GestureAction::TapCommand) {
        c = touch_command_to_key(ge.command_idx);
      } else if (ge.action == GestureAction::TapMode) {
        c = '`';
      } else if (ge.action == GestureAction::TapLine) {
        if (ge.line_idx >= 0 && ge.line_idx < 6) {
          c = (char)('1' + ge.line_idx);
        }
      } else if (ge.action == GestureAction::SwipeLeft || ge.action == GestureAction::SwipeRight) {
        int dir = (ge.action == GestureAction::SwipeLeft) ? 1 : -1;
        UIMode next = swipe_next_mode(ui_mode, dir);
        enter_mode(next);
        if (next == UIMode::RX) {
          ui_force_redraw_rx();
          ui_draw_rx();
        }
      } else if (ge.action == GestureAction::SwipeUp || ge.action == GestureAction::SwipeDown) {
        int delta = (ge.action == GestureAction::SwipeDown) ? 1 : -1;
        if (ui_mode == UIMode::RX) {
          ui_rx_scroll(delta);
        }
      }
    }
    if (g_startup_active && c == 0 && ge.action != GestureAction::None) {
      c = ' ';  // any gesture dismisses startup screen
    }
    // Merge injected keys from UART0 RX
    poll_uart0_keys();
    if (c == 0 && s_key_inject_queue) {
      char injected = 0;
      if (xQueueReceive(s_key_inject_queue, &injected, 0) == pdTRUE) {
        c = injected;
        last_key = 0;  // Reset debounce so same-key injection works
      }
    }
    if (c == 0) {
      BleUiInput ble_input{};
      if (ble_pop_input(ble_input)) {
#if ENABLE_BLE
        if (!g_ble_enabled || g_ble_dump_in_progress) {
          c_from_ble = false;
        } else {
          c_from_ble = true;
          last_key = 0;  // allow repeated BLE commands without local debounce suppression
          if (g_ble_text_mode) {
            ble_commit_text_input(ble_input);
          } else {
            c = ble_parse_ui_command(ble_input.data, ble_input.len);
            if (c == 0) c_from_ble = false;
          }
        }
#endif
      }
    }
    if (touch_keyboard_active()) {
      c = 0;  // Touch keyboard owns edit input while active.
    }

    // BLE remote UI push model: always compare and send latest 7-line snapshot when changed.
    ble_mirror_tick();
    ble_countdown_tick();

    // Startup screen overlay on RX page: show until any key press, and only once
    if (g_startup_active) {
      if (c == 0) {
        last_key = 0;
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      if (c == last_key) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      const bool direct_mode_entry = is_startup_direct_mode_key(c);
      g_startup_active = false;
      save_station_data();

      if (!direct_mode_entry) {
        last_key = c;
        // Non-mode startup dismissal keeps prior behavior: show RX and consume key.
        ui_force_redraw_rx();
        ui_draw_rx();
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      // Let the first mode key both dismiss startup and perform normal mode switch.
      last_key = 0;
    }

    rtc_tick();
    update_countdown();
    check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
    tx_tick();              // Process TX state machine (single-threaded, non-blocking)

  // CONTROL mode: legacy host serial protocol over USB only
  if (ui_mode == UIMode::CONTROL) {
    poll_host_uart();
    if (host_bin_active) { // block keyboard exits during binary upload
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c == 0) {
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c == last_key) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    last_key = c;
    if ((c == 'c' || c == 'C') && !c_from_ble) {
      enter_mode(UIMode::RX);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }

    // Global TX cancel (Esc/` in RX/TX/Status when not editing)
    if (c == '`' &&
        (ui_mode == UIMode::RX || ui_mode == UIMode::TX || ui_mode == UIMode::STATUS) &&
        status_edit_idx == -1) {
      g_tx_cancel_requested = true;
      if (cat_cdc_ready()) {
        const char* rx = "RX;";
        cat_cdc_send(reinterpret_cast<const uint8_t*>(rx), strlen(rx), 50);
      }
      debug_log_line("TX cancel requested");
      last_key = c;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (c == 0) {
      if (g_rx_dirty && ui_mode == UIMode::RX) {
        ui_set_rx_list(g_rx_lines);
        ui_draw_rx(rx_flash_idx);
        g_rx_dirty = false;
      }
      if (ui_mode == UIMode::TX && g_tx_view_dirty) {
        g_tx_view_dirty = false;
        redraw_tx_view();
      }
      // NOTE: Beacon scheduling moved to decode_monitor_results() to match
      // reference architecture - beacon CQ is only added after decodes processed
      ui_draw_waterfall_if_dirty();
      menu_flash_tick();
      rx_flash_tick();
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
  if (c == last_key) {
    // No new keypress - still need to refresh dirty views
    if (ui_mode == UIMode::TX && g_tx_view_dirty) {
      g_tx_view_dirty = false;
      redraw_tx_view();
    }
    // NOTE: Beacon scheduling moved to decode_monitor_results()
    ui_draw_waterfall_if_dirty();
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }
  last_key = c;

  rtc_tick();
  update_countdown();
  check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
  tx_tick();              // Process TX state machine (single-threaded, non-blocking)
  menu_flash_tick();
  rx_flash_tick();
  apply_pending_sync();

  // NOTE: TX scheduling now follows reference architecture:
  // 1. decode_monitor_results() sets g_qso_xmit flag after processing
  // 2. check_slot_boundary() triggers TX at slot boundary when parity matches
  // 3. autoseq_tick() is called at slot boundary AFTER TX slot ends

  // Refresh TX view if autoseq state changed
  if (ui_mode == UIMode::TX && g_tx_view_dirty) {
    g_tx_view_dirty = false;
    redraw_tx_view();
  }

  static int last_status_uac = -1; // -1 forces a redraw on first entry
  int cur_uac = uac_is_streaming() ? 1 : 0;
  const bool c_mode_blocked = control_mode_blocked_by_uac();
  static int last_c_mode_blocked = -1;
  if ((c_mode_blocked ? 1 : 0) != last_c_mode_blocked) {
    ui_set_command_enabled('C', !c_mode_blocked);
    last_c_mode_blocked = c_mode_blocked ? 1 : 0;
  }
  if (ui_mode == UIMode::STATUS && cur_uac != last_status_uac) {
    draw_status_view();
  }
  if (ui_mode != UIMode::STATUS) {
    last_status_uac = -1;
  } else {
    last_status_uac = cur_uac;
  }

  // Ensure decode is enabled whenever streaming becomes active.
  if (uac_is_streaming() && !g_decode_enabled) {
    g_decode_enabled = true;
    ui_set_paused(false);
  }

  if (g_rx_dirty && ui_mode == UIMode::RX) {
      ui_set_rx_list(g_rx_lines);
      ui_draw_rx(rx_flash_idx);
      g_rx_dirty = false;
  }
  ui_draw_waterfall_if_dirty();

  bool switched = false;
  auto cancel_status_edit = []() {
    if (status_edit_idx != -1) {
      status_edit_idx = -1;
      status_edit_buffer.clear();
      status_cursor_pos = -1;
    }
  };
  if (!(ui_mode == UIMode::MENU && (menu_edit_idx >= 0 || menu_long_edit || menu_delete_confirm))) {
      // Mode switch keys (disabled while editing in MENU)
      if (c == 'r' || c == 'R') { cancel_status_edit(); enter_mode(UIMode::RX); ui_force_redraw_rx(); ui_draw_rx(); switched = true; }
      else if (c == 't' || c == 'T') { cancel_status_edit(); enter_mode(ui_mode == UIMode::TX ? UIMode::RX : UIMode::TX); switched = true; }
      else if (c == 'b' || c == 'B') { cancel_status_edit(); enter_mode(ui_mode == UIMode::BAND ? UIMode::RX : UIMode::BAND); switched = true; }
      else if (c == 'm' || c == 'M') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          if (menu_page == 0) {
            enter_mode(UIMode::RX);
          } else {
            menu_page = 0;
            draw_menu_view();
          }
        } else {
          enter_mode(UIMode::MENU);
        }
        switched = true;
      }
      else if (c == 'n' || c == 'N') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          if (menu_page == 1) {
            enter_mode(UIMode::RX);
          } else {
            menu_page = 1;
            draw_menu_view();
          }
        } else {
          menu_page = 0;
          enter_mode(UIMode::MENU);
          if (menu_page < 2) menu_page++;  // one "." press
          draw_menu_view();
        }
        switched = true;
      }
      else if (c == 'o' || c == 'O') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          if (menu_page == 2) {
            enter_mode(UIMode::RX);
          } else {
            menu_page = 2;
            draw_menu_view();
          }
        } else {
          menu_page = 0;
          enter_mode(UIMode::MENU);
          if (menu_page < 2) menu_page++;  // first "."
          if (menu_page < 2) menu_page++;  // second "."
          draw_menu_view();
        }
        switched = true;
      }
      else if ((c == 'f' || c == 'F') && c_from_ble) {
        cancel_status_edit();
        ble_start_qso_pick_mode();
        switched = true;
      }
      else if (c == 'q' || c == 'Q') { cancel_status_edit(); enter_mode(ui_mode == UIMode::QSO ? UIMode::RX : UIMode::QSO); switched = true; }
      else if (c == 'c' || c == 'C') {
#if ENABLE_BLE
        if (c_from_ble) {
          // BLE 'C' is intentionally ignored.
          switched = true;
        } else
#endif
        if (c_mode_blocked) {
          switched = true;  // C mode disabled while UAC host is connected.
        } else {
          cancel_status_edit();
          enter_mode(ui_mode == UIMode::CONTROL ? UIMode::RX : UIMode::CONTROL);
          switched = true;
        }
      }
      else if (c == 'd' || c == 'D') { cancel_status_edit(); enter_mode(ui_mode == UIMode::DEBUG ? UIMode::RX : UIMode::DEBUG); switched = true; }
      else if (c == 's' || c == 'S') { cancel_status_edit(); enter_mode(ui_mode == UIMode::STATUS ? UIMode::RX : UIMode::STATUS); switched = true; }
    }

  if (!switched && c) {
    // Mode-specific handling
    switch (ui_mode) {
      case UIMode::RX: {
        int sel = ui_handle_rx_key(c);
        if (sel >= 0 && sel < (int)g_rx_lines.size()) {
          if (rx_flash_idx >= 0 && rx_flash_idx != sel) {
            ui_flash_rx_line(rx_flash_idx, false);
          }
          // User tapped on a decoded message - let autoseq handle it
          autoseq_on_touch(g_rx_lines[sel]);
          g_tx_view_dirty = true;
          // Set TX flags - actual TX at slot boundary
          AutoseqTxEntry pending;
          if (autoseq_fetch_pending_tx(pending)) {
            g_qso_xmit = true;
            g_target_slot_parity = pending.slot_id & 1;
            g_pending_tx = pending;
            g_pending_tx_valid = true;
          }
          rx_flash_idx = sel;
          rx_flash_deadline = rtc_now_ms() + 500;
          ui_flash_rx_line(rx_flash_idx, true);
        }
        break;
      }
      case UIMode::TX: {
        // TX view shows QSO states from autoseq
        // Pagination through QSO list (max 9 QSOs)
        int qso_count = autoseq_queue_size();
        int start_idx = tx_page * 5;
        if (c == ';') {
          if (tx_page > 0) { tx_page--; redraw_tx_view(); }
        } else if (c == '.') {
          if (start_idx + 5 < qso_count) { tx_page++; redraw_tx_view(); }
        } else if (c >= '2' && c <= '6') {
          int idx = start_idx + (c - '2');
          if (autoseq_drop_index(idx)) {
            g_pending_tx_valid = false;
            redraw_tx_view();
            // Re-evaluate TX after queue change
            AutoseqTxEntry pending;
            if (autoseq_fetch_pending_tx(pending)) {
              g_qso_xmit = true;
              g_target_slot_parity = pending.slot_id & 1;
              g_pending_tx = pending;
              g_pending_tx_valid = true;
            }
          }
        } else if (c == '1') {
          if (autoseq_rotate_same_parity()) {
            g_pending_tx_valid = false;
            redraw_tx_view();
            // Re-evaluate TX after queue change
            AutoseqTxEntry pending;
            if (autoseq_fetch_pending_tx(pending)) {
              g_qso_xmit = true;
              g_target_slot_parity = pending.slot_id & 1;
              g_pending_tx = pending;
              g_pending_tx_valid = true;
            }
          }
        } else if (c == 'e' || c == 'E') {
          encode_and_log_pending_tx();
        }
        break;
      }
        case UIMode::BAND: {
          if (c_from_ble && band_edit_idx >= 0) {
            break;  // BLE does not participate in band frequency text edit.
          }
          if (band_edit_idx >= 0) {
            if (c >= '0' && c <= '9') { band_edit_buffer.push_back(c); draw_band_view(); }
            else if (c == 0x08 || c == 0x7f) {
              if (!band_edit_buffer.empty()) { band_edit_buffer.pop_back(); draw_band_view(); }
            } else if (c == '\r' || c == '\n') {
              if (!band_edit_buffer.empty()) {
                int val = std::stoi(band_edit_buffer);
                g_bands[band_edit_idx].freq = val;
                save_station_data();
              }
              band_edit_idx = -1;
              band_edit_buffer.clear();
              draw_band_view();
            }
          } else {
            if (c == ';') {
              if (band_page > 0) { band_page--; draw_band_view(); }
            } else if (c == '.') {
              if ((band_page + 1) * 6 < (int)g_bands.size()) { band_page++; draw_band_view(); }
            } else if (c >= '1' && c <= '6') {
              int idx = band_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_bands.size()) {
                band_edit_idx = idx;
                band_edit_buffer = std::to_string(g_bands[idx].freq);
                draw_band_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              }
            }
          }
          break;
        }
        case UIMode::STATUS: {
        if (c_from_ble && status_edit_idx != -1) {
          break;  // BLE does not participate in any status edit flow.
        }
        if (status_edit_idx == -1) {
          if (c == '1') { g_status_beacon_temp = (BeaconMode)(((int)g_status_beacon_temp + 1) % 3); draw_status_view(); }
          else if (c == '2') {
            status_edit_idx = 1;
            draw_status_view();
            if (!uac_is_streaming()) {
              debug_log_line("Starting UAC host...");
              if (!uac_start()) {
                debug_log_line("UAC start failed");
              } else {
                g_decode_enabled = true;
                ui_set_paused(false);
                ui_clear_waterfall();
              }
            } else {
              int freq_hz = g_bands[g_band_sel].freq * 1000;
              char cmd[32];
              snprintf(cmd, sizeof(cmd), "FA%011d;", freq_hz);
              bool ok = false;
              if (cat_cdc_ready()) {
                if (cat_cdc_send(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), 200) == ESP_OK) {
                  ok = true;
                }
                const char* md = "MD6;";
                cat_cdc_send(reinterpret_cast<const uint8_t*>(md), strlen(md), 200);
              }
              debug_log_line(ok ? "CAT sync sent" : "CAT not ready");
            }
            status_edit_idx = -1;
            draw_status_view();
          }
          else if (c == '3') {
            advance_active_band(1);
            save_station_data();
            draw_status_view();
              debug_log_line("Band changed");
            }
              else if (c == '4') {
                g_tune = !g_tune;
                if (cat_cdc_ready()) {
                  int freq_hz = g_bands[g_band_sel].freq * 1000;
                  char cmd[32];
                  snprintf(cmd, sizeof(cmd), "FA%011d;", freq_hz);
                  cat_cdc_send(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), 200); // set VFO
                  cat_cdc_send(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), 200); // confirm VFO
                  const char* md = "MD6;";
                  cat_cdc_send(reinterpret_cast<const uint8_t*>(md), strlen(md), 200);   // USB mode
                  if (g_tune) {
                const char* tx = "TX;";
                cat_cdc_send(reinterpret_cast<const uint8_t*>(tx), strlen(tx), 200); // key down
                // Use current cursor/random offset ~1500 Hz as tune tone
                int tune_hz = (g_offset_src == OffsetSrc::CURSOR) ? g_offset_hz : 1500;
                char ta[16];
                snprintf(ta, sizeof(ta), "TA%04d.%02d;", tune_hz, 0);
                cat_cdc_send(reinterpret_cast<const uint8_t*>(ta), strlen(ta), 200); // start tune carrier
                debug_log_line("CAT tune: TX TA");
                  } else {
                    const char* rx = "RX;";
                    cat_cdc_send(reinterpret_cast<const uint8_t*>(rx), strlen(rx), 200); // release
                    debug_log_line("CAT tune: RX");
                  }
                } else {
                  ESP_LOGW(TAG, "CAT not ready; tune skipped");
                }
                draw_status_view();
              }
              else if (c == '5') {
                status_edit_idx = 4;
                status_edit_buffer = digits_only_limited(g_date, 8);
                status_cursor_pos = 0;
                draw_status_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              }
              else if (c == '6') {
                status_edit_idx = 5;
                status_edit_buffer = digits_only_limited(g_time, 6);
                status_cursor_pos = 0;
                draw_status_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              }
            } else {
              if (status_edit_idx == 1) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); draw_status_view(); }
                if (c == ';') { g_offset_hz += 100; draw_status_view(); }
                else if (c == '.') { g_offset_hz -= 100; draw_status_view(); }
                else if (c == ',') { g_offset_hz -= 10; draw_status_view(); }
                else if (c == '/') { g_offset_hz += 10; draw_status_view(); }
                else if (c == '\n') { save_station_data(); status_edit_idx = -1; draw_status_view(); }
              } else if (status_edit_idx == 4 || status_edit_idx == 5) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == ',') { // left
                  int pos = status_cursor_pos - 1;
                  while (pos >= 0 && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos--;
                  if (pos >= 0) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c == '/') { // right
                  int pos = status_cursor_pos + 1;
                  while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                  if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c >= '0' && c <= '9') {
                  if (status_cursor_pos >= 0 && status_cursor_pos < (int)status_edit_buffer.size()) {
                    status_edit_buffer[status_cursor_pos] = c;
                    int pos = status_cursor_pos + 1;
                    while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                    if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  }
                  draw_status_view();
                } else if (c == '\n') {
                  if (status_edit_idx == 4) g_date = status_edit_buffer;
                  else g_time = status_edit_buffer;
                  save_station_data();
                  rtc_set_from_strings();
                  rtc_sync_to_hw();  // Persist to hardware RTC
                  status_edit_idx = -1;
                  status_cursor_pos = -1;
                  status_edit_buffer.clear();
                  draw_status_view();
                }
              } else {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == '\n') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
              }
            }
            break;
          }
        case UIMode::DEBUG: {
          if (c == ';') {
            if (debug_page > 0) { debug_page--; ui_draw_debug(g_debug_lines, debug_page); }
          } else if (c == '.') {
            if ((debug_page + 1) * 6 < (int)g_debug_lines.size()) { debug_page++; ui_draw_debug(g_debug_lines, debug_page); }
          }
          break;
        }
        case UIMode::QSO: {
#if ENABLE_BLE
          if (g_ble_qso_pick_mode && c_from_ble) {
            if (c == ';') {
              if (q_page > 0) { q_page--; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c >= '1' && c <= '6') {
              ble_try_dump_qso_file_by_key(c);
            } else if (c == '`') {
              ble_cancel_qso_pick_mode();
            }
            break;
          }
#endif
          if (!g_q_show_entries) {
            if (c == ';') {
              if (q_page > 0) { q_page--; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c >= '1' && c <= '6') {
              int idx = q_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_q_files.size()) {
                g_q_current_file = g_q_files[idx];
                qso_load_entries(g_q_current_file);
                g_q_show_entries = true;
                q_page = 0;
                ui_draw_list(g_q_lines, q_page, -1);
              }
            }
          } else {
            if (c == ';') {
              if (q_page > 0) { q_page--; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c == '`') {
              // back to file list
              g_q_show_entries = false;
              q_page = 0;
              qso_load_file_list();
              ui_draw_list(g_q_lines, q_page, -1);
            }
          }
          break;
        }
        case UIMode::CONTROL:
          break;
        case UIMode::MENU: {
          if (ui_mode == UIMode::MENU) {
            if (c_from_ble && (menu_long_edit || menu_edit_idx >= 0)) {
              break;  // BLE does not support text edit input.
            }
            if (menu_long_edit) {
              if (c == '\n' || c == '\r') {
                if (menu_long_kind == LONG_FT) {
                  g_free_text = menu_long_buf;
                  if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
                  update_autoseq_cq_type();
                } else if (menu_long_kind == LONG_COMMENT) {
                  g_comment1 = menu_long_buf;
                } else if (menu_long_kind == LONG_ACTIVE) {
                  g_active_band_text = menu_long_buf;
                  rebuild_active_bands();
                } else if (menu_long_kind == LONG_IGNORE) {
                  g_ignore_prefix_text = clamp_ignore_prefix_text(menu_long_buf);
                  rebuild_ignore_prefixes();
                }
                save_station_data();
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == '`') {
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_long_buf.empty()) menu_long_buf.pop_back();
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_long_kind == LONG_FT || menu_long_kind == LONG_IGNORE) {
                  ch = toupper((unsigned char)ch);
                }
                if (!(menu_long_kind == LONG_IGNORE &&
                      menu_long_buf.size() >= kIgnorePrefixTextMaxLen)) {
                  menu_long_buf.push_back(ch);
                }
                draw_menu_view();
              }
              break;
            } else if (menu_edit_idx >= 0) {
              if (c == '\n' || c == '\r') {
                // Absolute indices across pages
                if (menu_edit_idx == 3) { g_call = menu_edit_buf; autoseq_set_station(g_call, g_grid); }
                else if (menu_edit_idx == 4) { g_grid = menu_edit_buf; autoseq_set_station(g_call, g_grid); }
                else if (menu_edit_idx == 7) { g_offset_hz = atoi(menu_edit_buf.c_str()); }
                else if (menu_edit_idx == 10) { g_comment1 = menu_edit_buf; }
                else if (menu_edit_idx == 15) {
                  int v = atoi(menu_edit_buf.c_str());
                  if (v < 0) v = 0;
                  g_autoseq_max_retry = v;
                  autoseq_set_max_retry(g_autoseq_max_retry);
                }
                if (menu_edit_idx == 3) {
                  ble_update_name_from_station(true);
                }
                save_station_data();
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_edit_buf.empty()) menu_edit_buf.pop_back();
                draw_menu_view();
              } else if (c == '`') {
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_edit_idx == 15 && !(ch >= '0' && ch <= '9')) {
                  break;
                }
                if (menu_edit_idx == 7 && !((ch >= '0' && ch <= '9') || ch == '-')) {
                  break;
                }
                if (menu_edit_idx % 6 == 3 || menu_edit_idx % 6 == 4 || menu_edit_idx % 6 == 5) {
                  ch = toupper((unsigned char)ch);
                }
                menu_edit_buf.push_back(ch);
                draw_menu_view();
              }
              break;
            }
            if (menu_delete_confirm) {
              // Confirmation prompt for "Delete Logs" (page 2 line 6)
              // Temporary touch-first confirm mapping:
              // '^' command key -> ';' => YES, 'v' command key -> '.' => NO.
              if (c == 'Y' || c == 'y' || c == ';') {
                esp_err_t err = delete_logs_on_spiffs_keep_stationdata();
                menu_delete_confirm = false;
                menu_flash_idx = 17; // abs index of line 6 on page 2
                menu_flash_deadline = rtc_now_ms() + 500;
                debug_log_line(err == ESP_OK ? "Logs deleted" : "Delete failed");
                draw_menu_view();
              } else if (c == 'N' || c == 'n' || c == '.' || c == '`') {
                menu_delete_confirm = false;
                draw_menu_view();
              }
              break;
            }

        if (c == ';') {
          if (menu_page > 0) { menu_page--; draw_menu_view(); }
        } else if (c == '.') {
          if (menu_page < 2) { menu_page++; draw_menu_view(); }
        } else if (menu_page == 0) {
              if (c == '1') {
                g_cq_type = (CqType)(((int)g_cq_type + 1) % 6);
                if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
                save_station_data();
                update_autoseq_cq_type();
                draw_menu_view();
              } else if (c == '2') {
                // Send freetext - one-off transmission, bypass autoseq
                // If autoseq already has pending TX, ignore to avoid races
                if (!autoseq_has_pending_tx()) {
                  int64_t now_slot = rtc_now_ms() / 15000;
                  AutoseqTxEntry ft{};
                  ft.text = g_free_text;
                  ft.dxcall = "FreeText";
                  ft.offset_hz = g_offset_hz;
                  ft.slot_id = (int)((now_slot + 1) & 1); // next slot
                  ft.repeat_counter = 1;
                  ft.is_signoff = false;
                  if (schedule_manual_pending_tx(ft)) {
                    menu_flash_idx = 1; // absolute index of "Send FreeText"
                    menu_flash_deadline = rtc_now_ms() + 500;
                    draw_menu_view();
                    debug_log_line(std::string("Queued: ") + g_free_text);
                  }
                }
              } else if (c == '3') {
                menu_long_edit = true;
                menu_long_kind = LONG_FT;
                menu_long_buf = g_free_text;
                menu_long_backup = g_free_text;
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '4') {
                menu_edit_idx = 3; // Call (line index 3)
                menu_edit_buf = g_call;
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '5') {
                menu_edit_idx = 4; // Grid (line index 4)
                menu_edit_buf = g_grid;
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '6') {
                ESP_LOGI(TAG, "Entering deep sleep (GPIO0 wake)");
                // Save current accurate time for compensation after wake-up
                if (rtc_valid) {
                  g_rtc_sleep_epoch = rtc_epoch_base +
                      (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
                  rtc_sync_to_hw();  // Sync to hardware RTC
                  save_station_data();
                  ESP_LOGI(TAG, "Saved sleep epoch: %ld, comp=%d",
                           (long)g_rtc_sleep_epoch, g_rtc_comp);
                }
                M5.Display.sleep();
                vTaskDelay(pdMS_TO_TICKS(100));
                // Configure GPIO0 as wake-up source (active low)
                esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
                esp_deep_sleep_start();
              }
            } else if (menu_page == 1) {
                if (c == '1') {
                  g_offset_src = (OffsetSrc)(((int)g_offset_src + 1) % 3);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '2') {
                  menu_edit_idx = 7; // Cursor line
                  menu_edit_buf = std::to_string(g_offset_hz);
                  draw_menu_view();
#if ENABLE_BLE
                  if (c_from_ble) ble_enter_text_mode();
#endif
                } else if (c == '3') {
                  g_radio = (RadioType)(((int)g_radio + 1) % 3);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '4') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_IGNORE;
                  menu_long_buf = g_ignore_prefix_text;
                  menu_long_backup = g_ignore_prefix_text;
                  draw_menu_view();
#if ENABLE_BLE
                  if (c_from_ble) ble_enter_text_mode();
#endif
                } else if (c == '5') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_COMMENT;
                  menu_long_buf = g_comment1;
                  menu_long_backup = g_comment1;
                  draw_menu_view();
#if ENABLE_BLE
                  if (c_from_ble) ble_enter_text_mode();
#endif
                } else if (c == '6') {
                  g_ble_enabled = !g_ble_enabled;
                  apply_ble_enabled_policy(true);
                  save_station_data();
                  draw_menu_view();
                }
            } else if (menu_page == 2) {
              if (c == '1') {
                g_rxtx_log = !g_rxtx_log;
                save_station_data();
                draw_menu_view();
              } else if (c == '2') {
                g_skip_tx1 = !g_skip_tx1;
                autoseq_set_skip_tx1(g_skip_tx1);
                save_station_data();
                draw_menu_view();
              } else if (c == '3') {
                menu_long_edit = true;
                menu_long_kind = LONG_ACTIVE;
                menu_long_buf = g_active_band_text;
                menu_long_backup = g_active_band_text;
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '4') {
                menu_edit_idx = 15; // Max Retry line
                menu_edit_buf = std::to_string(g_autoseq_max_retry);
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              } else if (c == '5') {
                esp_err_t err = copy_logs_spiffs_to_sd_overwrite();
                menu_flash_idx = 16; // abs index of page 2 line 5
                menu_flash_deadline = rtc_now_ms() + 500;
                if (err == ESP_OK) {
                  debug_log_line("Copied SPIFFS files to SD");
                } else {
                  //char buf[64];
                  //snprintf(buf, sizeof(buf), "f:%x %s", (unsigned)err, esp_err_to_name(err));
                  //debug_log_line(buf);
                }

                draw_menu_view();
              } else if (c == '6') {
                menu_delete_confirm = true;
                draw_menu_view();
#if ENABLE_BLE
                if (c_from_ble) ble_enter_text_mode();
#endif
              }
            }
          }
          break;
        }
      }
    }

#if ENABLE_BLE
    if (g_ble_text_mode && !ble_text_target_active()) {
      ble_exit_text_mode();
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  // Run the main application loop on core0.
  xTaskCreatePinnedToCore(app_task_core0, "app_core0", 12288, nullptr, 5, nullptr, 0);
}
static void draw_status_line(int idx, const std::string& text, bool highlight) {
  const UiLayout& lay = ui_layout();
  const int line_h = lay.line_h;
  const int start_y = lay.text_area.y;
  int y = start_y + idx * line_h;
  uint16_t bg = TFT_WHITE;
  M5.Display.fillRect(0, y, lay.screen_w, line_h, bg);
  M5.Display.setTextColor(TFT_BLACK, bg);
  M5.Display.setTextDatum(middle_left);
  M5.Display.setTextSize(4);
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%d %c%s", idx + 1, highlight ? '>' : ' ', text.c_str());
  ui_set_visible_text_line(idx, buf);
  M5.Display.drawString(buf, 24, y + (line_h / 2));
}
[[maybe_unused]] static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging) {
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  // Outline
  M5.Display.startWrite();
  M5.Display.fillRect(x, y, w, h, TFT_BLACK);
  M5.Display.drawRect(x, y, w - 3, h, TFT_WHITE);
  M5.Display.fillRect(x + w - 3, y + h / 4, 3, h / 2, TFT_WHITE); // tab
  // Fill
  int inner_w = w - 5;
  int inner_h = h - 4;
  int fill_w = (inner_w * level) / 100;
  uint16_t fill_color = (level > 30) ? M5.Display.color565(0, 200, 0)
                        : (level > 15) ? M5.Display.color565(200, 180, 0)
                                        : M5.Display.color565(200, 0, 0);
  M5.Display.fillRect(x + 2, y + 2, fill_w, inner_h, fill_color);
  // Charging bolt
  if (charging) {
    int bx = x + w / 2 - 2;
    int by = y + 2;
    M5.Display.fillTriangle(bx, by, bx + 4, by + h / 2, bx + 2, by, M5.Display.color565(255, 255, 0));
    M5.Display.fillTriangle(bx + 2, by + h / 2, bx + 6, by + h - 2, bx + 4, by + h - 2, M5.Display.color565(255, 255, 0));
  }
  M5.Display.endWrite();
}
