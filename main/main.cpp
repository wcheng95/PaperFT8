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

#define ENABLE_BLE 0

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

#endif
#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif

#define UART_SVC_UUID   0xFFE0
#define UART_RX_UUID    0xFFE1
#define UART_TX_UUID    0xFFE2

#if ENABLE_BLE
static const ble_uuid16_t uart_svc_uuid =
    BLE_UUID16_INIT(UART_SVC_UUID);
static const ble_uuid16_t uart_rx_uuid =
    BLE_UUID16_INIT(UART_RX_UUID);
static const ble_uuid16_t uart_tx_uuid =
    BLE_UUID16_INIT(UART_TX_UUID);
#endif


#if ENABLE_BLE

static QueueHandle_t ble_rx_queue = nullptr;
static uint16_t gatt_tx_handle = 0;
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static const char* BT_TAG = "BLE_INIT";

static int gap_cb(struct ble_gap_event *event, void *arg);
static void nimble_host_task(void *param);
static void ble_on_sync(void);

// RX write callback (ok as you have)
static int uart_rx_cb(uint16_t conn_handle,
                      uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt,
                      void *arg)
{
    if (!ble_rx_queue) return 0;
    ESP_LOGI(BT_TAG, "RX write cb: conn=%u len=%u", conn_handle, (unsigned)ctxt->om->om_len);
    const uint8_t *p = ctxt->om->om_data;
    uint16_t len = ctxt->om->om_len;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = p[i];
        xQueueSend(ble_rx_queue, &b, 0);
    }
    return 0;
}

// TX characteristic doesn't need reads/writes; return success.
static int uart_tx_cb(uint16_t conn_handle,
                      uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt,
                      void *arg)
{
    return 0;
}

#include "esp_nimble_hci.h"   // <-- add

// C++-safe static characteristics table
static const struct ble_gatt_chr_def gatt_uart_chrs[] = {
    {
        .uuid = &uart_rx_uuid.u,
        .access_cb = uart_rx_cb,
        .arg = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &uart_tx_uuid.u,              // <-- IMPORTANT: no BLE_UUID16_DECLARE here
        .access_cb = uart_tx_cb,
        .arg = nullptr,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &gatt_tx_handle,  // CCCD will follow
    },
    { 0 }  // terminator
};

// C++-safe service table
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uart_svc_uuid.u,
        .characteristics = gatt_uart_chrs,
    },
    { 0 }  // terminator
};


static void init_bluetooth(void)
{
    static bool inited = false;
    if (inited) return;
    inited = true;
    ESP_LOGI(BT_TAG, "init_bluetooth start");

    esp_err_t nvrc = nvs_flash_init();
    if (nvrc == ESP_ERR_NVS_NO_FREE_PAGES || nvrc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvrc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvrc);

    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "nimble_port_init failed: %d", rc);
        return;
    }
    ESP_LOGI(BT_TAG, "nimble_port_init OK");

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_LOGI(BT_TAG, "GAP/GATT init done");

    ble_rx_queue = xQueueCreate(256, 1);
    assert(ble_rx_queue);

    ble_svc_gap_device_name_set("PaperFT8");

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }
    ESP_LOGI(BT_TAG, "Services added");

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(BT_TAG, "Host task started");
}


static void ble_app_advertise(void);

static int gap_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            ESP_LOGI(BT_TAG, "GAP connect, handle=%u", g_conn_handle);
        } else {
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(BT_TAG, "GAP connect failed; restarting adv");
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGW(BT_TAG, "GAP disconnect; restarting adv");
        ble_app_advertise();
        break;

    default:
        ESP_LOGI(BT_TAG, "GAP event type=%d", event->type);
        break;
    }
    return 0;
}


// BLE UART-style service (Nordic-like) UUIDs
[[maybe_unused]] static uint8_t ble_rx_placeholder = 0;
[[maybe_unused]] static uint8_t ble_tx_placeholder = 0;
#endif // ENABLE_BLE

int64_t rtc_now_ms();
static esp_err_t copy_file_overwrite(const char* src_path, const char* dst_path);

static void debug_log_line(const std::string& msg);
//exported symbol (linkable from other .cpp)
void debug_log_line_public(const std::string& msg) {
  debug_log_line(msg);
}

//static const char *TAG = "sdtest";

#define PIN_NUM_MISO GPIO_NUM_39
#define PIN_NUM_MOSI GPIO_NUM_14
#define PIN_NUM_CLK  GPIO_NUM_40
#define PIN_NUM_CS   GPIO_NUM_12

void mount_sd_spi(void)
{
    esp_err_t ret;
    const char mount_point[] = "/sdcard";

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = PIN_NUM_MOSI;
    bus_cfg.miso_io_num = PIN_NUM_MISO;
    bus_cfg.sclk_io_num = PIN_NUM_CLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &g_sd_card);
    if (ret != ESP_OK) {
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

static bool ends_with_adi(const char* name) {
  size_t n = strlen(name);
  return (n >= 4) && (strcasecmp(name + (n - 4), ".adi") == 0);
}

static bool is_rxtx_log_name(const char* name) {
  if (!name) return false;
  // Match RT[YYMMDD].log -> total length 12
  // Example: RT260319.log
  if (strlen(name) != 12) return false;
  if (name[0] != 'R' || name[1] != 'T') return false;
  for (int i = 2; i < 8; ++i) {
    if (name[i] < '0' || name[i] > '9') return false;
  }
  return strcmp(name + 8, ".log") == 0;
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

static bool is_log_file_on_spiffs(const char* name) {
  if (!name) return false;
  if (ends_with_adi(name)) return true;
  return is_rxtx_log_name(name) ||
         (strcmp(name, "Station.ini") == 0) ||
         (strcmp(name, "fieldday.log") == 0);
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

// Copy all log files from SPIFFS -> SD card, overwriting destination.
// Copies: *.adi, RT[YYMMDD].log, Station.ini
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
    if (!is_log_file_on_spiffs(name)) continue;

    std::string src = std::string("/spiffs/") + name;

    struct stat st;
    bool ok = false;
    for (int i = 0; i < 5; ++i) {
      if (stat(src.c_str(), &st) == 0 && S_ISREG(st.st_mode)) { ok = true; break; }
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!ok) { last_err = ESP_FAIL; continue; }

    std::string dst = std::string("/sdcard/") + name;
    esp_err_t err = copy_file_overwrite(src.c_str(), dst.c_str());
    if (err != ESP_OK) last_err = err;
  }

  closedir(d);
  unmount_sd_spi("/sdcard");
  return last_err;
}
// Delete log files on SPIFFS (keep Station.ini).
// Deletes: *.adi, RT[YYMMDD].log, fieldday.log

static esp_err_t delete_logs_on_spiffs_keep_stationdata() {
  DIR* d = opendir("/spiffs");
  if (!d) return ESP_FAIL;

  struct dirent* ent;
  while ((ent = readdir(d)) != nullptr) {
    const char* name = ent->d_name;
    if (!name || name[0] == '.') continue;

    if (ends_with_adi(name) || is_rxtx_log_name(name) || strcmp(name, "fieldday.log") == 0) {
      std::string path = std::string("/spiffs/") + name;
      unlink(path.c_str());  // ignore missing/err
    }
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
    int start_idx = idx;

    // Linear probing: must match add()
    while (callsign_hashtable[idx].callsign[0] != '\0')
    {
        uint32_t existing_hash = callsign_hashtable[idx].hash & 0x003FFFFFu;

        if ((existing_hash >> hash_shift) == hash)
        {
            strcpy(callsign, callsign_hashtable[idx].callsign);

            // Reset age to 0 on successful hit, preserve 22-bit payload
            callsign_hashtable[idx].hash = existing_hash;
            return true;
        }

        idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
        if (idx == start_idx)
            break;
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
enum class UIMode { RX, TX, BAND, MENU, HOST, CONTROL, DEBUG, LIST, STATUS, QSO };
static UIMode ui_mode = UIMode::RX;
static int tx_page = 0;
static std::vector<UiRxLine> g_rx_lines;
static volatile bool g_tx_view_dirty = false;  // Set when autoseq state changes
static volatile bool g_auto_switch_to_tx = false;  // Auto-switch to TX screen when transmitting
static volatile bool g_auto_switch_to_rx = false;  // Auto-switch to RX screen when decodes arrive
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
static void host_send_bt(const std::string& s);
static std::vector<std::string> g_debug_lines;
static int debug_page = 0;
static const size_t DEBUG_MAX_LINES = 18; // 3 pages

static void host_handle_line(const std::string& line);
static void save_station_data();
// TX entry for display and scheduling (populated by autoseq)
static AutoseqTxEntry g_pending_tx;
static bool g_pending_tx_valid = false;
static volatile bool g_tx_cancel_requested = false;
static void host_process_bytes(const uint8_t* buf, size_t len);
static void poll_host_uart();
static void poll_ble_uart();
struct GestureEvent;
static bool poll_gesture(GestureEvent& out);
static UIMode swipe_next_mode(UIMode current, int dir);
static void enter_mode(UIMode new_mode);
static bool g_rx_dirty = false;



static std::vector<std::string> g_list_lines = {
    "10:34 20m WA4HR",
    "10:36 40m K4ABC",
    "10:40 17m DL2XYZ",
    "10:45 30m JA1ZZZ",
    "10:50 15m VK2AAA",
    "10:52 10m KH6BBB"
};
static int list_page = 0;
static std::vector<std::string> g_uac_lines = {
    "HOST MODE: USB serial",
    "Commands:",
    "WRITEBIN <file> <size> <crc32_hex>",
    "WRITE/APPEND",
    "READ/DELETE",
    "LIST/INFO/HELP",
    "EXIT to leave"
};
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
    "PaperFT8 V1.4.1",
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
// rtc_comp: compensation factor in seconds per 10000 seconds (e.g., +150 = 1.5% fast)
static time_t g_rtc_sleep_epoch = 0;
static int g_rtc_comp = 0;

enum class CqType { CQ, CQSOTA, CQPOTA, CQQRP, CQFD, CQFREETEXT };
enum class OffsetSrc { RANDOM, CURSOR, RX };
enum class RadioType { NONE, TRUSDX, QMX };
static CqType g_cq_type = CqType::CQ;
static std::string g_cq_freetext = "FreeText";
static bool g_skip_tx1 = false;
static std::string g_free_text = "TNX 73";
static std::string g_call = "YOURCALL";
static std::string g_grid = "CM97";
bool g_decode_enabled = true;
static OffsetSrc g_offset_src = OffsetSrc::RANDOM;
static RadioType g_radio = RadioType::QMX;
static std::string g_ant = "EFHW";
static std::string g_comment1 = "MiniFT8 /Radio /Ant";
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
static enum { LONG_NONE, LONG_FT, LONG_COMMENT, LONG_ACTIVE } menu_long_kind = LONG_NONE;
static std::string menu_long_buf;
static std::string menu_long_backup;
static int menu_flash_idx = -1;          // absolute index to flash highlight
static int64_t menu_flash_deadline = 0;  // ms timestamp when flash ends
static bool menu_delete_confirm = false;  // confirmation state for Delete Logs
static int rx_flash_idx = -1;
static int64_t rx_flash_deadline = 0;
bool g_streaming = false;
static void draw_menu_view();
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging);
static void draw_status_view();
static void draw_status_line(int idx, const std::string& text, bool highlight);
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
static void update_countdown();
static void menu_flash_tick();
static void rx_flash_tick();
#if ENABLE_BLE
static uint8_t g_own_addr_type;
#endif
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
    if (s.find("<call:") == std::string::npos) continue;
    auto get_field = [&](const std::string& tag)->std::string {
      size_t p = s.find("<" + tag);
      if (p == std::string::npos) return "";
      size_t gt = s.find('>', p);
      if (gt == std::string::npos) return "";
      size_t end = s.find(' ', gt);
      if (end == std::string::npos) end = s.size();
      return s.substr(gt + 1, end - gt - 1);
    };
    std::string call = get_field("call:");
    std::string time_on = get_field("time_on:");
    std::string freq = get_field("freq:");
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
    if (call.empty()) call = "?";
    if (band.empty()) band = freq.empty() ? "?" : freq;
    g_q_lines.push_back(time_on + " " + band + " " + call);
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
  // Expand placeholders using current radio/ant strings
  auto radio_name_local = [](RadioType r) {
    switch (r) {
      case RadioType::TRUSDX: return "QMX";
      case RadioType::QMX: return "QMX";
      default: return "None";
    }
  };
  repl(comment_expanded, "/Radio", radio_name_local(g_radio));
  repl(comment_expanded, "/Ant", g_ant);
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
  host_send_bt(s);
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
  mon_cfg.time_osr = 1;
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
  std::vector<std::string> lines;
  lines.reserve(g_bands.size());
  for (size_t i = 0; i < g_bands.size(); ++i) {
    std::string freq_str;
    if ((int)i == band_edit_idx && !band_edit_buffer.empty()) {
      freq_str = band_edit_buffer;
    } else {
      freq_str = std::to_string(g_bands[i].freq);
    }
    lines.push_back(std::string(g_bands[i].name) + ": " + freq_str);
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
    case OffsetSrc::CURSOR: return "Cursor";
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
  repl(out, "/Ant", g_ant);
  return out;
}

static std::string battery_status_line() {
  int level = (int)M5.Power.getBatteryLevel();
  bool charging = M5.Power.isCharging();
  if (level < 0 || level > 100) level = 0;
  char buf[32];
  snprintf(buf, sizeof(buf), "Batt:%3d%%%s", level, charging ? " CHG" : "");
  return buf;
}

static std::string elide_right(const std::string& s, size_t max_len = 22) {
  if (s.size() <= max_len) return s;
  if (max_len <= 3) return s.substr(s.size() - max_len);
  return std::string("...") + s.substr(s.size() - (max_len - 3));
}

static std::string head_trim(const std::string& s, size_t max_len = 16) {
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

static void draw_status_view();

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

  time_t now = rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
  struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
  settimeofday(&tv, NULL);
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
    rx_flash_idx = -1;
    rx_flash_deadline = 0;
    if (ui_mode == UIMode::RX) {
      ui_draw_rx();
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
      int idx = ui_rx_hit_test(last_x, last_y);
      if (idx >= 0) {
        out.action = GestureAction::TapLine;
        out.line_idx = idx;
      } else {
        const UiLayout& lay = ui_layout();
        if (lay.mode_box.contains(last_x, last_y)) {
          out.action = GestureAction::TapMode;
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
  std::array<uint8_t, 240> row{};
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
      int t_index = candidates[i].time_offset * mon->wf.time_osr + candidates[i].time_sub;
      int f_index = candidates[i].freq_offset * mon->wf.freq_osr + candidates[i].freq_sub;
      size_t offset2 = (size_t)t_index * (size_t)mon->wf.block_stride + (size_t)f_index;
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
    if (!to_me.empty()) {
      autoseq_on_decodes(to_me);
      g_tx_view_dirty = true;
      g_last_reply_text = to_me.front().text;
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
  if (merged.size() > 12) merged.resize(12);

  g_rx_lines = merged;

  if (!merged.empty()) g_auto_switch_to_rx = true;

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

static void draw_menu_long_edit() {
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

  // Auto-switch to TX screen when transmission starts
  g_auto_switch_to_tx = true;

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
    if (menu_long_edit) {
      draw_menu_long_edit();
      return;
    }
  std::vector<std::string> lines;
  lines.reserve(12);

  std::string cq_line = std::string("CQ Type:");
  if (g_cq_type == CqType::CQFREETEXT) cq_line += g_cq_freetext;
  else cq_line += cq_type_name(g_cq_type);
  lines.push_back(cq_line);
  lines.push_back("Send FreeText");
  lines.push_back(std::string("F:") + head_trim(g_free_text, 16));
  lines.push_back(std::string("Call:") + elide_right(menu_edit_idx == 3 ? menu_edit_buf : g_call));
  lines.push_back(std::string("Grid:") + elide_right(menu_edit_idx == 4 ? menu_edit_buf : g_grid));
  lines.push_back(std::string("Sleep:") + (M5.Power.isCharging() ? "press" : "USB?"));

  lines.push_back(std::string("Offset:") + offset_name(g_offset_src));
  if (menu_edit_idx == 7) {
    lines.push_back(std::string("Cursor:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("Cursor:") + std::to_string(g_offset_hz));
  }
  lines.push_back(std::string("Radio:") + radio_name(g_radio));
  lines.push_back(std::string("Antenna:") + elide_right(menu_edit_idx == 9 ? menu_edit_buf : g_ant));
  lines.push_back(std::string("C:") + head_trim(expand_comment1(), 16));
  lines.push_back(battery_status_line());

  // Page 2 content (index 12+)
  lines.push_back(std::string("RxTxLog:") + (g_rxtx_log ? "ON" : "OFF"));
  lines.push_back(std::string("SkipTX1:") + (g_skip_tx1 ? "ON" : "OFF"));
  lines.push_back(std::string("ActiveBand:") + head_trim(g_active_band_text, 16));
  // RTC compensation: seconds per 10000 seconds (e.g., +150 = 1.5% fast)
  if (menu_edit_idx == 15) {
    lines.push_back(std::string("RTC Comp:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("RTC Comp:") + std::to_string(g_rtc_comp));
  }
  lines.push_back("Copy Logs to SD");
  lines.push_back(menu_delete_confirm ? "Are you sure Y/N?" : "Delete Logs");

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
  int battery_abs_idx = (int)lines.size() - 1;
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

static void host_send_bt(const std::string& s)
{
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    if (!gatt_tx_handle) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(s.data(), s.size());
    if (!om) return;

    ble_gatts_notify_custom(g_conn_handle, gatt_tx_handle, om);
}

static void ble_on_sync(void);

static void ble_on_sync(void)
{
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
    uint8_t addr_val[6];
    ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
    ESP_LOGI(BT_TAG, "Sync, address type %d, addr %02x:%02x:%02x:%02x:%02x:%02x",
             g_own_addr_type,
             addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    ble_app_advertise();
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv{};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t*)"PaperFT8";
    fields.name_len = strlen("PaperFT8");
    fields.name_is_complete = 1;

    ble_gap_adv_stop();  // safe if not advertising
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "adv_set_fields failed: %d", rc);
        return;
    }
    rc = ble_gap_adv_start(g_own_addr_type, nullptr,
                           BLE_HS_FOREVER,
                           &adv, gap_cb, nullptr);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "adv_start failed: %d", rc);
    } else {
        ESP_LOGI(BT_TAG, "Advertising as PaperFT8");
    }
}

#else  // ENABLE_BLE
static void host_send_bt(const std::string& s) { (void)s; }
static void init_bluetooth(void) {}
[[maybe_unused]] static void poll_ble_uart() {}
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

#if ENABLE_BLE
static void poll_ble_uart() {
  if (!ble_rx_queue) return;
  uint8_t buf[256];
  size_t n = 0;
  uint8_t b = 0;
  while (xQueueReceive(ble_rx_queue, &b, 0) == pdTRUE) {
    buf[n++] = b;
    if (n == sizeof(buf)) {
      ESP_LOGI(BT_TAG, "BLE RX chunk %u bytes", (unsigned)n);
      host_process_bytes(buf, n);
      // If no newline was seen, synthesize one to flush short commands over BLE.
      if (!host_bin_active && n > 0 && buf[n - 1] != '\n' && buf[n - 1] != '\r') {
        const uint8_t nl = '\n';
        host_process_bytes(&nl, 1);
      }
      n = 0;
    }
  }
  if (n > 0) {
    ESP_LOGI(BT_TAG, "BLE RX chunk %u bytes", (unsigned)n);
    host_process_bytes(buf, n);
    if (!host_bin_active && buf[n - 1] != '\n' && buf[n - 1] != '\r') {
      const uint8_t nl = '\n';
      host_process_bytes(&nl, 1);
    }
  }
}

#endif // ENABLE_BLE

static void load_station_data() {
  // If Station.ini exists on SD, prefer it by copying onto SPIFFS first.
  // If mount/copy fails, fall back to the on-device SPIFFS Station.ini.
  sync_station_ini_from_sd_to_spiffs();

  FILE* f = fopen(STATION_FILE, "r");
  if (!f) return;
  char line[64];
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
    } else if (sscanf(line, "date=%63s", line) == 1) {
      g_date = line;
    } else if (sscanf(line, "time=%63s", line) == 1) {
      g_time = line;
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
    } else if (strncmp(line, "ant=", 4) == 0) {
      g_ant = trim_copy(line + 4);
    } else if (strncmp(line, "comment1=", 9) == 0) {
      g_comment1 = trim_copy(line + 9);
    } else if (sscanf(line, "rxtx_log=%d", &val) == 1) {
      g_rxtx_log = (val != 0);
    } else if (sscanf(line, "skiptx1=%d", &val) == 1) {
      g_skip_tx1 = (val != 0); autoseq_set_skip_tx1(g_skip_tx1);
    } else if (sscanf(line, "active_band=%d", &val) == 1) { // legacy single value
      g_active_band_text = std::to_string(val);
    } else if (strncmp(line, "active_bands=", 13) == 0) {
      g_active_band_text = trim_copy(line + 13);
    } else if (sscanf(line, "rtc_comp=%d", &g_rtc_comp) == 1) {
      // Loaded rtc_comp
    } else {
      long long epoch_tmp = 0;
      if (sscanf(line, "rtc_sleep_epoch=%lld", &epoch_tmp) == 1) {
        g_rtc_sleep_epoch = (time_t)epoch_tmp;
      }
    }
  }
  fclose(f);
  // Try hardware RTC first (persists through deep sleep), fall back to saved strings
  if (!rtc_init_from_hw()) {
    ESP_LOGI(TAG, "Hardware RTC not valid, using saved time strings");
    rtc_set_from_strings();
  }
  rebuild_active_bands();
  g_beacon = BeaconMode::OFF; // force off on load
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
  fprintf(f, "ant=%s\n", g_ant.c_str());
  fprintf(f, "comment1=%s\n", g_comment1.c_str());
  fprintf(f, "rxtx_log=%d\n", g_rxtx_log ? 1 : 0);
  fprintf(f, "active_bands=%s\n", g_active_band_text.c_str());
  fprintf(f, "rtc_sleep_epoch=%lld\n", (long long)g_rtc_sleep_epoch);
  fprintf(f, "rtc_comp=%d\n", g_rtc_comp);
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
  ui_mode = new_mode;
  rx_flash_idx = -1;
  switch (ui_mode) {
    case UIMode::RX:
      ui_draw_mode_box("R");
      // Force RX list redraw
      ui_force_redraw_rx();
      ui_draw_rx();
      break;
    case UIMode::TX:
      ui_draw_mode_box("T");
      tx_page = 0;
      redraw_tx_view();
      break;
    case UIMode::BAND:
      band_page = 0;
      band_edit_idx = -1;
      draw_band_view();
      break;
    case UIMode::MENU:
      ui_draw_mode_box("S");
      menu_page = 0;
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      menu_delete_confirm = false;
      draw_menu_view();
      break;
    case UIMode::DEBUG:
      debug_page = (int)((g_debug_lines.size() - 1) / 6);
      ui_draw_debug(g_debug_lines, debug_page);
      break;
    case UIMode::CONTROL:
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
  case UIMode::HOST:
    ui_draw_mode_box("S");
    // Start UAC audio streaming
    g_uac_lines.clear();
    g_uac_lines.push_back("USB Audio Host Mode");
    if (uac_start()) {
      g_decode_enabled = true;
      ui_set_paused(false);
      ui_clear_waterfall();
      g_uac_lines.push_back("Starting USB host...");
      g_uac_lines.push_back("Connect 24-bit/48kHz");
      g_uac_lines.push_back("stereo USB mic");
    } else {
      g_uac_lines.push_back("Failed to start UAC");
        debug_log_line("UAC start failed");
      }
      ui_draw_list(g_uac_lines, 0, -1);
      break;
    case UIMode::LIST:
      ui_draw_mode_box("R");
      list_page = 0;
      ui_draw_list(g_list_lines, list_page, -1);
      break;
    case UIMode::QSO:
      ui_draw_mode_box("Q");
      g_q_show_entries = false;
      q_page = 0;
      qso_load_file_list();
      ui_draw_list(g_q_lines, q_page, -1);
      break;
    case UIMode::STATUS:
      ui_draw_mode_box("S");
      g_status_beacon_temp = g_beacon;
      status_edit_idx = -1;
      status_cursor_pos = -1;
      draw_status_view();
      break;
  }
}

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
  update_autoseq_cq_type();

  // Update autoseq with station info after loading
  autoseq_set_station(g_call, g_grid);

  // Prepare RX list (but don't draw yet - startup screen may be shown)
  std::vector<UiRxLine> empty;
  ui_set_rx_list(empty);

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
    GestureEvent ge;
    if (poll_gesture(ge)) {
      if (ge.action == GestureAction::TapMode) {
        c = '`';
      } else if (ge.action == GestureAction::TapLine && ui_mode == UIMode::RX) {
        if (ge.line_idx >= 0 && ge.line_idx < (int)g_rx_lines.size()) {
          autoseq_on_touch(g_rx_lines[ge.line_idx]);
          g_tx_view_dirty = true;
          AutoseqTxEntry pending;
          if (autoseq_fetch_pending_tx(pending)) {
            g_qso_xmit = true;
            g_target_slot_parity = pending.slot_id & 1;
            g_pending_tx = pending;
            g_pending_tx_valid = true;
          }
          rx_flash_idx = ge.line_idx;
          rx_flash_deadline = rtc_now_ms() + 500;
          ui_draw_rx(rx_flash_idx);
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
      last_key = c;

      g_startup_active = false;
      save_station_data();

      // Now show the real RX page; consume this key so it doesn't trigger actions.
      ui_force_redraw_rx();
      ui_draw_rx();

      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    rtc_tick();
    update_countdown();
    check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
    tx_tick();              // Process TX state machine (single-threaded, non-blocking)

  // HOST mode: UAC audio streaming - update status display
  if (ui_mode == UIMode::HOST) {
    // Update status display periodically
    static TickType_t last_update = 0;
    TickType_t now = xTaskGetTickCount();
    if (now - last_update > pdMS_TO_TICKS(500)) {
      last_update = now;
      // Update status line (keep only header)
      if (g_uac_lines.size() > 1) {
        g_uac_lines.resize(1);
      }
      g_uac_lines.push_back(uac_get_status_string());
      if (uac_is_streaming()) {
        g_uac_lines.push_back("Decoding FT8...");
        // Show debug lines with raw USB sample data
        const char* dbg1 = uac_get_debug_line1();
        const char* dbg2 = uac_get_debug_line2();
        if (dbg1[0]) g_uac_lines.push_back(dbg1);
        if (dbg2[0]) g_uac_lines.push_back(dbg2);
      }
      ui_draw_list(g_uac_lines, 0, -1);
    }
    // Handle keyboard - only 'h'/'H' to exit
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
    if (c == 'h' || c == 'H') {
      enter_mode(UIMode::RX);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }

  // CONTROL mode: legacy host serial protocol over USB (and BLE)
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
    if (c == 'c' || c == 'C' || c == 'h' || c == 'H') {
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
      // Auto-switch between RX and TX screens (only when in RX or TX mode)
      if (g_auto_switch_to_tx && ui_mode == UIMode::RX) {
        g_auto_switch_to_tx = false;
        g_auto_switch_to_rx = false;  // Clear both to avoid ping-pong
        enter_mode(UIMode::TX);
        redraw_tx_view();
      } else if (g_auto_switch_to_rx && ui_mode == UIMode::TX) {
        g_auto_switch_to_rx = false;
        g_auto_switch_to_tx = false;
        enter_mode(UIMode::RX);
        ui_force_redraw_rx();
        ui_draw_rx();
      } else {
        // Clear flags if in other modes (don't interrupt MENU, BAND, etc.)
        g_auto_switch_to_tx = false;
        g_auto_switch_to_rx = false;
      }

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
  if (!(ui_mode == UIMode::MENU && (menu_edit_idx >= 0 || menu_long_edit))) {
      // Mode switch keys (disabled while editing in MENU)
      if (c == 'r' || c == 'R') { cancel_status_edit(); enter_mode(UIMode::RX); ui_force_redraw_rx(); ui_draw_rx(); switched = true; }
      else if (c == 't' || c == 'T') { cancel_status_edit(); enter_mode(ui_mode == UIMode::TX ? UIMode::RX : UIMode::TX); switched = true; }
      else if (c == 'b' || c == 'B') { cancel_status_edit(); enter_mode(ui_mode == UIMode::BAND ? UIMode::RX : UIMode::BAND); switched = true; }
      else if (c == 'm' || c == 'M') { cancel_status_edit(); enter_mode(ui_mode == UIMode::MENU ? UIMode::RX : UIMode::MENU); switched = true; }
      else if (c == 'n' || c == 'N') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          enter_mode(UIMode::RX);
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
          enter_mode(UIMode::RX);
        } else {
          menu_page = 0;
          enter_mode(UIMode::MENU);
          if (menu_page < 2) menu_page++;  // first "."
          if (menu_page < 2) menu_page++;  // second "."
          draw_menu_view();
        }
        switched = true;
      }
      else if (c == 'q' || c == 'Q') { cancel_status_edit(); enter_mode(ui_mode == UIMode::QSO ? UIMode::RX : UIMode::QSO); switched = true; }
      else if (c == 'h' || c == 'H') { cancel_status_edit(); enter_mode(ui_mode == UIMode::HOST ? UIMode::RX : UIMode::HOST); switched = true; }
      else if (c == 'c' || c == 'C') { cancel_status_edit(); enter_mode(ui_mode == UIMode::CONTROL ? UIMode::RX : UIMode::CONTROL); switched = true; }
      else if (c == 'd' || c == 'D') { cancel_status_edit(); enter_mode(ui_mode == UIMode::DEBUG ? UIMode::RX : UIMode::DEBUG); switched = true; }
      else if (c == 'l' || c == 'L') { cancel_status_edit(); enter_mode(ui_mode == UIMode::LIST ? UIMode::RX : UIMode::LIST); switched = true; }
      else if (c == 's' || c == 'S') { cancel_status_edit(); enter_mode(ui_mode == UIMode::STATUS ? UIMode::RX : UIMode::STATUS); switched = true; }
    }

  if (!switched && c) {
    // Mode-specific handling
    switch (ui_mode) {
      case UIMode::RX: {
        int sel = ui_handle_rx_key(c);
        if (sel >= 0 && sel < (int)g_rx_lines.size()) {
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
          ui_draw_rx(rx_flash_idx);
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
              }
            }
          }
          break;
        }
        case UIMode::STATUS: {
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
              else if (c == '5') { status_edit_idx = 4; status_edit_buffer = g_date; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == '-')) status_cursor_pos++; draw_status_view(); }
              else if (c == '6') { status_edit_idx = 5; status_edit_buffer = g_time; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == ':')) status_cursor_pos++; draw_status_view(); }
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
        case UIMode::LIST: {
          if (c == ';') {
            if (list_page > 0) { list_page--; ui_draw_list(g_list_lines, list_page, -1); }
          } else if (c == '.') {
            if ((list_page + 1) * 6 < (int)g_list_lines.size()) { list_page++; ui_draw_list(g_list_lines, list_page, -1); }
          }
          break;
        }
        case UIMode::QSO: {
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
        case UIMode::HOST:
        case UIMode::MENU: {
          if (ui_mode == UIMode::MENU) {
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
                if (menu_long_kind == LONG_FT) ch = toupper((unsigned char)ch);
                menu_long_buf.push_back(ch);
                draw_menu_view();
              }
              break;
            } else if (menu_edit_idx >= 0) {
              if (c == '\n' || c == '\r') {
                // Absolute indices across pages
                if (menu_edit_idx == 3) { g_call = menu_edit_buf; autoseq_set_station(g_call, g_grid); }
                else if (menu_edit_idx == 4) { g_grid = menu_edit_buf; autoseq_set_station(g_call, g_grid); }
                else if (menu_edit_idx == 7) { g_offset_hz = atoi(menu_edit_buf.c_str()); }
                else if (menu_edit_idx == 9) { g_ant = menu_edit_buf; }
                else if (menu_edit_idx == 10) { g_comment1 = menu_edit_buf; }
                else if (menu_edit_idx == 15) { g_rtc_comp = atoi(menu_edit_buf.c_str()); }
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
              if (c == 'Y' || c == 'y') {
                esp_err_t err = delete_logs_on_spiffs_keep_stationdata();
                menu_delete_confirm = false;
                menu_flash_idx = 17; // abs index of line 6 on page 2
                menu_flash_deadline = rtc_now_ms() + 500;
                debug_log_line(err == ESP_OK ? "Logs deleted" : "Delete failed");
                draw_menu_view();
              } else if (c == 'N' || c == 'n' || c == '`') {
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
              } else if (c == '4') {
                menu_edit_idx = 3; // Call (line index 3)
                menu_edit_buf = g_call;
                draw_menu_view();
              } else if (c == '5') {
                menu_edit_idx = 4; // Grid (line index 4)
                menu_edit_buf = g_grid;
                draw_menu_view();
              } else if (c == '6') {
                  if (M5.Power.isCharging()) {
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
                  } else {
                    debug_log_line("Sleep skipped: not charging");
                    draw_menu_view();
                  }
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
                } else if (c == '3') {
                  g_radio = (RadioType)(((int)g_radio + 1) % 3);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '4') {
                  menu_edit_idx = 9; // Antenna line
                  menu_edit_buf = g_ant;
                  draw_menu_view();
                } else if (c == '5') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_COMMENT;
                  menu_long_buf = g_comment1;
                  menu_long_backup = g_comment1;
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
              } else if (c == '4') {
                menu_edit_idx = 15; // RTC Comp line
                menu_edit_buf = std::to_string(g_rtc_comp);
                draw_menu_view();
              } else if (c == '5') {
                esp_err_t err = copy_logs_spiffs_to_sd_overwrite();
                menu_flash_idx = 16; // abs index of page 2 line 5
                menu_flash_deadline = rtc_now_ms() + 500;
                if (err == ESP_OK) {
                  debug_log_line("Copied logs to SD");
                } else {
                  //char buf[64];
                  //snprintf(buf, sizeof(buf), "f:%x %s", (unsigned)err, esp_err_to_name(err));
                  //debug_log_line(buf);
                }

                draw_menu_view();
              } else if (c == '6') {
                menu_delete_confirm = true;
                draw_menu_view();
              }
            }
          }
          break;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  // Run the main application loop on core0; BLE host stays on core0.
  xTaskCreatePinnedToCore(app_task_core0, "app_core0", 12288, nullptr, 5, nullptr, 0);
  init_bluetooth();   // runs on core0
}
static void draw_status_line(int idx, const std::string& text, bool highlight) {
  const int line_h = 19;
  const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
  int y = start_y + idx * line_h;
  uint16_t bg = highlight ? M5.Display.color565(30, 30, 60) : TFT_BLACK;
  M5.Display.setTextSize(2);
  M5.Display.fillRect(0, y, 240, line_h, bg);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.setCursor(0, y);
  M5.Display.printf("%d %s", idx + 1, text.c_str());
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
