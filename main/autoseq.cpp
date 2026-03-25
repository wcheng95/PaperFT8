/*
 * autoseq.cpp - FT8 CQ/QSO auto-sequencing engine for Mini-FT8
 *
 * Ported from DX-FT8-MULTIBAND-TABLET-TRANSCEIVER autoseq_engine.c
 * Adapted for ESP32/C++ with integrated TX scheduling.
 */

#include "autoseq.h"
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include "esp_log.h"
#include <string>
#include <unordered_map>

//void debug_log_line_public(const std::string& msg);
static const char* TAG = "AUTOSEQ";

// ============== Internal state ==============

static QsoContext s_queue[AUTOSEQ_MAX_QUEUE];
static int s_queue_size = 0;

// Station configuration
static std::string s_my_call;
static std::string s_my_grid;
static bool s_skip_tx1 = false;

// Forward declarations (must appear before format_tx_text uses them)
static std::string trim_copy(const std::string& s);
static bool parse_fd_exchange(const std::string& in, std::string& out);

// CQ configuration
static AutoseqCqType s_cq_type = AutoseqCqType::CQ;
static std::string s_cq_freetext;

// ADIF callback
static AdifLogCallback s_adif_callback;

// Cabrillo Field Day callback (for ARRL-FD logging)
static CabrilloFdLogCallback s_cabrillo_fd_callback = nullptr;

// Track latest received FD exchange per dxcall
static std::unordered_map<std::string, std::string> s_fd_rx_exchange;

// TX scheduling state
static bool s_pending_valid = false;
static AutoseqTxEntry s_pending;
static int s_pending_ctx_idx = -1;
static int64_t s_last_tx_slot_idx = -1000;
static int s_last_tx_parity = -1;

// ============== Forward declarations ==============

static void set_state(QsoContext* ctx, AutoseqState s, TxMsgType first_tx, int limit);
static void format_tx_text(QsoContext* ctx, TxMsgType id, std::string& out);
static TxMsgType parse_rcvd_msg(QsoContext* ctx, const UiRxLine& msg);
static bool generate_response(QsoContext* ctx, const UiRxLine& msg, bool override);
static void on_decode(const UiRxLine& msg);
static bool compare_ctx(const QsoContext& left, const QsoContext& right);
static void pop_front();
static QsoContext* append_ctx();
static void sort_and_clean();
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static void log_qso_if_needed(QsoContext* ctx);
static std::string normalize_call_token(const std::string& s);

// ============== Public API ==============

void autoseq_init() {
    s_queue_size = 0;
    s_pending_valid = false;
    s_pending_ctx_idx = -1;
    s_last_tx_slot_idx = -1000;
    s_last_tx_parity = -1;
}

void autoseq_clear() {
    autoseq_init();
}

/*
static void dlogf(const char* fmt, ...) {
  char b[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  debug_log_line_public(b);
}
*/

bool autoseq_drop_index(int idx) {
    if (idx < 0 || idx >= s_queue_size) return false;
    for (int i = idx; i + 1 < s_queue_size; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    --s_queue_size;
    return true;
}

bool autoseq_rotate_same_parity() {
    if (s_queue_size < 2) return false;

    int parity = s_queue[0].slot_id & 1;

    int last = -1;
    for (int i = 1; i < s_queue_size; ++i) {
        if ((s_queue[i].slot_id & 1) == parity) last = i;
        else break; // optional: only rotate within the front same-parity run
    }
    if (last == -1) return false;

    QsoContext head = s_queue[0];
    for (int i = 0; i < last; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    s_queue[last] = head;
    return true;
}

void autoseq_start_cq(int slot_parity) {
    // Don't add duplicate CQ at bottom
    if (s_queue_size > 0 && s_queue_size < AUTOSEQ_MAX_QUEUE) {
        if (s_queue[s_queue_size - 1].state == AutoseqState::CALLING) {
            return;
        }
    }
    if (s_queue_size >= AUTOSEQ_MAX_QUEUE) {
        return;
    }

    QsoContext* ctx = append_ctx();
    ctx->dxcall = "CQ";
    ctx->dxgrid.clear();
    ctx->snr_tx = -99;
    ctx->snr_rx = -99;
    ctx->slot_id = slot_parity;  // Use the specified slot parity
    set_state(ctx, AutoseqState::CALLING, TxMsgType::TX6, 0);
    // No sort needed - CQ always at bottom
    ESP_LOGI(TAG, "Started CQ on slot %d", slot_parity);
}

void autoseq_on_touch(const UiRxLine& msg) {
    // If queue is full, remove the last one
    if (s_queue_size == AUTOSEQ_MAX_QUEUE) {
        --s_queue_size;
    }

    QsoContext* ctx = append_ctx();

    // Determine the DX callsign from the message (normalize to handle <> wrapped hashed calls)
    std::string dxcall;
    if (!msg.field2.empty()) {
        dxcall = normalize_call_token(msg.field2);  // field2 is the sender
    } else if (!msg.field1.empty() && msg.field1 != "CQ") {
        dxcall = normalize_call_token(msg.field1);
    }

    // Check if it's addressed to me (normalize to handle <> wrapped hashed calls)
    std::string f1_norm = normalize_call_token(msg.field1);
    std::string my_norm = normalize_call_token(s_my_call);

    if (!my_norm.empty() && f1_norm == my_norm) {
        generate_response(ctx, msg, true);
        sort_and_clean();
        return;
    }

    // Treat as calling CQ - we're initiating contact
    ctx->dxcall = dxcall;
    if (looks_like_grid(msg.field3)) {
        ctx->dxgrid = msg.field3;
    }
    //dlogf("r:=%d %s %s %s",
    //  msg.snr, msg.field1.c_str(), msg.field2.c_str(), msg.field3.c_str());
    //dlogf("s:=%d", ctx->snr_tx);

    ctx->snr_tx = msg.snr;  // Our measurement of their signal
    ctx->offset_hz = msg.offset_hz;
    ctx->slot_id = msg.slot_id ^ 1;  // TX on opposite slot

    // Mark this QSO as Field Day only if the *received* message is CQ FD
    // (This avoids sending FD exchange when answering a normal CQ while our CQType is FD.)
    {
        std::string f1 = msg.field1;
        for (auto& ch : f1) ch = toupper((unsigned char)ch);
        // Handle both "FD" (decoder strips CQ prefix) and "CQ FD" (raw format)
        ctx->is_fd = (f1 == "FD" || f1 == "CQ FD" || f1.rfind("CQ FD", 0) == 0);
    }

    //dlogf("TH: %s %s %s snr=%d",
    //  msg.field1.c_str(), msg.field2.c_str(), msg.field3.c_str(), msg.snr);

    //dlogf("TH: bf snr_tx=%d skip=%d",
    //  ctx->snr_tx, (int)s_skip_tx1);

    set_state(ctx, s_skip_tx1 ? AutoseqState::REPORT : AutoseqState::REPLYING,
              s_skip_tx1 ? TxMsgType::TX2 : TxMsgType::TX1, AUTOSEQ_MAX_RETRY);
    sort_and_clean();

    //dlogf("TH: af snr_tx=%d state=%d",
    //  ctx->snr_tx, (int)ctx->state);

    //ESP_LOGI(TAG, "Touch: %s grid=%s snr=%d", ctx->dxcall.c_str(),
    //         ctx->dxgrid.c_str(), ctx->snr_tx);
}

void autoseq_on_decodes(const std::vector<UiRxLine>& to_me_messages) {
    ESP_LOGI(TAG, "on_decodes: %d messages, queue_size=%d",
             (int)to_me_messages.size(), s_queue_size);
    for (const auto& msg : to_me_messages) {
        ESP_LOGI(TAG, "  msg: %s %s %s snr=%d",
                 msg.field1.c_str(), msg.field2.c_str(), msg.field3.c_str(), msg.snr);
        on_decode(msg);
    }
    sort_and_clean();
    if (s_queue_size > 0) {
        ESP_LOGI(TAG, "on_decodes done: queue[0] state=%d, next_tx=%d, dxcall=%s",
                 (int)s_queue[0].state, (int)s_queue[0].next_tx, s_queue[0].dxcall.c_str());
    }
}

// Called AFTER TX completes to set up retry for next attempt
// This is the reference architecture - tick is for retry management, not scheduling
void autoseq_tick(int64_t slot_idx, int slot_parity, int ms_to_boundary) {
    (void)slot_idx; (void)slot_parity; (void)ms_to_boundary;  // unused for now

    if (s_queue_size == 0) return;

    QsoContext* ctx = &s_queue[0];

    // Advance retry counter or timeout - sets up NEXT TX attempt
    switch (ctx->state) {
        case AutoseqState::REPLYING:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX1;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::REPORT:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX2;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::ROGER_REPORT:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX3;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::ROGERS:
            if (ctx->retry_counter < ctx->retry_limit) {
                ctx->next_tx = TxMsgType::TX4;
                ctx->retry_counter++;
            } else {
                ctx->state = AutoseqState::IDLE;
                ctx->next_tx = TxMsgType::TX_UNDEF;
            }
            break;
        case AutoseqState::CALLING:  // CQ only once (controlled by beacon)
        case AutoseqState::SIGNOFF:  // QSO complete, remove from queue
            ctx->state = AutoseqState::IDLE;
            ctx->next_tx = TxMsgType::TX_UNDEF;
            break;
        default:
            break;
    }

    if (ctx->state == AutoseqState::IDLE) {
        pop_front();
    }

    ESP_LOGI(TAG, "Tick: queue_size=%d, state=%d, next_tx=%d, retry=%d/%d",
             s_queue_size, s_queue_size > 0 ? (int)s_queue[0].state : -1,
             s_queue_size > 0 ? (int)s_queue[0].next_tx : -1,
             s_queue_size > 0 ? s_queue[0].retry_counter : 0,
             s_queue_size > 0 ? s_queue[0].retry_limit : 0);
}

// Get the next TX message text based on current state (does NOT modify state)
// Returns true if there's a TX ready, false otherwise
bool autoseq_get_next_tx(std::string& out_text) {
    out_text.clear();

    if (s_queue_size == 0) return false;

    QsoContext* ctx = &s_queue[0];
    if (ctx->state == AutoseqState::IDLE || ctx->next_tx == TxMsgType::TX_UNDEF) {
        return false;
    }

    format_tx_text(ctx, ctx->next_tx, out_text);
    return !out_text.empty();
}

// Get the pending TX entry - populates from current context state
// Does NOT modify state - just reads current next_tx
bool autoseq_fetch_pending_tx(AutoseqTxEntry& out) {
    if (s_queue_size == 0) return false;

    QsoContext* ctx = &s_queue[0];
    if (ctx->state == AutoseqState::IDLE || ctx->next_tx == TxMsgType::TX_UNDEF) {
        return false;
    }

    std::string tx_text;
    format_tx_text(ctx, ctx->next_tx, tx_text);
    if (tx_text.empty()) return false;

    out.text = tx_text;
    out.dxcall = ctx->dxcall;
    out.offset_hz = ctx->offset_hz;
    out.slot_id = ctx->slot_id;
    out.repeat_counter = ctx->retry_limit - ctx->retry_counter;
    out.is_signoff = (ctx->next_tx == TxMsgType::TX4 ||
                      ctx->next_tx == TxMsgType::TX5);

    ESP_LOGI(TAG, "Fetch TX: %s (state=%d, next_tx=%d)",
             tx_text.c_str(), (int)ctx->state, (int)ctx->next_tx);
    return true;
}

void autoseq_mark_sent(int64_t slot_idx) {
    if (s_queue_size == 0) return;

    s_last_tx_slot_idx = slot_idx;
    s_last_tx_parity = s_queue[0].slot_id & 1;

    // Logging happens in generate_response() when state transitions to ROGERS/SIGNOFF
    ESP_LOGI(TAG, "TX sent on slot %lld", slot_idx);
}

void autoseq_get_qso_states(std::vector<std::string>& out) {
    out.clear();
    static const char* state_names[] = {
        "CALL", "RPLY", "RPRT", "RRPT", "RGRS", "SOFF", ""
    };

    for (int i = 0; i < s_queue_size; ++i) {
        const QsoContext* ctx = &s_queue[i];
        if (ctx->state == AutoseqState::IDLE) continue;

        char buf[32];
        snprintf(buf, sizeof(buf), "%-8.8s %.4s %d/%d",
                 ctx->dxcall.c_str(),
                 state_names[(int)ctx->state],
                 ctx->retry_counter, ctx->retry_limit);
        out.push_back(buf);
    }
}

bool autoseq_has_active_qso() {
    for (int i = 0; i < s_queue_size; ++i) {
        if (s_queue[i].state != AutoseqState::IDLE &&
            s_queue[i].state != AutoseqState::CALLING) {
            return true;
        }
    }
    return false;
}

int autoseq_queue_size() {
    return s_queue_size;
}

void autoseq_set_adif_callback(AdifLogCallback cb) {
    s_adif_callback = cb;
}


void autoseq_set_cabrillo_fd_callback(CabrilloFdLogCallback cb) {
    s_cabrillo_fd_callback = cb;
}

void autoseq_set_station(const std::string& call, const std::string& grid) {
    s_my_call = call;
    s_my_grid = grid;
}

void autoseq_set_skip_tx1(bool skip) {
    s_skip_tx1 = skip;
}

void autoseq_set_cq_type(AutoseqCqType type, const std::string& freetext) {
    s_cq_type = type;
    s_cq_freetext = freetext;
}

// ============== Internal helpers ==============

static void set_state(QsoContext* ctx, AutoseqState s, TxMsgType first_tx, int limit) {
    ctx->state = s;
    ctx->next_tx = first_tx;
    ctx->retry_counter = 0;
    ctx->retry_limit = limit;
}

static void format_tx_text(QsoContext* ctx, TxMsgType id, std::string& out) {
    out.clear();
    if (!ctx || ctx->state == AutoseqState::IDLE) return;

    char buf[64];

    switch (id) {
        case TxMsgType::TX1:
            snprintf(buf, sizeof(buf), "%s %s %s",
                     ctx->dxcall.c_str(), s_my_call.c_str(), s_my_grid.c_str());
            out = buf;
            break;

        case TxMsgType::TX2:
            if (ctx->is_fd) {
                // Field Day: send our exchange (from CQ freetext) instead of SNR report
                std::string myex;
                if (!parse_fd_exchange(s_cq_freetext, myex)) {
                    myex = trim_copy(s_cq_freetext);
                }
                snprintf(buf, sizeof(buf), "%s %s %s",
                         ctx->dxcall.c_str(), s_my_call.c_str(), myex.c_str());
                out = buf;
            } else {
                snprintf(buf, sizeof(buf), "%s %s %+d",
                         ctx->dxcall.c_str(), s_my_call.c_str(), ctx->snr_tx);
                out = buf;
            }
            break;

        case TxMsgType::TX3:
            if (ctx->is_fd) {
                // Field Day: acknowledge with 'R' + our exchange
                std::string myex;
                if (!parse_fd_exchange(s_cq_freetext, myex)) {
                    myex = trim_copy(s_cq_freetext);
                }
                snprintf(buf, sizeof(buf), "%s %s R %s",
                         ctx->dxcall.c_str(), s_my_call.c_str(), myex.c_str());
                out = buf;
            } else {
                snprintf(buf, sizeof(buf), "%s %s R%+d",
                         ctx->dxcall.c_str(), s_my_call.c_str(), ctx->snr_tx);
                out = buf;
            }
            break;

        case TxMsgType::TX4:
            snprintf(buf, sizeof(buf), "%s %s RR73",
                     ctx->dxcall.c_str(), s_my_call.c_str());
            out = buf;
            // Logging moved to generate_response() state transitions to avoid
            // dual-core race: format_tx_text is called from both core 0 (UI)
            // and core 1 (decode), causing concurrent ADIF writes.
            break;

        case TxMsgType::TX5:
            snprintf(buf, sizeof(buf), "%s %s 73",
                     ctx->dxcall.c_str(), s_my_call.c_str());
            out = buf;
            break;

        case TxMsgType::TX6: {
            const char* cq_prefix = "CQ";
            switch (s_cq_type) {
                case AutoseqCqType::SOTA: cq_prefix = "CQ SOTA"; break;
                case AutoseqCqType::POTA: cq_prefix = "CQ POTA"; break;
                case AutoseqCqType::QRP:  cq_prefix = "CQ QRP";  break;
                case AutoseqCqType::FD:   cq_prefix = "CQ FD";   break;
                case AutoseqCqType::FREETEXT:
                    out = s_cq_freetext;
                    return;
                default: break;
            }
            snprintf(buf, sizeof(buf), "%s %s %s",
                     cq_prefix, s_my_call.c_str(), s_my_grid.c_str());
            out = buf;
            break;
        }

        default:
            break;
    }
}

static inline std::string trim_copy(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
    while (b > a && (s[b-1] == ' ' || s[b-1] == '\t' || s[b-1] == '\r' || s[b-1] == '\n')) --b;
    return s.substr(a, b - a);
}

// Parse "1B SCV" or "R 1B SCV". Returns normalized exchange without leading "R " (e.g. "1B SCV").
static bool parse_fd_exchange(const std::string& in, std::string& out_norm) {
    std::string s = trim_copy(in);
    if (s.empty()) return false;
    if (s.size() >= 2 && (s[0] == 'R') && (s[1] == ' ')) {
        s = trim_copy(s.substr(2));
    }
    // Expect: "<num><class> <section>"
    size_t sp = s.find(' ');
    if (sp == std::string::npos) return false;
    std::string tok1 = s.substr(0, sp);
    std::string tok2 = trim_copy(s.substr(sp + 1));
    if (tok1.size() < 2 || tok2.empty()) return false;

    char cls = tok1.back();
    if (cls < 'A' || cls > 'F') return false;

    int num = 0;
    for (size_t i = 0; i + 1 < tok1.size(); ++i) {
        char c = tok1[i];
        if (c < '0' || c > '9') return false;
        num = num * 10 + (c - '0');
        if (num > 99) return false;
    }
    if (num < 1 || num > 32) return false;

    // Section: typically 2-3 chars (or "DX"), accept 2-4 alnum.
    if (tok2.size() < 2 || tok2.size() > 4) return false;
    for (char c : tok2) {
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
    }

    out_norm = tok1 + " " + tok2;
    return true;
}


static TxMsgType parse_rcvd_msg(QsoContext* ctx, const UiRxLine& msg) {



    TxMsgType rcvd = TxMsgType::TX_UNDEF;

    std::string f3 = msg.field3;
    for (auto& ch : f3) ch = toupper((unsigned char)ch);

    // Keywords first
    if (f3 == "RR73" || f3 == "RRR") {
        rcvd = TxMsgType::TX4;
    } else if (f3 == "73") {
        rcvd = TxMsgType::TX5;
    } else {
        // FD exchange shortcut
        std::string norm;
        if (ctx && parse_fd_exchange(msg.field3, norm) && !ctx->dxcall.empty()) {
            s_fd_rx_exchange[ctx->dxcall] = norm;

            std::string t = trim_copy(msg.field3);
            if (!t.empty() && (t[0] == 'R' || t[0] == 'r')) rcvd = TxMsgType::TX3;
            else rcvd = TxMsgType::TX2;

            ctx->rcvd_msg_type = rcvd;
            return rcvd;
        }

        // Normal FT8
        if (looks_like_grid(f3)) {
            rcvd = TxMsgType::TX1;
            if (ctx && ctx->dxgrid.empty()) ctx->dxgrid = f3;
        } else if (!f3.empty() && f3[0] == 'R' && f3.size() > 1) {
            int rpt = 0;
            if (looks_like_report(f3.substr(1), rpt)) {
                rcvd = TxMsgType::TX3;
                if (ctx) ctx->snr_rx = rpt;
            }
        } else {
            int rpt = 0;
            if (looks_like_report(f3, rpt)) {
                rcvd = TxMsgType::TX2;
                if (ctx) ctx->snr_rx = rpt;
            }
        }
    }

    if (ctx) ctx->rcvd_msg_type = rcvd;
    return rcvd;
}

static void log_qso_if_needed(QsoContext* ctx) {
    if (!ctx || ctx->logged) return;

    // Cabrillo Field Day log (optional, independent of ADIF)
    if (ctx->is_fd && s_cabrillo_fd_callback && !ctx->dxcall.empty()) {
        auto it = s_fd_rx_exchange.find(ctx->dxcall);
        if (it != s_fd_rx_exchange.end()) {
            s_cabrillo_fd_callback(ctx->dxcall, it->second);
        }
    }

    if (!s_adif_callback) return;

    ctx->logged = true;
    s_adif_callback(ctx->dxcall, ctx->dxgrid, ctx->snr_tx, ctx->snr_rx);

    ESP_LOGI(TAG, "Logged QSO: %s grid=%s rst_sent=%d rst_rcvd=%d",
             ctx->dxcall.c_str(), ctx->dxgrid.c_str(), ctx->snr_tx, ctx->snr_rx);
}

static bool generate_response(QsoContext* ctx, const UiRxLine& msg, bool override) {
    // Get DX callsign from field2 (the sender), normalize to handle <> wrapped hashed calls
    std::string dxcall = normalize_call_token(msg.field2);
    if (dxcall.empty()) dxcall = normalize_call_token(msg.field1);

    // Set dxcall BEFORE parse_rcvd_msg so FD exchange check sees a non-empty dxcall
    if (override && ctx->dxcall.empty()) {
        ctx->dxcall = dxcall;
    }

    TxMsgType rcvd = parse_rcvd_msg(ctx, msg);

    ESP_LOGI(TAG, "generate_response: override=%d, rcvd=%d, ctx->state=%d",
             override, (int)rcvd, (int)ctx->state);

    if (rcvd == TxMsgType::TX_UNDEF) {
        ESP_LOGW(TAG, "generate_response: rcvd=TX_UNDEF, returning false");
        return false;
    }

    // Update SNR we report to them on initial messages
    if (rcvd == TxMsgType::TX1 || rcvd == TxMsgType::TX2) {
        ctx->snr_tx = msg.snr;
    }

    if (override) {
        ctx->dxcall = dxcall;
        ctx->offset_hz = msg.offset_hz;
        ctx->slot_id = msg.slot_id ^ 1;  // TX on opposite slot

        // Determine if this is a Field Day QSO
        {
            std::string f1 = msg.field1;
            for (auto& ch : f1) ch = toupper((unsigned char)ch);
            // Accept "CQ FD" in field1 (touch on CQ FD message)
            if (f1 == "CQ FD" || f1 == "CQFD" || (f1.rfind("CQ FD", 0) == 0)) {
                ctx->is_fd = true;
            }
            // Also mark as FD when we're in FD mode and received a valid FD exchange
            // (handles DX replying to our CQ FD — field1 is our callsign, not "CQ FD")
            else if (s_cq_type == AutoseqCqType::FD) {
                std::string norm;
                if (parse_fd_exchange(msg.field3, norm)) {
                    ctx->is_fd = true;
                }
            }
        }

        // Reset state based on received message type
        switch (rcvd) {
            case TxMsgType::TX1:
                set_state(ctx, AutoseqState::CALLING, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX2:
                set_state(ctx, AutoseqState::REPLYING, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX3:
                set_state(ctx, AutoseqState::REPORT, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX4:
                set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX_UNDEF, 0);
                break;
            case TxMsgType::TX5:
                set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX_UNDEF, 0);
                break;
            default:
                break;
        }
    }

    // State machine transitions
    switch (ctx->state) {
        case AutoseqState::CALLING:  // We sent CQ
            switch (rcvd) {
                case TxMsgType::TX1:
                    set_state(ctx, AutoseqState::REPORT, TxMsgType::TX2, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX2:
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, AUTOSEQ_MAX_RETRY);
                    log_qso_if_needed(ctx);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::REPLYING:  // We sent TX1
            switch (rcvd) {
                case TxMsgType::TX2:
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, AUTOSEQ_MAX_RETRY);
                    log_qso_if_needed(ctx);
                    return true;
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, 0);
                    log_qso_if_needed(ctx);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::REPORT:  // We sent TX2
            switch (rcvd) {
                case TxMsgType::TX2:
                    // DX sent their own report (no R prefix) — they either
                    // didn't copy our TX2 or changed from TX1 to TX2.
                    // Both sides have exchanged reports; advance to TX3.
                    set_state(ctx, AutoseqState::ROGER_REPORT, TxMsgType::TX3, AUTOSEQ_MAX_RETRY);
                    return true;
                case TxMsgType::TX3:
                    set_state(ctx, AutoseqState::ROGERS, TxMsgType::TX4, AUTOSEQ_MAX_RETRY);
                    log_qso_if_needed(ctx);
                    return true;
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, 0);
                    log_qso_if_needed(ctx);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::ROGER_REPORT:  // We sent TX3
            switch (rcvd) {
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, AUTOSEQ_MAX_RETRY);
                    log_qso_if_needed(ctx);
                    return true;
                default:
                    return false;
            }

        case AutoseqState::ROGERS:  // We sent TX4
            switch (rcvd) {
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    // Already logged at ROGERS entry - just mark complete
                    set_state(ctx, AutoseqState::IDLE, TxMsgType::TX_UNDEF, 0);
                    break;
                default:
                    break;
            }
            return false;

        case AutoseqState::SIGNOFF:  // We sent TX5
            switch (rcvd) {
                case TxMsgType::TX4:
                case TxMsgType::TX5:
                    // Already logged at SIGNOFF entry; send another 73
                    set_state(ctx, AutoseqState::SIGNOFF, TxMsgType::TX5, AUTOSEQ_MAX_RETRY);
                    return true;
                default:
                    return false;
            }
            return false;

        default:
            break;
    }

    return false;
}

static void on_decode(const UiRxLine& msg) {
    // Check if it's addressed to us (normalize to handle <> wrapped hashed calls)
    std::string f1_norm = normalize_call_token(msg.field1);
    std::string my_norm = normalize_call_token(s_my_call);

    if (my_norm.empty() || f1_norm != my_norm) {
        return;
    }

    // Get DX call from field2 (normalize to handle <> wrapped hashed calls)
    std::string dxcall = normalize_call_token(msg.field2);
    if (dxcall.empty()) return;

    // Check if it matches an existing context (case-insensitive)
    for (int i = 0; i < s_queue_size; ++i) {
        QsoContext* ctx = &s_queue[i];
        std::string ctx_dxcall = ctx->dxcall;
        for (auto& ch : ctx_dxcall) ch = toupper((unsigned char)ch);
        if (ctx_dxcall == dxcall) {
            ESP_LOGI(TAG, "on_decode: found ctx for %s, state=%d, next_tx=%d",
                     dxcall.c_str(), (int)ctx->state, (int)ctx->next_tx);
            generate_response(ctx, msg, false);
            ESP_LOGI(TAG, "on_decode: after response, state=%d, next_tx=%d",
                     (int)ctx->state, (int)ctx->next_tx);
            return;
        }
    }

    // Check message type before creating new context
    // Don't create context for signoff messages (RR73/73) - these are late
    // messages from completed QSOs
    std::string f3 = msg.field3;
    for (auto& ch : f3) ch = toupper((unsigned char)ch);
    if (f3 == "RR73" || f3 == "RRR" || f3 == "73") {
        ESP_LOGW(TAG, "on_decode: ignoring late signoff from %s (no active ctx)",
                 dxcall.c_str());
        return;
    }

    ESP_LOGW(TAG, "on_decode: NO ctx found for %s, creating new (queue_size=%d)",
             dxcall.c_str(), s_queue_size);

    // No matching context - create new if queue not full
    if (s_queue_size >= AUTOSEQ_MAX_QUEUE) {
        return;
    }

    QsoContext* ctx = append_ctx();
    generate_response(ctx, msg, true);
    ESP_LOGI(TAG, "on_decode: NEW ctx %s, state=%d, next_tx=%d",
             ctx->dxcall.c_str(), (int)ctx->state, (int)ctx->next_tx);
}

// Comparison for std::sort: IDLE at top (to be popped), CALLING at bottom
// Returns true if left should come before right
static bool compare_ctx(const QsoContext& left, const QsoContext& right) {
    // Same state? Lower retry count gets priority — round-robin among
    // contacts so we probe for viable propagation paths rather than
    // burning all retries on one potentially dead contact.
    // State priority (below) ensures responsive contacts that advance
    // in the QSO sequence still get completed first.
    if (left.state == right.state) {
        return left.retry_counter < right.retry_counter;
    }

    // Higher state value wins (more advanced in QSO)
    // DESCENDING order: IDLE(6) > SIGNOFF(5) > ROGERS(4) > ROGER_REPORT(3) > ...
    // IDLE at front gets popped; more advanced QSOs processed first
    return left.state > right.state;  // Higher state comes first
}

static void pop_front() {
    if (s_queue_size <= 0) return;

    // Shift array
    for (int i = 0; i < s_queue_size - 1; ++i) {
        s_queue[i] = s_queue[i + 1];
    }
    --s_queue_size;
}

static QsoContext* append_ctx() {
    if (s_queue_size >= AUTOSEQ_MAX_QUEUE) return nullptr;

    QsoContext* ctx = &s_queue[s_queue_size++];
    *ctx = QsoContext{};  // Reset to defaults
    return ctx;
}

static void sort_and_clean() {
    if (s_queue_size == 0) return;

    // Use std::sort instead of qsort - qsort does byte-wise swap which
    // corrupts std::string members in QsoContext
    std::sort(s_queue, s_queue + s_queue_size, compare_ctx);

    // Pop IDLE entries from front
    while (s_queue_size > 0 && s_queue[0].state == AutoseqState::IDLE) {
        pop_front();
    }
}

static bool looks_like_grid(const std::string& s) {
    if (s.size() != 4) return false;
    // Pattern: AA00 (letters, letters, digits, digits)
    return isalpha((unsigned char)s[0]) && isalpha((unsigned char)s[1]) &&
           isdigit((unsigned char)s[2]) && isdigit((unsigned char)s[3]);
}

static bool looks_like_report(const std::string& s, int& out) {
    if (s.empty()) return false;

    // Parse optional sign and digits
    size_t idx = 0;
    bool neg = false;

    if (s[idx] == '+') {
        idx++;
    } else if (s[idx] == '-') {
        neg = true;
        idx++;
    }

    if (idx >= s.size() || !isdigit((unsigned char)s[idx])) return false;

    int val = 0;
    while (idx < s.size() && isdigit((unsigned char)s[idx])) {
        val = val * 10 + (s[idx] - '0');
        idx++;
    }

    // Must consume entire string
    if (idx != s.size()) return false;

    // Valid FT8 report range: -30 to +30
    if (val > 30) return false;

    out = neg ? -val : val;
    return true;
}

// Normalize a callsign token: strip <> wrappers used for hashed nonstd calls
// and convert to uppercase
static std::string normalize_call_token(const std::string& s) {
    std::string out = s;
    // Trim <> wrappers used for hashed nonstd calls
    if (!out.empty() && out.front() == '<') out.erase(out.begin());
    if (!out.empty() && out.back()  == '>') out.pop_back();
    // Convert to uppercase
    for (auto& ch : out) ch = (char)toupper((unsigned char)ch);
    return out;
}
