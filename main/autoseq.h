#pragma once

#include <string>
#include <vector>
#include <functional>
#include "ui.h"

// Maximum concurrent QSOs in queue
constexpr int AUTOSEQ_MAX_QUEUE = 9;
// Maximum retries before giving up on a QSO
constexpr int AUTOSEQ_MAX_RETRY = 5;

// High-level auto-sequencer states
// Order matters for priority sorting (higher = more advanced in QSO)
enum class AutoseqState {
    CALLING = 0,   // We sent CQ (TX6)
    REPLYING,      // We sent TX1 (grid)
    REPORT,        // We sent TX2 (SNR report)
    ROGER_REPORT,  // We sent TX3 (R+SNR)
    ROGERS,        // We sent TX4 (RR73)
    SIGNOFF,       // We sent TX5 (73)
    IDLE           // QSO complete (auto-removed)
};

// FT8 message types
enum class TxMsgType {
    TX_UNDEF = 0,
    TX1,  // <DXCALL> <MYCALL> <GRID>
    TX2,  // <DXCALL> <MYCALL> ##
    TX3,  // <DXCALL> <MYCALL> R##
    TX4,  // <DXCALL> <MYCALL> RR73
    TX5,  // <DXCALL> <MYCALL> 73
    TX6   // CQ <MYCALL> <GRID>
};

// QSO context - one per active contact
struct QsoContext {
    AutoseqState state = AutoseqState::IDLE;
    TxMsgType next_tx = TxMsgType::TX_UNDEF;
    TxMsgType rcvd_msg_type = TxMsgType::TX_UNDEF;

    std::string dxcall;     // Remote station callsign
    std::string dxgrid;     // Remote grid (preserved from initial exchange!)

    int snr_tx = -99;       // What we report to them (our measurement of their signal)
    int snr_rx = -99;       // What they reported about us

    int retry_counter = 0;
    int retry_limit = AUTOSEQ_MAX_RETRY;
    bool logged = false;    // Prevents duplicate ADIF logging
    bool is_fd = false;

    int offset_hz = 1500;   // TX audio offset
    int slot_id = 0;        // TX slot (0=even, 1=odd)
};

// TX entry for scheduling
struct AutoseqTxEntry {
    std::string text;       // Full FT8 message text
    std::string dxcall;     // Target callsign
    int offset_hz = 1500;
    int slot_id = 0;
    int repeat_counter = 5;
    bool is_signoff = false; // True for TX4/TX5 (priority scheduling)
};

// ADIF logging callback type
using AdifLogCallback = std::function<void(const std::string& dxcall,
                                            const std::string& dxgrid,
                                            int rst_sent, int rst_rcvd)>;

// ============== Public API ==============

// Initialize/reset the autoseq engine
void autoseq_init();

// Clear all active QSOs
void autoseq_clear();

// Drop a QSO by index (0-based in display order). Returns true if removed.
bool autoseq_drop_index(int idx);

// Rotate to the next QSO with the same slot parity as the current head.
// Returns true if a rotation occurred.
bool autoseq_rotate_same_parity();

// Start a CQ call (adds CQ to queue)
// slot_parity: 0 for even slots, 1 for odd slots
void autoseq_start_cq(int slot_parity);

// Manual response: user taps on a decoded message
void autoseq_on_touch(const UiRxLine& msg);

// Automatic response: process all decoded messages addressed to us
void autoseq_on_decodes(const std::vector<UiRxLine>& to_me_messages);

// TX retry tick - call AFTER TX completes to set up retry
// This advances retry counter and sets next_tx for the next attempt
void autoseq_tick(int64_t slot_idx, int slot_parity, int ms_to_boundary);

// Get next TX text based on current state (does NOT modify state)
bool autoseq_get_next_tx(std::string& out_text);

// Fetch pending TX entry based on current state (does NOT modify state)
bool autoseq_fetch_pending_tx(AutoseqTxEntry& out);

// Mark TX as sent (called after transmission completes)
// Note: ADIF logging happens in generate_response() state transitions
void autoseq_mark_sent(int64_t slot_idx);

// Get display strings for active QSOs
void autoseq_get_qso_states(std::vector<std::string>& out);

// Check if there's any active QSO (not IDLE)
bool autoseq_has_active_qso();

// Get current queue size
int autoseq_queue_size();

// Set the ADIF logging callback
void autoseq_set_adif_callback(AdifLogCallback cb);

// Cabrillo Field Day callback type (for ARRL-FD logging)
using CabrilloFdLogCallback = void (*)(const std::string& dxcall, const std::string& their_fd_exchange);
void autoseq_set_cabrillo_fd_callback(CabrilloFdLogCallback cb);

// Configuration setters (called when station data changes)
void autoseq_set_station(const std::string& call, const std::string& grid);
void autoseq_set_skip_tx1(bool skip);  // Skip TX1 and start with TX2

// CQ type configuration
enum class AutoseqCqType { CQ = 0, SOTA, POTA, QRP, FD, FREETEXT };
void autoseq_set_cq_type(AutoseqCqType type, const std::string& freetext = "");
