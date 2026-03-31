#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace touchkbd {

struct Rect {
  int16_t x = 0;
  int16_t y = 0;
  int16_t w = 0;
  int16_t h = 0;

  bool contains(int16_t px, int16_t py) const {
    return (px >= x) && (py >= y) && (px < (x + w)) && (py < (y + h));
  }
};

enum class KeyKind : uint8_t {
  Character,
  Backspace,
  Escape,
  MoveLeft,
  MoveRight,
  ClearWordLeft,
  Space,
  Enter,
};

struct KeyDef {
  uint8_t index = 0;
  uint8_t row = 0;
  uint8_t col = 0;
  KeyKind kind = KeyKind::Character;
  char literal = 0;
  const char* label = "";
  Rect rect;
};

struct KeyVisualState {
  bool pressed = false;
  bool control = false;
  bool repeatable = false;
};

struct BufferState {
  const char* text = nullptr;
  size_t length = 0;
  size_t cursor = 0;
  size_t capacity = 0;
  bool editing = false;
};

struct VisibleLines {
  static constexpr size_t kRows = 2;
  static constexpr size_t kMaxCols = 40;

  // Number of visible columns currently in use (<= kMaxCols).
  size_t cols = 12;
  std::array<std::array<char, kMaxCols + 1>, kRows> lines {};
  size_t cursorLine = 0;
  size_t cursorCol = 0;
  size_t viewportStart = 0;
  bool cursorVisible = false;
};

class IHost {
 public:
  virtual ~IHost() = default;
  virtual uint32_t nowMs() const = 0;

  virtual void onBufferChanged(const BufferState&) {}
  virtual void onCommit(const BufferState&) {}
  virtual void onCancel(const BufferState&) {}
};

class IRenderer {
 public:
  virtual ~IRenderer() = default;
  virtual void drawKeyboardBackground(const Rect& bounds) = 0;
  virtual void drawKey(const KeyDef& key, const KeyVisualState& state) = 0;
};

struct Config {
  uint32_t firstRepeatDelayMs = 450;
  uint32_t repeatPeriodMs = 80;
  uint32_t wordNavAccelerationMs = 1800;
  bool repeatSpace = false;
  // Edit viewport width in monospace columns. Independent from key layout.
  // Clamped to [1, VisibleLines::kMaxCols].
  size_t viewportCols = 12;
};

class TouchKeyboard {
 public:
  static constexpr size_t kLayoutRows = 4;
  static constexpr size_t kLayoutCols = 12;
  static constexpr size_t kKeyCount = kLayoutRows * kLayoutCols;
  static constexpr size_t kSnapshotCapacity = 256;

  explicit TouchKeyboard(IHost& host, IRenderer* renderer = nullptr, const Config& config = Config{});

  bool attachBuffer(char* buffer, size_t capacity);
  void detachBuffer();

  void setBounds(int16_t x, int16_t y, int16_t w, int16_t h);
  Rect bounds() const { return bounds_; }

  void onTouchDown(int16_t x, int16_t y);
  void onTouchMove(int16_t x, int16_t y);
  void onTouchUp(int16_t x, int16_t y);
  void tick();

  void drawAll();
  void redrawDirty();

  const char* text() const { return buffer_ ? buffer_ : ""; }
  const char* originalText() const { return snapshot_.data(); }
  BufferState bufferState() const;

  VisibleLines getVisibleLines() const;
  bool copyVisibleLine(size_t lineIndex, char* out, size_t outCap) const;

  void cancelEdit();
  void commitEdit();

  size_t keyCount() const { return kKeyCount; }
  const KeyDef& keyDef(size_t index) const { return keys_[index]; }

  bool isEditing() const { return editingActive_; }

 private:
  bool hasBuffer() const { return buffer_ != nullptr && capacity_ > 0; }
  bool isRepeatableKey(const KeyDef& key) const;
  int hitTest(int16_t x, int16_t y) const;
  void rebuildKeyRects();
  void clearDirty();
  void markAllDirty();
  void markKeyDirty(int keyIndex);
  void notifyBufferChanged();

  bool applyKeyAction(int keyIndex, bool fromRepeat, bool wordAccelerated);

  bool insertChar(char ch);
  bool doBackspace();
  bool moveCursorLeft(bool byWord);
  bool moveCursorRight(bool byWord);
  bool clearWordToLeft();

  void refreshSnapshotFromBuffer();
  void restoreBufferFromSnapshot();
  void normalizeBufferState();
  void updateViewportForCursor();

  static bool isSpaceChar(char ch);
  static bool isWordChar(char ch);
  static size_t boundedStrLen(const char* s, size_t maxLen);

 private:
  IHost& host_;
  IRenderer* renderer_ = nullptr;
  Config config_;

  char* buffer_ = nullptr;
  size_t capacity_ = 0;
  size_t length_ = 0;
  size_t cursor_ = 0;

  std::array<char, kSnapshotCapacity> snapshot_ {};
  size_t snapshotLen_ = 0;

  size_t viewportStart_ = 0;
  size_t viewportCols_ = 12;
  bool editingActive_ = false;

  Rect bounds_ {};
  std::array<KeyDef, kKeyCount> keys_ {};

  bool pointerDown_ = false;
  int downKeyIndex_ = -1;
  int pressedKeyIndex_ = -1;
  uint32_t touchDownMs_ = 0;
  uint32_t nextRepeatMs_ = 0;
  bool repeatTriggered_ = false;

  bool dirtyAll_ = true;
  std::array<bool, kKeyCount> keyDirty_ {};
};

}  // namespace touchkbd

