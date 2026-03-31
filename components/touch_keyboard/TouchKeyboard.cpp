#include "TouchKeyboard.h"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace touchkbd {

namespace {

struct LayoutEntry {
  const char* label;
  KeyKind kind;
  char literal;
};

constexpr LayoutEntry kLayout[TouchKeyboard::kLayoutRows][TouchKeyboard::kLayoutCols] = {
    {{"1", KeyKind::Character, '1'},
     {"2", KeyKind::Character, '2'},
     {"3", KeyKind::Character, '3'},
     {"4", KeyKind::Character, '4'},
     {"5", KeyKind::Character, '5'},
     {"6", KeyKind::Character, '6'},
     {"7", KeyKind::Character, '7'},
     {"8", KeyKind::Character, '8'},
     {"9", KeyKind::Character, '9'},
     {"0", KeyKind::Character, '0'},
     {"BS", KeyKind::Backspace, 0},
     {"ESC", KeyKind::Escape, 0}},

    {{"Q", KeyKind::Character, 'Q'},
     {"W", KeyKind::Character, 'W'},
     {"E", KeyKind::Character, 'E'},
     {"R", KeyKind::Character, 'R'},
     {"T", KeyKind::Character, 'T'},
     {"Y", KeyKind::Character, 'Y'},
     {"U", KeyKind::Character, 'U'},
     {"I", KeyKind::Character, 'I'},
     {"O", KeyKind::Character, 'O'},
     {"P", KeyKind::Character, 'P'},
     {"<--", KeyKind::MoveLeft, 0},
     {"-->", KeyKind::MoveRight, 0}},

    {{"A", KeyKind::Character, 'A'},
     {"S", KeyKind::Character, 'S'},
     {"D", KeyKind::Character, 'D'},
     {"F", KeyKind::Character, 'F'},
     {"G", KeyKind::Character, 'G'},
     {"H", KeyKind::Character, 'H'},
     {"J", KeyKind::Character, 'J'},
     {"K", KeyKind::Character, 'K'},
     {"L", KeyKind::Character, 'L'},
     {"/", KeyKind::Character, '/'},
     {"?", KeyKind::Character, '?'},
     {"CLR", KeyKind::ClearWordLeft, 0}},

    {{"Z", KeyKind::Character, 'Z'},
     {"X", KeyKind::Character, 'X'},
     {"C", KeyKind::Character, 'C'},
     {"V", KeyKind::Character, 'V'},
     {"B", KeyKind::Character, 'B'},
     {"N", KeyKind::Character, 'N'},
     {"M", KeyKind::Character, 'M'},
     {"+", KeyKind::Character, '+'},
     {"-", KeyKind::Character, '-'},
     {".", KeyKind::Character, '.'},
     {"SPC", KeyKind::Space, ' '},
     {"ENTER", KeyKind::Enter, 0}}};

}  // namespace

TouchKeyboard::TouchKeyboard(IHost& host, IRenderer* renderer, const Config& config)
    : host_(host), renderer_(renderer), config_(config) {
  if (config_.viewportCols < 1) {
    viewportCols_ = 1;
  } else if (config_.viewportCols > VisibleLines::kMaxCols) {
    viewportCols_ = VisibleLines::kMaxCols;
  } else {
    viewportCols_ = config_.viewportCols;
  }

  markAllDirty();
  rebuildKeyRects();
}

bool TouchKeyboard::attachBuffer(char* buffer, size_t capacity) {
  if (buffer == nullptr || capacity < 2) {
    return false;
  }

  buffer_ = buffer;
  capacity_ = capacity;
  normalizeBufferState();
  refreshSnapshotFromBuffer();
  editingActive_ = true;
  cursor_ = length_;
  updateViewportForCursor();
  notifyBufferChanged();
  markAllDirty();
  return true;
}

void TouchKeyboard::detachBuffer() {
  buffer_ = nullptr;
  capacity_ = 0;
  length_ = 0;
  cursor_ = 0;
  viewportStart_ = 0;
  editingActive_ = false;
  pointerDown_ = false;
  downKeyIndex_ = -1;
  pressedKeyIndex_ = -1;
  repeatTriggered_ = false;
  snapshot_.fill(0);
  snapshotLen_ = 0;
  markAllDirty();
}

void TouchKeyboard::setBounds(int16_t x, int16_t y, int16_t w, int16_t h) {
  bounds_ = {x, y, w, h};
  rebuildKeyRects();
  markAllDirty();
}

void TouchKeyboard::onTouchDown(int16_t x, int16_t y) {
  if (!editingActive_) return;

  pointerDown_ = true;
  downKeyIndex_ = hitTest(x, y);
  repeatTriggered_ = false;
  touchDownMs_ = host_.nowMs();
  nextRepeatMs_ = touchDownMs_ + config_.firstRepeatDelayMs;

  if (pressedKeyIndex_ != downKeyIndex_) {
    if (pressedKeyIndex_ >= 0) markKeyDirty(pressedKeyIndex_);
    pressedKeyIndex_ = downKeyIndex_;
    if (pressedKeyIndex_ >= 0) markKeyDirty(pressedKeyIndex_);
  }
}

void TouchKeyboard::onTouchMove(int16_t, int16_t) {
  // Intentionally ignore move-hit switching:
  // key selection is latched from touch-down until touch-up.
}

void TouchKeyboard::onTouchUp(int16_t x, int16_t y) {
  if (!pointerDown_) return;

  (void)x;
  (void)y;
  const int actionKey = downKeyIndex_;
  // Selection is latched on touch-down; release position does not retarget.
  const bool shouldTrigger = (actionKey >= 0) && !repeatTriggered_;

  pointerDown_ = false;
  downKeyIndex_ = -1;
  repeatTriggered_ = false;

  if (pressedKeyIndex_ >= 0) {
    markKeyDirty(pressedKeyIndex_);
  }
  pressedKeyIndex_ = -1;

  if (shouldTrigger) {
    applyKeyAction(actionKey, false, false);
  }
}

void TouchKeyboard::tick() {
  if (!editingActive_) return;
  if (!pointerDown_ || downKeyIndex_ < 0) return;

  const KeyDef& key = keys_[downKeyIndex_];
  if (!isRepeatableKey(key)) return;

  uint32_t now = host_.nowMs();
  if (now < nextRepeatMs_) return;

  while (now >= nextRepeatMs_) {
    const bool accelWord = (key.kind == KeyKind::MoveLeft || key.kind == KeyKind::MoveRight) &&
                           ((now - touchDownMs_) >= config_.wordNavAccelerationMs);
    applyKeyAction(downKeyIndex_, true, accelWord);
    repeatTriggered_ = true;
    nextRepeatMs_ += config_.repeatPeriodMs;
    if (!editingActive_) break;
  }
}

void TouchKeyboard::drawAll() {
  if (!renderer_) return;

  renderer_->drawKeyboardBackground(bounds_);
  for (size_t i = 0; i < kKeyCount; ++i) {
    const auto& key = keys_[i];
    KeyVisualState state;
    state.pressed = (static_cast<int>(i) == pressedKeyIndex_);
    state.control = (key.kind != KeyKind::Character);
    state.repeatable = isRepeatableKey(key);
    renderer_->drawKey(key, state);
  }
  clearDirty();
}

void TouchKeyboard::redrawDirty() {
  if (!renderer_) return;

  if (dirtyAll_) {
    drawAll();
    return;
  }

  for (size_t i = 0; i < kKeyCount; ++i) {
    if (!keyDirty_[i]) continue;
    const auto& key = keys_[i];
    KeyVisualState state;
    state.pressed = (static_cast<int>(i) == pressedKeyIndex_);
    state.control = (key.kind != KeyKind::Character);
    state.repeatable = isRepeatableKey(key);
    renderer_->drawKey(key, state);
    keyDirty_[i] = false;
  }
}

BufferState TouchKeyboard::bufferState() const {
  BufferState st;
  st.text = text();
  st.length = length_;
  st.cursor = cursor_;
  st.capacity = capacity_;
  st.editing = editingActive_;
  return st;
}

VisibleLines TouchKeyboard::getVisibleLines() const {
  VisibleLines out;
  out.cols = viewportCols_;

  for (size_t r = 0; r < VisibleLines::kRows; ++r) {
    for (size_t c = 0; c < out.cols; ++c) {
      out.lines[r][c] = ' ';
    }
    out.lines[r][out.cols] = '\0';
  }

  if (!hasBuffer()) return out;

  const size_t cols = out.cols;
  const size_t windowSize = VisibleLines::kRows * cols;
  size_t viewStart = viewportStart_;
  if (cursor_ < viewStart) {
    viewStart = (cursor_ / cols) * cols;
  } else if (cursor_ >= (viewStart + windowSize)) {
    size_t block = cursor_ / cols;
    size_t first_block = (block >= (VisibleLines::kRows - 1)) ? (block - (VisibleLines::kRows - 1)) : 0;
    viewStart = first_block * cols;
  }

  for (size_t r = 0; r < VisibleLines::kRows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      size_t src = viewStart + r * cols + c;
      if (src < length_) {
        out.lines[r][c] = buffer_[src];
      }
    }
  }

  out.viewportStart = viewStart;
  if (cursor_ >= viewStart && cursor_ < (viewStart + windowSize)) {
    size_t ofs = cursor_ - viewStart;
    out.cursorLine = ofs / cols;
    out.cursorCol = ofs % cols;
    out.cursorVisible = true;
  }

  return out;
}

bool TouchKeyboard::copyVisibleLine(size_t lineIndex, char* out, size_t outCap) const {
  if (out == nullptr || outCap == 0 || lineIndex >= VisibleLines::kRows) {
    return false;
  }

  VisibleLines v = getVisibleLines();
  size_t n = std::min(v.cols, outCap - 1);
  std::memcpy(out, v.lines[lineIndex].data(), n);
  out[n] = '\0';
  return true;
}

void TouchKeyboard::cancelEdit() {
  if (!hasBuffer()) return;

  restoreBufferFromSnapshot();
  editingActive_ = false;
  pointerDown_ = false;
  downKeyIndex_ = -1;
  if (pressedKeyIndex_ >= 0) markKeyDirty(pressedKeyIndex_);
  pressedKeyIndex_ = -1;
  updateViewportForCursor();
  notifyBufferChanged();
  host_.onCancel(bufferState());
  markAllDirty();
}

void TouchKeyboard::commitEdit() {
  if (!hasBuffer()) return;

  refreshSnapshotFromBuffer();
  editingActive_ = false;
  pointerDown_ = false;
  downKeyIndex_ = -1;
  if (pressedKeyIndex_ >= 0) markKeyDirty(pressedKeyIndex_);
  pressedKeyIndex_ = -1;
  updateViewportForCursor();
  notifyBufferChanged();
  host_.onCommit(bufferState());
  markAllDirty();
}

bool TouchKeyboard::isRepeatableKey(const KeyDef& key) const {
  if (key.kind == KeyKind::Backspace) return true;
  if (key.kind == KeyKind::MoveLeft) return true;
  if (key.kind == KeyKind::MoveRight) return true;
  if (config_.repeatSpace && key.kind == KeyKind::Space) return true;
  return false;
}

int TouchKeyboard::hitTest(int16_t x, int16_t y) const {
  for (size_t i = 0; i < kKeyCount; ++i) {
    if (keys_[i].rect.contains(x, y)) return static_cast<int>(i);
  }
  return -1;
}

void TouchKeyboard::rebuildKeyRects() {
  for (size_t r = 0; r < kLayoutRows; ++r) {
    for (size_t c = 0; c < kLayoutCols; ++c) {
      size_t idx = r * kLayoutCols + c;
      int16_t x0 = bounds_.x + static_cast<int16_t>((bounds_.w * c) / kLayoutCols);
      int16_t x1 = bounds_.x + static_cast<int16_t>((bounds_.w * (c + 1)) / kLayoutCols);
      int16_t y0 = bounds_.y + static_cast<int16_t>((bounds_.h * r) / kLayoutRows);
      int16_t y1 = bounds_.y + static_cast<int16_t>((bounds_.h * (r + 1)) / kLayoutRows);

      keys_[idx].index = static_cast<uint8_t>(idx);
      keys_[idx].row = static_cast<uint8_t>(r);
      keys_[idx].col = static_cast<uint8_t>(c);
      keys_[idx].kind = kLayout[r][c].kind;
      keys_[idx].literal = kLayout[r][c].literal;
      keys_[idx].label = kLayout[r][c].label;
      keys_[idx].rect = {x0, y0, static_cast<int16_t>(x1 - x0), static_cast<int16_t>(y1 - y0)};
    }
  }
}

void TouchKeyboard::clearDirty() {
  dirtyAll_ = false;
  keyDirty_.fill(false);
}

void TouchKeyboard::markAllDirty() {
  dirtyAll_ = true;
  keyDirty_.fill(true);
}

void TouchKeyboard::markKeyDirty(int keyIndex) {
  if (keyIndex < 0 || keyIndex >= static_cast<int>(kKeyCount)) return;
  keyDirty_[static_cast<size_t>(keyIndex)] = true;
}

void TouchKeyboard::notifyBufferChanged() {
  host_.onBufferChanged(bufferState());
}

bool TouchKeyboard::applyKeyAction(int keyIndex, bool, bool wordAccelerated) {
  if (keyIndex < 0 || keyIndex >= static_cast<int>(kKeyCount) || !editingActive_) {
    return false;
  }

  const KeyDef& key = keys_[static_cast<size_t>(keyIndex)];
  bool changed = false;

  switch (key.kind) {
    case KeyKind::Character:
      changed = insertChar(key.literal);
      break;
    case KeyKind::Space:
      changed = insertChar(' ');
      break;
    case KeyKind::Backspace:
      changed = doBackspace();
      break;
    case KeyKind::MoveLeft:
      changed = moveCursorLeft(wordAccelerated);
      break;
    case KeyKind::MoveRight:
      changed = moveCursorRight(wordAccelerated);
      break;
    case KeyKind::ClearWordLeft:
      changed = clearWordToLeft();
      break;
    case KeyKind::Escape:
      cancelEdit();
      return true;
    case KeyKind::Enter:
      commitEdit();
      return true;
  }

  if (changed) {
    updateViewportForCursor();
    notifyBufferChanged();
  }
  return changed;
}

bool TouchKeyboard::insertChar(char ch) {
  if (!hasBuffer()) return false;
  if (length_ + 1 >= capacity_) return false;

  std::memmove(buffer_ + cursor_ + 1, buffer_ + cursor_, (length_ - cursor_) + 1);
  buffer_[cursor_] = ch;
  ++length_;
  ++cursor_;
  return true;
}

bool TouchKeyboard::doBackspace() {
  if (!hasBuffer()) return false;
  if (cursor_ == 0 || length_ == 0) return false;

  std::memmove(buffer_ + cursor_ - 1, buffer_ + cursor_, (length_ - cursor_) + 1);
  --cursor_;
  --length_;
  return true;
}

bool TouchKeyboard::moveCursorLeft(bool byWord) {
  if (!hasBuffer()) return false;
  if (cursor_ == 0) return false;

  if (!byWord) {
    --cursor_;
    return true;
  }

  size_t i = cursor_;
  while (i > 0 && isSpaceChar(buffer_[i - 1])) --i;
  if (i > 0 && isWordChar(buffer_[i - 1])) {
    while (i > 0 && isWordChar(buffer_[i - 1])) --i;
  } else {
    while (i > 0 && !isSpaceChar(buffer_[i - 1]) && !isWordChar(buffer_[i - 1])) --i;
  }

  if (i == cursor_) return false;
  cursor_ = i;
  return true;
}

bool TouchKeyboard::moveCursorRight(bool byWord) {
  if (!hasBuffer()) return false;
  if (cursor_ >= length_) return false;

  if (!byWord) {
    ++cursor_;
    return true;
  }

  size_t i = cursor_;
  while (i < length_ && isSpaceChar(buffer_[i])) ++i;
  if (i < length_ && isWordChar(buffer_[i])) {
    while (i < length_ && isWordChar(buffer_[i])) ++i;
  } else {
    while (i < length_ && !isSpaceChar(buffer_[i]) && !isWordChar(buffer_[i])) ++i;
  }

  if (i == cursor_) return false;
  cursor_ = i;
  return true;
}

bool TouchKeyboard::clearWordToLeft() {
  if (!hasBuffer()) return false;
  if (cursor_ == 0) return false;

  size_t start = cursor_;
  while (start > 0 && isSpaceChar(buffer_[start - 1])) --start;
  while (start > 0 && !isSpaceChar(buffer_[start - 1])) --start;

  if (start == cursor_) return false;
  size_t removeCount = cursor_ - start;
  std::memmove(buffer_ + start, buffer_ + cursor_, (length_ - cursor_) + 1);
  length_ -= removeCount;
  cursor_ = start;
  return true;
}

void TouchKeyboard::refreshSnapshotFromBuffer() {
  snapshot_.fill(0);
  if (!hasBuffer()) {
    snapshotLen_ = 0;
    return;
  }

  snapshotLen_ = std::min(length_, kSnapshotCapacity - 1);
  std::memcpy(snapshot_.data(), buffer_, snapshotLen_);
  snapshot_[snapshotLen_] = '\0';
}

void TouchKeyboard::restoreBufferFromSnapshot() {
  if (!hasBuffer()) return;

  size_t copyLen = std::min(snapshotLen_, capacity_ - 1);
  std::memcpy(buffer_, snapshot_.data(), copyLen);
  buffer_[copyLen] = '\0';
  length_ = copyLen;
  cursor_ = length_;
}

void TouchKeyboard::normalizeBufferState() {
  if (!hasBuffer()) {
    length_ = 0;
    cursor_ = 0;
    return;
  }

  length_ = boundedStrLen(buffer_, capacity_ - 1);
  buffer_[length_] = '\0';
  if (cursor_ > length_) cursor_ = length_;
}

void TouchKeyboard::updateViewportForCursor() {
  const size_t cols = viewportCols_;
  const size_t window = VisibleLines::kRows * cols;

  if (cursor_ < viewportStart_) {
    viewportStart_ = (cursor_ / cols) * cols;
  } else if (cursor_ >= (viewportStart_ + window)) {
    size_t block = cursor_ / cols;
    size_t first_block = (block >= (VisibleLines::kRows - 1)) ? (block - (VisibleLines::kRows - 1)) : 0;
    viewportStart_ = first_block * cols;
  }
}

bool TouchKeyboard::isSpaceChar(char ch) {
  return std::isspace(static_cast<unsigned char>(ch)) != 0;
}

bool TouchKeyboard::isWordChar(char ch) {
  unsigned char u = static_cast<unsigned char>(ch);
  return std::isalnum(u) != 0 || ch == '_';
}

size_t TouchKeyboard::boundedStrLen(const char* s, size_t maxLen) {
  size_t n = 0;
  while (n < maxLen && s[n] != '\0') ++n;
  return n;
}

}  // namespace touchkbd

