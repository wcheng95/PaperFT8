#include "TouchKeyboard.h"

#include <cstdio>
#include <cstring>

namespace {

using touchkbd::BufferState;
using touchkbd::IHost;
using touchkbd::IRenderer;
using touchkbd::KeyDef;
using touchkbd::KeyVisualState;
using touchkbd::Rect;
using touchkbd::TouchKeyboard;
using touchkbd::VisibleLines;

class MockHost : public IHost {
 public:
  uint32_t nowMs() const override { return nowMs_; }

  void advanceMs(uint32_t delta) { nowMs_ += delta; }

  void onBufferChanged(const BufferState& st) override {
    std::printf("changed: text=\"%s\" len=%u cursor=%u\n",
                st.text,
                static_cast<unsigned>(st.length),
                static_cast<unsigned>(st.cursor));
  }

  void onCommit(const BufferState& st) override {
    std::printf("commit: \"%s\"\n", st.text);
  }

  void onCancel(const BufferState& st) override {
    std::printf("cancel: \"%s\"\n", st.text);
  }

 private:
  uint32_t nowMs_ = 0;
};

class MockRenderer : public IRenderer {
 public:
  void drawKeyboardBackground(const Rect& bounds) override {
    std::printf("draw keyboard @ (%d,%d %dx%d)\n", bounds.x, bounds.y, bounds.w, bounds.h);
  }

  void drawKey(const KeyDef&, const KeyVisualState&) override {
    // No-op in this tiny mock. Real adapters draw key caps and pressed state.
  }
};

int findKeyByLabel(const TouchKeyboard& kb, const char* label) {
  for (size_t i = 0; i < kb.keyCount(); ++i) {
    if (std::strcmp(kb.keyDef(i).label, label) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void tapKey(TouchKeyboard& kb, const char* label) {
  const int idx = findKeyByLabel(kb, label);
  if (idx < 0) return;

  const Rect r = kb.keyDef(static_cast<size_t>(idx)).rect;
  const int16_t cx = static_cast<int16_t>(r.x + r.w / 2);
  const int16_t cy = static_cast<int16_t>(r.y + r.h / 2);
  kb.onTouchDown(cx, cy);
  kb.onTouchUp(cx, cy);
}

void holdKey(TouchKeyboard& kb, MockHost& host, const char* label, uint32_t holdMs, uint32_t tickStepMs = 20) {
  const int idx = findKeyByLabel(kb, label);
  if (idx < 0) return;

  const Rect r = kb.keyDef(static_cast<size_t>(idx)).rect;
  const int16_t cx = static_cast<int16_t>(r.x + r.w / 2);
  const int16_t cy = static_cast<int16_t>(r.y + r.h / 2);

  kb.onTouchDown(cx, cy);
  uint32_t elapsed = 0;
  while (elapsed < holdMs) {
    host.advanceMs(tickStepMs);
    kb.tick();
    elapsed += tickStepMs;
  }
  kb.onTouchUp(cx, cy);
}

void printEditWindow(const TouchKeyboard& kb) {
  const VisibleLines v = kb.getVisibleLines();

  std::printf("+");
  for (size_t i = 0; i < v.cols; ++i) std::printf("-");
  std::printf("+\n");
  std::printf("|%s|\n", v.lines[0].data());
  std::printf("|%s|\n", v.lines[1].data());
  std::printf("+");
  for (size_t i = 0; i < v.cols; ++i) std::printf("-");
  std::printf("+\n");

  if (v.cursorVisible) {
    char marker[VisibleLines::kMaxCols + 1];
    std::memset(marker, ' ', v.cols);
    marker[v.cols] = '\0';
    if (v.cursorCol < v.cols) marker[v.cursorCol] = '^';
    std::printf(" %s  cursor(line=%u col=%u)\n",
                marker,
                static_cast<unsigned>(v.cursorLine),
                static_cast<unsigned>(v.cursorCol));
  }
}

}  // namespace

int main() {
  MockHost host;
  MockRenderer renderer;
  TouchKeyboard kb(host, &renderer);

  kb.setBounds(0, 80, 240, 160);

  char text[128] = "CQ TEST K6ABC FN31";
  kb.attachBuffer(text, sizeof(text));
  kb.drawAll();

  std::printf("\nInitial window\n");
  printEditWindow(kb);

  tapKey(kb, "BS");
  tapKey(kb, "?");
  tapKey(kb, "SPC");
  tapKey(kb, "A");

  std::printf("\nAfter key taps\n");
  printEditWindow(kb);

  holdKey(kb, host, "<--", 2100);
  std::printf("\nAfter long-hold left (word-accelerated near end)\n");
  printEditWindow(kb);

  tapKey(kb, "ENTER");

  kb.attachBuffer(text, sizeof(text));
  tapKey(kb, "1");
  tapKey(kb, "2");
  std::printf("\nBefore cancel\n");
  printEditWindow(kb);

  tapKey(kb, "ESC");
  std::printf("\nAfter cancel (restored snapshot)\n");
  std::printf("text=\"%s\"\n", text);

  return 0;
}
