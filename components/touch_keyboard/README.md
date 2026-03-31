# TouchKeyboard

Portable on-screen touch keyboard core for embedded devices.

This module keeps keyboard logic platform-independent and uses thin host interfaces for:
- time (`IHost::nowMs`)
- drawing (`IRenderer`)
- app callbacks (`onBufferChanged`, `onCommit`, `onCancel`)

No LVGL dependency exists in the core. If needed, keep LVGL in a renderer/host adapter layer.

## Layout (fixed 4x12)

1. `1 2 3 4 5 6 7 8 9 0 BS ESC`
2. `Q W E R T Y U I O P <-- -->`
3. `A S D F G H J K L / ? CLR`
4. `Z X C V B N M + - . SPC ENTER`

## Behavior summary

- Touch action triggers on touch-up.
- No drag-across key switching.
- Single-touch style model with pressed-key visual state while finger is down.
- Long-press repeat:
  - enabled for `BS`, `<--`, `-->`
  - optional for `SPC` (`Config::repeatSpace`, default `false`)
- Recommended defaults (configurable):
  - first repeat delay: `450 ms`
  - repeat period: `80 ms`
  - left/right word acceleration threshold: `1800 ms`

Editing semantics:
- Caller provides `char*` buffer and capacity.
- Keyboard edits in-place.
- Internal snapshot (`256` bytes) is captured at `attachBuffer`.
- `ESC`: restore snapshot and exit.
- `ENTER`: commit current text, refresh snapshot, and exit.

Text viewport semantics:
- Monospace model with 2 visible lines and configurable columns (`Config::viewportCols`, default `12`).
- Continuous backing buffer with scrolling viewport.
- Cursor visibility maintained by viewport logic.

## Main API

- `attachBuffer(char* buffer, size_t capacity)`
- `setBounds(int16_t x, int16_t y, int16_t w, int16_t h)`
- `onTouchDown(...)`, `onTouchMove(...)`, `onTouchUp(...)`
- `tick()` for repeat timing
- `Config::viewportCols` controls edit-window wrap width (independent of 4x12 key grid)
- `drawAll()`, `redrawDirty()`
- `getVisibleLines()`, `copyVisibleLine(...)`
- `commitEdit()`, `cancelEdit()`

See `example/TouchKeyboardMock.cpp` for a tiny adapter/mock integration example.

## ESP-IDF integration

This folder is an IDF component:
- `CMakeLists.txt` registers `TouchKeyboard.cpp`
- public header is in `include/`

Use your platform layer to:
1. map touch coordinates to `onTouchDown/Up/Move`
2. call `tick()` periodically (for repeat behavior)
3. implement drawing in an `IRenderer`
4. render visible text window from `getVisibleLines()`
