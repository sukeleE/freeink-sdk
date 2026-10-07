#pragma once

#include <BoardConfig.h>

#include "Ssd1677Luts.h"
#include "StickyCombinedAa.h"

#ifndef FREEINK_SSD1677_TEXT_TURN_AA
#define FREEINK_SSD1677_TEXT_TURN_AA                                                         \
  (FREEINK_SSD1677_TEXT_ROUTING && (FREEINK_DEVICE_METALIO_EINK4 || FREEINK_DEVICE_STICKY || \
                                    FREEINK_DEVICE_MURPHY_M4 || FREEINK_DEVICE_WAVESHARE_EPAPER_397 || \
                                    FREEINK_DEVICE_WAVESHARE_EPAPER_397_V2))
#endif
#ifndef FREEINK_SSD1677_TEXT_WHITE_FRAMES
#define FREEINK_SSD1677_TEXT_WHITE_FRAMES 32
#endif
#ifndef FREEINK_SSD1677_TEXT_BLACK_DELAY
#define FREEINK_SSD1677_TEXT_BLACK_DELAY 0
#endif
static_assert(FREEINK_SSD1677_TEXT_WHITE_FRAMES == 32 || FREEINK_SSD1677_TEXT_WHITE_FRAMES == 40 ||
                  FREEINK_SSD1677_TEXT_WHITE_FRAMES == 48,
              "Text-turn white calibration uses 32, 40 or 48 frames");
static_assert(FREEINK_SSD1677_TEXT_BLACK_DELAY == 0 || FREEINK_SSD1677_TEXT_BLACK_DELAY == 8 ||
                  FREEINK_SSD1677_TEXT_BLACK_DELAY == 16 || FREEINK_SSD1677_TEXT_BLACK_DELAY == 24,
              "Text-turn black delay uses 0, 8, 16 or 24 frames");

namespace freeink::combinedAa {
// Per-panel calibration in nominal 5 ms frames. Keep independent entries so
// tuning an unvalidated panel never changes Sticky or Paper Mono.
struct TextTurnTiming {
  uint8_t grayWhite = 32;
  uint8_t whiteStart = 24;
  uint8_t white = FREEINK_SSD1677_TEXT_WHITE_FRAMES;
  uint8_t blackStart = 24 + FREEINK_SSD1677_TEXT_BLACK_DELAY;
};
struct Calibration {
  uint8_t kick, gray, black, frameRate;
  const uint8_t* voltages;
  uint8_t border, powerOff;
  bool powerUpFirst;
  uint8_t sleepKey = 0x03;
  TextTurnTiming textTurn{};
};
inline const Calibration sticky{stickyCombinedAa::KICK_FRAMES,
                                stickyCombinedAa::GRAY_FRAMES,
                                stickyCombinedAa::BLACK_FRAMES,
                                0x08,
                                stickyCombinedAa::VOLTAGES,
                                0x80,
                                0x03,
                                true};
inline const Calibration x4pro{16, 24, 32, 0x08, lut_grayscale + 105, 0xC0, 0x03, false};
inline const Calibration x4classic{16, 24, 32, 0x08, lut_grayscale + 105, 0xC0, 0x03, false};
inline const Calibration murphy{16, 24, 32, 0x08, lut_grayscale + 105, 0xC0, 0x03, false};
inline const Calibration waveshare{16, 24, 32, 0x08, lut_grayscale_waveshare + 105, 0x80, 0x03, true, 0x01};
inline const Calibration metalio{16, 24, 32, 0x08, lut_grayscale + 105, 0x80, 0x83, true};
inline const Calibration& calibration() {
#if FREEINK_DEVICE_X4PRO
  return x4pro;
#elif FREEINK_DEVICE_X4CLASSIC
  return x4classic;
#elif FREEINK_DEVICE_MURPHY_M4
  return murphy;
#elif FREEINK_DEVICE_WAVESHARE_EPAPER_397
  return waveshare;
#elif FREEINK_DEVICE_METALIO_EINK4
  return metalio;
#else
  return sticky;
#endif
}
}  // namespace freeink::combinedAa
