#include "AudioManager.h"

#include <BoardConfig.h>

// Capability-gated like FrontlightManager: devices without FREEINK_CAP_AUDIO
// compile the stub bodies at the bottom and link no I2S/codec code.
#if FREEINK_CAP_AUDIO

#include <Wire.h>
#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <mp3dec.h>

#include <memory>

namespace freeink {

namespace {

constexpr uint32_t CODEC_I2C_HZ = 100000;  // OEM bus speed (shared with touch)
constexpr size_t READ_CHUNK = 1024;        // mono source bytes per loop pass

// MP3 task buffer sizes. Helix needs ~4 KB of input to cover the largest frame
// plus its bit-reservoir look-back; output is at most 1152 stereo samples.
constexpr size_t MP3_IN_CAP = 4096;
constexpr size_t MP3_OUT_SAMPS = 2304;
// Consecutive decode failures before giving up on the file (auto-skip).
constexpr uint8_t MP3_ERROR_BUDGET = 16;

// ES8388 playback init recovered from the Murphy OEM firmware — exact register
// order matters (staged mute -> clocks/format -> mixers -> power -> unmute).
struct RegVal {
  uint8_t reg, val;
};
constexpr RegVal ES8388_INIT[] = {
    {0x19, 0x04}, {0x01, 0x50}, {0x02, 0x00}, {0x08, 0x00}, {0x04, 0x3e}, {0x00, 0x12},
    {0x17, 0x18}, {0x18, 0x02}, {0x26, 0x1b}, {0x27, 0x90}, {0x2a, 0x90}, {0x2b, 0x80},
    {0x2d, 0x00}, {0x1b, 0x00}, {0x1a, 0x00}, {0x03, 0xff}, {0x09, 0x88}, {0x10, 0xf0},
    {0x0b, 0x80}, {0x0c, 0x0e}, {0x0d, 0x02}, {0x10, 0x20}, {0x11, 0x20}, {0x2e, 0x1e},
    {0x2f, 0x1e}, {0x30, 0x1e}, {0x31, 0x1e}, {0x04, 0x3c}, {0x19, 0x00},
};

constexpr uint8_t ES8388_VOL_MAX_REG = 0x21;  // register full-scale for OUT volumes

// ES8311 playback init, mirroring M5Unified's PaperColor speaker bring-up
// (_speaker_enabled_cb_papercolor). The codec derives its internal MCLK from
// BCLK (reg 0x01 bit7 + reg 0x02 MULT_PRE=3, i.e. 32*fs * 8 = 256*fs), so no
// MCLK line is needed and the same init covers every sample rate. The 16-bit
// I2S SDP format (0x09) is set explicitly — M5Unified relies on the default.
constexpr RegVal ES8311_INIT[] = {
    {0x00, 0x80},  // RESET: CSM power on, slave mode
    {0x01, 0xB5},  // CLK_MANAGER: internal MCLK from BCLK pin, clocks on
    {0x02, 0x18},  // CLK_MANAGER: MULT_PRE x8 -> internal MCLK = 256*fs
    {0x09, 0x0C},  // SDP-in: I2S format, 16-bit
    {0x0D, 0x01},  // SYSTEM: power up analog circuitry
    {0x12, 0x00},  // SYSTEM: power up DAC
    {0x13, 0x10},  // SYSTEM: enable output to HP drive
    {0x32, 0xCF},  // DAC volume +16 dB (M5's default for the 1W speaker)
    {0x37, 0x08},  // DAC: bypass equalizer
};

constexpr RegVal ES8311_MCLK_16K_INIT[] = {
    {0x00, 0x80}, {0x01, 0x3F}, {0x02, 0x00}, {0x03, 0x10}, {0x04, 0x20},
    {0x05, 0x00}, {0x06, 0x03}, {0x07, 0x00}, {0x08, 0xFF}, {0x09, 0x0C},
    {0x0D, 0x01}, {0x12, 0x00}, {0x13, 0x10}, {0x32, 0xCF}, {0x37, 0x08},
};

constexpr uint8_t ES8311_VOL_MAX_REG = 0xCF;  // +16 dB, 0.5 dB/step (0xBF = 0 dB)

uint32_t readLE32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
uint32_t readBE32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
uint16_t readLE16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

bool isEs8311(BoardConfig::AudioOutput output) {
  return output == BoardConfig::AudioOutput::I2sEs8311 ||
         output == BoardConfig::AudioOutput::I2sEs8311Mclk16k;
}

}  // namespace

bool AudioManager::present() const {
  return BoardConfig::ACTIVE.audio.output == BoardConfig::AudioOutput::I2sEs8388 ||
         isEs8311(BoardConfig::ACTIVE.audio.output) ||
         BoardConfig::ACTIVE.audio.output == BoardConfig::AudioOutput::I2sDac;
}

bool AudioManager::codecWrite(uint8_t reg, uint8_t value) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  Wire.beginTransmission(cfg.codecAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool AudioManager::codecInit() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return true;  // plain I2S DAC, nothing to configure

  const RegVal* seq;
  size_t seqLen;
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311Mclk16k) {
    seq = ES8311_MCLK_16K_INIT;
    seqLen = sizeof(ES8311_MCLK_16K_INIT) / sizeof(ES8311_MCLK_16K_INIT[0]);
  } else if (cfg.output == BoardConfig::AudioOutput::I2sEs8311) {
    seq = ES8311_INIT;
    seqLen = sizeof(ES8311_INIT) / sizeof(ES8311_INIT[0]);
  } else {
    seq = ES8388_INIT;
    seqLen = sizeof(ES8388_INIT) / sizeof(ES8388_INIT[0]);
  }

  Wire.begin(cfg.codecSda, cfg.codecScl, CODEC_I2C_HZ);

  // The OEM firmware retries until the codec ACKs; three attempts is plenty
  // for a codec already powered.
  for (int attempt = 0; attempt < 3; ++attempt) {
    bool ok = true;
    for (size_t i = 0; i < seqLen; ++i) {
      if (!codecWrite(seq[i].reg, seq[i].val)) {
        ok = false;
        break;
      }
    }
    if (ok) return true;
    delay(100);
  }
  return false;
}

bool AudioManager::begin() {
  if (begun_) return true;
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (!present()) return false;

  if (cfg.ampEnable != BoardConfig::PIN_UNASSIGNED) {
    pinMode(cfg.ampEnable, OUTPUT);
    digitalWrite(cfg.ampEnable, LOW);  // amp comes up only during playback
  }
  if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
    pinMode(cfg.enable, OUTPUT);
    digitalWrite(cfg.enable, cfg.enableActiveHigh ? HIGH : LOW);
    delay(10);  // codec rail ramp before the first I2C access
  }

  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311Mclk16k && !ensureI2s(16000)) {
    log_e("audio mclk setup failed");
    rollbackBegin();
    return false;
  }

  if (!codecInit()) {
    log_e("audio codec init failed");
    rollbackBegin();
    return false;
  }
  begun_ = true;
  return true;
}

void AudioManager::setVolume(uint8_t percent) {
  if (percent > 100) percent = 100;
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return;
  if (isEs8311(cfg.output)) {
    // Single DAC volume register, 0.5 dB/step; full scale matches the init
    // value M5 ships for this speaker (+16 dB).
    codecWrite(0x32, (uint8_t)((uint16_t)percent * ES8311_VOL_MAX_REG / 100));
    return;
  }
  const uint8_t reg = (uint8_t)((uint16_t)percent * ES8388_VOL_MAX_REG / 100);
  // OUT1 (0x2e/0x2f) and OUT2 (0x30/0x31) pairs, like the OEM volume path.
  codecWrite(0x2e, reg);
  codecWrite(0x2f, reg);
  codecWrite(0x30, reg);
  codecWrite(0x31, reg);
}

// DAC mute between alarms so nothing residual reaches the output.
void AudioManager::codecMute(bool mute) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.codecAddr == 0) return;
  if (isEs8311(cfg.output)) {
    codecWrite(0x31, mute ? 0x60 : 0x00);  // DAC_MUTE bits
  } else {
    codecWrite(0x19, mute ? 0x04 : 0x00);  // ES8388 DACCONTROL3 soft mute
  }
}

void AudioManager::setAmp(bool on) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.ampEnable == BoardConfig::PIN_UNASSIGNED) return;
  digitalWrite(cfg.ampEnable, on ? HIGH : LOW);
}

void AudioManager::rollbackBegin() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  setAmp(false);
  teardownI2s();
  if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
    digitalWrite(cfg.enable, cfg.enableActiveHigh ? LOW : HIGH);
  }
}

void AudioManager::powerDown() {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (!begun_) return;
  stop();
  teardownI2s();
  if (isEs8311(cfg.output)) {
    // Like M5Unified's disable path: drop the amp and the codec rail.
    setAmp(false);
    if (cfg.enable != BoardConfig::PIN_UNASSIGNED) {
      digitalWrite(cfg.enable, cfg.enableActiveHigh ? LOW : HIGH);
    }
  } else if (cfg.codecAddr != 0) {
    codecWrite(0x02, 0xff);  // ES8388 CHIPPOWER: everything off
  }
  begun_ = false;
}

bool AudioManager::parseWavHeader(const WavSource& source, WavInfo& info) {
  uint8_t hdr[12];
  if (!source.seek(0)) return false;
  if (source.read(hdr, 12) != 12) return false;
  if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) return false;

  size_t pos = 12;
  bool haveFmt = false;
  for (int guard = 0; guard < 32; ++guard) {
    uint8_t chunk[8];
    if (!source.seek(pos) || source.read(chunk, 8) != 8) return false;
    const uint32_t size = readLE32(chunk + 4);
    pos += 8;

    if (memcmp(chunk, "fmt ", 4) == 0) {
      uint8_t fmt[16];
      if (size < 16 || source.read(fmt, 16) != 16) return false;
      const uint16_t audioFormat = readLE16(fmt);
      info.channels = readLE16(fmt + 2);
      info.sampleRate = readLE32(fmt + 4);
      info.bitsPerSample = readLE16(fmt + 14);
      if (audioFormat != 1) return false;  // PCM only
      haveFmt = true;
    } else if (memcmp(chunk, "data", 4) == 0) {
      info.dataStart = pos;
      info.dataLength = size;
      break;
    }
    pos += size + (size & 1);  // chunks are word-aligned
  }

  return haveFmt && info.dataStart != 0 && info.bitsPerSample == 16 &&
         (info.channels == 1 || info.channels == 2) && info.sampleRate >= 8000 &&
         info.sampleRate <= 48000;
}

bool AudioManager::ensureI2s(uint32_t sampleRate) {
  const auto& cfg = BoardConfig::ACTIVE.audio;
  if (cfg.output == BoardConfig::AudioOutput::I2sEs8311Mclk16k && sampleRate != 16000) {
    log_e("external-mclk ES8311 supports 16 kHz PCM only");
    return false;
  }
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;

  if (tx && currentRate_ == sampleRate) {
    // Channel exists but was disabled when the last playback drained.
    if (!chanEnabled_) {
      if (i2s_channel_enable(tx) != ESP_OK) return false;
      chanEnabled_ = true;
    }
    return true;
  }

  if (tx) {
    if (chanEnabled_) i2s_channel_disable(tx);
    chanEnabled_ = false;
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
    clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;  // codec runs 256x MCLK/LRCK
    if (i2s_channel_reconfig_std_clock(tx, &clk) != ESP_OK) return false;
    if (i2s_channel_enable(tx) != ESP_OK) return false;
    chanEnabled_ = true;
    currentRate_ = sampleRate;
    return true;
  }

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // Without auto_clear the DMA replays its last buffers on underrun — heard
  // as a looping stutter after playback stops.
  chanCfg.auto_clear = true;
  if (i2s_new_channel(&chanCfg, &tx, nullptr) != ESP_OK) return false;

  i2s_std_config_t std = {};
  std.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
  std.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  std.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  std.gpio_cfg.mclk = cfg.mclk == BoardConfig::PIN_UNASSIGNED ? I2S_GPIO_UNUSED
                                                              : (gpio_num_t)cfg.mclk;
  std.gpio_cfg.bclk = (gpio_num_t)cfg.bclk;
  std.gpio_cfg.ws = (gpio_num_t)cfg.lrclk;
  std.gpio_cfg.dout = (gpio_num_t)cfg.dout;
  std.gpio_cfg.din = I2S_GPIO_UNUSED;

  if (i2s_channel_init_std_mode(tx, &std) != ESP_OK) {
    i2s_del_channel(tx);
    return false;
  }
  if (i2s_channel_enable(tx) != ESP_OK) {
    i2s_del_channel(tx);
    return false;
  }
  txChan_ = tx;
  chanEnabled_ = true;
  currentRate_ = sampleRate;
  return true;
}

void AudioManager::teardownI2s() {
  if (!txChan_) return;
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;
  if (chanEnabled_) i2s_channel_disable(tx);
  i2s_del_channel(tx);
  txChan_ = nullptr;
  chanEnabled_ = false;
  currentRate_ = 0;
}

bool AudioManager::play(const WavSource& source, bool loop) {
  if (!begun_ && !begin()) return false;
  stop();

  WavInfo info;
  if (!parseWavHeader(source, info)) {
    log_e("unsupported WAV (need 16-bit PCM, 1-2ch, 8-48 kHz)");
    return false;
  }
  if (!ensureI2s(info.sampleRate)) {
    log_e("i2s setup failed");
    return false;
  }
  if (!source.seek(info.dataStart)) return false;

  return startPlayback(source, info, loop);
}

bool AudioManager::playPcm(const PcmSource& source, uint32_t sampleRate, uint8_t channels) {
  if (!source.read || sampleRate < 8000 || sampleRate > 48000 || (channels != 1 && channels != 2)) {
    log_e("unsupported PCM (need 16-bit LE, 1-2ch, 8-48 kHz)");
    return false;
  }
  if (!begun_ && !begin()) return false;
  stop();
  if (source.seek && !source.seek(0)) {
    log_e("PCM source seek failed");
    return false;
  }
  if (!ensureI2s(sampleRate)) {
    log_e("i2s setup failed");
    return false;
  }

  WavInfo info;
  info.sampleRate = sampleRate;
  info.channels = channels;
  info.bitsPerSample = 16;

  WavSource wavSource;
  wavSource.read = source.read;
  wavSource.seek = source.seek;
  return startPlayback(wavSource, info, false);
}

bool AudioManager::startPlayback(const WavSource& source, const WavInfo& info, bool loop) {
  // Unmute the DAC (stop() mutes it). Codec writes stay on the caller's core
  // so the shared I2C bus is never touched from the audio task. The speaker
  // amp comes up in the playback task once silence is flowing — enabling it
  // against an idle I2S line is an audible pop.
  codecMute(false);

  kind_ = SourceKind::Wav;
  source_ = source;
  wav_ = info;
  loop_ = loop;
  positionMs_.store(0, std::memory_order_release);
  playbackState_.store(PlaybackState::Playing, std::memory_order_release);

  // Same shape as the OEM "musicTask" (high priority, core 0 — the Arduino
  // loop owns core 1); the stack covers on-task buffers plus SdFat call depth.
  if (xTaskCreatePinnedToCore(taskEntry, "audio_play", TASK_STACK, this, 10, &task_, 0) != pdPASS) {
    task_ = nullptr;
    playbackState_.store(PlaybackState::Idle, std::memory_order_release);
    return false;
  }
  return true;
}

bool AudioManager::playBuffer(const uint8_t* data, size_t len, bool loop) {
  // Shared offset state lives in the lambdas; play() copies them.
  auto offset = std::make_shared<size_t>(0);
  WavSource src;
  src.read = [data, len, offset](uint8_t* dst, size_t want) -> int {
    const size_t left = len - *offset;
    const size_t n = want < left ? want : left;
    memcpy(dst, data + *offset, n);
    *offset += n;
    return (int)n;
  };
  src.seek = [len, offset](size_t pos) {
    if (pos > len) return false;
    *offset = pos;
    return true;
  };
  return play(src, loop);
}

bool AudioManager::requestRestart() {
  if (!source_.seek) return false;
  PlaybackState state = playbackState_.load(std::memory_order_acquire);
  for (;;) {
    if (state == PlaybackState::RestartPending) return true;
    if (state != PlaybackState::Playing) return false;
    if (playbackState_.compare_exchange_weak(state, PlaybackState::RestartPending,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
      return true;
    }
  }
}

bool AudioManager::pause() {
  PlaybackState state = playbackState_.load(std::memory_order_acquire);
  for (;;) {
    if (state == PlaybackState::Paused) return true;
    if (state != PlaybackState::Playing) return false;
    if (playbackState_.compare_exchange_weak(state, PlaybackState::Paused,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
      // Mute now; the amp is dropped by the task once it reaches the pause
      // gate (it may be finishing a blocking I2S write).
      codecMute(true);
      return true;
    }
  }
}

bool AudioManager::resume() {
  PlaybackState state = playbackState_.load(std::memory_order_acquire);
  for (;;) {
    if (state == PlaybackState::Playing) return true;
    if (state != PlaybackState::Paused) return false;
    if (playbackState_.compare_exchange_weak(state, PlaybackState::Playing,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
      codecMute(false);
      if (task_ != nullptr) xTaskNotifyGive(task_);  // wake the blocked task
      return true;
    }
  }
}

bool AudioManager::getPlaybackStatus(PlaybackStatus& out) const {
  const PlaybackState state = playbackState_.load(std::memory_order_acquire);
  out.active = state != PlaybackState::Idle;
  out.paused = state == PlaybackState::Paused;
  out.positionMs = positionMs_.load(std::memory_order_acquire);
  out.durationMs = 0;
  if (kind_ == SourceKind::Mp3) {
    out.durationMs = mp3_.durationMs;
  } else if (wav_.dataLength > 0 && wav_.sampleRate > 0) {
    out.durationMs = (uint32_t)((uint64_t)wav_.dataLength * 1000 /
                                ((uint32_t)wav_.sampleRate * wav_.channels * 2));
  }
  return out.active;
}

bool AudioManager::takeNaturalEnd() { return naturalEnd_.exchange(false); }

void AudioManager::stop() {
  if (!begun_) return;
  PlaybackState state = playbackState_.load(std::memory_order_acquire);
  while (state == PlaybackState::Playing || state == PlaybackState::RestartPending ||
         state == PlaybackState::Paused) {
    if (playbackState_.compare_exchange_weak(state, PlaybackState::Stopping,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
      // A paused task blocks on its notification; wake it so it can finish.
      if (task_ != nullptr) xTaskNotifyGive(task_);
      break;
    }
  }
  for (int i = 0; i < 200 && playbackState_.load(std::memory_order_acquire) != PlaybackState::Idle; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // Drop the amp and mute the DAC so nothing residual reaches the output
  // between alarms.
  setAmp(false);
  codecMute(true);
}

void AudioManager::primeLine(void* txRaw, void* silence, size_t bytes) {
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txRaw;
  memset(silence, 0, bytes);
  // Prime the line with silence, then raise the amp: the amp pops loudly
  // when enabled against an idle or just-started I2S line.
  for (int i = 0; i < 2; ++i) {
    size_t written = 0;
    if (i2s_channel_write(tx, silence, bytes, &written, pdMS_TO_TICKS(200)) != ESP_OK) break;
  }
  setAmp(true);
  if (BoardConfig::ACTIVE.audio.ampSettleMs > 0) {
    vTaskDelay(pdMS_TO_TICKS(BoardConfig::ACTIVE.audio.ampSettleMs));
  }
}

bool AudioManager::waitWhilePaused() {
  setAmp(false);
  while (playbackState_.load(std::memory_order_acquire) == PlaybackState::Paused) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  return playbackState_.load(std::memory_order_acquire) != PlaybackState::Stopping;
}

void AudioManager::finale(void* txRaw, bool natural) {
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txRaw;
  setAmp(false);
  i2s_channel_disable(tx);
  chanEnabled_ = false;
  naturalEnd_.store(natural, std::memory_order_release);

  task_ = nullptr;
  playbackState_.store(PlaybackState::Idle, std::memory_order_release);
  vTaskDelete(nullptr);
}

void AudioManager::taskEntry(void* self) {
  auto* me = static_cast<AudioManager*>(self);
  if (me->kind_ == SourceKind::Mp3) {
    me->mp3TaskLoop();
  } else {
    me->taskLoop();
  }
}

void AudioManager::taskLoop() {
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;
  alignas(int16_t) uint8_t inBuf[READ_CHUNK];
  // Mono is duplicated into both slots, so the out buffer is 2x.
  int16_t outBuf[READ_CHUNK];

  primeLine(tx, outBuf, sizeof(outBuf));

  size_t consumed = 0;
  bool draining = false;
  bool natural = false;
  uint8_t drainBuffers = 0;
  for (;;) {
    PlaybackState state = playbackState_.load(std::memory_order_acquire);

    if (state == PlaybackState::Paused) {
      if (!waitWhilePaused()) {
        finale(tx, false);
        return;
      }
      primeLine(tx, outBuf, sizeof(outBuf));
      continue;
    }

    if (state == PlaybackState::RestartPending) {
      if (!source_.seek || !source_.seek(wav_.dataStart)) {
        log_e("PCM source restart failed");
        playbackState_.store(PlaybackState::Stopping, std::memory_order_release);
      } else {
        consumed = 0;
        draining = false;
        drainBuffers = 0;
        PlaybackState pending = PlaybackState::RestartPending;
        playbackState_.compare_exchange_strong(pending, PlaybackState::Playing,
                                               std::memory_order_acq_rel,
                                               std::memory_order_acquire);
      }
      continue;
    }
    if (state == PlaybackState::Stopping && !draining) {
      draining = true;
      natural = false;
      drainBuffers = 0;
    }

    if (draining) {
      if (drainBuffers >= 6) {
        if (state == PlaybackState::Stopping) {
          finale(tx, natural);
          return;
        }
        // Natural drain finished while still Playing.
        PlaybackState playing = PlaybackState::Playing;
        if (playbackState_.compare_exchange_strong(playing, PlaybackState::Stopping,
                                                   std::memory_order_acq_rel,
                                                   std::memory_order_acquire)) {
          finale(tx, natural);
          return;
        }
        continue;
      }
      memset(outBuf, 0, sizeof(outBuf));
      size_t written = 0;
      if (i2s_channel_write(tx, outBuf, sizeof(outBuf), &written, pdMS_TO_TICKS(200)) != ESP_OK) {
        finale(tx, false);
        return;
      }
      // A user stop landing during the natural tail turns it into a user stop.
      if (playbackState_.load(std::memory_order_acquire) == PlaybackState::Stopping) natural = false;
      ++drainBuffers;
      continue;
    }

    size_t want = READ_CHUNK;
    if (wav_.dataLength > 0) {
      const size_t left = wav_.dataLength - consumed;
      if (left == 0) {
        if (loop_ && source_.seek(wav_.dataStart)) {
          consumed = 0;
          continue;
        }
        draining = true;
        natural = true;
        continue;
      }
      if (want > left) want = left;
    }
    const size_t frameBytes = static_cast<size_t>(wav_.channels) * sizeof(int16_t);
    want -= want % frameBytes;
    if (want == 0) {
      draining = true;
      natural = true;
      continue;
    }

    const int n = source_.read(inBuf, want);
    if (n <= 0) {
      if (loop_ && source_.seek(wav_.dataStart)) {
        consumed = 0;
        continue;
      }
      draining = true;
      natural = true;
      continue;
    }
    if (n > static_cast<int>(want) || static_cast<size_t>(n) % frameBytes != 0) {
      log_e("PCM source returned an invalid byte count");
      finale(tx, false);
      return;
    }
    consumed += n;
    positionMs_.store((uint32_t)((uint64_t)consumed * 1000 /
                                 ((uint32_t)wav_.sampleRate * wav_.channels * 2)),
                      std::memory_order_release);

    const int16_t* samples = (const int16_t*)inBuf;
    size_t outBytes;
    if (wav_.channels == 1) {
      const int frames = n / 2;
      for (int i = 0; i < frames; ++i) {
        outBuf[i * 2] = samples[i];
        outBuf[i * 2 + 1] = samples[i];
      }
      outBytes = (size_t)frames * 4;
    } else {
      memcpy(outBuf, inBuf, n);
      outBytes = (size_t)n;
    }

    size_t written = 0;
    if (i2s_channel_write(tx, outBuf, outBytes, &written, pdMS_TO_TICKS(1000)) != ESP_OK) {
      finale(tx, false);
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// MP3
// ---------------------------------------------------------------------------

bool AudioManager::analyzeMp3(const Mp3Source& source, Mp3Info& info) {
  if (!source.read || !source.seek || !source.size) return false;
  if (!source.seek(0)) return false;

  uint8_t hdr[10];
  if (source.read(hdr, sizeof(hdr)) != (int)sizeof(hdr)) return false;

  size_t start = 0;
  if (memcmp(hdr, "ID3", 3) == 0) {
    // ID3v2.2+: the size at offset 6 is four syncsafe bytes (7 bits each).
    const uint32_t tagSize = ((uint32_t)(hdr[6] & 0x7f) << 21) |
                             ((uint32_t)(hdr[7] & 0x7f) << 14) |
                             ((uint32_t)(hdr[8] & 0x7f) << 7) | (uint32_t)(hdr[9] & 0x7f);
    start = 10 + tagSize;
    if (hdr[5] & 0x10) start += 10;  // footer flag
    if (!source.seek(start)) return false;
  }

  uint8_t probe[8192];
  const int got = source.read(probe, sizeof(probe));
  if (got < 4) return false;

  const int syncOff = MP3FindSyncWord(probe, got);
  if (syncOff < 0) {
    log_e("mp3: no frame sync");
    return false;
  }
  start += syncOff;

  HMP3Decoder probeDec = MP3InitDecoder();
  if (probeDec == nullptr) return false;

  MP3FrameInfo fi;
  if (MP3GetNextFrameInfo(probeDec, &fi, probe + syncOff) != ERR_MP3_NONE) {
    log_e("mp3: invalid first frame (free bitrate?)");
    MP3FreeDecoder(probeDec);
    return false;
  }
  MP3FreeDecoder(probeDec);

  if (fi.samprate <= 0 || (fi.nChans != 1 && fi.nChans != 2) || fi.bitrate <= 0) {
    log_e("mp3: unusable frame params");
    return false;
  }

  // Duration. Prefer the Xing/Info table in the first frame, then VBRI,
  // otherwise estimate from the average bitrate and audio payload size.
  const bool mpeg1 = fi.version == MPEG1;
  const uint16_t spf = mpeg1 ? 1152 : 576;
  uint32_t numFrames = 0;
  bool haveFrames = false;

  const int xingOff = mpeg1 ? (fi.nChans == 1 ? 21 : 36) : (fi.nChans == 1 ? 13 : 21);
  if (syncOff + xingOff + 8 <= (size_t)got) {
    const uint8_t* p = probe + syncOff + xingOff;
    if (memcmp(p, "Xing", 4) == 0 || memcmp(p, "Info", 4) == 0) {
      const uint32_t flags = readBE32(p + 4);
      if (flags & 0x00000001) {  // number-of-frames field present
        numFrames = readBE32(p + 8);
        haveFrames = numFrames > 0;
      }
    }
  }
  // VBRI sits at a fixed 36-byte offset from the frame start.
  if (!haveFrames && syncOff + 36 + 18 <= (size_t)got) {
    const uint8_t* p = probe + syncOff + 36;
    if (memcmp(p, "VBRI", 4) == 0) {
      numFrames = readBE32(p + 14);  // total frames
      haveFrames = numFrames > 0;
    }
  }

  uint32_t durationMs;
  if (haveFrames) {
    durationMs = (uint32_t)((uint64_t)numFrames * spf * 1000 / fi.samprate);
  } else {
    const uint64_t total = source.size();
    const uint64_t audioBytes = total > start ? total - start : 0;
    durationMs = (uint32_t)(audioBytes * 8000ULL / fi.bitrate);
    log_i("mp3: no Xing/VBRI, duration estimated from bitrate");
  }

  info.audioStart = start;
  info.sampleRate = fi.samprate;
  info.channels = fi.nChans;
  info.samplesPerFrame = spf;
  info.durationMs = durationMs;
  return true;
}

bool AudioManager::playMp3(const Mp3Source& source) {
  if (!begun_ && !begin()) return false;
  stop();

  Mp3Info info;
  if (!analyzeMp3(source, info)) {
    log_e("unsupported MP3");
    return false;
  }
  if (!source.seek(info.audioStart)) return false;
  if (!ensureI2s(info.sampleRate)) {
    log_e("i2s setup failed");
    return false;
  }
  return startMp3Playback(source, info);
}

bool AudioManager::startMp3Playback(const Mp3Source& source, const Mp3Info& info) {
  codecMute(false);

  kind_ = SourceKind::Mp3;
  mp3Source_ = source;
  mp3_ = info;
  positionMs_.store(0, std::memory_order_release);
  playbackState_.store(PlaybackState::Playing, std::memory_order_release);

  HMP3Decoder dec = MP3InitDecoder();
  if (dec == nullptr) {
    playbackState_.store(PlaybackState::Idle, std::memory_order_release);
    return false;
  }
  mp3Decoder_ = dec;

  if (xTaskCreatePinnedToCore(taskEntry, "audio_play", TASK_STACK, this, 10, &task_, 0) != pdPASS) {
    MP3FreeDecoder(dec);
    mp3Decoder_ = nullptr;
    task_ = nullptr;
    playbackState_.store(PlaybackState::Idle, std::memory_order_release);
    return false;
  }
  return true;
}

void AudioManager::mp3TaskLoop() {
  i2s_chan_handle_t tx = (i2s_chan_handle_t)txChan_;

  // All big buffers off the task stack — Helix plus SdFat already need depth.
  // Internal RAM first; fall back to PSRAM (slower reads, logged).
  uint8_t* in = (uint8_t*)heap_caps_malloc(MP3_IN_CAP, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (in == nullptr) {
    in = (uint8_t*)heap_caps_malloc(MP3_IN_CAP, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (in != nullptr) log_w("mp3 input buffer located in PSRAM");
  }
  int16_t* out = (int16_t*)heap_caps_malloc(MP3_OUT_SAMPS * sizeof(int16_t),
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (out == nullptr) {
    out = (int16_t*)heap_caps_malloc(MP3_OUT_SAMPS * sizeof(int16_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (out != nullptr) log_w("mp3 output buffer located in PSRAM");
  }
  if (in == nullptr || out == nullptr) {
    log_e("mp3 task buffer allocation failed");
    if (in != nullptr) heap_caps_free(in);
    if (out != nullptr) heap_caps_free(out);
    HMP3Decoder dec = (HMP3Decoder)mp3Decoder_;
    if (dec != nullptr) MP3FreeDecoder(dec);
    mp3Decoder_ = nullptr;
    finale(tx, true);  // nothing to play — let the engine advance
    return;
  }

  primeLine(tx, out, MP3_OUT_SAMPS * sizeof(int16_t));

  HMP3Decoder dec = (HMP3Decoder)mp3Decoder_;
  size_t inLen = 0;
  bool sourceEof = false;
  uint8_t errorCount = 0;
  uint8_t frameLog = 0;
  uint32_t posMs = 0;

  bool draining = false;
  bool natural = false;
  uint8_t drainBuffers = 0;

  for (;;) {
    PlaybackState state = playbackState_.load(std::memory_order_acquire);

    if (state == PlaybackState::Paused) {
      if (!waitWhilePaused()) {
        heap_caps_free(in);
        heap_caps_free(out);
        MP3FreeDecoder(dec);
        mp3Decoder_ = nullptr;
        finale(tx, false);
        return;
      }
      primeLine(tx, out, MP3_OUT_SAMPS * sizeof(int16_t));
      continue;
    }
    if (state == PlaybackState::Stopping && !draining) {
      draining = true;
      natural = false;
      drainBuffers = 0;
    }

    if (draining) {
      if (drainBuffers >= 6) {
        if (state == PlaybackState::Stopping) {
          natural = false;
          break;
        }
        PlaybackState playing = PlaybackState::Playing;
        if (playbackState_.compare_exchange_strong(playing, PlaybackState::Stopping,
                                                   std::memory_order_acq_rel,
                                                   std::memory_order_acquire)) {
          natural = true;
          break;
        }
        continue;
      }
      memset(out, 0, MP3_OUT_SAMPS * sizeof(int16_t));
      size_t written = 0;
      if (i2s_channel_write(tx, out, MP3_OUT_SAMPS * sizeof(int16_t), &written,
                            pdMS_TO_TICKS(200)) != ESP_OK) {
        natural = false;
        break;
      }
      if (playbackState_.load(std::memory_order_acquire) == PlaybackState::Stopping) natural = false;
      ++drainBuffers;
      continue;
    }

    // Top up the input window (remaining bytes are always compacted to front).
    if (!sourceEof && inLen < MP3_IN_CAP) {
      const int rd = mp3Source_.read(in + inLen, MP3_IN_CAP - inLen);
      if (rd <= 0) {
        sourceEof = true;
      } else {
        inLen += rd;
      }
    }

    if (inLen == 0) {
      if (sourceEof) {
        draining = true;
        natural = true;
      }
      continue;
    }

    unsigned char* p = in;
    int bytesLeft = (int)inLen;
    const int rc = MP3Decode(dec, &p, &bytesLeft, out, 0);

    const int consumed = (int)inLen - bytesLeft;
    if (bytesLeft > 0) memmove(in, p, bytesLeft);
    inLen = bytesLeft;

    if (rc == ERR_MP3_MAINDATA_UNDERFLOW) {
      // Bit reservoir needs bytes from later in the stream: refill and retry.
      // Not counted as an error.
      if (sourceEof) {
        draining = true;
        natural = true;
      }
      continue;
    }
    if (rc == ERR_MP3_INDATA_UNDERFLOW) {
      if (sourceEof && inLen == 0) {
        draining = true;
        natural = true;
      }
      continue;
    }
    if (rc != ERR_MP3_NONE) {
      if (errorCount < 4) log_w("mp3 decode error %d — resyncing", rc);
      ++errorCount;
      if (errorCount >= MP3_ERROR_BUDGET) {
        log_e("mp3: too many decode errors, ending track");
        draining = true;
        natural = true;
        continue;
      }
      const int syncOff = MP3FindSyncWord(in, (int)inLen);
      if (syncOff > 0) {
        memmove(in, in + syncOff, inLen - syncOff);
        inLen -= syncOff;
      } else if (syncOff < 0) {
        inLen = 0;
      }
      continue;
    }

    errorCount = 0;
    MP3FrameInfo fi;
    MP3GetLastFrameInfo(dec, &fi);
    if (frameLog < 3) {
      log_i("mp3 frame: %lu Hz, %d ch, %d bps, %d samps", (unsigned long)fi.samprate, fi.nChans,
            fi.bitrate, fi.outputSamps);
      ++frameLog;
    }

    posMs += (uint32_t)((uint64_t)mp3_.samplesPerFrame * 1000 / fi.samprate);
    positionMs_.store(posMs, std::memory_order_release);

    size_t outBytes = (size_t)fi.outputSamps * sizeof(int16_t);
    if (fi.nChans == 1) {
      // Expand mono to stereo in place, backwards so samples aren't clobbered.
      for (int i = fi.outputSamps - 1; i >= 0; --i) {
        out[i * 2] = out[i];
        out[i * 2 + 1] = out[i];
      }
      outBytes = (size_t)fi.outputSamps * 2 * sizeof(int16_t);
    }

    size_t written = 0;
    if (i2s_channel_write(tx, out, outBytes, &written, pdMS_TO_TICKS(1000)) != ESP_OK) {
      natural = false;
      break;
    }
  }

  heap_caps_free(in);
  heap_caps_free(out);
  MP3FreeDecoder(dec);
  mp3Decoder_ = nullptr;
  finale(tx, natural);
}

}  // namespace freeink

#else  // !FREEINK_CAP_AUDIO — stubs so callers need no #ifdefs

namespace freeink {
bool AudioManager::present() const { return false; }
bool AudioManager::begin() { return false; }
void AudioManager::setVolume(uint8_t) {}
bool AudioManager::play(const WavSource&, bool) { return false; }
bool AudioManager::playPcm(const PcmSource&, uint32_t, uint8_t) { return false; }
bool AudioManager::playMp3(const Mp3Source&) { return false; }
bool AudioManager::playBuffer(const uint8_t*, size_t, bool) { return false; }
bool AudioManager::requestRestart() { return false; }
bool AudioManager::pause() { return false; }
bool AudioManager::resume() { return false; }
bool AudioManager::getPlaybackStatus(PlaybackStatus&) const { return false; }
bool AudioManager::takeNaturalEnd() { return false; }
void AudioManager::stop() {}
void AudioManager::powerDown() {}
bool AudioManager::parseWavHeader(const WavSource&, WavInfo&) { return false; }
bool AudioManager::analyzeMp3(const Mp3Source&, Mp3Info&) { return false; }
bool AudioManager::startPlayback(const WavSource&, const WavInfo&, bool) { return false; }
bool AudioManager::startMp3Playback(const Mp3Source&, const Mp3Info&) { return false; }
bool AudioManager::ensureI2s(uint32_t) { return false; }
void AudioManager::teardownI2s() {}
bool AudioManager::codecInit() { return false; }
bool AudioManager::codecWrite(uint8_t, uint8_t) { return false; }
void AudioManager::codecMute(bool) {}
void AudioManager::setAmp(bool) {}
void AudioManager::rollbackBegin() {}
void AudioManager::primeLine(void*, void*, size_t) {}
bool AudioManager::waitWhilePaused() { return false; }
void AudioManager::finale(void*, bool) {}
void AudioManager::taskEntry(void*) {}
void AudioManager::taskLoop() {}
void AudioManager::mp3TaskLoop() {}
}  // namespace freeink

#endif  // FREEINK_CAP_AUDIO
