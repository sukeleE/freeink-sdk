#pragma once

// FreeInk audio output.
//
// Drives the audio path described by BoardConfig::ACTIVE.audio: bring up the
// control codec over its (possibly shared) I2C bus, master the I2S bus with
// the new ESP-IDF i2s_std driver, and stream 16-bit PCM data from a
// caller-supplied byte source. Two codecs are supported, selected by
// AudioConfig::output: ES8388 (Murphy M3, OEM-recovered register sequence)
// and ES8311 (M5 PaperColor, mirroring M5Unified's speaker bring-up — the
// codec clocks itself from BCLK, plus the AW8737A amp on its ampEnable pin).
//
// Two stream kinds are supported: WAV (header parsed here) and MP3 (decoded
// with the framework-bundled Helix fixed-point decoder). Playback runs in a
// dedicated FreeRTOS task (priority above typical workers, like the OEM
// "musicTask"), so play() returns immediately; with loop=true the WAV source
// is rewound and replayed until stop() is called — the alarm use case.
// Playback can be paused (task blocks, amp off) and later resumed.
//
// Sources are pairs of callbacks instead of a FILE/Stream so the SDK stays
// storage-agnostic: firmware can serve bytes from LittleFS, SD, or a PROGMEM
// array with the same API.

#include <Arduino.h>

#include <atomic>
#include <functional>

namespace freeink {

class AudioManager {
  enum class PlaybackState : uint8_t { Idle, Playing, Paused, Stopping, RestartPending };

 public:
  struct WavSource {
    // Copy up to len bytes to dst, returning the count (0 = EOF, <0 = error).
    std::function<int(uint8_t* dst, size_t len)> read;
    // Absolute seek from the start of the WAV; used for chunk walking and loop
    // rewind. Return false if unsupported (loop and header re-parse then fail).
    std::function<bool(size_t pos)> seek;
  };

  struct PcmSource {
    // Copy interleaved signed 16-bit little-endian PCM to dst. The callback
    // may block; 0 ends playback and a negative result reports a source error.
    std::function<int(uint8_t* dst, size_t len)> read;
    // Optional absolute byte seek. playPcm() starts at zero; restart requests
    // require this callback.
    std::function<bool(size_t pos)> seek;
  };

  struct Mp3Source {
    // Same contract as WavSource::read; the decoder input is fed through it.
    std::function<int(uint8_t* dst, size_t len)> read;
    // Mandatory absolute seek — frame analysis rewinds before the task starts.
    std::function<bool(size_t pos)> seek;
    // Total file size in bytes, used for the bitrate-based duration estimate.
    std::function<uint64_t()> size;
  };

  struct PlaybackStatus {
    bool active = false;
    bool paused = false;
    uint32_t positionMs = 0;
    uint32_t durationMs = 0;  // 0 when unknown
  };

  // Initializes the codec + enable pin. Returns false when the active board
  // has no audio path (callers can treat audio as absent).
  bool begin();
  bool present() const;

  // Analog output volume, 0-100 (maps onto the codec's OUT1/OUT2 registers).
  void setVolume(uint8_t percent);

  // Starts WAV playback (16-bit PCM, mono or stereo, 8-48 kHz). Stops any
  // current playback first. loop=true replays until stop().
  bool play(const WavSource& source, bool loop);

  // Starts headerless PCM playback. Mono samples are duplicated to both I2S
  // slots. Only 16-bit little-endian, 1-2 channels, and 8-48 kHz are accepted.
  bool playPcm(const PcmSource& source, uint32_t sampleRate, uint8_t channels);

  // Starts MP3 playback (MPEG-1/2/2.5 layer III, mono or stereo). The header
  // is analyzed up front (ID3v2 skip, Xing/VBRI duration when present).
  bool playMp3(const Mp3Source& source);

  // Convenience: play from a memory buffer (e.g. an embedded default sound).
  bool playBuffer(const uint8_t* data, size_t len, bool loop);

  // Request a rewind at the next PCM buffer boundary. Returns false when no
  // active seekable source can accept it, so callers may start new playback.
  bool requestRestart();

  // Pause/resume. Pausing drops the amp and mutes the DAC; the task blocks
  // until resume. Returns false in the wrong state (e.g. while idle).
  bool pause();
  bool resume();

  // Snapshot for UI progress display. Returns false when nothing is playing.
  bool getPlaybackStatus(PlaybackStatus& out) const;

  // One-shot flag: the current stream reached its natural end (as opposed to
  // a user stop()). The first caller after the end gets true.
  bool takeNaturalEnd();

  void stop();
  bool isPlaying() const { return playbackState_.load(std::memory_order_acquire) != PlaybackState::Idle; }

  // Codec power-down (CHIPPOWER off). begin() restores it.
  void powerDown();

 private:
  enum class SourceKind : uint8_t { Wav, Mp3 };

  struct WavInfo {
    uint32_t sampleRate = 0;
    uint16_t channels = 0;
    uint16_t bitsPerSample = 0;
    size_t dataStart = 0;
    size_t dataLength = 0;
  };

  struct Mp3Info {
    uint32_t sampleRate = 0;
    uint16_t channels = 0;
    uint32_t durationMs = 0;
    size_t audioStart = 0;
    uint16_t samplesPerFrame = 1152;  // 1152 for MPEG1, 576 for MPEG2/2.5
  };

  static void taskEntry(void* self);
  void taskLoop();
  void mp3TaskLoop();
  bool parseWavHeader(const WavSource& source, WavInfo& info);
  bool analyzeMp3(const Mp3Source& source, Mp3Info& info);
  bool startPlayback(const WavSource& source, const WavInfo& info, bool loop);
  bool startMp3Playback(const Mp3Source& source, const Mp3Info& info);
  bool ensureI2s(uint32_t sampleRate);
  void teardownI2s();

  // Shared playback-task helpers. Codec register access stays on the caller's
  // core; these only touch the amp GPIO and the I2S channel.
  // Write two buffers of silence, then raise the amp and let it settle.
  void primeLine(void* tx, void* silence, size_t bytes);
  // Common tail: drop the amp, disable the channel, publish state. `natural`
  // distinguishes EOF from a user stop (see takeNaturalEnd). Deletes the task.
  void finale(void* tx, bool natural);
  // Block while Paused (amp already down, DAC already muted by pause()).
  // Returns false when the wait ended because of Stopping instead of resume.
  bool waitWhilePaused();

  bool codecInit();
  bool codecWrite(uint8_t reg, uint8_t value);
  void codecMute(bool mute);
  void setAmp(bool on);
  void rollbackBegin();

  static constexpr uint32_t TASK_STACK = 12288;

  bool begun_ = false;
  std::atomic<PlaybackState> playbackState_{PlaybackState::Idle};
  TaskHandle_t task_ = nullptr;

  SourceKind kind_ = SourceKind::Wav;
  WavSource source_;
  Mp3Source mp3Source_;
  WavInfo wav_;
  Mp3Info mp3_;
  bool loop_ = false;
  void* mp3Decoder_ = nullptr;  // HMP3Decoder (void* to slim the header)

  std::atomic<uint32_t> positionMs_{0};
  std::atomic<bool> naturalEnd_{false};

  void* txChan_ = nullptr;  // i2s_chan_handle_t (kept void* to slim the header)
  volatile bool chanEnabled_ = false;
  uint32_t currentRate_ = 0;
};

}  // namespace freeink

using AudioManager = freeink::AudioManager;
