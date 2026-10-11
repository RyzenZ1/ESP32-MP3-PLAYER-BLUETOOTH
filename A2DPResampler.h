/*
 * A2DPResampler.h
 *
 * Resamples decoded MP3 PCM to a fixed sink rate (48 kHz stereo by
 * default for the JBL Tune 770NC).  Handles any source rate
 * (16000/22050/32000/44100/48000) and mono or stereo input.
 * Pitch and speed are both preserved.
 */

#ifndef A2DP_RESAMPLER_H
#define A2DP_RESAMPLER_H

#include <Arduino.h>
#include "MP3DecoderHelix.h"

class A2DPResampler {
public:
    static const int BUF_SIZE = 4096;   // stereo frames (~93 ms @ 44.1 kHz)

    A2DPResampler() { reset(); }

    void begin(float sinkRateHz) {
        _sinkRate = sinkRateHz;
        reset();
    }

    void setSinkRate(float hz) { _sinkRate = hz; }
    float sinkRate() const     { return _sinkRate; }

    void reset() {
        _write = 0;
        _read = 0;
        _srcPos = 0.0f;
        _srcRate = 44100;
        _channels = 2;
        _hold = { 0, 0 };
    }

    // Called from MP3 decoder callback
    void feed(MP3FrameInfo &info, int16_t *pcm, size_t len) {
        _srcRate  = info.samprate;
        _channels = info.nChans;

        if (info.nChans == 1) {
            for (size_t i = 0; i < len; i++) {
                int next = (_write + 1) % BUF_SIZE;
                if (next == _read) return;   // full
                _buf[_write] = { pcm[i], pcm[i] };
                _write = next;
            }
        } else {
            for (size_t i = 0; i + 1 < len; i += 2) {
                int next = (_write + 1) % BUF_SIZE;
                if (next == _read) return;
                _buf[_write] = { pcm[i], pcm[i + 1] };
                _write = next;
            }
        }
    }

    // Called from BT A2DP callback
    void pull(int16_t *out, int framesWanted) {
        const float ratio = (float)_srcRate / _sinkRate;

        for (int i = 0; i < framesWanted; i++) {
            int avail = (_write - _read + BUF_SIZE) % BUF_SIZE;
            int off0  = (int)_srcPos;
            float t   = _srcPos - (float)off0;

            int16_t l, r;
            if (off0 + 1 < avail) {
                Frame &f0 = _buf[(_read + off0)     % BUF_SIZE];
                Frame &f1 = _buf[(_read + off0 + 1) % BUF_SIZE];
                l = (int16_t)((float)f0.l + t * ((float)f1.l - (float)f0.l));
                r = (int16_t)((float)f0.r + t * ((float)f1.r - (float)f0.r));
            } else {
                l = _hold.l;
                r = _hold.r;
            }
            _hold = { l, r };

            out[i * 2]     = l;
            out[i * 2 + 1] = r;

            _srcPos += ratio;
            while (_srcPos >= 1.0f) {
                _srcPos -= 1.0f;
                if (_read != _write) {
                    _read = (_read + 1) % BUF_SIZE;
                }
            }
        }
    }

    int available() const { return (_write - _read + BUF_SIZE) % BUF_SIZE; }

private:
    struct Frame { int16_t l, r; };
    Frame          _buf[BUF_SIZE];
    volatile int   _write = 0;
    volatile int   _read  = 0;

    float          _srcPos    = 0.0f;
    float          _sinkRate  = 48000.0f;
    uint32_t       _srcRate   = 44100;
    int            _channels  = 2;
    Frame          _hold      = { 0, 0 };
};

#endif