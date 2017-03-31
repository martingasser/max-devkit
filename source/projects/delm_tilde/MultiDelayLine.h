//
//  MultiDelayLine.h
//  max-devkit
//
//  Created by Martin Gasser on 25/03/2017.
//
//

#ifndef MultiDelayLine_h
#define MultiDelayLine_h

#include <numeric>
#include <vector>
#include <algorithm>
#include <iostream>

#include <boost/simd/algorithm/transform.hpp>
#include <boost/simd/function/all.hpp>
#include <boost/simd/function/sum.hpp>
#include <boost/simd/memory.hpp>

#include "CallbackStatistics.h"

int nextPower2(unsigned int n) {
  n--;
  n |= n >> 1;
  n |= n >> 2;
  n |= n >> 4;
  n |= n >> 8;
  n |= n >> 16;
  n++;
  return n;
}

class MultiDelayLine {
public:
  using DelayTimes = std::vector<float>;

  MultiDelayLine() {}

  MultiDelayLine(MultiDelayLine const &) = delete;

  ~MultiDelayLine() {}

  void init(float sampleRate, float maxDelayTime, int bufferSize,
            int numChannels) {

    sampleRate_ = sampleRate;
    maxDelayTime_ = maxDelayTime;
    bufferSize_ = bufferSize;
    numChannels_ = numChannels;

    int l = maxDelayTime * sampleRate_;
    if (l < 1)
      l = 1;
    delaySize_ = nextPower2(l);

    mask_ = delaySize_ - 1;

    DelayLine line(delaySize_, 0.0f);
    delayLines_.resize(numChannels_, line);

    mixBuffer_.resize(numChannels_ * delaySize_, 0.0f);

    int numDelays = numChannels_ * numChannels_;
    DelayValues values(numChannels_, 0);
    delays_.resize(numDelays, values);
    for (auto& d : delays_) {
      int n{2000};
      std::generate(d.begin(), d.end(),
                    [this, &n] { return (n -= 500); });
    }
    
    
    channelInd_.resize(numChannels_, std::vector<int>(numChannels_) );
    
    for (int i = 0; i < numChannels_; ++i) {
      auto& c = channelInd_[i];
      std::iota(c.begin(), c.end(), 0);
      
      auto& delays = delays_[i];

      std::sort(c.begin(), c.end(), [c, delays](int d1, int d2) {
        return delays[d1] > delays[d2];
      });
    }
    
    writeIndex_ = 0;
    canAdd_.resize(numChannels_, false);
    
    stats_.reset();
  }

  void reset() {
    for (auto &dl : delayLines_) {
      dl.assign(delaySize_, 0.0f);
    }

    mixBuffer_.assign(numChannels_ * bufferSize_, 0.0f);

    writeIndex_ = 0;
  }

  void setDelays(int channel, DelayTimes &times) {
    const float samplesPerMS = sampleRate_ / 1000.0f;

    DelayValues& delays = delays_[channel];

    for (int c = 0; c < numChannels_; ++c) {
      delays[c] = int(times[c] * samplesPerMS);
    }
  }

  void process(double **ins, double **outs) {

    CallbackStatistics::ScopedTimer timer =
        stats_.scopedTimer(bufferSize_, sampleRate_);

    canAdd_.assign(numChannels_, false);
    
    for (int inChannel = 0; inChannel < numChannels_; ++inChannel) {
      float *delayLine = delayLines_[inChannel].data(); // get delay line for inChannel
      const double *input = ins[inChannel];
      writeSamples(input, delayLine);
      
      DelayValues& delays = delays_[inChannel];  
      
      //for (int outChannel = 0; outChannel < numChannels_; ++outChannel) {
      for (int outChannel : channelInd_[inChannel]) {
        const int currentDelay = delays[outChannel];
        const int delayPos = (writeIndex_ - currentDelay) & mask_;

        float *mixPtr = &mixBuffer_[outChannel * bufferSize_];

        float *dp = &delayLine[delayPos];
        const float *ep = delayLine + delaySize_;

        if (canAdd_[outChannel]) {
          
          pack_t pack1;
          pack_t pack2;

          int n = bufferSize_ / packCard_;
          while (n--) {
            pack1 = boost::simd::aligned_load<pack_t>(mixPtr);
            pack2 = boost::simd::aligned_load<pack_t>(dp);
            pack1 = pack2 + pack1;
            boost::simd::aligned_store(pack1, mixPtr);
            mixPtr += packCard_;
            dp += packCard_;

            if (dp == ep) {
              dp = delayLine;
            }
          }
          
          /*
          int n = bufferSize_;
          while (n--) {
            *mixPtr++ += *dp++;
            if (dp == ep) {
              dp = delayLine;
            }
          }*/
        }
        else {

          int n = bufferSize_;
          while (n--) {
            *mixPtr++ = *dp++;
            if (dp == ep) {
              dp = delayLine;
            }
          }
          canAdd_[outChannel] = true;
        }
      }
    }

    for (int outChannel = 0; outChannel < numChannels_; ++outChannel) {
      const float *m = &mixBuffer_[outChannel * bufferSize_];
      std::copy(m, m + bufferSize_, outs[outChannel]);
    }

    writeIndex_ = (writeIndex_ + bufferSize_) & mask_;
  }

private:
  void writeSamples(const double *input, float *delayLine) const {

    int n = bufferSize_;
    const float *ep = delayLine + delaySize_;
    float *bp = delayLine + writeIndex_;

    while (n--) {
      *bp++ = *input++;
      if (bp == ep) {
        bp = delayLine;
      }
    }
  }

  using pack_t = boost::simd::pack<float>;
  static const size_t packCard_ = boost::simd::cardinal_of<pack_t>();

  using float_alloc_t = boost::simd::allocator<float, packCard_>;
  using DelayValues = std::vector<int>;
  using DelayLine = std::vector<float, float_alloc_t>;

  float sampleRate_ = 0.0f;
  float maxDelayTime_ = 0.0f;
  int bufferSize_ = 0;
  int numChannels_ = 0;
  int delaySize_ = 0;
  int writeIndex_ = 0;

  int mask_ = 0;

  std::vector<DelayLine> delayLines_;
  std::vector<DelayValues> delays_;
  std::vector<float, float_alloc_t> mixBuffer_;

  std::vector<bool> canAdd_;
  std::vector< std::vector< int > > channelInd_;
  
  CallbackStatistics stats_;
};

#endif /* MultiDelayLine_h */
