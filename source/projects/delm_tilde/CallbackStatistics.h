//
//  CallbackStatistics.h
//  Greenspec
//
//  Created by Martin Gasser on 25/01/2017.
//
//

#ifndef CallbackStatistics_h
#define CallbackStatistics_h

#include <limits>
#include <boost/lockfree/spsc_queue.hpp>
#include <iostream>
#include <chrono>
#include <thread>

struct StatData {
  double computation;
  //double period;
};

class CallbackStatistics {
public:
  
  class ScopedTimer {
    friend class CallbackStatistics;
  public:
    ~ScopedTimer() {
      stat_.timeStop();
    }
    
  private:
    ScopedTimer(CallbackStatistics& stat) :
      stat_(stat) {
        stat_.timeStart();
    }
    
    CallbackStatistics& stat_;
  };


  CallbackStatistics() :
    measurements_(512),
    running_{true},
    mean_(0.0),
    variance_(0.0),
    maximum_(0.0),
    minimum_(std::numeric_limits<double>::max()),
    bufferSize_(0),
    preCount_(10) {

     calculator_ = std::thread([this]() {
       while (this->running_) {
         this->measurements_.consume_all([this](auto statData) -> void {
           auto computation = statData.computation;
           //auto period = statData.period;
           auto period = bufferSize_ * 1000000.0/sampleRate_;
           std::lock_guard<std::mutex> guard(this->mutex_);
           if (this->preCount_) {
             this->preCount_--;
             return;
           }
           
           this->updateStatistics(computation, period);
         });
       }
     });
  }
  
  ~CallbackStatistics() {
     running_.store(false);
     calculator_.join();    
  }
  
  void reset() {
    std::lock_guard<std::mutex> guard(this->mutex_);
    mean_ = 0.0;
    variance_ = 0.0;
    maximum_ = 0.0;
    minimum_ = std::numeric_limits<double>::max();
    preCount_ = 10;
    reset_ = true;
  }
  
  void updateStatistics(float computation, float period) {
    const float alpha = 0.01;
    const float diff = (computation-mean_);
    const float incr = alpha*diff;
    mean_ = mean_+incr;
    variance_ = (1.0-alpha) * (variance_  + diff*incr);
    
    // const double stddev = sqrt(variance_);
    
    maximum_ = computation > maximum_ ? computation : maximum_;
    minimum_ = computation < minimum_ ? computation : minimum_;
    
    if (computation > 0.8*period) {
      std::cout << "Glitch: ";
      std::cout << "Buffersize: " << bufferSize_ << ", callback period: "  << period << ", current computation: " << computation << ", mean: " <<  mean_ << ", min: " << minimum_ <<  ", max: " << maximum_ <<  std::endl;
    }
    else {
      static int ctr = 50;
      
      
      if (ctr-- == 0) {
        std::cout << "Buffersize: " << bufferSize_ << ", callback period: "  << period << ", current computation: " << computation << ", mean: " <<  mean_ << ", min: " << minimum_ <<  ", max: " << maximum_ << std::endl;
        ctr = 50;
      }
    }

  }
  
  void timeStart() {
    const auto time = std::chrono::high_resolution_clock::now();
    //if (! reset_) {
    //  period_ = time-start_;
    //}
    start_ = time;
    reset_ = false;
  }

  
  void timeStop() {
    std::chrono::duration<double> computation = std::chrono::high_resolution_clock::now()-start_;
    double c = std::chrono::duration_cast<std::chrono::microseconds>(computation).count();
    //double p = std::chrono::duration_cast<std::chrono::microseconds>(period_).count();
    //measurements_.push({ c, p});
    
    measurements_.push({ {c} });
  }
  
  ScopedTimer scopedTimer(int bufferSize, double sampleRate) {
    bufferSize_ = bufferSize;
    sampleRate_ = sampleRate;
    return ScopedTimer(*this);
  }
  
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  boost::lockfree::spsc_queue<StatData> measurements_;
  
  std::thread calculator_;
  
  std::atomic<bool> running_;
  
  bool reset_;
  std::chrono::duration<double> period_;
  double mean_;
  double variance_;
  double maximum_;
  double minimum_;
  int bufferSize_;
  double sampleRate_;
  
  std::mutex mutex_;
  int preCount_;
};

#endif /* CallbackStatistics_h */
