#ifndef MOVING_AVG_FILTER_H
#define MOVING_AVG_FILTER_H

#include "Filter.h"

/**
 * Class for a moving average filter implementing the Filter interface.
 * @tparam T 
 */
template<typename T>
class MovingAvgFilter: public Filter<T> {
  public:
    MovingAvgFilter(int nsamples) {
        this->nsamples = nsamples;
        last = 0;
        total = (T)0;
        samples = new T[nsamples];
        for(int i = 0; i<nsamples; i++) samples[i] = (T)0;
    }

    /**
     * Computes the filtered output taking into consideration the last nsamples values.
     * @param value to be filtered. 
     * @return T filtered output.
     */
    T filter(T value) {
        total -= samples[last];
        samples[last] = value;
        total += value;

        if(++last > nsamples) last = 0;

        return (T)(total/nsamples);
    }
    
  private:
    T total;
    T *samples;
    int nsamples, last;
};

#endif
