#ifndef FILTERS_H
#define FILTERS_H

#include <cstdint>
#include <type_traits>

#include "VitiMath.hpp"

template <typename T>
class Filter {
    static_assert(std::is_floating_point<T>(), "Only floating points allowed");

  protected:
    T output;

  public:
    Filter()
        : output(0) {
    }

    const T& getOutput() const { return output; }
    void     setOutput(T v) { output = v; }

    virtual T step(T dt, T value) = 0;

    virtual ~Filter() = default;
};

template <typename T>
class FirstOrderFilter : public Filter<T> {
  protected:
    using Filter<T>::output;

  public:
    FirstOrderFilter() : FirstOrderFilter(0.1) {}
    explicit FirstOrderFilter(T rc)
        : Filter<T>(), _rc(rc) {}

    T _rc;

    void setRC(T p_rc) { this->_rc = p_rc; }
    void setF(T f) { setRC(1 / (2 * M_PI * f)); }

  protected:
    T alpha(T dt) const {
        return dt / (dt + _rc);
    }
};

template <typename T>
class LowPassFilter : public FirstOrderFilter<T> {
  protected:
    using FirstOrderFilter<T>::output;
    using FirstOrderFilter<T>::alpha;

  public:
    LowPassFilter() : LowPassFilter(0.1) {}
    explicit LowPassFilter(T rc)
        : FirstOrderFilter<T>(rc) {}

    T step(T dt, T value) override {
        T a = alpha(dt);

        return output = (1 - a) * output + (a)*value;
    }
};

template <typename T>
class SlopeFilter : public Filter<T> {
  protected:
    using Filter<T>::output;

  public:
    T s;
    SlopeFilter() : SlopeFilter(1) {}
    explicit SlopeFilter(T slope)
        : s(slope) {}

    T step(T dt, T value) override {
        T change = value - output;
        return output += constrainAbs(change, dt * s);
    }
};

template <typename T>
class EMAFilter final : public Filter<T> {
    // For more information about EMA, see https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average

  protected:
    using Filter<T>::output;

  private:
    const T alpha;

  public:
    explicit EMAFilter(std::size_t N) : alpha(2.0 / (N + 1.0)) {}

    T step(T dt, T value) override {
        (void)dt;
        output = alpha * value + (1 - alpha) * output;
        return output;
    }

    T step(T value) {
        return step(0, value);
    }
};

#endif // FILTERS_H
