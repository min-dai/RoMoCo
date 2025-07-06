/**
 * @author Eric Cousineau <eacousineau@gmail.com>, member of Dr. Aaron Ames's AMBER Lab
 * @author Jenna Reher <jreher@caltech.edu>, member of Dr. Aaron Ames's AMBER Lab
 * @author Min Dai <mdai@caltech.edu>, member of Dr. Aaron Ames's AMBER Lab
 */
#ifndef CONTROL_UTILITIES_FILTERS_HPP_
#define CONTROL_UTILITIES_FILTERS_HPP_

#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

namespace control_utilities
{

/**
 * @brief Compute low-pass filter coefficient
 * @param dt Sample frequency time step
 * @param dt_cutoff Cutoff frequency time step
 */
inline double low_pass_alpha(double dt, double dt_cutoff)
{
    double alpha = dt / (dt_cutoff + dt);
    return alpha;
}

inline VectorXd low_pass_alpha_vec(double dt, VectorXd dt_cutoff)
{
    VectorXd alpha = VectorXd::Zero(dt_cutoff.size());
    for(int i=0; i<dt_cutoff.size(); i++){
        alpha(i) = dt / (dt_cutoff(i) + dt);
    }
    
    return alpha;
}

/**
 * @brief Compute low-pass filter coefficient
 * @param freq Sampling frequency
 * @param freq_sample Cutoff frequency time step
 */
inline double low_pass_alpha_freq(double freq, double freq_cutoff)
{
    return low_pass_alpha(1 / freq, 1 / freq_cutoff);
}

/**
 * @brief Simple first-order low-pass filter
 * @param alpha Filter coefficient
 * @param prev_filt Previous filtered value. If nan, then returns the raw value
 * @param cur_raw Current measurement
 * @return
 */
inline double low_pass_first_order(double alpha, double prev_filt, double cur_raw)
{
    double cur_filt;
    if (std::isnan(prev_filt))
        cur_filt = cur_raw;
    else
        cur_filt = alpha * cur_raw + (1 - alpha) * prev_filt;
    return cur_filt;
}

/**
 * @brief Basic structure for a first-order low-pass IIR filter
 */
class LowPassFilter
{
    /** @brief Filtering coefficient */
    double alpha_;
    /** @brief Previously filtered value */
    double prev_filt_;
public:
    LowPassFilter(double dt, double dt_cutoff)
    {
        reconfigure(dt, dt_cutoff);
    }

    /**
     * @brief reset Reset the value
     * @param value
     */
    void reset(double value = NAN)
    {
        prev_filt_ = value;
    }

    /**
     * @brief reconfigure Reconfigure coefficient, resetting previous value to NAN
     * @param alpha
     */
    void reconfigure(double alpha)
    {
        alpha_ = alpha;
        reset();
    }

    /**
     * @brief reconfigure Reconfigure the filter, resetting the previous value
     * @param dt
     * @param dt_cutoff
     */
    void reconfigure(double dt, double dt_cutoff)
    {
        reconfigure(low_pass_alpha(dt, dt_cutoff));
    }

    /**
     * @brief update Update filter with current raw value, updating memory
     * @param cur_raw
     */
    double update(double cur_raw)
    {
        double cur_filt = low_pass_first_order(alpha_, prev_filt_, cur_raw);
        prev_filt_ = cur_filt;
        return cur_filt;
    }

    double getAlpha() const
    {
        return alpha_;
    }

    double getValue() const
    {
        return prev_filt_;
    }
};

class LowPassFilterVec
{
    /** @brief Filtering coefficient */
    VectorXd alpha_;
    /** @brief Previously filtered value */
    VectorXd prev_filt_;
    bool initialized;

public:
    LowPassFilterVec(double dt, VectorXd dt_cutoff, int size)
    {
        reconfigure(dt, dt_cutoff,size);
        prev_filt_.resize(size);
    }

    /**
     * @brief reset Reset the value
     * @param value
     */
    void reset(double value = NAN)
    {
        prev_filt_ = value*VectorXd::Ones(prev_filt_.size());
        initialized = false;
    }

    /**
     * @brief reconfigure Reconfigure coefficient, resetting previous value to NAN
     * @param alpha
     */
    void reconfigure(VectorXd alpha)
    {
        alpha_ = alpha;
        reset();
    }

    /**
     * @brief reconfigure Reconfigure the filter, resetting the previous value
     * @param dt
     * @param dt_cutoff
     */
    void reconfigure(double dt, VectorXd dt_cutoff, int size)
    {
        reconfigure(low_pass_alpha_vec(dt, dt_cutoff));
        prev_filt_.resize(size);
    }

    /**
     * @brief update Update filter with current raw value, updating memory
     * @param cur_raw
     */
    VectorXd update(VectorXd cur_raw)
    {
        VectorXd cur_filt= VectorXd::Zero(cur_raw.size());
        if (initialized)
        for(int i =0; i < cur_raw.size(); i++){
            cur_filt(i) = alpha_(i) * cur_raw(i) + (1 - alpha_(i)) * prev_filt_(i);
        }
            
        else {
            cur_filt = cur_raw;
            initialized = true;
        }

            
        prev_filt_ = cur_filt;
        return cur_filt;
    }

    VectorXd getAlpha() const
    {
        return alpha_;
    }

    VectorXd getValue() const
    {
        return prev_filt_;
    }
};


}

#endif // CONTROL_UTILITIES_FILTERS_HPP_
