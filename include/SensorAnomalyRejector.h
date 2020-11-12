/**
 * @author Vincent Spruyt
 * @brief Filter class that sets anomalous sensor readings to a predefined value.
 *
 * NOTE: You have to define FILTER_UPDATE_RATE_HZ 
 * and ANOMALY_REJECTION_WINDOW_SIZE_S before including this file!
 * 
 * We want to deal with two types of outliers:
 * 1) The magnitude of the sensor reading differs significantly from the running average
 * 2) The signal has been fluctuating a lot lately, even though the current value isn't far from the average
 *
 * In both cases, we can not trust the current sensor reading.
 * Therefore, we calculate the running mean and running standard deviation of the magnitude
 * of the sensor data, and define two thresholds:
 * The 'difference_threshold' kicks in to eliminate short-term noise, by invalidating
 * readings that are more than 'difference_threshold' units away from the rolling mean.
 * The 'stddev_treshold' kicks in to eliminate long-term noise, by invalidating readings
 * when the signal has not been stable recently, even if the current value seems fine.
 * This makes sure that we only start trusting sensor readings again when the signal has 
 * been stable for a while, and thus acts as kind of a hystersis on the first threshold.
 * Rolling statistics are calculated using the Welford algorithm, to ensure numerical stability.
 */

#ifndef SENSORANOMALYREJECTOR_H
#define SENSORANOMALYREJECTOR_H

//------------------------------------------------------------------------------
// Includes
#include <CircularBuffer.h>
#include <FusionTypes.h>
#include "Arduino.h"
#include <math.h>

// Assumes that FILTER_UPDATE_RATE_HZ has been defined before including this header file
#define ANOMALY_BUFFER_SIZE (FILTER_UPDATE_RATE_HZ*ANOMALY_REJECTION_WINDOW_SIZE_S)

class SensorAnomalyRejector{
public:
    SensorAnomalyRejector(float difference_threshold, float stddev_threshold, float replacement_value);
    FusionVector3 update(float x, float y, float z);
    bool is_rejecting_magnitude = false;  
    bool is_rejecting_variance = false;  
    bool is_initialized = false;      

private:        
    CircularBuffer<float,ANOMALY_BUFFER_SIZE> values_real_mag;    
    float difference_threshold;
    float stddev_threshold;  
    float replacement_value;  
    float mean_sensor_real_mag = 0.0;
    float varsum_sensor_real_mag = 0.0;
    float std_dev = 0.0;

    // Use Welford's algorithm for online estimate of mean and variance, to avoid numerical instability
    float welford_mean(float prev_mean, float latest_sample, float oldest_sample, int window_size);
    float welford_variance_sum(float prev_varsum, float prev_mean, float new_mean, float latest_sample, float oldest_sample);
};

SensorAnomalyRejector::SensorAnomalyRejector(float difference_threshold, float stddev_threshold, float replacement_value)
                :difference_threshold(difference_threshold), stddev_threshold(stddev_threshold), replacement_value(replacement_value){};

FusionVector3 SensorAnomalyRejector::update(float x, float y, float z){
    
    if(this->values_real_mag.isFull())        
        is_initialized = true;
    else
        is_initialized = false;
        
    float real_mag = sqrt(x*x + y*y + z*z);    

    if(is_initialized){
        float abs_diff = abs((real_mag - this->mean_sensor_real_mag));
        if(abs_diff > this->difference_threshold){
            x = this->replacement_value;
            y = this->replacement_value;
            z = this->replacement_value;
            is_rejecting_magnitude = true;            
        }        
        else
            is_rejecting_magnitude = false;

    }

    // Rolling statistics for the magnitude
    if(this->values_real_mag.size()==0)
        this->mean_sensor_real_mag = real_mag;
    float prev_mean = this->mean_sensor_real_mag;
    this->mean_sensor_real_mag = welford_mean(this->mean_sensor_real_mag, real_mag, this->values_real_mag.first(), this->values_real_mag.capacity);
    this->varsum_sensor_real_mag = welford_variance_sum(this->varsum_sensor_real_mag, prev_mean, this->mean_sensor_real_mag, real_mag, this->values_real_mag.first());        
            
    this->values_real_mag.push(real_mag);
    this->std_dev = sqrt(this->varsum_sensor_real_mag/(this->values_real_mag.size()-1));

    if(is_initialized){
        if(this->std_dev > stddev_threshold){
            x = this->replacement_value;
            y = this->replacement_value;
            z = this->replacement_value;
            is_rejecting_variance = true;
        }        
        else
            is_rejecting_variance = false;
    }    
    
    return {x, y, z};
};

float SensorAnomalyRejector::welford_mean(float prev_mean, float latest_sample, float oldest_sample, int window_size){
        return prev_mean + (latest_sample - oldest_sample)/window_size;
};

float SensorAnomalyRejector::welford_variance_sum(float prev_varsum, float prev_mean, float new_mean, float latest_sample, float oldest_sample){
        return prev_varsum + (latest_sample - prev_mean) * (latest_sample - new_mean) - (oldest_sample - prev_mean) * (oldest_sample - new_mean);
};


#endif
//------------------------------------------------------------------------------
// End of file
