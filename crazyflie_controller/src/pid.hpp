#pragma once

#include <ros/ros.h>

class PID {
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        const std::string &name)
        : m_kp              (kp)
        , m_kd              (kd)
        , m_ki              (ki)
        , m_minOutput       (minOutput)
        , m_maxOutput       (maxOutput)
        , m_integratorMin   (integratorMin)
        , m_integratorMax   (integratorMax)
        , m_integral        (0)
        , m_previousError   (0)
        , m_previousTime    (ros::Time::now()) {}

    void reset() {
        m_integral      = 0;
        m_previousError = 0;
        m_previousTime  = ros::Time::now();
    }

    void setIntegral(float integral) {
        m_integral = integral;
    }

    float ki() const {
        return m_ki;
    }

    float update(float value, float targetValue) {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = targetValue - value;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        float P = m_kp * error;
        float D = 0;
        
        if (dt > 0)
            D = m_kd * (error - m_previousError) / dt;
        
        float I = m_ki * m_integral;
        float output = P + I + D;
        m_previousError = error;
        m_previousTime = time;

        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    ros::Time m_previousTime;
};
