#ifndef SBG_ROS_SIMU_H
#define SBG_ROS_SIMU_H

// Standard headers
#include <iostream>
#include <map>
#include <string>
#include <memory>

// ROS headers
#include <rclcpp/rclcpp.hpp>
// #include <std_srvs/srv/set_bool.hpp>
// #include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// SbgECom headers
#include <sbgEComLib.h>

// Project headers

namespace sbg
{
    class SbgSimu : public rclcpp::Node
    {
    private:
        //---------------------------------------------------------------------//
        //- Private variables                                                 -//
        //---------------------------------------------------------------------//

        SbgEComHandle m_com_handle_;
        SbgInterface m_sbg_interface_;
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Time m_last_imu_time_;

        //---------------------------------------------------------------------//
        //- Private  methods                                                  -//
        //---------------------------------------------------------------------//

        SbgErrorCode sendDeviceInfo(SbgEComHandle *pHandle);

        void connect(void);

        SbgErrorCode navDataToStream(SbgLogEkfNavData *pNavData, SbgStreamBuffer *pOutputStream);
        SbgErrorCode imuDataToStream(SbgLogImuData *pImuData, SbgStreamBuffer *pOutputStream);
        SbgErrorCode quatDataToStream(SbgLogEkfQuatData *pQuatData, SbgStreamBuffer *pOutputStream);

    public:
        //---------------------------------------------------------------------//
        //- Constructor                                                       -//
        //---------------------------------------------------------------------//

        /*!
         * Default constructor.
         *
         * \param[in] node_name   ROS2 Node name.
         */
        SbgSimu();

        /*!
         * Default destructor.
         */
        ~SbgSimu(void);

        /*!
         * Subscribers.
         */
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subFix;

        /*!
         * IMU callback.
         */
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        /*!
         * GPS callback.
         */
        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    };
}

#endif // SBG_ROS_SIMU_H