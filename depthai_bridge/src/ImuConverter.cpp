
#include <depthai_bridge/ImuConverter.hpp>
#include <tf/transform_datatypes.h>
namespace dai {

namespace ros {

ImuConverter::ImuConverter(const std::string& frameName, ImuSyncMethod syncMode, double linear_accel_cov, double angular_velocity_cov)
    : _frameName(frameName),
      _syncMode(syncMode),
      _linear_accel_cov(linear_accel_cov),
      _angular_velocity_cov(angular_velocity_cov),
      _sequenceNum(0),
      _steadyBaseTime(std::chrono::steady_clock::now()) {
#ifdef IS_ROS2
    _rosBaseTime = rclcpp::Clock().now();
#else
    _rosBaseTime = ::ros::Time::now();
#endif
}

void ImuConverter::FillImuData_LinearInterpolation(std::vector<IMUPacket>& imuPackets, std::deque<ImuMsgs::Imu>& imuMsgs) {
    // int accelSequenceNum = -1, gyroSequenceNum = -1;
    static std::deque<dai::IMUReportAccelerometer> accelHist;
    static std::deque<dai::IMUReportGyroscope> gyroHist;
    static std::deque<dai::IMUReportRotationVectorWAcc> rotationVecHist;

    for(int i = 0; i < imuPackets.size(); ++i) {
        if(accelHist.size() == 0) {
            accelHist.push_back(imuPackets[i].acceleroMeter);
        } else if(accelHist.back().sequence != imuPackets[i].acceleroMeter.sequence) {
            accelHist.push_back(imuPackets[i].acceleroMeter);
        }

        if(gyroHist.size() == 0) {
            gyroHist.push_back(imuPackets[i].gyroscope);
        } else if(gyroHist.back().sequence != imuPackets[i].gyroscope.sequence) {
        	gyroHist.push_back(imuPackets[i].gyroscope);
        }

        if(rotationVecHist.size() == 0) {
        	rotationVecHist.push_back(imuPackets[i].rotationVector);
        } else if(rotationVecHist.back().sequence != imuPackets[i].rotationVector.sequence) {
        	rotationVecHist.push_back(imuPackets[i].rotationVector);
        }

        if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
            if(accelHist.size() < 3) {
                continue;
            } else {
                dai::IMUReportAccelerometer accel0, accel1;
                dai::IMUReportGyroscope currGyro;
                dai::IMUReportRotationVectorWAcc currRot;
                accel0.sequence = -1;
                DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ", " Interpolating LINEAR_INTERPOLATE_ACCEL mode " << std::endl);
                while(accelHist.size()) {
                    if(accel0.sequence == -1) {
                        accel0 = accelHist.front();
                        accelHist.pop_front();
                    } else {
                        accel1 = accelHist.front();
                        accelHist.pop_front();
                        // auto dt = 1e-9
                        //           * static_cast<double>(
                        //               std::chrono::duration_cast<std::chrono::nanoseconds>(accel1.timestamp.get() - accel0.timestamp.get()).count());

                        // remove std::milli to get in seconds
                        std::chrono::duration<double, std::milli> duration_ms = accel1.timestamp.get() - accel0.timestamp.get();
                        double dt = duration_ms.count();

                        if(!gyroHist.size()) {
                            DEPTHAI_ROS_WARN_STREAM("IMU INTERPOLATION: ", "Gyro data not found. Dropping accel data points");
                        }
                        if(!rotationVecHist.size()){
                        	DEPTHAI_ROS_WARN_STREAM("IMU INTERPOLATION: ", "Rotation data not found. Dropping accel data points");
                        }

                        while(gyroHist.size() && rotationVecHist.size()) {
                            currGyro = gyroHist.front();
                            currRot = rotationVecHist.front();


                            DEPTHAI_ROS_DEBUG_STREAM(
                                "IMU INTERPOLATION: ",
                                "Accel 0: Seq => " << accel0.sequence << " timeStamp => " << (accel0.timestamp.get() - _steadyBaseTime).count() << std::endl);
                            DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ",
                                                     "currGyro 0: Seq => " << currGyro.sequence << "timeStamp => "
                                                                           << (currGyro.timestamp.get() - _steadyBaseTime).count() << std::endl);
                            DEPTHAI_ROS_DEBUG_STREAM(
                                "IMU INTERPOLATION: ",
                                "Accel 1: Seq => " << accel1.sequence << " timeStamp => " << (accel1.timestamp.get() - _steadyBaseTime).count() << std::endl);
                            if(currGyro.timestamp.get() > accel0.timestamp.get() && currGyro.timestamp.get() <= accel1.timestamp.get()) {
                                // auto alpha = std::chrono::duration_cast<std::chrono::nanoseconds>(currGyro.timestamp.get() - accel0.timestamp.get()).count();
                                // / dt;
                                // remove std::milli to get in seconds
                                std::chrono::duration<double, std::milli> diff = currGyro.timestamp.get() - accel0.timestamp.get();
                                const double alpha = diff.count() / dt;
                                dai::IMUReportAccelerometer interpAccel = lerpImu(accel0, accel1, alpha);
                                imuMsgs.push_back(CreateUnitMessage(interpAccel, currGyro, currRot));
                                gyroHist.pop_front();
                                rotationVecHist.pop_front();
                            } else if(currGyro.timestamp.get() > accel1.timestamp.get()) {
                                accel0 = accel1;
                                if(accelHist.size()) {
                                    accel1 = accelHist.front();
                                    accelHist.pop_front();
                                    duration_ms = accel1.timestamp.get() - accel0.timestamp.get();
                                    dt = duration_ms.count();
                                } else {
                                    break;
                                }
                            } else {
                                gyroHist.pop_front();
                                rotationVecHist.pop_front();
                                DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ", "Droppinh GYRO with old timestamps which are below accel10 \n");
                            }
                        }
                        // gyroHist.push_back(currGyro); // Undecided whether this is necessary
                        accel0 = accel1;
                    }
                }
                DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ", "Count  ->" << i << " Placing Accel 0 Seq Number :" << accel0.sequence << std::endl);

                accelHist.push_back(accel0);
            }
        } else if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
            if(gyroHist.size() < 3) {
                continue;
            } else {
                dai::IMUReportGyroscope gyro0, gyro1;
                dai::IMUReportAccelerometer currAccel;
                dai::IMUReportRotationVectorWAcc currRot;
                gyro0.sequence = -1;
                DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ", " Interpolating LINEAR_INTERPOLATE_GYRO mode " << std::endl);
                while(gyroHist.size()) {
                    if(gyro0.sequence == -1) {
                        gyro0 = gyroHist.front();
                        gyroHist.pop_front();
                    } else {
                        gyro1 = gyroHist.front();
                        gyroHist.pop_front();
                        // remove std::milli to get in seconds
                        std::chrono::duration<double, std::milli> duration_ms = gyro1.timestamp.get() - gyro0.timestamp.get();
                        double dt = duration_ms.count();

                        if(!accelHist.size()) {
                            DEPTHAI_ROS_WARN_STREAM("IMU INTERPOLATION: ", "Accel data not found. Dropping data");
                        }
                        if(!rotationVecHist.size()){
                        	DEPTHAI_ROS_WARN_STREAM("IMU INTERPOLATION: ", "Rotation data not found. Dropping accel data points");
                        }
                        while(accelHist.size()) {
                            currAccel = accelHist.front();
                            currRot = rotationVecHist.front();

                            DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ",
                                                     "gyro 0: Seq => " << gyro0.sequence << std::endl
                                                                       << "       timeStamp => " << (gyro0.timestamp.get() - _steadyBaseTime).count()
                                                                       << std::endl);
                            DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ",
                                                     "currAccel 0: Seq => " << currAccel.sequence << std::endl
                                                                            << "       timeStamp => " << (currAccel.timestamp.get() - _steadyBaseTime).count()
                                                                            << std::endl);
                            DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ",
                                                     "gyro 1: Seq => " << gyro1.sequence << std::endl
                                                                       << "       timeStamp => " << (gyro1.timestamp.get() - _steadyBaseTime).count()
                                                                       << std::endl);
                            if(currAccel.timestamp.get() > gyro0.timestamp.get() && currAccel.timestamp.get() <= gyro1.timestamp.get()) {
                                // remove std::milli to get in seconds
                                std::chrono::duration<double, std::milli> diff = currAccel.timestamp.get() - gyro0.timestamp.get();
                                const double alpha = diff.count() / dt;
                                dai::IMUReportGyroscope interpGyro = lerpImu(gyro0, gyro1, alpha);
                                imuMsgs.push_back(CreateUnitMessage(currAccel, interpGyro, currRot));
                                accelHist.pop_front();
                                rotationVecHist.pop_front();
                            } else if(currAccel.timestamp.get() > gyro1.timestamp.get()) {
                                gyro0 = gyro1;
                                if(gyroHist.size()) {
                                    gyro1 = gyroHist.front();
                                    gyroHist.pop_front();
                                    duration_ms = gyro1.timestamp.get() - gyro0.timestamp.get();
                                    dt = duration_ms.count();
                                } else {
                                    break;
                                }
                            } else {
                                accelHist.pop_front();
                                rotationVecHist.pop_front();
                                DEPTHAI_ROS_DEBUG_STREAM("IMU INTERPOLATION: ", "Droppinh ACCEL with old timestamps which are below accel10 \n");
                            }
                        }
                        gyro0 = gyro1;
                    }
                }
                gyroHist.push_back(gyro0);
            }
        }
    }
}

double ImuConverter::toDegrees(double ang)
{
	return 180.0 * (ang / M_PI);
}

double ImuConverter::toRadian(double ang)
{
	return M_PI * (ang / 180.0);
}

ImuMsgs::Imu ImuConverter::CreateUnitMessage(dai::IMUReportAccelerometer accel, dai::IMUReportGyroscope gyro, dai::IMUReportRotationVectorWAcc rotation)
{
    ImuMsgs::Imu interpMsg;
    interpMsg.linear_acceleration.x = accel.x;
    interpMsg.linear_acceleration.y = accel.y;
    interpMsg.linear_acceleration.z = accel.z;

    interpMsg.angular_velocity.x = gyro.x;
    interpMsg.angular_velocity.y = gyro.y;
    interpMsg.angular_velocity.z = gyro.z;


    //transform the imu to oak-FLU
    tf::Quaternion q1(rotation.i, rotation.j, rotation.k, rotation.real);
    tf::Quaternion q2(0.0, -0.707, 0.0, 0.707);
    tf::Quaternion q3;
    q3 = q1*q2;
    q3.normalize();

    interpMsg.orientation.x = q3.x();
    interpMsg.orientation.y = q3.y();
    interpMsg.orientation.z = q3.z();
    interpMsg.orientation.w = q3.w();

    interpMsg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    interpMsg.linear_acceleration_covariance = {_linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov};
    interpMsg.angular_velocity_covariance = {_angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov};

    interpMsg.header.frame_id = _frameName;
#ifndef IS_ROS2
    interpMsg.header.seq = _sequenceNum;
    _sequenceNum++;
#endif

    if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
        interpMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, gyro.timestamp.get());
    } else if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
        interpMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, accel.timestamp.get());
    } else {
        interpMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, accel.timestamp.get());
    }

    return interpMsg;
}

void ImuConverter::toRosMsg(std::shared_ptr<dai::IMUData> inData, std::deque<ImuMsgs::Imu>& outImuMsgs) {
    if(_syncMode != ImuSyncMethod::COPY) {
        FillImuData_LinearInterpolation(inData->packets, outImuMsgs);
    } else {
        for(int i = 0; i < inData->packets.size(); ++i) {
            auto accel = inData->packets[i].acceleroMeter;
            auto gyro = inData->packets[i].gyroscope;
            auto rotation = inData->packets[i].rotationVector;

            //printf("[%f, %f, %f, %f]\n", rotation.i, rotation.j, rotation.k, rotation.real);
            //printf("[%f, %f, %f]\n", accel.x, accel.y, accel.z);

            outImuMsgs.push_back(CreateUnitMessage(accel, gyro, rotation));
        }
    }
}

}  // namespace ros
}  // namespace dai
