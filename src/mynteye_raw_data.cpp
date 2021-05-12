#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/clock.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/device/context.h"
#include "mynteye/device/device.h"

#define GRAVITY 9.8
#define QOS_LENGTH 7
#define ROS_FRAMERATE_CUT 2
#define MATCH_CHECK_THRESHOLD 3

using namespace mynteye;

class MynteyeRawData : public rclcpp::Node
{
  public:
    ~MynteyeRawData()
    {
        if(device != nullptr) 
        {
            device->Stop(Source::ALL);
        }
    }

    MynteyeRawData() : Node("mynteye_raw_data"), qos_profile_(0), skip_tag_(-1), skip_tmp_left_tag_(0), skip_tmp_right_tag_(0)
    {
        unit_hard_time_ *= 10;
        rosParameters();
        initDevice();
        pthread_mutex_init(&mutex_data_, nullptr);
        createPublishers();
        int frame_rate = device->GetOptionValue(Option::FRAME_RATE);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / frame_rate), std::bind(&MynteyeRawData::publishTopics, this));
    }

  private:
    void rosParameters()
    {
        request_index_ = 0;
        left_frame_id_ = "mynteye_left_frame";
        right_frame_id_ = "mynteye_right_frame";
        imu_frame_id_ = "mynteye_imu_frame";
        temperature_frame_id_ = "mynteye_temperature_frame";
        gravity_ = GRAVITY;
        option_names_ = {{Option::GAIN, "gain"},
          {Option::BRIGHTNESS, "brightness"},
          {Option::CONTRAST, "contrast"},
          {Option::FRAME_RATE, "frame_rate"},
          {Option::IMU_FREQUENCY, "imu_frequency"},
          {Option::EXPOSURE_MODE, "exposure_mode"},
          {Option::MAX_GAIN, "max_gain"},
          {Option::MAX_EXPOSURE_TIME, "max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "desired_brightness"},
          {Option::IR_CONTROL, "ir_control"},
          {Option::HDR_MODE, "hdr_mode"},
          {Option::ACCELEROMETER_RANGE, "accel_range"},
          {Option::GYROSCOPE_RANGE, "gyro_range"}};
        qos_length_ = QOS_LENGTH;
        ros_framerate_cut_ = ROS_FRAMERATE_CUT;

        this->declare_parameter("request_index", request_index_);
        for(auto &&it = option_names_.begin(); it != option_names_.end(); ++it)
        {
            int value = -1;
            this->declare_parameter(it->second, value);
        }
        this->declare_parameter("qos_length", qos_length_);
        this->declare_parameter("ros_framerate_cut", ros_framerate_cut_);
    }

    void initDevice()
    {   
        device = selectDevice();
        RCLCPP_FATAL_EXPRESSION(this->get_logger(), device == nullptr, "No Supported Device found :(");

        auto &&requests = device->GetStreamRequests();
        std::size_t m = requests.size();
        RCLCPP_FATAL_EXPRESSION(this->get_logger(), m <= 0, "No MYNT EYE devices :(");

        int request_index = 0;
        this->get_parameter("request_index", request_index);
        if(m <= 1) 
        {
            RCLCPP_INFO(this->get_logger(),"Only one stream request, selecting index: 0");
            device->ConfigStreamRequest(requests[0]);
        }
        else 
        {
            if(request_index >= m)
            {
                RCLCPP_WARN(this->get_logger(),"Request_index out of range");
                device->ConfigStreamRequest(requests[0]);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Selecting stream, index: %d", request_index);
                device->ConfigStreamRequest(requests[request_index]);
            }
        }

        for (auto &&it = option_names_.begin(); it != option_names_.end(); ++it)
        {
            if (!device->Supports(it->first)) continue;
            int value = -1;
            this->get_parameter(it->second, value);
            if (value != -1)
            {
                RCLCPP_INFO(this->get_logger(), "Set %s to %d", it->second.c_str(), value);
                device->SetOptionValue(it->first, value);
            }
            RCLCPP_INFO(this->get_logger(), "option::%s : %d", it->second.c_str(), device->GetOptionValue(it->first));
        }

        int framerate_cut;
        this->get_parameter("ros_framerate_cut", framerate_cut);
        if(framerate_cut > 0 && framerate_cut < 7)
        {
            skip_tag_ = framerate_cut;
        }

        is_started_ = false;
    }

    std::shared_ptr<Device> selectDevice()
    {
        Context context;
        auto &&devices = context.devices();

        size_t n = devices.size();
        RCLCPP_FATAL_EXPRESSION(this->get_logger(), n <= 0, "No MYNT EYE devices :(");
        for (size_t i = 0; i < n; i++)
        {
            auto &&device = devices[i];
            auto &&model = device->GetModel();
            if(model == Model::STANDARD)
            {
                auto &&name = device->GetInfo(Info::DEVICE_NAME);
                auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
                RCLCPP_INFO(this->get_logger(), "Selecting MYNT EYE device; name: %s serial number: %s", name.c_str(), serial_number.c_str());
                return device;
            }
        }
        return nullptr;
    }

    void createPublishers()
    {
        size_t qos_length;
        this->get_parameter("qos_length", qos_length);
        qos_profile_ = rclcpp::QoS(rclcpp::QoSInitialization(rclcpp::KeepLast(qos_length)));
        qos_profile_.reliable();
        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_raw", qos_profile_);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_raw", qos_profile_);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", qos_profile_);
        temperature_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature/data_raw", qos_profile_);
    }

    void publishTopics()
    {
        if(this->count_subscribers("left/image_raw") > 0 && !is_published_left_)
        {
            device->SetStreamCallback(Stream::LEFT, [&](const device::StreamData &data)
                                        {
                                            ++left_count_;
                                            if(left_count_ > 10)
                                            {
                                                rclcpp::Time stamp = checkUpTimeStamp(data.img->timestamp, Stream::LEFT);
                                                if(skip_tag_ > 0)
                                                {
                                                    if(skip_tmp_left_tag_ == 0)
                                                    {
                                                        skip_tmp_left_tag_ = skip_tag_;
                                                    }
                                                    else
                                                    {
                                                        skip_tmp_left_tag_--;
                                                        return;
                                                    }
                                                    if(left_timestamps_.size() < MATCH_CHECK_THRESHOLD)
                                                    {
                                                        left_timestamps_.insert(left_timestamps_.begin(), data.img->timestamp);
                                                    }
                                                    else
                                                    {
                                                        left_timestamps_.insert(left_timestamps_.begin(), data.img->timestamp);
                                                        left_timestamps_.pop_back();
                                                    }
                                                }
                                                publishImage(left_frame_id_, data, stamp);
                                            }
                                        });
            is_published_left_ = 1;
        }

        if(this->count_subscribers("right/image_raw") > 0 && !is_published_right_)
        {
            device->SetStreamCallback(Stream::RIGHT, [&](const device::StreamData &data)
                                        {
                                            ++right_count_;
                                            if(right_count_ > 10)
                                            {
                                                rclcpp::Time stamp = checkUpTimeStamp(data.img->timestamp, Stream::RIGHT);
                                                if(skip_tag_ > 0)
                                                {
                                                    if(skip_tmp_right_tag_ == 0)
                                                    {
                                                        skip_tmp_right_tag_ = skip_tag_;
                                                    }
                                                    else
                                                    {
                                                        skip_tmp_right_tag_--;
                                                        return;
                                                    }
                                                    if(right_timestamps_.size() < MATCH_CHECK_THRESHOLD)
                                                    {
                                                        right_timestamps_.insert(right_timestamps_.begin(), data.img->timestamp);
                                                    }
                                                    else
                                                    {
                                                        right_timestamps_.insert(right_timestamps_.begin(), data.img->timestamp);
                                                        right_timestamps_.pop_back();
                                                        bool is_match = false;
                                                        for(size_t i = 0; i < right_timestamps_.size(); i++)
                                                        {
                                                            for(size_t j = 0; j < left_timestamps_.size(); j++)
                                                            {
                                                                if(right_timestamps_[i] == left_timestamps_[j])
                                                                {
                                                                    is_match = true;
                                                                    break;
                                                                }
                                                            }
                                                            if(is_match)
                                                            {
                                                                break;
                                                            }
                                                        }
                                                        if(!is_match)
                                                        {
                                                            skip_tmp_right_tag_++;
                                                        }
                                                    }
                                                }
                                                publishImage(right_frame_id_, data, stamp);
                                            }
                                        });
            is_published_right_ = 1;
        }

        if(!is_published_motion_)
        {
            device->SetMotionCallback([this](const device::MotionData &data)
                                        {
                                            rclcpp::Time stamp = checkUpImuTimeStamp(data.imu->timestamp);
                                            ++motion_count_;
                                            if (motion_count_ > 50 && data.imu)
                                            {
                                                if (data.imu)
                                                {
                                                    if (data.imu->flag == 1) // accelerometer
                                                    {  
                                                        imu_accel_ = data.imu;
                                                        publishImuBySync();
                                                    }
                                                    else if (data.imu->flag == 2) // gyroscope
                                                    {
                                                        imu_gyro_ = data.imu;
                                                        publishImuBySync();
                                                    }
                                                    else
                                                    {
                                                        publishImu(data, stamp);
                                                        publishTemperature(data.imu->temperature, stamp);
                                                    }
                                                }
                                                else
                                                {
                                                    RCLCPP_WARN(this->get_logger(), "Motion data is empty");
                                                }
                                                rclcpp::Rate rate(std::chrono::milliseconds(1));
                                                rate.sleep();
                                            }
                                        });
            is_published_motion_ = 1;
        }

        if (!is_started_)
        {
            device->Start(Source::ALL);
            is_started_ = true;
        }
    }

    rclcpp::Time checkUpTimeStamp(std::uint64_t _hard_time, const Stream &stream)
    {
        static std::map<Stream, std::uint64_t> hard_time_now;
        static std::map<Stream, std::uint64_t> acc;
        if(is_overflow(_hard_time, hard_time_now[stream])) 
        {
            acc[stream]++;
        }
        hard_time_now[stream] = _hard_time;
        return hardTimeToSoftTime(acc[stream] * unit_hard_time_ + _hard_time);
    }

    rclcpp::Time checkUpImuTimeStamp(std::uint64_t _hard_time)
    {
        static std::uint64_t hard_time_now(0), acc(0);
        if (is_overflow(_hard_time, hard_time_now))
        {
            acc++;
        }
        hard_time_now = _hard_time;
        return hardTimeToSoftTime(acc * unit_hard_time_ + _hard_time);
    }

    inline bool is_overflow(std::uint64_t now, std::uint64_t pre)
    {
        return (now < pre) && ((pre - now) > (unit_hard_time_ / 2));
    }

    rclcpp::Time hardTimeToSoftTime(std::uint64_t _hard_time)
    {
        static bool isInited = false;
        static double soft_time_begin(0);
        static std::uint64_t hard_time_begin(0);
        if (false == isInited)
        {
            rclcpp::Clock clock_temp;
            soft_time_begin = clock_temp.now().seconds();
            hard_time_begin = _hard_time;
            isInited = true;
        }
        std::uint64_t time_ns_detal = (_hard_time - hard_time_begin);
        std::uint64_t time_ns_detal_s = time_ns_detal / 1000000;
        std::uint64_t time_ns_detal_ns = time_ns_detal % 1000000;
        double time_sec_double =
        rclcpp::Time(time_ns_detal_s, time_ns_detal_ns * 1000).seconds();
        return rclcpp::Time(soft_time_begin + time_sec_double);
    }

    void publishImage(const std::string &frame_id, const device::StreamData &data, rclcpp::Time stamp)
    {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id;
        pthread_mutex_lock(&mutex_data_);
        cv::Mat img(data.frame->height(), data.frame->width(), CV_8UC1, data.frame->data());
        auto &&msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
        pthread_mutex_unlock(&mutex_data_);
        if(frame_id == "mynteye_left_frame")
        {
            left_pub_->publish(std::move(msg));
        }
        else if(frame_id == "mynteye_right_frame")
        {
            right_pub_->publish(std::move(msg));
        }  
    }

    void publishImuBySync()
    {
        timestampAlign();
        for(int i = 0; i < imu_align_.size(); i++)
        {
            rclcpp::Time stamp = checkUpImuTimeStamp(imu_align_[i].timestamp);
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = imu_frame_id_;
            msg.linear_acceleration.x = imu_align_[i].accel[0] * gravity_;
            msg.linear_acceleration.y = imu_align_[i].accel[1] * gravity_;
            msg.linear_acceleration.z = imu_align_[i].accel[2] * gravity_;
            msg.angular_velocity.x = imu_align_[i].gyro[0] * M_PI / 180;
            msg.angular_velocity.y = imu_align_[i].gyro[1] * M_PI / 180;
            msg.angular_velocity.z = imu_align_[i].gyro[2] * M_PI / 180;
            for (int j = 0; j < 9; j++)
            {
                msg.linear_acceleration_covariance[j] = 0;
                msg.angular_velocity_covariance[j] = 0;
            }
            imu_pub_->publish(std::move(msg));
            publishTemperature(imu_align_[i].temperature, stamp);
        }
    }

    void timestampAlign()
    {
        static std::vector<ImuData> acc_buf;
        static std::vector<ImuData> gyro_buf;
        if(imu_accel_ != nullptr)
        {
            acc_buf.push_back(*imu_accel_);
        }
        if(imu_gyro_ != nullptr)
        {
            gyro_buf.push_back(*imu_gyro_);
        }
        imu_accel_ = nullptr;
        imu_gyro_ = nullptr;
        imu_align_.clear();
        if (acc_buf.empty() || gyro_buf.empty()) return;
        ImuData imu_temp;
        auto itg = gyro_buf.end();
        auto ita = acc_buf.end();
        for (auto it_gyro = gyro_buf.begin(); it_gyro != gyro_buf.end(); it_gyro++)
        {
            for (auto it_acc = acc_buf.begin(); it_acc+1 != acc_buf.end(); it_acc++)
            {
                if (it_gyro->timestamp >= it_acc->timestamp && it_gyro->timestamp <= (it_acc+1)->timestamp)
                {
                    double k = static_cast<double>((it_acc+1)->timestamp - it_acc->timestamp);
                    k = static_cast<double>(it_gyro->timestamp - it_acc->timestamp) / k;
                    imu_temp = *it_gyro;
                    imu_temp.accel[0] = it_acc->accel[0] + ((it_acc+1)->accel[0] - it_acc->accel[0]) * k;
                    imu_temp.accel[1] = it_acc->accel[1] + ((it_acc+1)->accel[1] - it_acc->accel[1]) * k;
                    imu_temp.accel[2] = it_acc->accel[2] + ((it_acc+1)->accel[2] - it_acc->accel[2]) * k;
                    imu_align_.push_back(imu_temp);
                    itg = it_gyro;
                    ita = it_acc;
                }
            }
        }
        if (itg != gyro_buf.end())
        {
            gyro_buf.erase(gyro_buf.begin(), itg + 1);
        }
        if (ita != acc_buf.end())
        {
            acc_buf.erase(acc_buf.begin(), ita);
        }
    }

    void publishImu(const device::MotionData &data, rclcpp::Time stamp)
    {
        if (this->count_subscribers("imu/data_raw") == 0) return;
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = imu_frame_id_;
        msg.linear_acceleration.x = data.imu->accel[0] * gravity_;
        msg.linear_acceleration.y = data.imu->accel[1] * gravity_;
        msg.linear_acceleration.z = data.imu->accel[2] * gravity_;
        msg.angular_velocity.x = data.imu->gyro[0] * M_PI / 180;
        msg.angular_velocity.y = data.imu->gyro[1] * M_PI / 180;
        msg.angular_velocity.z = data.imu->gyro[2] * M_PI / 180;
        for (int i = 0; i < 9; i++)
        {
            msg.linear_acceleration_covariance[i] = 0;
            msg.angular_velocity_covariance[i] = 0;
        }
        imu_pub_->publish(std::move(msg));
    }

    void publishTemperature(float temperature, rclcpp::Time stamp)
    {
        if (this->count_subscribers("temperature/data_raw") == 0) return;
        sensor_msgs::msg::Temperature msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = temperature_frame_id_;
        msg.temperature = temperature;
        msg.variance = 0;
        temperature_pub_->publish(std::move(msg));
    }

    std::uint64_t unit_hard_time_ = std::numeric_limits<std::uint32_t>::max();

    int request_index_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    std::string imu_frame_id_;
    std::string temperature_frame_id_;
    double gravity_;
    std::map<Option, std::string> option_names_;
    int qos_length_;
    int ros_framerate_cut_;

    std::shared_ptr<Device> device;

    rclcpp::QoS qos_profile_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_published_left_ = false;
    bool is_published_right_ = false;
    bool is_published_motion_= false;
    std::size_t left_count_ = 0;
    std::size_t right_count_ = 0;
    std::size_t motion_count_ = 0;
    int skip_tag_;
    int skip_tmp_left_tag_;
    int skip_tmp_right_tag_;
    std::vector<int64_t> left_timestamps_;
    std::vector<int64_t> right_timestamps_;
    std::vector<ImuData> imu_align_;
    std::shared_ptr<ImuData> imu_accel_;
    std::shared_ptr<ImuData> imu_gyro_;
    bool is_started_;

    pthread_mutex_t mutex_data_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MynteyeRawData>());
    rclcpp::shutdown();
    return 0;
}
