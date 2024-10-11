
#include "lifecycle_single_laser.hpp"

// 构造函数
SingleLaserLifecycle::SingleLaserLifecycle(const std::string& name) : nav2_util::LifecycleNode(name) {
    lidar_comm_ = std::make_shared<ldlidar_ssl::RTRNet>();
    cmd_port_ = std::make_shared<ldlidar_ssl::CmdInterfaceLinux>();
}

// 析构函数
SingleLaserLifecycle::~SingleLaserLifecycle() {
    if (data_thread_.joinable()) {
        stop_thread_ = true;
        data_thread_.join();
    }
}

// 配置阶段
nav2_util::CallbackReturn SingleLaserLifecycle::on_configure(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Configuring...");

    // 声明参数
    this->declare_parameter<std::string>("product_type", setting_.product_type);
    this->declare_parameter<std::string>("port_name", setting_.port_name);
    this->declare_parameter<std::string>("frame_id", setting_.frame_id);
    this->declare_parameter<std::string>("topic_name", setting_.topic_name);

    // 获取参数
    this->get_parameter("product_type", setting_.product_type);
    this->get_parameter("port_name", setting_.port_name);
    this->get_parameter("frame_id", setting_.frame_id);
    this->get_parameter("topic_name", setting_.topic_name);

    RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] SDK Pack Version is " << lidar_comm_->GetSdkPackVersionNumber());

    ldlidar_ssl::LDType type;
    if ("SSL20L" == setting_.product_type) {
        type = ldlidar_ssl::LDType::LD_SSL20_L;
        sleep_rate_ = 25;
    } else if ("SSL20N" == setting_.product_type) {
        type = ldlidar_ssl::LDType::LD_SSL20_N;
        sleep_rate_ = 30;
    } else if ("SSL20P" == setting_.product_type) {
        type = ldlidar_ssl::LDType::LD_SSL20_P;
        sleep_rate_ = 60;
    } else if ("LD07N" == setting_.product_type) {
        type = ldlidar_ssl::LDType::LD_07N;
        sleep_rate_ = 28;
    } else {
        type = ldlidar_ssl::LDType::LD_NO_TYPE;
    }

    if (!lidar_comm_->SetProductType(type)) {
        RCLCPP_ERROR(this->get_logger(), "[ldrobot] set product type is fail");
        return nav2_util::CallbackReturn::FAILURE;
    }

    if (!cmd_port_->Open(setting_.port_name, ldlidar_ssl::STDBaudRateType::BAUD_921600)) {
        RCLCPP_ERROR(this->get_logger(), "[ldrobot] open %s device is fail", setting_.port_name.c_str());
        return nav2_util::CallbackReturn::FAILURE;
    } else {
        RCLCPP_INFO(this->get_logger(), "[ldrobot] open %s device is success", setting_.port_name.c_str());
    }

    cmd_port_->SetReadCallback(std::bind(&ldlidar_ssl::RTRNet::CommReadCallback, lidar_comm_.get(),
                                         std::placeholders::_1, std::placeholders::_2));

    int error = 0;
    while (!lidar_comm_->IsParametersReady()) {
        if (!lidar_comm_->SendCmd(cmd_port_.get(), 0, ldlidar_ssl::PACK_CONFIG_ADDRESS)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_CONFIG_ADDRESS cmd fail");
            return nav2_util::CallbackReturn::FAILURE;
        }
        sleep(1);
        if (!lidar_comm_->SendCmd(cmd_port_.get(), THIS_DEVICE_ADDREESS, ldlidar_ssl::PACK_GET_COE)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_GET_COE cmd fail");
            return nav2_util::CallbackReturn::FAILURE;
        }
        sleep(1);
        if (!lidar_comm_->SendCmd(cmd_port_.get(), THIS_DEVICE_ADDREESS, ldlidar_ssl::PACK_VIDEO_SIZE)) {
            RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_VIDEO_SIZE cmd fail");
            return nav2_util::CallbackReturn::FAILURE;
        }
        error++;
        if (error > 2) { /* Exit if the number of errors is more than 2*/
            RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] Error: get ssl20 lidar parameters fail");
            return nav2_util::CallbackReturn::FAILURE;
        }
    }
    lidar_comm_->ResetParametersReady();

    //插值距离模式输出
    uint8_t send_parbuf = 0;
    send_parbuf = 0;  // 1：开启插值模式，0：关闭插值模式
    if (!lidar_comm_->sendbytes(cmd_port_.get(), THIS_DEVICE_ADDREESS, ldlidar_ssl::PACK_COE_INTERPOLATION,
                                &send_parbuf, 1, 0)) {
        std::cout << "[ldrobot] send to PACK_COE_INTERPOLATION cmd fail" << std::endl;
        return nav2_util::CallbackReturn::FAILURE;
    } else {
        std::cout << "[ldrobot] send to PACK_COE_INTERPOLATION cmd success" << std::endl;
    }
    lidar_comm_->SetDistanceModule(send_parbuf);
    sleep(1);

    RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] get ssl20 lidar parameters success");

    if (!lidar_comm_->SendCmd(cmd_port_.get(), THIS_DEVICE_ADDREESS, ldlidar_ssl::PACK_GET_DISTANCE)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_GET_DISTANCE cmd fail");
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_GET_DISTANCE cmd success");
    }

    lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(setting_.topic_name, rclcpp::SensorDataQoS());

    return nav2_util::CallbackReturn::SUCCESS;
}

// 激活阶段
nav2_util::CallbackReturn SingleLaserLifecycle::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Activating...");
    lidar_pub_->on_activate();
    stop_thread_ = false;
    data_thread_ = std::thread(&SingleLaserLifecycle::data_publish_loop, this);
    return nav2_util::CallbackReturn::SUCCESS;
}

// 失活阶段
nav2_util::CallbackReturn SingleLaserLifecycle::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Deactivating...");
    lidar_pub_->on_deactivate();
    stop_thread_ = true;
    if (data_thread_.joinable()) {
        data_thread_.join();
    }
    return nav2_util::CallbackReturn::SUCCESS;
}

// 清理阶段
nav2_util::CallbackReturn SingleLaserLifecycle::on_cleanup(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Cleaning up...");
    cmd_port_->Close();
    return nav2_util::CallbackReturn::SUCCESS;
}

// 关闭阶段
nav2_util::CallbackReturn SingleLaserLifecycle::on_shutdown(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    if (!lidar_comm_->SendCmd(cmd_port_.get(), THIS_DEVICE_ADDREESS, ldlidar_ssl::PACK_STOP)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_STOP_DISTANCE_TRANS cmd fail");
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_STOP_DISTANCE_TRANS cmd success");
    }
    cmd_port_->Close();
    return nav2_util::CallbackReturn::SUCCESS;
}

// 数据发布循环
void SingleLaserLifecycle::data_publish_loop() {
    rclcpp::WallRate r(sleep_rate_);
    auto last_time = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !stop_thread_) {
        if (lidar_comm_->IsFrameReady()) {
            lidar_comm_->ResetFrameReady();
            last_time = std::chrono::steady_clock::now();
            ldlidar_ssl::Points2D laserscan = lidar_comm_->GetLaserScanData();
            ToLaserscanMessagePublish(laserscan);
        }

        // if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
        // last_time).count() > 1000) {
        //   // 数据发布超时或者串口设备拔出处理
        //   RCLCPP_ERROR(this->get_logger(), "[ldrobot] lidar pub data is time out, please check lidar device");
        //   stop_thread_ = true;
        // }

        r.sleep();
    }

    if (!lidar_comm_->SendCmd(cmd_port_.get(), THIS_DEVICE_ADDREESS, ldlidar_ssl::PACK_STOP)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_STOP_DISTANCE_TRANS cmd fail");
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "[ldrobot] send to PACK_STOP_DISTANCE_TRANS cmd success");
    }
}

// 将数据转换并发布
void SingleLaserLifecycle::ToLaserscanMessagePublish(ldlidar_ssl::Points2D& src) {
    float angle_min, angle_max, range_min, range_max, angle_increment;

    rclcpp::Time start_scan_time;
    static rclcpp::Time end_scan_time;
    static bool first_pub = false;
    float scan_time = 0;

    start_scan_time = this->now();
    scan_time = start_scan_time.seconds() - end_scan_time.seconds();

    if (first_pub == false) {
        scan_time = (float)(1 / 30.f);  // 1/30Hz 按理论设计估计
        first_pub = true;
    }

    /*Adjust the parameters according to the demand*/
    angle_min = ANGLE_TO_RADIAN(0);
    angle_max = ANGLE_TO_RADIAN(360);
    range_min = 0.015;
    range_max = 1.5;
    /*Angle resolution, the smaller the resolution, the smaller the error after conversion*/
    double fov = lidar_comm_->GetFovAngleVal();
    double total_point_number = lidar_comm_->GetTotalPointNumberVal();
    angle_increment = ANGLE_TO_RADIAN(fov / total_point_number);
    /*Calculate the number of scanning points*/
    unsigned int beam_size = ceil((angle_max - angle_min) / angle_increment);
    sensor_msgs::msg::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting_.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;

    output.time_increment = scan_time / (beam_size - 1);
    output.scan_time = scan_time;

    // RCLCPP_INFO(this->get_logger(), "angle_min : %f ,angle_max : %f ,angle_increment : %f , beam_size : %d",
    // angle_min,
    //             angle_max, angle_increment, beam_size);

    /*First fill all the data with NaN*/
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : src) {
        float range = point.distance / 1000.f;  // distance unit (mm) transform to (m)
        float dir_angle = point.angle;
        // RCLCPP_INFO(this->get_logger(), "point.angle : %f",point.angle);
        dir_angle = 360.f - dir_angle;         // clockwise transform to counterclockwise
        float angle = dir_angle / 180 * M_PI;  // degree transform to radian
        int index = (int)((angle - output.angle_min) / output.angle_increment);
        if (index >= 0 && index < (int)beam_size) {
            /*If the current content is NaN, it is assigned directly*/
            if (std::isnan(output.ranges[index])) {
                output.ranges[index] = range;
            } else { /*Otherwise, only when the distance is less than the current value, it can be re-assigned*/
                if (range < output.ranges[index]) {
                    output.ranges[index] = range;
                }
            }
            output.intensities[index] = point.intensity;
        }
    }
    lidar_pub_->publish(output);
    end_scan_time = start_scan_time;
    // RCLCPP_INFO(this->get_logger(), "[ldrobot] pub lidar data");
    // int index_zero = (int)((ANGLE_TO_RADIAN(302.756195) - output.angle_min) / output.angle_increment);
    // RCLCPP_INFO(this->get_logger(), "distance of zero angle : %f",output.ranges[index_zero]);
}
