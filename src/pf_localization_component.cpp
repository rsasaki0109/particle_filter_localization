#include <particle_filter_localization/pf_localization_component.h>
#include <chrono>
using namespace std::chrono_literals;

namespace particle_filter_localization
{
    PfLocalizationComponent::PfLocalizationComponent(const rclcpp::NodeOptions & options)
    : Node("pf_localization", options),
        clock_(RCL_ROS_TIME), 
        tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
        listener_(tfbuffer_)
    {
        declare_parameter("reference_frame_id","map");
        get_parameter("reference_frame_id",reference_frame_id_);
        declare_parameter("robot_frame_id","base_link");
        get_parameter("robot_frame_id",robot_frame_id_);
        declare_parameter("initial_pose_topic",get_name() + std::string("/initial_pose"));
        get_parameter("initial_pose_topic",initial_pose_topic_);
        declare_parameter("imu_topic",get_name() + std::string("/imu"));
        get_parameter("imu_topic",imu_topic_);
        declare_parameter("odom_topic",get_name() + std::string("/odom"));
        get_parameter("odom_topic",odom_topic_);
        declare_parameter("gnss_pose_topic",get_name() + std::string("/gnss_pose"));
        get_parameter("gnss_pose_topic",gnss_pose_topic_);
        declare_parameter("map_topic",get_name() + std::string("/map"));
        get_parameter("map_topic",map_topic_);
        declare_parameter("cloud_topic",get_name() + std::string("/cloud"));
        get_parameter("cloud_topic",cloud_topic_);

        declare_parameter("pub_period",10);
        get_parameter("pub_period",pub_period_);
        declare_parameter("num_state",10);
        get_parameter("num_state",num_state_);
        declare_parameter("num_error_state",9);
        get_parameter("num_error_state",num_error_state_);
        declare_parameter("num_particles",100);
        get_parameter("num_particles",num_particles_);
        declare_parameter("selected_estimator","WeightedAverage");
        get_parameter("selected_estimator",selected_estimator_);
        declare_parameter("voxel_leaf_size",0.2);
        get_parameter("voxel_leaf_size",voxel_leaf_size_);
        declare_parameter("var_initial_pose",0.2);
        get_parameter("var_initial_pose",var_initial_pose_);
        declare_parameter("sigma_imu_w",0.01);
        get_parameter("sigma_imu_w",sigma_imu_w_);
        declare_parameter("sigma_imu_acc",0.01);
        get_parameter("sigma_imu_acc",sigma_imu_acc_);
        declare_parameter("sigma_gnss_xy",0.1);
        get_parameter("sigma_gnss_xy",sigma_gnss_xy_);
        declare_parameter("sigma_gnss_z",0.15);
        get_parameter("sigma_gnss_z",sigma_gnss_z_);
        declare_parameter("sigma_odom_xyz",0.2);
        get_parameter("sigma_odom_xyz",sigma_odom_xyz_);
        declare_parameter("sigma_lidar",0.02);
        get_parameter("sigma_lidar",sigma_lidar_);
        declare_parameter("use_gnss",true);
        get_parameter("use_gnss",use_gnss_);
        declare_parameter("use_odom",false);
        get_parameter("use_odom",use_odom_);

        declare_parameter("debug",false);
        get_parameter("debug",debug_);

        set_on_parameters_set_callback(
        [this](const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult 
        {
            auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
            for(auto param : params)
            {
                if(param.get_name() == "num_state")
                {
                    if(num_state_ >0)
                    {
                        num_state_ = param.as_int();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "number of states must over 0";
                    }
                }
                if(param.get_name() == "num_error_state")
                {
                    if(num_error_state_ >0)
                    {
                        num_error_state_ = param.as_int();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "number of error states must over 0";
                    }
                }
                if(param.get_name() == "num_obs")
                {
                    if(num_obs_ >0)
                    {
                        num_obs_ = param.as_int();
                        results->successful = true;
                        results->reason = "";
                    }
                    else
                    {
                        results->successful = false;
                        results->reason = "number of observation must over 0";
                    }
                }
            }
            if(!results->successful)
            {
                results->successful = false;
                results->reason = "";
            }
            return *results;
        }
        );
        
        // Init 
        previous_time_imu_ = -1;
        x_ = Eigen::VectorXd::Zero(num_state_);
        x_(STATE::QW) = 1;
        P_ = Eigen::MatrixXd::Identity(num_error_state_,num_error_state_) * 100;//todo:set initial value properly
        gravity_ << 0,0,-9.80665;
        vg_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);  

        sigma_gnss_[0] = sigma_gnss_xy_;
        sigma_gnss_[1] = sigma_gnss_xy_;
        sigma_gnss_[2] = sigma_gnss_z_;
        sigma_odom_[0] = sigma_odom_xyz_;
        sigma_odom_[1] = sigma_odom_xyz_;
        sigma_odom_[2] = sigma_odom_xyz_;

        initial_pose_recieved_ = false;
        map_recieved_ = false;
        
        // Setup Publisher 
        std::string output_pose_name = get_name() + std::string("/current_pose");
        current_pose_pub_ = 
            create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_name,10);
        std::string output_particles_name = get_name() + std::string("/particles");
        particles_pub_ = 
            create_publisher<geometry_msgs::msg::PoseArray>(output_particles_name,10);

        // Setup Subscriber 
        auto initial_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            std::cout << "initial pose callback" << std::endl;
            initial_pose_recieved_ = true;
            current_pose_ = *msg;
            Particle initial_particle;
            initial_particle.pos.x() = current_pose_.pose.position.x;
            initial_particle.pos.y() = current_pose_.pose.position.y;
            initial_particle.pos.z() = current_pose_.pose.position.z;
            initial_particle.quat.x() = current_pose_.pose.orientation.x;
            initial_particle.quat.y() = current_pose_.pose.orientation.y;
            initial_particle.quat.z() = current_pose_.pose.orientation.z;
            initial_particle.quat.w() = current_pose_.pose.orientation.w;
            pf_.init(num_particles_, var_initial_pose_,sigma_imu_w_, sigma_imu_acc_, initial_particle);
        };

        auto imu_callback =
        [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
        {
            current_stamp_ = msg->header.stamp;

            // dt_imu
            double current_time_imu = msg->header.stamp.sec 
                                      + msg->header.stamp.nanosec * 1e-9;
            if(previous_time_imu_ == -1){
                previous_time_imu_ = current_time_imu;
                return;
            }
            double dt_imu = current_time_imu - previous_time_imu_;
            previous_time_imu_ = current_time_imu;

            if(initial_pose_recieved_){
                sensor_msgs::msg::Imu transformed_msg;
                try
                {
                    geometry_msgs::msg::Vector3Stamped acc_in, acc_out, w_in, w_out;
                    acc_in.vector.x = msg->linear_acceleration.x;
                    acc_in.vector.y = msg->linear_acceleration.y;
                    acc_in.vector.z = msg->linear_acceleration.z;
                    w_in.vector.x = msg->angular_velocity.x;
                    w_in.vector.y = msg->angular_velocity.y;
                    w_in.vector.z = msg->angular_velocity.z;
                    tf2::TimePoint time_point = tf2::TimePoint(
                        std::chrono::seconds(msg->header.stamp.sec) +
                        std::chrono::nanoseconds(msg->header.stamp.nanosec));
                    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
                        robot_frame_id_, msg->header.frame_id, time_point);
                    tf2::doTransform(acc_in, acc_out, transform);
                    tf2::doTransform(w_in, w_out, transform);
                    transformed_msg.header.stamp = msg->header.stamp;
                    transformed_msg.angular_velocity.x = w_out.vector.x;
                    transformed_msg.angular_velocity.y = w_out.vector.y;
                    transformed_msg.angular_velocity.z = w_out.vector.z;
                    transformed_msg.linear_acceleration.x = acc_out.vector.x;
                    transformed_msg.linear_acceleration.y = acc_out.vector.y;
                    transformed_msg.linear_acceleration.z = acc_out.vector.z; 
                    Eigen::Vector3d w(w_out.vector.x, w_out.vector.y, w_out.vector.z);
                    Eigen::Vector3d acc(acc_out.vector.x, acc_out.vector.y, acc_out.vector.z);
                       
                    double alpha = 0.8;//a low-pass filter parameter
                    gravity_.x() = alpha * gravity_.x() + (1 - alpha) * acc.x();
                    gravity_.y() = alpha * gravity_.y() + (1 - alpha) * acc.y();
                    gravity_.z() = alpha * gravity_.z() + (1 - alpha) * acc.z();
                    Eigen::Vector3d acc_gravity_corrected = acc - gravity_;
                    if(map_recieved_){
                        if(!debug_){
                            pf_.predict(w, acc_gravity_corrected, dt_imu);
                        }   
                    }
                }
                catch (tf2::TransformException& e){
                    RCLCPP_ERROR(this->get_logger(),"%s",e.what());
                    return;
                }
            }       
        };

        auto odom_callback =
        [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_ && use_odom_){
                geometry_msgs::msg::PoseStamped pose;
                pose.header = msg->header;
                pose.pose.position.x = msg->pose.pose.position.x;
                pose.pose.position.y = msg->pose.pose.position.y;
                pose.pose.position.z = msg->pose.pose.position.z;
                measurementUpdate(pose, sigma_odom_); 
            }    
        };

        auto gnss_pose_callback =
        [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_ && use_gnss_){
                measurementUpdate(*msg, sigma_gnss_); 
            }    
        };

        auto map_callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            std::cout << "map callback" << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            map_recieved_ = true;
            pcl::fromROSMsg(*msg,*map_ptr);
            pf_.setMap(map_ptr);

        };

        cloud_received_ = false;

        auto cloud_callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            if(initial_pose_recieved_ && map_recieved_){
                sensor_msgs::msg::PointCloud2 transformerd_msg;
                try{
                    tf2::TimePoint time_point = tf2::TimePoint(
                        std::chrono::seconds(msg->header.stamp.sec) +
                        std::chrono::nanoseconds(msg->header.stamp.nanosec));
                    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
                        robot_frame_id_, msg->header.frame_id, time_point);
                    tf2::doTransform(*msg, transformerd_msg, transform);//TODO:slow(https://github.com/ros/geometry2/pull/432)
                }
                catch (tf2::TransformException& e){
                    RCLCPP_ERROR(this->get_logger(),"%s",e.what());
                    return;
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(transformerd_msg,*cloud_ptr);
                vg_filter_.setInputCloud(cloud_ptr);
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                vg_filter_.filter(*filtered_cloud_ptr);

                if(!cloud_received_){
                    cloud_received_ = true;
                    first_cloud_ += *filtered_cloud_ptr;
                }
                pcl::PointCloud<pcl::PointXYZI>::Ptr first_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(first_cloud_));
                
                if(debug_){
                    measurementUpdate(first_cloud_ptr);
                }
                else{
                    measurementUpdate(filtered_cloud_ptr);
                }
            }
        };


        sub_initial_pose_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(initial_pose_topic_, 1,
                initial_pose_callback);
        sub_imu_ = 
            create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 1,
                imu_callback);
        sub_odom_ = 
            create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1,
                odom_callback);
        sub_gnss_pose_ = 
            create_subscription<geometry_msgs::msg::PoseStamped>(gnss_pose_topic_, 1,
                gnss_pose_callback);
        sub_map_  = 
            create_subscription<sensor_msgs::msg::PointCloud2>(map_topic_, 1,
                map_callback);
        sub_cloud_  = 
            create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, 1,
                cloud_callback);
        std::chrono::milliseconds period(pub_period_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&PfLocalizationComponent::broadcastPose, this));
    }   

    //TODO:like a Fast SLAM(estimaterのみでKFしてGaussian PFのように粒子をまき直したほうが効率良さそう？)
    /* 
     * y = pobs = [xobs yobs zobs]
     * 
     * K = P_k H^T (H P_k H^T + R)^{-1}
     * 
     * dx = K (y_k - p_k )
     * 
     * p_x = p_{k-1} + dp_k
     * v_k = v_{k-1} + dv_k
     * q_k = Rot(dth) q_{k-1}
     * 
     * P_k = (I - KH)*P_{k-1} 
     */
    void PfLocalizationComponent::measurementUpdate(const geometry_msgs::msg::PoseStamped input_pose_msg, const double variance[])
    {
        // error state
        current_stamp_ = input_pose_msg.header.stamp;
        Eigen::Matrix3d R;
        R << variance[0], 0, 0,
             0, variance[1], 0 ,
             0, 0, variance[2];
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, num_error_state_);
        H.block<3,3>(0, 0) =  Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse(); 
        Eigen::Vector3d y = Eigen::Vector3d(input_pose_msg.pose.position.x, input_pose_msg.pose.position.y, input_pose_msg.pose.position.z);
        Eigen::VectorXd dx = K *(y - x_.segment(STATE::X, 3));

        // state
        x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dx.segment(ERROR_STATE::DX, 3);
        x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dx.segment(ERROR_STATE::DVX, 3);
        double norm_quat = sqrt(dx(ERROR_STATE::DTHX)*dx(ERROR_STATE::DTHX) + dx(ERROR_STATE::DTHY)*dx(ERROR_STATE::DTHY) + dx(ERROR_STATE::DTHZ)*dx(ERROR_STATE::DTHZ));
        if (norm_quat < 1e-10) x_.segment(STATE::QX, 4) = Eigen::Vector4d(0, 0, 0, cos(norm_quat/2));
        else x_.segment(STATE::QX, 4) = Eigen::Vector4d(sin(norm_quat/2) * dx(ERROR_STATE::DTHX)/norm_quat, sin(norm_quat/2) * dx(ERROR_STATE::DTHY)/norm_quat,
                                                 sin(norm_quat/2) * dx(ERROR_STATE::DTHZ)/norm_quat, cos(norm_quat/2));
                                                 
        P_ = (Eigen::MatrixXd::Identity(num_error_state_, num_error_state_) - K*H) * P_;

        return;
    }

    void PfLocalizationComponent::measurementUpdate(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
    {
        pf_.update(cloud_ptr);
    }

    void PfLocalizationComponent::broadcastPose()
    {
        if(initial_pose_recieved_){
            Particle selected_posatt; 
            if(selected_estimator_ == "WeightedAverage"){
                selected_posatt = pf_.getWeightedAverage();
            }
            else{
                selected_posatt = pf_.getMAPestimate();
            }
            current_pose_.header.stamp = current_stamp_;
            current_pose_.header.frame_id = reference_frame_id_;
            current_pose_.pose.position.x = selected_posatt.pos.x();
            current_pose_.pose.position.y = selected_posatt.pos.y();
            current_pose_.pose.position.z = selected_posatt.pos.z();
            current_pose_.pose.orientation.x = selected_posatt.quat.x();
            current_pose_.pose.orientation.y = selected_posatt.quat.y();
            current_pose_.pose.orientation.z = selected_posatt.quat.z();
            current_pose_.pose.orientation.w = selected_posatt.quat.w();
            current_pose_pub_->publish(current_pose_);  
            
            geometry_msgs::msg::PoseArray particles_msg;
            std::vector<Particle> particles = pf_.getParticles();
            particles_msg.header.stamp = current_stamp_;
            particles_msg.header.frame_id = reference_frame_id_;
            particles_msg.poses.resize(num_particles_);
            for (int i = 0; i < num_particles_; i++) {
                particles_msg.poses[i].position.x = particles[i].pos.x();
                particles_msg.poses[i].position.y = particles[i].pos.y();
                particles_msg.poses[i].position.z = particles[i].pos.z();
                particles_msg.poses[i].orientation.x = particles[i].quat.x();
                particles_msg.poses[i].orientation.y = particles[i].quat.y();
                particles_msg.poses[i].orientation.z = particles[i].quat.z();
                particles_msg.poses[i].orientation.w = particles[i].quat.w();
            }
            particles_pub_->publish(particles_msg);

        }
        return;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(particle_filter_localization::PfLocalizationComponent)