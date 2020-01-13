#include <particle_filter_localization/pf.h>

namespace particle_filter_localization
{

    ParticleFilter::ParticleFilter(){}
    ParticleFilter::~ParticleFilter(){}

    void ParticleFilter::init(const int num_particles, const double initial_sigma_noize, const Particle initial_particle){

        particles_.resize(num_particles);
        
        double initial_mean_noize = 0;
        std::normal_distribution<double> addnoize(initial_mean_noize, initial_sigma_noize);
        for(int i = 0; i < num_particles; i++)
        {
            particles_[i].pos.x() = initial_particle.pos.x() + addnoize(random_seed_);
            particles_[i].pos.y() = initial_particle.pos.y() + addnoize(random_seed_);
            particles_[i].pos.z() = initial_particle.pos.z() + addnoize(random_seed_);
            particles_[i].weight = 1/double(num_particles);
        }
    }

    void ParticleFilter::predict(const Eigen::Vector3d imu_w, const Eigen::Vector3d imu_acc, const double dt_imu)
    {
        Eigen::Vector3d gravity_;
        gravity_ << 0,0,-9.80665;

        Eigen::Quaterniond quat_wdt =  Eigen::Quaterniond(Eigen::AngleAxisd(imu_w.x() * dt_imu, Eigen::Vector3d::UnitX()) 
                                            * Eigen::AngleAxisd(imu_w.y() * dt_imu, Eigen::Vector3d::UnitY())    
                                            * Eigen::AngleAxisd(imu_w.z() * dt_imu, Eigen::Vector3d::UnitZ()));  
        for (int i = 0; i< particles_.size(); i++){ 
            Eigen::Quaterniond previous_quat = Eigen::Quaterniond(particles_[i].quat.w(), particles_[i].quat.x(), particles_[i].quat.y(), particles_[i].quat.z());
            Eigen::Matrix3d rot_mat = previous_quat.toRotationMatrix();
            // pos
            particles_[i].pos = particles_[i].pos + dt_imu * particles_[i].vec 
                                            + 0.5 * dt_imu * dt_imu * (rot_mat * imu_acc - gravity_); 
            // vel
            particles_[i].vec = particles_[i].vec + dt_imu * (rot_mat * imu_acc - gravity_);
            // quat 
            Eigen::Quaterniond predicted_quat = quat_wdt * previous_quat;
            particles_[i].quat = Eigen::Vector4d(predicted_quat.x(), predicted_quat.y(), predicted_quat.z(), predicted_quat.w());
        }
    }

    void ParticleFilter::update(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
    {
        double radius = 0.2;//TODO
        double sum = 0;
        for (int i = 0; i< particles_.size(); i++){
            Eigen::Matrix4f transform = (Eigen::Isometry3d(Eigen::Translation3d(particles_[i].pos.x(), particles_[i].pos.y(), particles_[i].pos.z())
			                      * Eigen::Quaterniond(particles_[i].quat.w(), particles_[i].quat.x(), particles_[i].quat.y(), particles_[i].quat.z()))).matrix().cast<float>();
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, transform);
            for (int j = 0; j < cloud_ptr->points.size (); j++){
                std::vector<int> pointIndices;
                std::vector<float> pointDistances;
                kdtree_.radiusSearch(cloud_ptr->points[j],radius,pointIndices,pointDistances,1);
                double dist = double(pointDistances[0]);
                double sigma_lidar = 0.02;
                double p = 1.0 / sqrt(2.0 * M_PI * sigma_lidar * sigma_lidar) * exp(-dist * dist / (2 * sigma_lidar * sigma_lidar));
                particles_[i].weight = particles_[i].weight * p;//addition better than multiplication?
            }
            sum += particles_[i].weight;
        }

        double ess_inverse = 0;
        for (int i = 0; i< particles_.size(); i++){
            particles_[i].weight = particles_[i].weight/sum;
            ess_inverse += particles_[i].weight * particles_[i].weight; 
        }

        int n_eff = int(1/ess_inverse); // Effective particle number
        int n_thres = int(particles_.size()/2); 
        if(n_eff < n_thres)
        {
            resample();
        }
    }

    void ParticleFilter::resample()
    {

    }

    void ParticleFilter::setMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr)
    {
        kdtree_.setInputCloud(map_ptr);
    }
    
} // namespace particle_filter_localization
