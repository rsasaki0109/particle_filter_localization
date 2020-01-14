#include <particle_filter_localization/pf.h>

namespace particle_filter_localization
{

    ParticleFilter::ParticleFilter(){
        ind_MAP_ = 0;
        max_weight_ = 0;
    }
    ParticleFilter::~ParticleFilter(){}

    void ParticleFilter::init(const int num_particles, const double initial_sigma_noize, const Particle initial_particle){

        kdtree_radius_ = 0.2;

        particles_.resize(num_particles);
        
        double initial_mean_noize = 0;
        std::normal_distribution<double> addnoize(initial_mean_noize, initial_sigma_noize);
        particles_[0].pos.x() = initial_particle.pos.x();
        particles_[0].pos.y() = initial_particle.pos.y();
        particles_[0].pos.z() = initial_particle.pos.z();
        particles_[0].quat = initial_particle.quat;
        particles_[0].weight = 1/double(num_particles);
        for(int i = 1; i < num_particles; i++)
        {
            particles_[i].pos.x() = initial_particle.pos.x() + addnoize(random_seed_);
            particles_[i].pos.y() = initial_particle.pos.y() + addnoize(random_seed_);
            particles_[i].pos.z() = initial_particle.pos.z() + addnoize(random_seed_);
            particles_[i].quat.x() = initial_particle.quat.x();
            particles_[i].quat.y() = initial_particle.quat.y();
            particles_[i].quat.z() = initial_particle.quat.z();
            particles_[i].quat.w() = initial_particle.quat.w();
            particles_[i].weight = 1/double(num_particles);
        }
    }

    void ParticleFilter::predict(const Eigen::Vector3d imu_w, const Eigen::Vector3d imu_acc, const double dt_imu)
    {
        int num_particles = particles_.size();

        Eigen::Vector3d gravity_;
        gravity_ << 0,0,-9.80665;//TODO?

        Eigen::Quaterniond quat_wdt =  Eigen::Quaterniond(Eigen::AngleAxisd(imu_w.x() * dt_imu, Eigen::Vector3d::UnitX()) 
                                            * Eigen::AngleAxisd(imu_w.y() * dt_imu, Eigen::Vector3d::UnitY())    
                                            * Eigen::AngleAxisd(imu_w.z() * dt_imu, Eigen::Vector3d::UnitZ()));  
        for (int i = 0; i< num_particles; i++){ 
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
        std::cout << "start search" << std::endl;
        
        double sum_weight = 0;
        int num_particles = particles_.size();
        for (int i = 0; i< num_particles; i++){
            Eigen::Matrix4f transform = (Eigen::Isometry3d(Eigen::Translation3d(particles_[i].pos.x(), particles_[i].pos.y(), particles_[i].pos.z())
			                      * Eigen::Quaterniond(particles_[i].quat.w(), particles_[i].quat.x(), particles_[i].quat.y(), particles_[i].quat.z()))).matrix().cast<float>();
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, transform);
            int num_clouds = cloud_ptr->points.size();
            for (int j = 0; j < num_clouds; j++){
                std::vector<int> pointIndices;
                std::vector<float> pointDistances;
                if(kdtree_.radiusSearch(cloud_ptr->points[j], kdtree_radius_, pointIndices, pointDistances, 1) ==0){
                    continue;//dist_max?
                }
                double dist = double(pointDistances[0]);
                double sigma_lidar = 0.02;
                double p = 1.0 / sqrt(2.0 * M_PI * sigma_lidar * sigma_lidar) * exp(-dist * dist / (2 * sigma_lidar * sigma_lidar));
                particles_[i].weight = particles_[i].weight * p;//addition better than multiplication?
            }
            sum_weight += particles_[i].weight;
            if(max_weight_ < particles_[i].weight)
            {
                max_weight_ = particles_[i].weight;
                ind_MAP_ = i;
            }
        }
        std::cout << "end search" << std::endl;

        double ess_inverse = 0;
        for (int i = 0; i< num_particles; i++){
            particles_[i].weight = particles_[i].weight/sum_weight;
            ess_inverse += particles_[i].weight * particles_[i].weight; 
        }

        int n_eff = int(1/ess_inverse); // Effective particle number
        int n_thres = int(num_particles/2); 
        if(n_eff < n_thres)
        {
            resample();
        }
    }

    //reference:cpprobotics particlefilter(MIT LICENSE)
    //https://github.com/onlytailei/CppRobotics/blob/master/src/particle_filter.cpp#L120
    //TODO: change KLD-sampling
    void ParticleFilter::resample()
    {
        int num_particles = particles_.size();

        Eigen::Matrix<double, Eigen::Dynamic, 1> wcum; 
        wcum = Eigen::VectorXd::Zero(num_particles);
        Eigen::Matrix<double, Eigen::Dynamic, 1> base;
        base = Eigen::VectorXd::Zero(num_particles);
        wcum(0) = particles_[0].weight;
        base(0) = 0;
        for(int i=1; i<num_particles; i++){
            wcum(i) = wcum(i-1) + particles_[i].weight;
            base(i) = base(i-1) + 1.0/num_particles;
        }

        Eigen::Matrix<double, Eigen::Dynamic, 1> resampleid;
        resampleid = Eigen::VectorXd::Zero(num_particles);

        std::vector<Particle> new_particles;
        new_particles.resize(num_particles);

        std::uniform_real_distribution<> uni_d;
        for(int j=0; j<num_particles; j++){
        resampleid(j) = base(j) + uni_d(random_seed_)/num_particles;
        }

        int ind = 0;

        for(int i=0; i<num_particles; i++){
            while(resampleid(i) > wcum(ind) && ind<num_particles-1){
             ind += 1;
            }
            new_particles[i] = particles_[ind];
            new_particles[i].weight = 1.0/num_particles;
        }
        particles_ = new_particles;
    }

    void ParticleFilter::setMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr)
    {
        kdtree_.setInputCloud(map_ptr);
    }

    Particle ParticleFilter::getMAPestimate()
    {
        return particles_[ind_MAP_];
    }

} // namespace particle_filter_localization
