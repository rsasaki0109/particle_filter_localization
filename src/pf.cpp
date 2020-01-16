#include <particle_filter_localization/pf.h>

namespace particle_filter_localization
{

    ParticleFilter::ParticleFilter(){
        ind_MAP_ = 0;
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
            particles_[i].cov = Eigen::Matrix<double,9,9>::Zero();
            particles_[i].weight = 1/double(num_particles);
        }
    }

    //TODO:like a Fast SLAM(estimaterのみでKFしてGaussian PFのように粒子をまき直したほうが効率良さそう？)
    /* state
     * x  = [p v q] = [x y z vx vy vz qx qy qz qw]
     * dx = [dp dv dth] = [dx dy dz dvx dvy dvz dthx dthy dthz]
     * 
     * pos_k = pos_{k-1} + vel_k * dt + (1/2) * (Rot(q_{k-1}) acc_{k-1}^{imu} - g) *dt^2
     * vel_k = vel_{k-1} + (Rot(quat_{k-1})) acc_{k-1}^{imu} - g) *dt
     * quat_k = Rot(w_{k-1}^{imu}*dt)*quat_{k-1}
     * 
     * covariance
     * P_{k} = F_k P_{k-1} F_k^T + L Q_k L^T
     */
    void ParticleFilter::predict(const Eigen::Vector3d imu_w, const Eigen::Vector3d imu_acc_gravity_corrected, const double dt_imu)
    {
        int num_particles = particles_.size();
        double sigma_imu_w_ = 0.2;
        double sigma_imu_acc_ = 0.2;
        std::normal_distribution<double> addnoize_w(0, sigma_imu_w_);
        std::normal_distribution<double> addnoize_acc(0, sigma_imu_acc_);
        
        for (int i = 0; i< num_particles; i++){ 
            Eigen::Vector3d w_noize(addnoize_w(random_seed_), addnoize_w(random_seed_), addnoize_w(random_seed_));
            Eigen::Vector3d acc_noize(addnoize_acc(random_seed_), addnoize_acc(random_seed_), addnoize_acc(random_seed_));
            Eigen::Vector3d imu_w_particle = imu_w + w_noize;
            Eigen::Vector3d imu_acc_particle = imu_acc_gravity_corrected + acc_noize;
            Eigen::Quaterniond quat_wdt =  Eigen::Quaterniond(Eigen::AngleAxisd(imu_w_particle.x() * dt_imu, Eigen::Vector3d::UnitX()) 
                                            * Eigen::AngleAxisd(imu_w_particle.y() * dt_imu, Eigen::Vector3d::UnitY())    
                                            * Eigen::AngleAxisd(imu_w_particle.z() * dt_imu, Eigen::Vector3d::UnitZ()));  
            Eigen::Quaterniond previous_quat = Eigen::Quaterniond(particles_[i].quat.w(), particles_[i].quat.x(), particles_[i].quat.y(), particles_[i].quat.z());
            Eigen::Matrix3d rot_mat = previous_quat.toRotationMatrix();
            // pos
            particles_[i].pos = particles_[i].pos + dt_imu * particles_[i].vec 
                                            + 0.5 * dt_imu * dt_imu * rot_mat * imu_acc_particle; 
            // vel
            particles_[i].vec = particles_[i].vec + dt_imu * rot_mat * imu_acc_particle;
            // quat 
            Eigen::Quaterniond predicted_quat = quat_wdt * previous_quat;
            particles_[i].quat = Eigen::Vector4d(predicted_quat.x(), predicted_quat.y(), predicted_quat.z(), predicted_quat.w());

            // cov
            // F
            Eigen::Matrix<double,9,9> F = Eigen::MatrixXd::Identity(9,9);
            F.block<3,3>(0,3) = dt_imu *  Eigen::MatrixXd::Identity(3,3);
            Eigen::Vector3d acc = imu_acc_particle;
            Eigen::Matrix3d acc_skew ;
            acc_skew << 0      ,-acc(2), acc(1),
                        acc(2) ,0      ,-acc(0),
                        -acc(1),acc(0) ,0;
            F.block<3,3>(3,6) = rot_mat *(-acc_skew) * dt_imu;
            // Q
            Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6,6);
            Q.block<3,3>(0, 0) = sigma_imu_acc_ * Q.block<3,3>(0, 0);
            Q.block<3,3>(3, 3) = sigma_imu_w_ * Q.block<3,3>(3, 3);
            Q = Q * (dt_imu * dt_imu);
            // L
            Eigen::MatrixXd L = Eigen::MatrixXd::Zero(9,6);
            L.block<3,3>(3, 0) = Eigen::MatrixXd::Identity(3,3);
            L.block<3,3>(6, 3) = Eigen::MatrixXd::Identity(3,3);
            particles_[i].cov = F * particles_[i].cov * F.transpose() + L * Q * L.transpose();

        }
    }

    void ParticleFilter::update(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
    {
        //TODO:Likelihood function!!!!!!!
        double sum_weight = 0;
        double max_weight = 0;
        int num_particles = particles_.size();
        for (int i = 0; i< num_particles; i++){
            //std::cout << "a" << std::endl;
            //std::cout << particles_[i].weight << std::endl;
            Eigen::Matrix4f transform = (Eigen::Isometry3d(Eigen::Translation3d(particles_[i].pos.x(), particles_[i].pos.y(), particles_[i].pos.z())
			                      * Eigen::Quaterniond(particles_[i].quat.w(), particles_[i].quat.x(), particles_[i].quat.y(), particles_[i].quat.z()))).matrix().cast<float>();
            //std::cout << transform << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, transform);
            int num_clouds = cloud_ptr->points.size();
            double log_p = 0;
            for (int j = 0; j < num_clouds; j++){
                //double sigma_lidar = 0.2;
                std::vector<int> pointIndices;
                std::vector<float> pointDistances;
                int num_searched = kdtree_.radiusSearch(transformed_cloud_ptr->points[j], kdtree_radius_, pointIndices, pointDistances, 1);
                if(num_searched ==0){
                    log_p -= sqrt(kdtree_radius_)/10;///10;
                    //log_p -= 1.0 * (log(sqrt(M_PI))) - log(sigma_lidar) - ((kdtree_radius_ * kdtree_radius_)/(2* sigma_lidar * sigma_lidar));
                    continue;//dist_max?
                }
                double dist = double(pointDistances[0]);
                //std::cout << dist << std::endl;
                //double sigma_lidar = 0.02;
                //double p = 1.0 / sqrt(2.0 * M_PI * sigma_lidar * sigma_lidar) * exp(-dist * dist / (2 * sigma_lidar * sigma_lidar));
                //double weight_one_point = 0.7;
                //std::cout << p * weight_one_point << std::endl;
                //particles_[i].weight = particles_[i].weight * p * weight_one_point;//addition better than multiplication?
                log_p -= sqrt(dist)/10;
                //log_p -= 1.0 * (log(sqrt(M_PI))) - log(sigma_lidar) - ((dist * dist)/(2* sigma_lidar * sigma_lidar));
            }
            particles_[i].weight = exp(log_p);
            //std::cout << log_p << std::endl;
            //std::cout << particles_[i].weight << std::endl;//aaaaaaaaa
            sum_weight += particles_[i].weight;
            if(max_weight < particles_[i].weight)
            {
                max_weight = particles_[i].weight;
                ind_MAP_ = i;
            }
        }

        double ess_inverse = 0;
        for (int i = 0; i< num_particles; i++){
            particles_[i].weight = particles_[i].weight/sum_weight;
            ess_inverse += particles_[i].weight * particles_[i].weight; 
        }

        int n_eff = int(1/ess_inverse); // Effective particle number
        int n_thres = int(num_particles/2); 
        //std::cout << "n_eff" << std::endl;
        //std::cout << n_eff << std::endl;
        if(n_eff < n_thres)
        {
            //std::cout << "resample" << std::endl;
            resample();
        }
    }

    //reference:cpprobotics particlefilter(MIT LICENSE)
    //https://github.com/onlytailei/CppRobotics/blob/master/src/particle_filter.cpp#L120
    //TODO: change KLD-sampling?
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
    Particle ParticleFilter::getWeightedAverage()
    {
        Eigen::Vector3d pos_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d vec_sum = Eigen::Vector3d::Zero();
        Eigen::Matrix4d mat_sum = Eigen::Matrix4d::Zero();
        for (auto p:particles_){
            // pos and vec
            pos_sum += p.pos;
            vec_sum += p.vec;
            // quat
            Eigen::Vector4d quat_vec = Eigen::Vector4d(p.quat.w(), p.quat.x(), p.quat.y(), p.quat.z());
            mat_sum += quat_vec * quat_vec.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(mat_sum);
        Eigen::Vector4d eig_vec_max = solver.eigenvectors().col(solver.eigenvectors().row(0).size() - 1);
        Eigen::Quaterniond quat_ave(eig_vec_max(0), eig_vec_max(1), eig_vec_max(2), eig_vec_max(3));

        Particle average_particle;
        average_particle.pos = pos_sum/particles_.size();
        average_particle.vec = vec_sum/particles_.size();
        average_particle.quat = quat_ave;
        return average_particle;
    }

    std::vector<Particle> ParticleFilter::getParticles()
    {
        return particles_;
    }

} // namespace particle_filter_localization
