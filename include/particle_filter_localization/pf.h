#include <vector>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

//#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

namespace particle_filter_localization
{
    


struct Particle
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vec;
    Eigen::Quaterniond quat;
    Eigen::Matrix<double,9,9> cov;
    double weight;

    /*Particle() : pos(0, 0, 0),quat(1, 0, 0, 0),cov(1,0,0,0,0,0,0,0,0,
                                                   0,1,0,0,0,0,0,0,0,
                                                   0,0,1,0,0,0,0,0,0,
                                                   0,0,0,1,0,0,0,0,0,
                                                   0,0,0,0,1,0,0,0,0,
                                                   0,0,0,0,0,1,0,0,0,
                                                   0,0,0,0,0,0,1,0,0,
                                                   0,0,0,0,0,0,0,1,0,
                                                   0,0,0,0,0,0,0,0,1)*/
    Particle() : pos(0, 0, 0),quat(1, 0, 0, 0)
    {

    }
};

class ParticleFilter
{
public:
    explicit ParticleFilter();
    virtual ~ParticleFilter();

    void init(const int num_particles, const double initial_sigma_noize,const double sigma_imu_w, 
              const double sigma_imu_acc, const Particle initial_particle);
    void predict(const Eigen::Vector3d imu_w, const Eigen::Vector3d imu_acc_gravity_corrected, const double dt_imu);
    void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
    void update(const Eigen::Vector3d gnss_pose);
    void resample();
    void reset();
    void setMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr);
    Particle getMAPestimate();
    Particle getWeightedAverage();
    std::vector<Particle> getParticles();

    void calcCov();
    Eigen::Vector3d Quat2RotVec(Eigen::Quaterniond quat);

private:
    std::vector<Particle> particles_;

    int ind_MAP_;
    
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;

    std::mt19937 random_seed_;//Mersenne twister

    double kdtree_radius_;
    double sigma_imu_w_;
    double sigma_imu_acc_;

};

} // namespace particle_filter_localization