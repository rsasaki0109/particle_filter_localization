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
    double weight;

    Particle() : pos(0, 0, 0),quat(1, 0, 0, 0), weight(0)
    {

    }
};

class ParticleFilter
{
public:
    explicit ParticleFilter();
    virtual ~ParticleFilter();

    void init(const int num_particles, const double initial_sigma_noize, const Particle initial_particle);
    void predict(const Eigen::Vector3d imu_w, const Eigen::Vector3d imu_acc, const double dt_imu);
    void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
    void resample();
    void reset();
    void setMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr);

private:
    std::vector<Particle> particles_;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;

    std::mt19937 random_seed_;//Mersenne twister

    double kdtree_radius_;

};

} // namespace particle_filter_localization