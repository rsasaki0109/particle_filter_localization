#include <vector>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

//#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
namespace particle_filter_localization
{
    


struct Particle
{
    Eigen::Vector3d pos;
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
    void predict();
    void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
    void resample();

private:
    std::vector<Particle> particles_;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;

    std::mt19937 random_seed_;//Mersenne twister

};

} // namespace particle_filter_localization