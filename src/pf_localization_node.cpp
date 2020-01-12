#include <particle_filter_localization/pf_localization_component.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<particle_filter_localization::PfLocalizationComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}