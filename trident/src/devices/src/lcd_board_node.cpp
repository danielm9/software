// auvic libraries
#include <peripheral_manager.hpp>

int main(int argc, char ** argv) {
    // Setup ROS stuff
    ros::init(argc, argv, "lcd_board_node");
    // "peripherals" is a namespace used to connect with the periph topics
    ros::NodeHandle n_auvic("peripherals");
    // start powerboard node
    Lcd_Board lcd_board(&n_auvic);

    ros::spin();
    return 0;
}
    