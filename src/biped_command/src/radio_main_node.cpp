#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <QTimer>
#include "biped_command/radio_slider_gui.hpp"

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("radio_slider_node");
   auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("radio_slider_values", 10);
   
   QApplication app(argc, argv);
   RadioSliderGUI fake_radio;
   fake_radio.show();

   Eigen::VectorXd slider_value(9);

   // Use QTimer to process ROS while allowing Qt to handle UI events
   QTimer timer;
   QObject::connect(&timer, &QTimer::timeout, [&]()
                    {
    if (!rclcpp::ok()) {
        app.quit();  // Exit Qt app when ROS is shutting down
    }
    rclcpp::spin_some(node);
    slider_value = fake_radio.getSliderValues();

    //  Publish slider values as a ROS message
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(slider_value.size());
    for (int i = 0; i < slider_value.size(); ++i) {
        msg.data[i] = slider_value[i];
    }
    publisher->publish(msg);
   
   });

    
   timer.start(10); // Check every 10ms

   QObject::connect(&app, &QApplication::aboutToQuit, [&]()
                    { rclcpp::shutdown(); });

   return app.exec();
}
