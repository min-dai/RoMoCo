#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>  
#include <QTimer>
#include "biped_command/radio_slider_gui.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "slider_gui");
   ros::NodeHandle nh;
   ros::Publisher slider_pub = nh.advertise<std_msgs::Float64MultiArray>("radio_slider_values", 10);

   QApplication app(argc, argv);
   RadioSliderGUI fake_radio;
   fake_radio.show();

   Eigen::VectorXd slider_value(9);

   // Use QTimer to process ROS while allowing Qt to handle UI events
   QTimer timer;
   QObject::connect(&timer, &QTimer::timeout, [&]()
                    {
    if (!ros::ok()) {
        app.quit();  // Exit Qt app when ROS is shutting down
    }
    ros::spinOnce();
    slider_value = fake_radio.getSliderValues();

    //  Publish slider values as a ROS message
    std_msgs::Float64MultiArray msg;
    msg.data.resize(slider_value.size());
    for (int i = 0; i < slider_value.size(); ++i) {
        msg.data[i] = slider_value[i];
    }
    slider_pub.publish(msg);
   
   });

    
   timer.start(10); // Check every 10ms

   QObject::connect(&app, &QApplication::aboutToQuit, [&]()
                    { ros::shutdown(); });

   return app.exec();
}
