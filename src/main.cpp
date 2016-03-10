#include "../include/mainwindow.h"
#include <QApplication>
//#include "rosThread.h"

//class Spinner {
//public:
//        Spinner() :
//                shutdown_required(false),
//                thr(&Spinner::spin)
//        {}

//        ~Spinner() {
//                shutdown_required = true;
//                thr.join();
//        }

//        void spin() {
//                ros::Rate loop(10);
//                sleep(1);
//                while ( ros::ok() && !shutdown_required ) {
//                        ros::spinOnce();
//                        loop.sleep();
//                }
//        }

//private:
//        bool shutdown_required;
//        boost::thread thr; // similarly, a boost thread
//};
using namespace Qt;

int main(int argc, char **argv){

    QApplication app(argc,argv);

    ros::init(argc, argv, "robot_state_viewer_node");

    std::string objectsdb;
    std::string roidb;

    if(argc < 2)
    {
        std::cout << "Not enough input arguments!! Quitting..."<<std::endl;

        return -1;

    }
    else
    {
        objectsdb = argv[1];
        if(argc > 2)
            roidb = argv[2];

    }

    qRegisterMetaType<std::vector<std::string> >();
    qRegisterMetaType<std::vector<SOMA2ROINameID> >();

    //RosThread thread();

    MainWindow mw;


    mw.rosthread.setObjectsDBName(objectsdb);
    mw.rosthread.setROIDBName(roidb);


    mw.show();

    return app.exec();

//ssss

//    cv::Mat img(145,145,CV_8UC1);

//    img.at<uchar>(0,0) = 255;
//    img.at<uchar>(0,1) = 255;

//    cv::imshow("mehmet",img);

//    ros::init(argc, argv, "talker");

//    RosThread rosThread;

//    rosThread.start();

//    //cv::Vec1b vec ;
//   // vec[0] = 255;

//    cv::namedWindow("mehmet");

//   // ros::

//    ros::NodeHandle n;

//    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

//    // Message publisher for commanding velocity to our robot
//    ros::Publisher create = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

//    // This is the required message type for velocity command
//    geometry_msgs::Twist twist;

//    // Give a linear velocity of 0.1 m/s in x direction(forward)
//    twist.linear.x = 0.1;

//    //ros::ServiceServer service = n.advertiseService("add_two_ints", add);

//    // ROs loop rate is 10 HZ
//  //  ros::Rate loop_rate(10);

//  //  int count = 0;

//   /* while(ros::ok()){


//        std::string str = "/tank";
//       //std::string ser = "1 100 100 && sleep 0.5";
//       // ros::service::call(str,'1 100 100 && sleep 0.5');

//        std_msgs::String msg;

//        std::stringstream ss;

//        ss<<"Hello World "<<count;

//        msg.data = ss.str();

//       // ROS_INFO("%s", msg.data.c_str());

//        chatter_pub.publish(msg);

//        // Publish our message to the robot
//        create.publish(twist);

//        ros::spinOnce();

//        loop_rate.sleep();

//        count++;

//        int key_pressed = cvWaitKey(10) & 0xFF;

//      //  ROS_INFO("%d",key_pressed);

//        // if key pressed is equal to "q"
//        if(key_pressed == 113){

//            // set twist velocity to 0
//            twist.linear.x = 0.0;

//            //stop the robot
//            create.publish(twist);

//            ROS_INFO("KEY PRESSED");

//            cv::destroyAllWindows();



//        }

//    }*/

//    return 0;



}
