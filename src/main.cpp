#include "../include/mainwindow.h"
#include <QApplication>

using namespace Qt;

int main(int argc, char **argv){

    QApplication app(argc,argv);

    ros::init(argc, argv, "robot_state_viewer_node");

    std::string objectsdbname;
    std::string objectscollectionname;
    std::string mongodbhost;
    std::string mongodbport;
    std::string roidb;

    MainWindow mw;

    if(argc < 2)
    {

        std::cout<<
                    "Running the robot_state_viewer with default arguments: MongoDB HostName:localhost, MongoDB Port:62345, ObjectsDBName: soma2data, ObjectsCollectionName:soma2, ROIDBName:soma2data"
                 <<std::endl;
        // std::cout << "Not enough input arguments!! Quitting..."<<std::endl;

        //  return -1;

    }
    else
    {
        if(argc > 1){
            objectsdbname = argv[1];
            mw.rosthread.setSOMA2ObjectsDBName(objectsdbname);

        }
        if(argc >2){
            objectscollectionname = argv[2];
            mw.rosthread.setSOMA2ObjectsCollectionName(objectscollectionname);
        }
        if(argc >3){
            mongodbhost = argv[3];
            mw.setMongoDBHostName(mongodbhost);
        }
        if(argc >4){
            mongodbport = argv[4];
            mw.setMongoDBPort(mongodbport);
        }

        if(argc >5){
            roidb = argv[5];
            mw.rosthread.setSOMA2ROIDBName(roidb);
        }

    }

    qRegisterMetaType<std::vector<std::string> >();
    qRegisterMetaType<std::vector<SOMA2ROINameID> >();

    //RosThread thread();






    mw.show();

    return app.exec();




}
