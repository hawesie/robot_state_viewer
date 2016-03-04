#include <QThread>
#include <ros/ros.h>
#include <QVector>
#include <mongodb_store_modified/MongoFind.h>
#include <mongodb_store/message_store.h>
#include <mongo/bson/bson.h>
#include <soma2_msgs/SOMA2Object.h>
#include <soma2_map_manager/MapInfo.h>
#include <soma2_msgs/SOMA2ROIObject.h>
#include <soma2_roi_manager/DrawROI.h>
#include <algorithm>    // std::unique, std::distance
#include <vector>       // std::vector
struct SOMA2ROINameID{

    std::string id;
    std::string name;

};


class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    //RosThread(int argc, char **argv, std::string nodeName);

     void shutdownROS();
     sensor_msgs::PointCloud2 worldstate;
     sensor_msgs::PointCloud2 getSOMA2ObjectCloudsWithTimestep(int timestep);
     void publishSOMA2ObjectClouds(sensor_msgs::PointCloud2 msg);
     sensor_msgs::PointCloud2 getSOMA2ObjectClouds(const std::vector<soma2_msgs::SOMA2Object>& soma2objects);
     std::string getSOMA2ObjectDateWithTimestep(int timestep);
   //  std::vector<SOMA2ROINameID> getSOMA2ROINames();
     std::string getMapName();
     void drawROIwithID(std::string id);
     soma2_msgs::SOMA2ROIObject getSOMA2ROIwithID(int id);
     std::vector<soma2_msgs::SOMA2Object> querySOMA2Objects(const mongo::BSONObj& queryobj, int timestep);
     std::vector<soma2_msgs::SOMA2Object> querySOMA2ObjectsWithDate(const mongo::BSONObj& queryobj);

    // std::vector<std::string> getSOMA2ObjectLabels();


private:
     bool shutdown;
     ros::NodeHandle n;
     ros::Publisher pcp;
     ros::ServiceClient roiclient;
    // mongodb_store_modified::MongoFindRequest req;
     mongodb_store::MessageStoreProxy soma2messagestore;
     mongodb_store::MessageStoreProxy soma2messagestoreROI;
     void getSOMA2Objects();
     void fetchSOMA2ROINames();
     void fetchSOMA2ObjectLabels();
     std::string map_name;
     std::string map_unique_id;
     std::vector<SOMA2ROINameID> roinameids;
     std::vector<std::string> labelnames;
     std::vector <soma2_msgs::SOMA2ROIObject> roiarray;



signals:

   void  rosStarted();
   void  rosStartFailed();
   void  rosFinished();
   void  mapinfoReceived();
   void  SOMA2ObjectLabels(std::vector<std::string>);
   void  SOMA2ROINames(std::vector<SOMA2ROINameID>);
public slots:
   void getSliderValue(int val);

  // void handleVelocityCommand(QVector<double> velCommand);

   void loop();

};
Q_DECLARE_METATYPE (std::vector<std::string>)
Q_DECLARE_METATYPE (std::vector<SOMA2ROINameID>)
