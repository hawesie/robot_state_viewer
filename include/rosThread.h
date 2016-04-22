#include <QThread>
#include <ros/ros.h>
#include <QVector>
#include <QCollator>
#include <mongodb_store/MongoFind.h>
#include <mongodb_store/message_store.h>
#include <mongo/bson/bson.h>
#include <soma2_msgs/SOMA2Object.h>
#include <soma_map_manager/MapInfo.h>
#include <soma2_msgs/SOMA2ROIObject.h>
#include <soma_roi_manager/DrawROI.h>
#include <algorithm>    // std::unique, std::distance
#include <vector>       // std::vector

struct SOMA2ROINameID{

    std::string id;
    std::string name;

};

struct SOMA2TimeLimits{

    int maxtimestep;
    int mintimestep;

    long mintimestamp;
    long maxtimestamp;

};


class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    sensor_msgs::PointCloud2 worldstate;


     // Shutdown the node
     void shutdownROS();



     // Returns the combined cloud of soma2objects
     sensor_msgs::PointCloud2 getSOMA2CombinedObjectCloud(const std::vector<soma2_msgs::SOMA2Object>& soma2objects);



    // sensor_msgs::PointCloud2 getSOMA2ObjectCloudsWithTimestep(int timestep);


     // Publish SOMA2 Object Cloud
     void publishSOMA2ObjectCloud(sensor_msgs::PointCloud2 msg);

     // Return the log date of the object at timestep t
     std::string getSOMA2ObjectDateWithTimestep(int timestep);

     // Return the name of the current map
     std::string getMapName();

     // Service call for drawing roi with id
     void drawROIwithID(std::string id);

     // Get the SOMA2 ROI with id
     soma2_msgs::SOMA2ROIObject getSOMA2ROIwithID(int id);

     // Query the SOMA2 objects
     std::vector<soma2_msgs::SOMA2Object> querySOMA2Objects(const mongo::BSONObj& queryobj);

     // Query the SOMA2 objects using timeinterval
  //   std::vector<soma2_msgs::SOMA2Object> querySOMA2ObjectsWithDate(const mongo::BSONObj& queryobj);

     // Set the DB name for SOMA2 objects
     void setSOMA2ObjectsDBName(std::string name);

     // Set the collection name for SOMA2 objects
     void setSOMA2ObjectsCollectionName(std::string name);

     void setSOMA2ROIDBName(std::string name);

     // Get the DB name for SOMA2 objects
     std::string getSOMA2ObjectsDBName();

     // Get the collection name for SOMA2 objects
     std::string getSOMA2ObjectsCollectionName();

	
     SOMA2TimeLimits getSOMA2CollectionMinMaxTimelimits();


     std::string getROIDBName();

   //  std::vector<int> getSOMA2CollectionMinMaxTimestep();

    // std::vector<std::string> getSOMA2ObjectLabels();

private:
     bool shutdown;
     ros::NodeHandle n;
     ros::Publisher pcp;
     ros::ServiceClient roiclient;

     std::string objectsdbname;
     std::string objectscollectionname;
     std::string roibdname;
    // mongodb_store_modified::MongoFindRequest req;
   //  mongodb_store::MessageStoreProxy soma2messagestore;
   //  mongodb_store::MessageStoreProxy soma2messagestoreROI;


     // Get the ROIs from db
     void fetchSOMA2ROINames();

     // Get the object labels from db
     void fetchSOMA2ObjectTypesIDs();


     std::string map_name;
     std::string map_unique_id;
     std::vector<SOMA2ROINameID> roinameids;
     std::vector<std::string> labelnames;
     std::vector <soma2_msgs::SOMA2ROIObject> roiarray;
   //  std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> >  soma2objects;



signals:

   void  rosStarted();
   void  rosStartFailed();
   void  rosFinished();
   void  mapinfoReceived();
   void  SOMA2ObjectTypes(std::vector<std::string>);
   void  SOMA2ObjectIDs(std::vector<std::string>);
   void  SOMA2ROINames(std::vector<SOMA2ROINameID>);

public slots:
   //void getSliderValue(int val);

  // void handleVelocityCommand(QVector<double> velCommand);

   void loop();

};
Q_DECLARE_METATYPE (std::vector<std::string>)
Q_DECLARE_METATYPE (std::vector<SOMA2ROINameID>)
