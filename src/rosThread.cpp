#include "rosThread.h"
#include <QDebug>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl_ros/point_cloud.h>
#include <QJsonObject>
#include <QJsonDocument>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


RosThread::RosThread():soma2messagestore(n,"soma2","labelled_objects"),soma2messagestoreROI(n,"soma2_roi","message_store")
{
    shutdown = false;

}

void RosThread::loop()
{


    if(!ros::ok()){

        emit rosStartFailed();

        return;
    }

    emit rosStarted();

    qDebug()<<"Ros Thread is running!!";

    ros::ServiceClient client = n.serviceClient<soma2_map_manager::MapInfo>("soma2/map_info");

    this->roiclient = n.serviceClient<soma2_roi_manager::DrawROI>("soma2/draw_roi");

    soma2_map_manager::MapInfo srv;
    srv.request.request = 0;

    qDebug()<<"Waiting for map_info service from soma2_map_manager";

    client.waitForExistence();
    if (client.call(srv))
    {
        ROS_INFO("Map Info: %s Map Unique ID: %s", srv.response.map_name.data(), srv.response.map_unique_id.data());
        this->map_name = srv.response.map_name.data();
        this->map_unique_id = srv.response.map_unique_id.data();
        emit mapinfoReceived();

    }
    else
    {
        ROS_ERROR("Failed to call service map_info");

    }


    pcp = n.advertise<sensor_msgs::PointCloud2>("robot_state_viewer_node/world_state_3d",1);

    // ros::AsyncSpinner spinner(2);

    // spinner.start();

    ros::Rate loop(10);

    //worldstate =  getSOMA2ObjectsWithTimestep(0);

    pcp.publish(worldstate);

    this->fetchSOMA2ROINames();
    this->fetchSOMA2ObjectLabels();

    // getSOMA2Objects();

    while(ros::ok())
    {


        // velocityCommandPublisher.publish(velocityCommand);

        ros::spinOnce();

        loop.sleep();



    }
    qDebug()<<"Ros finished!!";

    emit rosFinished();



}
void RosThread::drawROIwithID(std::string id)
{

    soma2_roi_manager::DrawROI drawroi;

    drawroi.request.map_name = this->map_name;
    drawroi.request.roi_id = id;

    this->roiclient.call(drawroi);


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}
void RosThread::getSliderValue(int val)
{
    //worldstate = getSOMA2ObjectsWithTimestep(val);

}
void RosThread::getSOMA2Objects()
{
    mongo::BSONObjBuilder builder2;


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

    soma2messagestore.query(soma2objects);

    if(soma2objects.size()>0)
    {
        qDebug()<<"Size of the labelled SOMA2 Objects"<<soma2objects.size();
    }

}
void RosThread::fetchSOMA2ObjectLabels()
{
    mongodb_store::MessageStoreProxy soma2store(n,"soma2","labelled_objects");

    std::vector<std::string> res;

    mongo::BSONObjBuilder builder2;

    builder2.append("type",1);

    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

    soma2store.query(soma2objects);

    if(soma2objects.size()>0)
    {
        //qDebug()<<"Size of the labelled SOMA2 Objects"<<labelled_objects.size();
        for(int i = 0; i < soma2objects.size(); i++)
        {
            res.push_back(soma2objects[i]->type);

        }
    }
    soma2objects.clear();
    //std::vector<int>::iterator it;

    std::sort(res.begin(), res.end());

    auto last = std::unique(res.begin(), res.end());

    res.erase(last, res.end());

    //res.resize( std::distance(res.begin(),it) );

   // this->labelnames = res;


    emit SOMA2ObjectLabels(res);

  //  return res;

}
void RosThread::fetchSOMA2ROINames()
{
  // std::vector<SOMA2ROINameID> res;


   mongodb_store::MessageStoreProxy soma2store(n,"soma2_roi","message_store");

   mongo::BSONObjBuilder builder;


   QJsonObject jsonobj;

   jsonobj.insert("map_name",QString::fromStdString(this->map_name));

   QJsonDocument doc;

   doc.setObject(jsonobj);

   QString str(doc.toJson());

  // qDebug()<<str;

   builder.appendElements(mongo::fromjson(str.toStdString()));


   std::vector<boost::shared_ptr<soma2_msgs::SOMA2ROIObject> > rois;


   soma2store.query(rois,builder.obj());

   if(rois.size() > 0)
   {
       for(auto &roi:rois)
       {
          // res.push_back(roi->type.data());
           SOMA2ROINameID roinameid;
           roinameid.id = roi->id.data();
           roinameid.name = roi->type.data();
           this->roinameids.push_back(roinameid);
           this->roiarray.push_back(*roi);
       }
   }

   rois.clear();

   emit SOMA2ROINames(this->roinameids);

   //  return this->roinameids;



}
soma2_msgs::SOMA2ROIObject RosThread::getSOMA2ROIwithID(int id)
{
    soma2_msgs::SOMA2ROIObject obj;

    for(auto roi:this->roiarray)
    {
        if(id ==  QString::fromStdString(roi.id.data()).toInt())
        {
            qDebug()<<"ROI found";
            return roi;

        }
    }


    return obj;

}
sensor_msgs::PointCloud2 RosThread::getSOMA2ObjectClouds(const std::vector<soma2_msgs::SOMA2Object> &soma2objects)
{
    sensor_msgs::PointCloud2 result;

    Cloud cc;

    // If there are no soma objects send an empty cloud
    if(soma2objects.size() == 0)
    {
        pcl::PointXYZRGB point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        cc.push_back(point);

        cc.header.frame_id = "/map";
        pcl::toROSMsg(cc,result);

        return result;

    }

    for(int i = 0; i < soma2objects.size(); i++)
    {
        sensor_msgs::PointCloud2 cloud = soma2objects[i].cloud;
        Cloud pclcloud;
        pcl::fromROSMsg(cloud,pclcloud);
        cc.points.resize(cc.width*cc.height + pclcloud.height*pclcloud.width);
        cc += pclcloud;

        if(i == 30) break;


    }
    cc.header.frame_id = "/map";
    pcl::toROSMsg(cc,result);

    return result;
}
sensor_msgs::PointCloud2 RosThread::getSOMA2ObjectCloudsWithTimestep(int timestep)
{
    mongo::BSONObjBuilder builder2;

    sensor_msgs::PointCloud2 result;

    QJsonObject jsonobj;

    jsonobj.insert("timestep",QString::number(timestep));
    jsonobj.insert("map_name",QString::fromStdString(this->map_name));

    QJsonDocument doc;

    doc.setObject(jsonobj);

    QString str(doc.toJson());

    qDebug()<<str;

    std::stringstream ss;
  //  ss<<"{\"timestep\":\""<<timestep<<"\,\"map_name\":\""<<this->map_name<<"\"}";

    builder2.appendElements(mongo::fromjson(str.toStdString()));


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

  //  soma2messagestore.query(soma2objects,builder2.obj());

    if(soma2objects.size()>0)
    {
        qDebug()<<"Size of the labelled SOMA2 Objects"<<soma2objects.size();
        Cloud cc;
        for(int i = 0; i < soma2objects.size(); i++)
        {
            sensor_msgs::PointCloud2 cloud = soma2objects[i]->cloud;
            Cloud pclcloud;
            pcl::fromROSMsg(cloud,pclcloud);
            cc.points.resize(cc.width*cc.height + pclcloud.height*pclcloud.width);
            cc += pclcloud;


        }
        cc.header.frame_id = "/map";
        pcl::toROSMsg(cc,result);

    }

    return result;
}
std::string RosThread::getMapName()
{
    return this->map_name;

}
std::string RosThread::getSOMA2ObjectDateWithTimestep(int timestep)
{
    mongodb_store::MessageStoreProxy soma2store(n,"soma2","labelled_objects");

    mongo::BSONObjBuilder builder2;

    sensor_msgs::PointCloud2 result;

    QJsonObject jsonobj;

    jsonobj.insert("timestep",QString::number(timestep));
    jsonobj.insert("map_name",QString::fromStdString(this->map_name));

    QJsonDocument doc;

    doc.setObject(jsonobj);

    QString str(doc.toJson());

    qDebug()<<str;

    std::stringstream ss;
  //  ss<<"{\"timestep\":\""<<timestep<<"\,\"map_name\":\""<<this->map_name<<"\"}";

    builder2.appendElements(mongo::fromjson(str.toStdString()));


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

    soma2store.query(soma2objects,builder2.obj());
    soma2_msgs::SOMA2Object anobject;
    std::string date;
    if(soma2objects.size() > 0){
        anobject = *soma2objects[0];
        date = anobject.logtime.data();
    }


    return date;
}
void RosThread::publishSOMA2ObjectClouds(sensor_msgs::PointCloud2 msg)
{
    pcp.publish(msg);


}
std::vector<soma2_msgs::SOMA2Object> RosThread::querySOMA2ObjectsWithDate(const mongo::BSONObj &queryobj)
{

    std::vector<soma2_msgs::SOMA2Object> res;


    mongo::BSONObjBuilder builder;


    builder.appendElements(queryobj);


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;


   /* queryjson = queryjson.remove(queryjson.indexOf("\""),1);
    queryjson = queryjson.remove(queryjson.lastIndexOf("\""),1);*/

 //   qDebug()<<queryjson.toLatin1().data();

  //   std::string stdstr(queryjson.toLatin1().data());

  //  stdstr.erase(stdstr.begin(),stdstr.begin()+1);
   // stdstr.erase(stdstr.end()-1,stdstr.end());

   // soma2messagestore.query(labelled_objects,QUERY("\"geoloc\""<<stdstr).obj);


    qDebug()<<"I am here";

    mongo::BSONObj builderobj = builder.obj();

    soma2messagestore.query(soma2objects,builderobj);

    qDebug()<<QString::fromStdString(builderobj.jsonString());


    if(soma2objects.size() > 0)
    {
        for(auto &labelled_object:soma2objects)
        {
            res.push_back(*labelled_object);
        }

    }
    qDebug()<<"Query returned"<<res.size()<<"objects";

   // std::cout<<QUERY("\"geoloc\""<<stdstr).obj.toString()<<std::endl;

  //  std::cout<< BSON("geoloc"<<stdstr).jsonString();

    return res;

}
std::vector<soma2_msgs::SOMA2Object> RosThread::querySOMA2Objects(const mongo::BSONObj &queryobj, int timestep)
{
    std::vector<soma2_msgs::SOMA2Object> res;


    mongo::BSONObjBuilder builder;


    builder.appendElements(queryobj);

    std::stringstream ss;

    ss<<timestep;

    builder.append("timestep",ss.str().data());

    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;



   /* queryjson = queryjson.remove(queryjson.indexOf("\""),1);
    queryjson = queryjson.remove(queryjson.lastIndexOf("\""),1);*/

 //   qDebug()<<queryjson.toLatin1().data();

  //   std::string stdstr(queryjson.toLatin1().data());

  //  stdstr.erase(stdstr.begin(),stdstr.begin()+1);
   // stdstr.erase(stdstr.end()-1,stdstr.end());

   // soma2messagestore.query(labelled_objects,QUERY("\"geoloc\""<<stdstr).obj);


    qDebug()<<"I am here";

    mongo::BSONObj builderobj = builder.obj();

    soma2messagestore.query(soma2objects,builderobj);

    qDebug()<<QString::fromStdString(builderobj.jsonString());


    if(soma2objects.size() > 0)
    {
        for(auto &labelled_object:soma2objects)
        {
            res.push_back(*labelled_object);
        }

    }
    qDebug()<<"Query returned"<<res.size()<<"objects";

   // std::cout<<QUERY("\"geoloc\""<<stdstr).obj.toString()<<std::endl;

  //  std::cout<< BSON("geoloc"<<stdstr).jsonString();

    return res;


}
/*void RosThread::handleVelocityCommand(QVector<double> velCommand)
{
     qDebug()<<"Helllo"<<velCommand.size();

    if(velCommand.size() == 2){

        qDebug()<<"Helllo"<<velCommand.size();


        velocityCommand.linear.x = (double)velCommand.at(0);
        velocityCommand.angular.z = (double)velCommand.at(1);

        qDebug()<<"Hello"<<velocityCommand.linear.x<<" "<<velocityCommand.angular.z;
    }
    else
    {
        velocityCommand.linear.x = 0.0;
        velocityCommand.angular.z = 0.0;

    }
}*/

