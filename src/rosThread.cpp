#include "rosThread.h"
#include <QDebug>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl_ros/point_cloud.h>
#include <QJsonObject>
#include <QJsonDocument>
#include <QStringList>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


RosThread::RosThread()/*:soma2messagestore(n,"soma2","labelled_objects"),soma2messagestoreROI(n,"soma2_roi","message_store")*/
{
    shutdown = false;
    this->objectsdbname = "soma2data";
    this->objectscollectionname = "soma2";
    this->roibdname = "soma2data";

}

void RosThread::setSOMA2ObjectsDBName(std::string name)
{
    this->objectsdbname = name;


}
void RosThread::setSOMA2ObjectsCollectionName(std::string name)
{
    this->objectscollectionname = name;


}

void RosThread::setSOMA2ROIDBName(std::string name)
{
    this->roibdname = name;

}

std::string RosThread::getSOMA2ObjectsDBName()
{
    return this->objectsdbname;
}
std::string RosThread::getSOMA2ObjectsCollectionName()
{
    return this->objectscollectionname;
}
std::string RosThread::getROIDBName()
{
    return this->roibdname;
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


    }
    else
    {
        ROS_ERROR("Failed to call service map_info");

    }


    pcp = n.advertise<sensor_msgs::PointCloud2>("robot_state_viewer_node/world_state_3d",1);

    ros::Rate loop(10);


   // if(this->roibdname.size() > 0)
    this->fetchSOMA2ROINames();

    this->fetchSOMA2ObjectLabels();

    emit mapinfoReceived();

    // emit SOMA2ObjectLabels(res);

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
void RosThread::fetchSOMA2ObjectLabels()
{
    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy soma2store(nl,this->objectscollectionname,this->objectsdbname);

    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> >  soma2objects;
    std::vector<std::string> soma2labels;

    QString dir = QDir::homePath();

    dir.append("/").append(".soma2/");

    QDir labelsdir;

    if(!labelsdir.exists(dir))
        labelsdir.mkdir(dir);

    QString filename = "objectlabels.txt";

    dir.append(filename);

    QFile file(dir);

    if(!file.open(QFile::WriteOnly))
    {
        qDebug()<<"Cannot Open labels file! Returning...";
        return ;


    }

    // Query all objects,
    soma2store.query(soma2objects);

    // List that stores the object labels
    QStringList ls;

    nl.shutdown();

    // If we have any objects
    if(soma2objects.size()>0)
    {


        for(int i = 0; i < soma2objects.size(); i++)
        {
            QString str;



            //   spr = soma2objects[i];

            str.append(QString::fromStdString(soma2objects[i]->type));



            ls.append(str);



            //std::cout<<soma22objects[i].use_count()<<std::endl;


        }
    }

    //  soma2objects.clear();


    // Remove duplicate names
    ls.removeDuplicates();

    // Sort the labels
    ls.sort(Qt::CaseInsensitive);


    QTextStream stream(&file);

    foreach(QString st, ls)
    {
        stream<<st<<"\n";

        // Dump data to array
        // res[count].append(st.toStdString());

        // Then transfer it into vector
        //  soma2labels.push_back(res[count]);

        // this->labelnames.push_back(res[count]);

        //  count++;
        // qDebug()<<st;
    }


    file.close();



    /* std::sort(res.begin(), res.end());

    auto last = std::unique(res.begin(), res.end());

    res.erase(last, res.end());*/

    //res.resize( std::distance(res.begin(),it) );


    emit SOMA2ObjectLabels(soma2labels);

    return ;

}
std::vector<int> RosThread::getSOMA2CollectionMinMaxTimestep()
{
    std::vector<int> res;
    int min = -1;
    int max = -1;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy soma2store(nl,this->objectscollectionname,this->objectsdbname);

    mongo::BSONObjBuilder builder;

    builder.append("$natural",-1);

    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

    soma2store.query(soma2objects,mongo::BSONObj(),mongo::BSONObj(),builder.obj(),false,1);


    if(soma2objects.size() > 0)
        max = soma2objects[0]->timestep;
        //std::cout<<soma2objects[0]->timestep<<std::endl;

    soma2objects.clear();
    soma2store.query(soma2objects,mongo::BSONObj(),mongo::BSONObj(),mongo::BSONObj(),false,1);

    if(soma2objects.size() > 0)
        min = soma2objects[0]->timestep;

    res.push_back(min);
    res.push_back(max);

    return res;

}
void RosThread::fetchSOMA2ROINames()
{
    // std::vector<SOMA2ROINameID> res;

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy soma2store(nl,"soma2_roi",this->roibdname);

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

    nl.shutdown();

    // rois.clear();

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
sensor_msgs::PointCloud2 RosThread::getSOMA2CombinedObjectCloud(const std::vector<soma2_msgs::SOMA2Object> &soma2objects)
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

std::string RosThread::getMapName()
{
    return this->map_name;

}
std::string RosThread::getSOMA2ObjectDateWithTimestep(int timestep)
{
    ros::NodeHandle nl;
    mongodb_store::MessageStoreProxy soma2store(nl,this->objectscollectionname,this->objectsdbname);

    mongo::BSONObjBuilder builder2;

    sensor_msgs::PointCloud2 result;

    QJsonObject jsonobj;

    jsonobj.insert("timestep",timestep);
    jsonobj.insert("map_name",QString::fromStdString(this->map_name));

    QJsonDocument doc;

    doc.setObject(jsonobj);

    QString str(doc.toJson());

    //  qDebug()<<str;

    std::stringstream ss;
    //  ss<<"{\"timestep\":\""<<timestep<<"\,\"map_name\":\""<<this->map_name<<"\"}";

    builder2.appendElements(mongo::fromjson(str.toStdString()));


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

    soma2store.query(soma2objects,builder2.obj());
    nl.shutdown();
    soma2_msgs::SOMA2Object anobject;
    std::string date;
    if(soma2objects.size() > 0){
        anobject = *soma2objects[0];
        qint64 val = (float)anobject.logtimestamp*1000;
        QDateTime dt = QDateTime::fromMSecsSinceEpoch(val,Qt::UTC);
      //  qDebug()<<dt.toString(Qt::ISODate);
        date = dt.toString(Qt::ISODate).toStdString();
    }


    soma2objects.clear();


    return date;
}
void RosThread::publishSOMA2ObjectCloud(sensor_msgs::PointCloud2 msg)
{
    pcp.publish(msg);


}
std::vector<soma2_msgs::SOMA2Object> RosThread::querySOMA2ObjectsWithDate(const mongo::BSONObj &queryobj)
{

    ros::NodeHandle nl;
    mongodb_store::MessageStoreProxy soma2store(nl,this->objectscollectionname,this->objectsdbname);

    std::vector<soma2_msgs::SOMA2Object> res;


    mongo::BSONObjBuilder builder;


    builder.appendElements(queryobj);


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;


    mongo::BSONObj builderobj = builder.obj();

    soma2store.query(soma2objects,builderobj);


  //  qDebug()<<QString::fromStdString(builderobj.jsonString());


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
std::vector<soma2_msgs::SOMA2Object> RosThread::querySOMA2Objects(mongo::BSONObj &queryobj, int timestep)
{

    ros::NodeHandle nl;

    mongodb_store::MessageStoreProxy soma2store(nl,this->objectscollectionname,this->objectsdbname);


    std::vector<soma2_msgs::SOMA2Object> res;
    //  std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > res;


  //  mongo::BSONObjBuilder builder;


  //  builder.appendElements(queryobj);


   // builder.append("timestep",timestep);


    std::vector<boost::shared_ptr<soma2_msgs::SOMA2Object> > soma2objects;

  //  queryobj = builder.obj();

    soma2store.query(soma2objects,queryobj);


    if(soma2objects.size() > 0)
    {
        for(auto &labelled_object:soma2objects)
        {
            res.push_back(*labelled_object);
            //qDebug()<<labelled_object.use_count;
        }

    }


    qDebug()<<"Query returned"<<res.size()<<"objects";

    return res;


}

