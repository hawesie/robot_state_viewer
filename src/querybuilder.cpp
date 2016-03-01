#include "querybuilder.h"
QueryBuilder::QueryBuilder()
{


}

mongo::BSONObj QueryBuilder::buildROIQuery(soma2_msgs::SOMA2ROIObject roiobj)
{

    QJsonArray arr;



    mongo::BSONArrayBuilder b;


    //BSON_ARRAY("\"coordinates\""<<arr);

    for(int i = 0; i < roiobj.geoposearray.poses.size(); i++){

        mongo::BSONArrayBuilder b2;

        QJsonArray pose;

        geometry_msgs::Pose apose = roiobj.geoposearray.poses[i];

        pose.append(apose.position.x);
        pose.append(apose.position.y);

        b2.append(apose.position.x);
        b2.append(apose.position.y);

        //pose.append(0.001);
       // pose.append(0.002);

        arr.append(pose);
        b.append(b2.arr());
    }

    QJsonArray pose;
    mongo::BSONArrayBuilder b2;

   // pose.append(roiobj.geoposearray.poses[0].position.x);
  //  pose.append(roiobj.geoposearray.poses[0].position.y);

   // b2.append((float)roiobj.geoposearray.poses[0].position.x);
  //  b2.append((float)roiobj.geoposearray.poses[0].position.y);


    arr.append(pose);
   // b.append(b2.arr());

    QJsonValue jsonval(QString::fromStdString("Polygon"));

    QJsonObject jobj;

    QJsonArray arr2;

    mongo::BSONArrayBuilder b3;

    arr2.append(arr);
    b3.append(b.arr());

  // qDebug()<<QString::fromStdString(b3.obj().jsonString());

    mongo::BSONObjBuilder builder;

    builder.appendArray("coordinates",b3.arr());
    builder.append("type","Polygon");

    mongo::BSONObjBuilder builder2;
    builder2.appendElements(builder.obj());



    jobj.insert("type",jsonval);

    jobj.insert("coordinates",arr2);


    QJsonObject jobj2;
     jobj2.insert("$geometry",jobj);

    mongo::BSONObjBuilder builder3;

    builder3.append("$geometry",builder2.obj());

  //  qDebug()<<QString::fromStdString(builder3.obj().jsonString());

    QJsonObject jobj3;
    jobj3.insert("$geoWithin",jobj2);

    mongo::BSONObjBuilder builder4;

    builder4.append("$geoWithin",builder3.obj());

   // qDebug()<<QString::fromStdString(builder4.obj().jsonString());


    QJsonObject jobj4;
    jobj4.insert("geoloc",jobj3);

    mongo::BSONObjBuilder builder5;

    builder5.append("geoloc",builder4.obj());


    //std::string resstr =builder5.obj().jsonString();

  //  qDebug()<<"This is the bson: "<<resstr;


   // QJsonDocument doc(jobj4);
    //qDebug()<<doc.toJson(QJsonDocument::Compact);

    //QString resjson(doc.toJson(QJsonDocument::Compact));

  //  QString resjson1 = resjson.remove();
   // QString resjson2 = resjson1.remove(resjson1.length()-1,1);

   // resjson = resjson2;

    //qDebug()<<resjson.simplified();
   // std::cout<<resjson.simplified().toStdString();

    //bsoncxx::builder::stream::document order_builder;

    //order_builder<<resjson.simplified().toStdString();

    //resjson.remove(QRegExp("[\\n\\t\\r]"));

    return builder5.obj();

}
