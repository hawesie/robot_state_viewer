#ifndef QUERYBUILDER_H
#define QUERYBUILDER_H
#include <QJsonObject>
#include <QJsonDocument>
#include <QVariantMap>
#include <QString>
#include "mainwindow.h"
#include <QJsonArray>
#include <QJsonValue>
#include <mongo/bson/bson.h>
#include <mongo/bson/bsonelement.h>

class QueryBuilder
{
public:
    QueryBuilder();

    mongo::BSONObj buildROIQuery(soma2_msgs::SOMA2ROIObject roiobj);
private:
    RosThread rosthread;
};

#endif // QUERYBUILDER_H
