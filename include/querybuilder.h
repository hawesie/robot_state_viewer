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

    static mongo::BSONObj buildSOMA2ROIWithinQuery(const soma2_msgs::SOMA2ROIObject& roiobj);

    static mongo::BSONObj buildSOMA2LabelEqualsQuery(const std::string& labelname);

    static mongo::BSONObj buildSOMA2LabelContainsQuery(const std::string& text);

    static mongo::BSONObj buildSOMA2WeekdayQuery(int index);

    static mongo::BSONObj buildSOMA2DateQuery(ulong lowerdate, ulong upperdate, int mode);

    static mongo::BSONObj buildSOMA2TimeQuery(int lowerhour,int lowerminute, int upperhour, int upperminute,  int mode);

private:
    RosThread rosthread;
};

#endif // QUERYBUILDER_H
