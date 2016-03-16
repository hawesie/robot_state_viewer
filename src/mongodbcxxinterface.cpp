#include "mongodbcxxinterface.h"
#include <sstream>
#include <iostream>
#include <mongocxx/exception/query_exception.hpp>
MongoDBCXXInterface::MongoDBCXXInterface()
{

}
MongoDBCXXInterface::MongoDBCXXInterface(std::string hostaddress, std::string port, std::string database, std::string collectionname)
{

    std::stringstream ss;
    ss<<"mongodb://"<<hostaddress<<":"<<port;

    this->hostaddress = hostaddress;
    this->port = port;

    mongocxx::uri uri{ss.str()};

    mongocxx::client conn(uri);


    /*  auto collection = conn[database][collectionname];*/

    this->collectionname = collectionname;
    this->database = database;


}

int MongoDBCXXInterface::getMaxTimeStep()
{

    std::stringstream ss;
    ss<<"mongodb://"<<this->hostaddress<<":"<<this->port;

    mongocxx::uri uri{ss.str()};

    mongocxx::client conn(uri);


    auto collection = conn[this->database][this->collectionname];


    if(collection){

        mongocxx::options::find opts;

        bsoncxx::builder::stream::document order_builder;

        order_builder << "_id" << -1;

        opts.sort(order_builder.view());

        opts.limit(1);

        try{

            auto cursor = collection.find({},opts);


            QString timestepint ;

            for (auto&& doc : cursor) {

                auto val = doc["type"];

                //std::cout<<bsoncxx::to_json(doc) << std::endl;<<std::endl;



                //  qDebug()<< QString::fromStdString(bsoncxx::to_json(doc));
                QString result = QString::fromStdString(bsoncxx::to_json(doc));
                QStringList resultlines = result.split("\n");
                QStringList timestepline = resultlines.filter("timestep");
                timestepint = timestepline[0].section(":",1,1);

                qDebug()<<"Max timestep for the collection named"<<QString::fromStdString(this->collectionname)<<timestepint.remove(QRegExp("[\",]")).toInt();


                //  QJsonParseError err;
                // QJsonDocument d = QJsonDocument::fromJson(QString::fromStdString(bsoncxx::to_json(doc)).toUtf8(),&err);
                // qWarning() << d.isNull();
                // qDebug()<<err.errorString();
            }

            return timestepint.remove(QRegExp("[\",]")).toInt();

        }
        catch(mongocxx::v_noabi::query_exception e)
        {
            std::cout<<"Exception occured while querying db with host address "<<this->hostaddress<<" port "<<this->port<<"\n";
            return -1;

        }


    }
    else
    {
        std::cout<<"Cannot connect to DB "<<this->hostaddress<<" "<<this->port<<"\n";
        return -1;

    }

}
int MongoDBCXXInterface::getMinTimeStep()
{

    std::stringstream ss;
    ss<<"mongodb://"<<this->hostaddress<<":"<<this->port;

    mongocxx::uri uri{ss.str()};

    mongocxx::client conn(uri);


    auto collection = conn[this->database][this->collectionname];

    if(collection){

        mongocxx::options::find opts;

        bsoncxx::builder::stream::document order_builder;

        //  order_builder << "_id" << -1;

        //  opts.sort(order_builder.view());

        opts.limit(1);


        try{
            auto cursor = collection.find({},opts);

            QString timestepint ;

            for (auto&& doc : cursor) {

                auto val = doc["type"];

                //std::cout<<bsoncxx::to_json(doc) << std::endl;<<std::endl;



                //  qDebug()<< QString::fromStdString(bsoncxx::to_json(doc));
                QString result = QString::fromStdString(bsoncxx::to_json(doc));
                QStringList resultlines = result.split("\n");
                QStringList timestepline = resultlines.filter("timestep");
                timestepint = timestepline[0].section(":",1,1);

                qDebug()<<"Min timestep for the collection named"<<QString::fromStdString(this->collectionname)<<timestepint.remove(QRegExp("[\",]")).toInt();


                //  QJsonParseError err;
                // QJsonDocument d = QJsonDocument::fromJson(QString::fromStdString(bsoncxx::to_json(doc)).toUtf8(),&err);
                // qWarning() << d.isNull();
                // qDebug()<<err.errorString();
            }

            return timestepint.remove(QRegExp("[\",]")).toInt();
        }
        catch(mongocxx::v_noabi::query_exception e)
        {
            std::cout<<"Exception occured while querying db with host address "<<this->hostaddress<<" port "<<this->port<<"\n";
            return -1;

        }

    }
    else
    {
        std::cout<<"Cannot connect to DB "<<this->hostaddress<<" "<<this->port<<"\n";
        return -1;

    }
}
