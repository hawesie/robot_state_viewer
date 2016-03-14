#ifndef MONGODBCXXINTERFACE_H
#define MONGODBCXXINTERFACE_H

#include <QString>
#include <QStringList>
#include <QDebug>
#include "mongocxx/v_noabi/mongocxx/client.hpp"
#include <mongocxx/uri.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>

#include <mongocxx/instance.hpp>

class MongoDBCXXInterface
{

public:
    MongoDBCXXInterface();
    MongoDBCXXInterface(std::string hostaddress, std::string port, std::string database, std::string collectionname);
    int getMaxTimeStep();
    int getMinTimeStep();
private:
    std::string hostaddress;
    std::string port;
   // auto collection;
    std::string collectionname;
    std::string database;

};

#endif // MONGODBCXXINTERFACE_H
