#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDialog>
#include <QTextBrowser>
#include <QStringListModel>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->mapnamelabel->setText("DB Fetch is in progress...");

    this->setWindowTitle("Robot State Viewer");

    // this->mongodbhost = "localhost";
    // this->mongodbport = "62345";

    ui->tab->setEnabled(false);

    thread = new QThread(this);

    rosthread.moveToThread(thread);

    connect(thread, SIGNAL(started()),&rosthread,SLOT(loop()));

    connect(&rosthread,SIGNAL(mapinfoReceived()),this,SLOT(handleMapInfoReceived()));

    connect(&rosthread,SIGNAL(SOMA2ROINames(std::vector<SOMA2ROINameID>)),this,SLOT(handleSOMA2ROINames(std::vector<SOMA2ROINameID>)));

    connect(&rosthread,SIGNAL(SOMA2ObjectTypes(std::vector<std::string>)),this,SLOT(handleSOMA2ObjectTypes(std::vector<std::string>)));


    connect(&rosthread,SIGNAL(rosFinished()),this,SLOT(close()));

    this->thread->start();

    // We will wait for the map information
    ui->timestepSlider->setEnabled(false);


    // qDebug()<<map["timestep"];

}

MainWindow::~MainWindow()
{
    rosthread.shutdownROS();
    thread->quit();
    thread->wait();
    delete ui;
}


void MainWindow::on_timestepSlider_valueChanged(int value)
{
    // emit sliderValue(value);

    // If slider is Enabled
    if(ui->sliderCBox->isChecked()){

        QString labeltext =  QString::number(value);

        labeltext.append(" / ").append(QString::number(maxtimestep+1));

        ui->timesteplabel->setText(labeltext);

        std::string date = rosthread.getSOMA2ObjectDateWithTimestep(value-1);

        ui->datelabel->setText(QString::fromStdString(date));


        // bool lowerdate = ui->lowerDateCBox->isChecked();

        // bool upperdate = ui->upperDateCBox->isChecked();

        /*    if(lowerdate || upperdate)
        {
            std::vector<soma2_msgs::SOMA2Object> soma2objects =  rosthread.querySOMA2ObjectsWithDate(this->mainBSONObj);

            ui->noretrievedobjectslabel->setText(QString::number(soma2objects.size()));

            sensor_msgs::PointCloud2 state =  rosthread.getSOMA2CombinedObjectCloud(soma2objects);

            rosthread.publishSOMA2ObjectCloud(state);

            lastqueryjson = QString::fromStdString(this->mainBSONObj.jsonString(mongo::TenGen,0 ));
            //Reset the bson obj
            mongo::BSONObjBuilder mainbuilder;

            this->mainBSONObj = mainbuilder.obj();

            return;

        }*/


        mongo::BSONObjBuilder builder;


        builder.appendElements(this->mainBSONObj);

        mongo::BSONObj timestepobj = QueryBuilder::buildSOMA2TimestepQuery(value-1);

        builder.appendElements(timestepobj);
        //builder.append("timestep",value-1);

        mongo::BSONObj tempObject = builder.obj();

        std::vector< soma2_msgs::SOMA2Object > soma2objects =  rosthread.querySOMA2Objects(tempObject);

        ui->noretrievedobjectslabel->setText(QString::number(soma2objects.size()));

        sensor_msgs::PointCloud2 state =  rosthread.getSOMA2CombinedObjectCloud(soma2objects);

        rosthread.publishSOMA2ObjectCloud(state);


        lastqueryjson = QString::fromStdString(tempObject.jsonString());
        //Reset the bson obj
        // mongo::BSONObjBuilder mainbuilder;

        // this->mainBSONObj = mainbuilder.obj();

    }
}

void MainWindow::handleMapInfoReceived()
{
    ui->tab->setEnabled(true);

    // Enable the slider
    ui->timestepSlider->setEnabled(true);
    ui->sliderCBox->setChecked(true);


    /*************Set Map Name***********************/
    std::string map_name = rosthread.getMapName();

    ui->mapnamelabel->setText(QString::fromStdString(map_name));
    /*************************************************************/


    /***********************Set Timestep Interval *************************************/
    std::string objectsdbname = this->rosthread.getSOMA2ObjectsDBName();

    std::string objectscolname = this->rosthread.getSOMA2ObjectsCollectionName();

    // MongoDBCXXInterface mongointerface(this->mongodbhost,this->mongodbport,objectsdbname,objectscolname);

    std::vector<int> minmax = this->rosthread.getSOMA2CollectionMinMaxTimestep();

    this->mintimestep = minmax[0];
    this->maxtimestep = minmax[1];



    QString labeltext ;
    labeltext.append(QString::number(this->mintimestep+1));
    labeltext.append(" / ");
    // QString labeltext =  "1 / ";

    labeltext.append(QString::number(this->maxtimestep+1));

    ui->timesteplabel->setText(labeltext);

    ui->timestepSlider->setMaximum(maxtimestep+1);
    ui->timestepSlider->setMinimum(this->mintimestep+1);
    /**********************************************************************************/




    /****************************** Set Dates to Min/Max Values **********************/
    std::string date = rosthread.getSOMA2ObjectDateWithTimestep(this->mintimestep);

    QDateTime qdate = QDateTime::fromString(QString::fromStdString(date),Qt::ISODate);

    ui->lowerDateEdit->setDate(qdate.date());

    ui->datelabel->setText(QString::fromStdString(date));

    date = rosthread.getSOMA2ObjectDateWithTimestep(maxtimestep);

    qdate = QDateTime::fromString(QString::fromStdString(date),Qt::ISODate);

    ui->upperDateEdit->setDate(qdate.date());
    /*********************************************************************************/

    /********************** Prepare the Weekdays ComboBox ****************************/
    QStringList weekdays;
    weekdays.push_back("");
    weekdays.push_back("Monday");
    weekdays.push_back("Tuesday");
    weekdays.push_back("Wednesday");
    weekdays.push_back("Thursday");
    weekdays.push_back("Friday");
    weekdays.push_back("Saturday");
    weekdays.push_back("Sunday");

    ui->weekdaysComboBox->addItems(weekdays);
    /************************************************************************************/

    // Clear any remaining ROI's in the RVIZ
    rosthread.drawROIwithID("-1");


    /********************* Publish Objects at world state t = 0 ***********************/

    //  std::vector<std::string>  res =  RosThread::fetchSOMA2ObjectLabels();

    /*  std::vector<soma2_msgs::SOMA2Object > soma2objects =  rosthread.querySOMA2Objects(this->mainBSONObj,0);

    ui->noretrievedobjectslabel->setText(QString::number(soma2objects.size()));

    sensor_msgs::PointCloud2 state =  rosthread.getSOMA2CombinedObjectCloud(soma2objects);

    rosthread.publishSOMA2ObjectCloud(state);*/


    /**********************************************************************************/


}
void MainWindow::handleSOMA2ObjectTypes(std::vector<std::string> typenames)
{
    QString dir = QDir::homePath();

    dir.append("/").append(".soma2").append("/objecttypes.txt");

    QFile file(dir);

    if(file.open(QFile::ReadOnly))
    {

        QTextStream stream(&file);


        QStringListModel *model = new QStringListModel(this);

        QStringList list;

        // ui->listViewObjectTypes->setmo

        while(!stream.atEnd())
        {
            QString str = stream.readLine();

            list<<str;

            qDebug()<<str;

            //ui->labelsComboBox->addItem(str);

        }

        model->setStringList(list);

        ui->listViewObjectTypes->setModel(model);
        ui->listViewObjectTypes->setSelectionMode(QAbstractItemView::MultiSelection);
        ui->listViewObjectTypes->setEditTriggers(QAbstractItemView::NoEditTriggers);

        file.close();

    }



    dir = QDir::homePath();

    dir.append("/").append(".soma2").append("/objectids.txt");

    QFile file2(dir);

    if(file2.open(QFile::ReadOnly))
    {

        QTextStream stream(&file2);


        QStringListModel *model = new QStringListModel(this);

        QStringList list;

        // ui->listViewObjectTypes->setmo

        while(!stream.atEnd())
        {
            QString str = stream.readLine();

            list<<str;

            qDebug()<<str;

            //ui->labelsComboBox->addItem(str);

        }

        model->setStringList(list);

        ui->listViewObjectIDs->setModel(model);
        ui->listViewObjectIDs->setSelectionMode(QAbstractItemView::MultiSelection);
        ui->listViewObjectIDs->setEditTriggers(QAbstractItemView::NoEditTriggers);


        file2.close();

    }




}
void MainWindow::handleSOMA2ROINames(std::vector<SOMA2ROINameID> roinameids)
{
    qDebug()<<"ROI Info Received";

    this->roinameids = roinameids;

    ui->roiComboBox->addItem("");

    for(int i = 0; i < roinameids.size();i++)
    {
        QString str;
        str = QString::fromStdString(roinameids[i].name);

        str.append(" ").append(QString::fromStdString(roinameids[i].id));

        ui->roiComboBox->addItem(str);

    }


}

void MainWindow::on_roiComboBox_currentIndexChanged(const QString &arg1)
{
    QString str = arg1;
    QStringList numpart = str.split(" ");

    if(numpart.size()>=2){
        // qDebug()<<numpart[1];
        rosthread.drawROIwithID(numpart[1].toStdString());
    }
    else
        rosthread.drawROIwithID("-1");
}

void MainWindow::on_queryButton_clicked()
{
    int roiintindex = ui->roiComboBox->currentIndex();

    int weekdayindex = ui->weekdaysComboBox->currentIndex();


    bool idequals = ui->listViewIDCBox->isChecked();

    bool typeequals = ui->listViewObjectTypesCBox->isChecked();
    /***********************************************************************/

    bool slideractive = ui->sliderCBox->isChecked();

    bool lowertime = ui->lowerTimeCBox->isChecked();

    bool uppertime = ui->upperTimeCBox->isChecked();

    bool lowerdatecbox = ui->lowerDateCBox->isChecked();

    bool upperdatecbox = ui->upperDateCBox->isChecked();

    mongo::BSONObjBuilder mainbuilder;

    if(lowerdatecbox || upperdatecbox)
    {
        // ui->timestepSlider->setEnabled(false);

        QDateTime datetime;
        datetime.setDate(ui->lowerDateEdit->date());

        QTime time;

        time.setHMS(0,0,0);
        datetime.setTime(time);

        ulong lowerdate =  datetime.toMSecsSinceEpoch();


        datetime.setDate(ui->upperDateEdit->date());
        time.setHMS(23,59,0);
        datetime.setTime(time);

        ulong upperdate = datetime.toMSecsSinceEpoch();

        int mode = 0;

        if(lowerdatecbox &&  upperdatecbox)
        {
            mode = 2;

        }
        else if(upperdatecbox)
            mode = 1;

        mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2DateQuery(lowerdate,upperdate,mode);

        mainbuilder.appendElements(bsonobj);


    }

    if(lowertime || uppertime)
    {


        int lowhour = ui->lowerTimeEdit->time().hour();

        int lowmin = ui->lowerTimeEdit->time().minute();

        int upphour = ui->upperTimeEdit->time().hour();

        int uppmin = ui->upperTimeEdit->time().minute();

        int mode = 0;
        if(lowertime && uppertime)
            mode = 2;
        else if(uppertime)
            mode=1;

        mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2TimeQuery(lowhour,lowmin,upphour,uppmin,mode);

        mainbuilder.appendElements(bsonobj);

    }


    if(weekdayindex > 0)
    {

        mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2WeekdayQuery(weekdayindex-1);


        mainbuilder.appendElements(bsonobj);
    }


    if(typeequals || idequals)
    {
        if(typeequals && idequals)
        {
            QModelIndexList indexlist = ui->listViewObjectIDs->selectionModel()->selectedIndexes();

            std::vector<std::string> list;

            std::vector<std::string> fieldnames;
            std::vector<int> objectIndexes;
            fieldnames.push_back("id");
            fieldnames.push_back("type");




            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }

            objectIndexes.push_back(indexlist.size());


            indexlist = ui->listViewObjectTypes->selectionModel()->selectedIndexes();


            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }


            objectIndexes.push_back(indexlist.size());

            mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(list,fieldnames,objectIndexes,"$or");

             mainbuilder.appendElements(bsonobj);

        }
        else if(typeequals){
            QModelIndexList indexlist = ui->listViewObjectTypes->selectionModel()->selectedIndexes();

            std::vector<std::string> list;

            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }

            // std::string typename = ui->labelsComboBox->currentText().toStdString();
            std::vector<std::string> fieldnames;
            fieldnames.push_back("type");
            std::vector<int> objectIndexes;
            objectIndexes.push_back(list.size());
            mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(list,fieldnames,objectIndexes,"$or");

            mainbuilder.appendElements(bsonobj);
        }
        else if(idequals)
        {
            QModelIndexList indexlist = ui->listViewObjectIDs->selectionModel()->selectedIndexes();

            std::vector<std::string> list;

            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }

            // std::string typename = ui->labelsComboBox->currentText().toStdString();

            std::vector<std::string> fieldnames;
            fieldnames.push_back("id");
            std::vector<int> objectIndexes;
            objectIndexes.push_back(list.size());
            mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2StringArrayBasedQuery(list,fieldnames,objectIndexes,"$or");



            mainbuilder.appendElements(bsonobj);
        }

    }



    if(roiintindex > 0){

        //  qDebug()<<"Current Index"<<roiintindex<<this->roinameids.size();

        QString roiindex = QString::fromStdString(this->roinameids[roiintindex-1].id);

        soma2_msgs::SOMA2ROIObject obj =  rosthread.getSOMA2ROIwithID(roiindex.toInt());

        //  qDebug()<<"ROI Index"<<roiindex;


        // QueryBuilder builder;

        mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2ROIWithinQuery(obj);

        mainbuilder.appendElements(bsonobj);



    }

    if(slideractive)
    {

        mongo::BSONObj timestepobj = QueryBuilder::buildSOMA2TimestepQuery(ui->timestepSlider->value()-1);

        mainbuilder.appendElements(timestepobj);

    }

    this->mainBSONObj = mainbuilder.obj();

    mongo::BSONObj tempObject = this->mainBSONObj;

    std::vector< soma2_msgs::SOMA2Object > soma2objects =  rosthread.querySOMA2Objects(tempObject);

    ui->noretrievedobjectslabel->setText(QString::number(soma2objects.size()));

    sensor_msgs::PointCloud2 state =  rosthread.getSOMA2CombinedObjectCloud(soma2objects);

    rosthread.publishSOMA2ObjectCloud(state);


    lastqueryjson = QString::fromStdString(tempObject.jsonString());



}

/*void MainWindow::on_list_clicked(bool checked)
{
    if(checked)
        ui->labelcontainsCBox->setChecked(false);
}

void MainWindow::on_typecontainsCBox_clicked(bool checked)
{
    if(checked)
        ui->labelequalsCBox->setChecked(false);
}*/

// Reset the Query Fields
void MainWindow::on_resetqueryButton_clicked()
{
    ui->roiComboBox->setCurrentIndex(0);

    ui->weekdaysComboBox->setCurrentIndex(0);


    ui->sliderCBox->setChecked(true);


    ui->listViewObjectTypesCBox->setChecked(false);

    ui->listViewIDCBox->setChecked(false);

    ui->lowerTimeCBox->setChecked(false);

    ui->upperTimeCBox->setChecked(false);

    ui->lowerDateCBox->setChecked(false);

    ui->upperDateCBox->setChecked(false);

    ui->listViewObjectTypes->clearSelection();

    ui->listViewObjectIDs->clearSelection();



    //Reset the bson obj
    mongo::BSONObjBuilder mainbuilder;

    this->mainBSONObj = mainbuilder.obj();

    emit ui->timestepSlider->valueChanged(mintimestep+1);
    ui->timestepSlider->setSliderPosition(mintimestep+1);




}

void MainWindow::on_exportjsonButton_clicked()
{
    QDialog* dialog = new QDialog(this);

    dialog->setGeometry(100,100,400,400);

    QTextBrowser* browser = new QTextBrowser(dialog);

    browser->setGeometry(0,0,dialog->width(),dialog->height());

    browser->setReadOnly(true);


    /******* Add "new" before Date word for allowing Date queries to be executed directly in RoboMongo ***/

    int index = lastqueryjson.indexOf("Date");

    int lastindex = lastqueryjson.indexOf("Date",index+4);

    qDebug()<<lastindex;

    if(lastindex > 0)
        lastindex  = lastindex+6;

    if(index > 0 )
    {
        lastqueryjson = lastqueryjson.insert(index-1," new ");

    }
    if(lastindex > 0)
    {
        lastqueryjson = lastqueryjson.insert(lastindex-1," new ");
    }

    /**********************************************************************************************************/

    browser->setText(lastqueryjson);

    dialog->show();

    dialog->setWindowTitle("Query JSON");

}

void MainWindow::on_sliderCBox_clicked(bool checked)
{
    if(checked){

        ui->lowerDateCBox->setChecked(false);
        ui->upperDateCBox->setChecked(false);
    }

}



void MainWindow::on_upperDateCBox_clicked(bool checked)
{
    if(checked)
        ui->sliderCBox->setChecked(false);

}

void MainWindow::on_lowerDateCBox_clicked(bool checked)
{
    if(checked)
        ui->sliderCBox->setChecked(false);

}
