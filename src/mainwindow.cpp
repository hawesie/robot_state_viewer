#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDialog>
#include <QTextBrowser>


#include <qjson/parser.h>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    thread = new QThread(this);

    rosthread.moveToThread(thread);

    connect(thread, SIGNAL(started()),&rosthread,SLOT(loop()));

    connect(&rosthread,SIGNAL(mapinfoReceived()),this,SLOT(handleMapInfoReceived()));

    connect(&rosthread,SIGNAL(SOMA2ROINames(std::vector<SOMA2ROINameID>)),this,SLOT(handleSOMA2ROINames(std::vector<SOMA2ROINameID>)));

    connect(&rosthread,SIGNAL(SOMA2ObjectLabels(std::vector<std::string>)),this,SLOT(handleSOMA2ObjectLabels(std::vector<std::string>)));


    // connect(&rosthread,SIGNAL(rosFinished()),thread,SLOT(quit()));

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

    QString labeltext =  QString::number(value);

    labeltext.append(" / ").append(QString::number(maxtimestep+1));

    ui->timesteplabel->setText(labeltext);

    std::string date = rosthread.getSOMA2ObjectDateWithTimestep(value-1);

    ui->datelabel->setText(QString::fromStdString(date));

    /*if(this->mainBSONObj.isEmpty()){

        rosthread.worldstate =  rosthread.getSOMA2ObjectCloudsWithTimestep(value-1);

        rosthread.publishSOMA2ObjectClouds(rosthread.worldstate);

        return;
    }*/
 //   else
   // {

        bool lowerdate = ui->lowerDateCBox->isChecked();

        bool upperdate = ui->upperDateCBox->isChecked();

        if(lowerdate || upperdate)
        {
            std::vector<soma2_msgs::SOMA2Object> soma2objects =  rosthread.querySOMA2ObjectsWithDate(this->mainBSONObj);

            ui->noretrievedobjectslabel->setText(QString::number(soma2objects.size()));

            sensor_msgs::PointCloud2 state =  rosthread.getSOMA2ObjectClouds(soma2objects);

            rosthread.publishSOMA2ObjectClouds(state);

            return;

        }



        std::vector<soma2_msgs::SOMA2Object> soma2objects =  rosthread.querySOMA2Objects(this->mainBSONObj,value-1);

        ui->noretrievedobjectslabel->setText(QString::number(soma2objects.size()));

        sensor_msgs::PointCloud2 state =  rosthread.getSOMA2ObjectClouds(soma2objects);

        rosthread.publishSOMA2ObjectClouds(state);





  //  }




}

void MainWindow::handleMapInfoReceived()
{

    // Enable the slider
    ui->timestepSlider->setEnabled(true);
    connect(this,SIGNAL(sliderValue(int)),&rosthread,SLOT(getSliderValue(int)));


    /*************Set Map Name***********************/
    std::string map_name = rosthread.getMapName();

    ui->mapnamelabel->setText(QString::fromStdString(map_name));
    /*************************************************************/


    /***********************Set Timestep Interval *************************************/
    MongoDBCXXInterface mongointerface("localhost","62345","labelled_objects","soma2");

    this->maxtimestep = mongointerface.getMaxTimeStep();

    QString labeltext =  "1 / ";

    labeltext.append(QString::number(this->maxtimestep+1));

    ui->timesteplabel->setText(labeltext);

    ui->timestepSlider->setMaximum(maxtimestep+1);
    ui->timestepSlider->setMinimum(1);
    /**********************************************************************************/




    /****************************** Set Dates to Min/Max Values **********************/
    std::string date = rosthread.getSOMA2ObjectDateWithTimestep(0);

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



    std::vector<soma2_msgs::SOMA2Object> soma2objects =  rosthread.querySOMA2Objects(this->mainBSONObj,0);

    ui->noretrievedobjectslabel->setText(QString::number(soma2objects.size()));

    sensor_msgs::PointCloud2 state =  rosthread.getSOMA2ObjectClouds(soma2objects);

    rosthread.publishSOMA2ObjectClouds(state);


    /**********************************************************************************/


}
void MainWindow::handleSOMA2ObjectLabels(std::vector<std::string> labelnames)
{
    ui->labelsComboBox->addItem("");

    for(int i=0; i < labelnames.size(); i++)
    {
        ui->labelsComboBox->addItem(QString::fromStdString(labelnames[i]));

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
        qDebug()<<numpart[1];
        rosthread.drawROIwithID(numpart[1].toStdString());
    }
    else
        rosthread.drawROIwithID("-1");
}

void MainWindow::on_queryButton_clicked()
{
    int roiintindex = ui->roiComboBox->currentIndex();

    int weekdayindex = ui->weekdaysComboBox->currentIndex();

    bool labelequals = ui->labelequalsCBox->isChecked();

    bool labelcontains = ui->labelcontainsCBox->isChecked();

    bool lowertime = ui->lowerTimeCBox->isChecked();

    bool uppertime = ui->upperTimeCBox->isChecked();


    bool lowerdate = ui->lowerDateCBox->isChecked();

    bool upperdate = ui->upperDateCBox->isChecked();

    mongo::BSONObjBuilder mainbuilder;

    if(lowerdate || upperdate)
    {


       QDateTime datetime;
       datetime.setDate(ui->lowerDateEdit->date());

       QTime time;

       time.setHMS(0,0,0);
       datetime.setTime(time);

        ulong lowerdate =  datetime.toMSecsSinceEpoch();

        datetime;
        datetime.setDate(ui->upperDateEdit->date());
        datetime.setTime(time);

        ulong upperdate = datetime.toMSecsSinceEpoch();

        int mode = 0;

        if(lowerdate &&  upperdate)
        {
            mode = 2;

        }
        else if(upperdate)
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


    if(labelequals)
    {
        std::string labelname = ui->labelsComboBox->currentText().toStdString();
        mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2LabelEqualsQuery(labelname);

        mainbuilder.appendElements(bsonobj);
    }
    else if(labelcontains)
    {
        std::string text = ui->labelcontainsLEdit->text().toStdString();
        mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2LabelContainsQuery(text);

        mainbuilder.appendElements(bsonobj);
    }


    if(roiintindex > 0){

        qDebug()<<"Current Index"<<roiintindex<<this->roinameids.size();

        QString roiindex = QString::fromStdString(this->roinameids[roiintindex-1].id);

        soma2_msgs::SOMA2ROIObject obj =  rosthread.getSOMA2ROIwithID(roiindex.toInt());

        qDebug()<<"ROI Index"<<roiindex;


        // QueryBuilder builder;

        mongo::BSONObj bsonobj = QueryBuilder::buildSOMA2ROIWithinQuery(obj);

        mainbuilder.appendElements(bsonobj);



    }

    this->mainBSONObj = mainbuilder.obj();

    if(ui->timestepSlider->value() != 1)
        ui->timestepSlider->setValue(1);
    else
        emit ui->timestepSlider->valueChanged(1);
  //  this->on_timestepSlider_valueChanged(1);



    /*  std::vector<soma2_msgs::SOMA2Object> soma2objects =  rosthread.querySOMA2Objects(this->mainBSONObj,0);

    if(soma2objects.size() > 0)
    {

        rosthread.worldstate =  rosthread.getSOMA2ObjectClouds(soma2objects);

        rosthread.publishSOMA2ObjectClouds(rosthread.worldstate);


    }*/
}

void MainWindow::on_labelequalsCBox_clicked(bool checked)
{
    if(checked)
        ui->labelcontainsCBox->setChecked(false);
}

void MainWindow::on_labelcontainsCBox_clicked(bool checked)
{
    if(checked)
        ui->labelequalsCBox->setChecked(false);
}

// Reset the Query Fields
void MainWindow::on_resetqueryButton_clicked()
{
    ui->roiComboBox->setCurrentIndex(0);

    ui->weekdaysComboBox->setCurrentIndex(0);

    ui->labelsComboBox->setCurrentIndex(0);

    ui->labelequalsCBox->setChecked(false);

    ui->labelcontainsCBox->setChecked(false);

    ui->lowerTimeCBox->setChecked(false);

    ui->upperTimeCBox->setChecked(false);

    ui->lowerDateCBox->setChecked(false);

    ui->upperDateCBox->setChecked(false);

    ui->labelcontainsLEdit->setText("");


    emit ui->timestepSlider->valueChanged(1);
    ui->timestepSlider->setSliderPosition(1);




}

void MainWindow::on_exportjsonButton_clicked()
{
    QDialog* dialog = new QDialog(this);

    dialog->setGeometry(100,100,400,400);

    QTextBrowser* browser = new QTextBrowser(dialog);

    browser->setGeometry(0,0,dialog->width(),dialog->height());

    dialog->show();

    dialog->setWindowTitle("Query JSON");

}
