#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>


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

  rosthread.worldstate =  rosthread.getSOMA2ObjectCloudsWithTimestep(value-1);

  rosthread.publishSOMA2ObjectClouds(rosthread.worldstate);

  std::string date = rosthread.getSOMA2ObjectDateWithTimestep(value-1);

  ui->datelabel->setText(QString::fromStdString(date));
}

void MainWindow::handleMapInfoReceived()
{
    ui->timestepSlider->setEnabled(true);

    std::string map_name = rosthread.getMapName();

    ui->mapnamelabel->setText(QString::fromStdString(map_name));

    connect(this,SIGNAL(sliderValue(int)),&rosthread,SLOT(getSliderValue(int)));

    MongoDBCXXInterface mongointerface("localhost","62345","labelled_objects","soma2");

    this->maxtimestep = mongointerface.getMaxTimeStep();

    QString labeltext =  "1 / ";

    labeltext.append(QString::number(this->maxtimestep+1));

    ui->timesteplabel->setText(labeltext);

    ui->timestepSlider->setMaximum(maxtimestep+1);
    ui->timestepSlider->setMinimum(1);

    rosthread.worldstate =  rosthread.getSOMA2ObjectCloudsWithTimestep(0);

    rosthread.publishSOMA2ObjectClouds(rosthread.worldstate);

    std::string date = rosthread.getSOMA2ObjectDateWithTimestep(0);

    ui->datelabel->setText(QString::fromStdString(date));

   // std::vector<SOMA2ROINameID> roinameids =  rosthread.getSOMA2ROINames();

   // std::vector<std::string> labelnames = rosthread.getSOMA2ObjectLabels();




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

    rosthread.drawROIwithID("-1");

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
    int index = ui->roiComboBox->currentIndex();

    if(index > 0){

        qDebug()<<"Current Index"<<index<<this->roinameids.size();

        QString roiindex = QString::fromStdString(this->roinameids[index-1].id);

        soma2_msgs::SOMA2ROIObject obj =  rosthread.getSOMA2ROIwithID(roiindex.toInt());

        qDebug()<<"ROI Index"<<roiindex;


        QueryBuilder builder;

        mongo::BSONObj queryjson = builder.buildROIQuery(obj);

        rosthread.querySOMA2Objects(queryjson);

    }
}
