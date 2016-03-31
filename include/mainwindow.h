#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QThread>
#include <QFile>
#include <QDir>
#include "rosThread.h"
//#include "mongodbcxxinterface.h"
#include "querybuilder.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    RosThread rosthread;
    ~MainWindow();

    void setMongoDBHostName(std::string hostname);
    void setMongoDBPort(std::string port);

signals:
    void sliderValue(int val);
public slots:
    void handleMapInfoReceived();

private slots:
    void on_timestepSlider_valueChanged(int value);

    void on_roiComboBox_currentIndexChanged(const QString &arg1);

    void  handleSOMA2ObjectTypes(std::vector<std::string> typenames);

    void  handleSOMA2ROINames(std::vector<SOMA2ROINameID>);

    void on_queryButton_clicked();

    void on_resetqueryButton_clicked();

    void on_exportjsonButton_clicked();



    void on_sliderCBox_clicked(bool checked);


    void on_upperDateCBox_clicked(bool checked);

    void on_lowerDateCBox_clicked(bool checked);

private:
    Ui::MainWindow *ui;

    QThread* thread;
    int maxtimestep;
    int mintimestep;
    std::vector<SOMA2ROINameID> roinameids;
    mongo::BSONObj mainBSONObj;
    QString lastqueryjson;
   // std::string mongodbhost;
   // std::string mongodbport;
};

#endif // MAINWINDOW_H
