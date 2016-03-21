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

    void  handleSOMA2ObjectLabels(std::vector<std::string>);

    void  handleSOMA2ROINames(std::vector<SOMA2ROINameID>);

    void on_queryButton_clicked();

    void on_labelequalsCBox_clicked(bool checked);

    void on_labelcontainsCBox_clicked(bool checked);

    void on_resetqueryButton_clicked();

    void on_exportjsonButton_clicked();



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
