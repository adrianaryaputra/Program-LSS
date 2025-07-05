//
// Created by azindait on 08/02/20.
//

#ifndef RVIZ_PLUGIN_CAMERA_PLUGIN_H
#define RVIZ_PLUGIN_CAMERA_PLUGIN_H


# include <ros/ros.h>
#include <QLabel>
#include <QPoint>
#include <Qt>
#include <QMouseEvent>
#include <QPixmap>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QMutex>

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QFileDialog>
#include <QString>

# include <rviz/panel.h>

#include "imgreceiver.h"
#include "imagelabel.h"
#include "ui_camera_resp.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>

#include <rviz_plugin/BoatsideService.h>
#include <rviz_plugin/FloatService.h>
#include <rviz_plugin/StringService.h>



namespace Ui{
    class cameraResponsive;
}

class CameraPanel: public rviz::Panel{
Q_OBJECT

public:
    CameraPanel(QWidget* parent = 0);
    ImgReceiver* img_receiver;

public Q_SLOTS:
//    receivedImage(QImage)
    //Boatside view
    void setLeftBoat();
    void setRightBoat();
    void setLeftBoat2();
    void setRightBoat2();
    void setCritLine();
    // void setCritLine_2();
    void changeCritLine(int, int);

    void missionChanged(int curr_miss);



Q_SIGNALS:
    //Boatside view
    void critLineChanged(int, int);
    void boatSideChanged(QPoint**);
    void boatSideChanged2(QPoint**);

protected:
    Ui::cameraResponsive *ui_;
//    ImageLabel* cameraFrame;
//    QLineEdit* topic;
//    QString output_topic_string;
//    QPushButton* lbs1_push_button;
//    QPushButton* rbs1_push_button;
//    QPushButton* cl_push_button;
//    QPushButton* publish_push_button;
//    QMutex mtx_img;

private:
    ros::NodeHandle nh;
    ros::Publisher critline_pub, boatside_pub;
    ros::ServiceServer load_param_serv;

    int image_width, image_height;

    int crit_lines[7];
    int horizon[7];

    int current_mission;

    /* 0:left top, 1:left bottom, 2:right top, 3:right bottom */
    QPoint boatSide[4];
    QPoint boatSide_all[28];
//    QPoint boatSide2[4];

    QMutex mtx_img;
    void initUi();
    void connectWidget();

    void savePublishCurrentMission();
    QPoint normalizePoint(QPoint);

    bool loadFileService(rviz_plugin::StringService::Request  &req,
         rviz_plugin::StringService::Response &res);
    void loadFile(std::string filename_);

};


#endif //RVIZ_PLUGIN_CAMERA_PLUGIN_H
