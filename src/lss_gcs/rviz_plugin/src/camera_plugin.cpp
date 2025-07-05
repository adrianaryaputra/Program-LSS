#include <camera_plugin.h>
#include <thread>
#include <iostream>

CameraPanel::CameraPanel(QWidget *parent) : rviz::Panel(parent), ui_(new Ui::cameraResponsive())
{

    ui_->setupUi(this);
    img_receiver = new ImgReceiver();

    std::cout << "mari connect" << std::endl;

    image_height = 480;
    image_width = 640;

    connectWidget();
    initUi();

    critline_pub = nh.advertise<std_msgs::Int32MultiArray>("/camera/critline", 1);
    boatside_pub = nh.advertise<geometry_msgs::PoseArray>("/camera/boatside", 1);
    load_param_serv = nh.advertiseService("rviz_plugin/camera_load_file", &CameraPanel::loadFileService, (CameraPanel *)this);

    std::cout << "camera plugin" << std::endl;

    current_mission = 0;
    missionChanged(current_mission + 1);

    // reset all var
    for (int i = 0; i < 7; i++)
    {
        crit_lines[i] = 240;
        horizon[i] = 240;
        boatSide_all->setX(0);
        boatSide_all->setY(0);
    }

    std::string filename_to_load = "/home/azidanit/irc2022_ws/src/irc2022/contoh_savefile.json";
    // loadFile(filename_to_load);
    // ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
}

bool CameraPanel::loadFileService(rviz_plugin::StringService::Request &req,
                                  rviz_plugin::StringService::Response &res)
{
    res.status = "ok";
    std::cout << "LOADING FROM SERVICE " << req.data << std::endl;
    loadFile(req.data);
    return true;
}

void CameraPanel::initUi()
{
    Q_EMIT boatSideChanged(ui_->camera->getBoatSide());
    Q_EMIT boatSideChanged2(ui_->camera->getBoatSide2());
    Q_EMIT critLineChanged(0, ui_->camera->getCritLine(0));
    Q_EMIT critLineChanged(1, ui_->camera->getCritLine(1));
}

void CameraPanel::connectWidget()
{
    // Boat Side and CritLine
    connect(ui_->buttonLeftBoat, &QPushButton::clicked, this, &CameraPanel::setLeftBoat);
    connect(ui_->buttonRightBoat, &QPushButton::clicked, this, &CameraPanel::setRightBoat);
    connect(ui_->buttonLeftBoat_2, &QPushButton::clicked, this, &CameraPanel::setLeftBoat2);
    connect(ui_->buttonRightBoat_2, &QPushButton::clicked, this, &CameraPanel::setRightBoat2);
    connect(ui_->buttonCritLine, &QPushButton::clicked, this, &CameraPanel::setCritLine);
    connect(ui_->spinBoxMission, SIGNAL(valueChanged(int)), this, SLOT(missionChanged(int)));

    // connect(ui_->buttonCritLine_2, &QPushButton::clicked, this, &CameraPanel::setCritLine_2);

    connect(
        img_receiver, &ImgReceiver::imageReceived,
        this,
        [=](QImage image)
        {
            mtx_img.lock();
            ui_->camera->drawPixmap(image);
            //            delete image;
            mtx_img.unlock();
        },
        Qt::DirectConnection);
    //    img_receiver->start();
}

void CameraPanel::setLeftBoat()
{
    if (ui_->buttonLeftBoat->text() == "L")
    {
        ui_->camera->changeSetMode(3);
        ui_->buttonLeftBoat->setText("Done");
        ui_->buttonRightBoat->setDisabled(true);
    }
    else
    {
        ui_->camera->changeSetMode(0);
        ui_->buttonLeftBoat->setText("L");
        ui_->buttonRightBoat->setDisabled(false);
        Q_EMIT boatSideChanged(ui_->camera->getBoatSide());

        boatSide[0] = *(ui_->camera->getBoatSide())[0];
        boatSide[1] = *(ui_->camera->getBoatSide())[1];
        boatSide[2] = *(ui_->camera->getBoatSide())[2];
        boatSide[3] = *(ui_->camera->getBoatSide())[3];

        geometry_msgs::PoseArray msg_boatside;
        msg_boatside.header.stamp = ros::Time::now();
        for (int i = 0; i < 4; i++)
        {
            geometry_msgs::Pose pose_;
            pose_.position.x = boatSide[i].x();
            pose_.position.y = boatSide[i].y();
            msg_boatside.poses.push_back(pose_);
        }

        boatside_pub.publish(msg_boatside);

        savePublishCurrentMission();
    }
}

void CameraPanel::setRightBoat()
{ // USE THIS TO PUBLISH TOPIC AND CALL SERVICE
    if (ui_->buttonRightBoat->text() == "R")
    {
        ui_->camera->changeSetMode(4);
        ui_->buttonRightBoat->setText("Done");
        ui_->buttonLeftBoat->setDisabled(true);
    }
    else
    {
        ui_->camera->changeSetMode(0);
        ui_->buttonRightBoat->setText("R");
        ui_->buttonLeftBoat->setDisabled(false);
        Q_EMIT boatSideChanged(ui_->camera->getBoatSide());

        boatSide[0] = *(ui_->camera->getBoatSide())[0];
        boatSide[1] = *(ui_->camera->getBoatSide())[1];
        boatSide[2] = *(ui_->camera->getBoatSide())[2];
        boatSide[3] = *(ui_->camera->getBoatSide())[3];

        geometry_msgs::PoseArray msg_boatside;
        msg_boatside.header.stamp = ros::Time::now();
        for (int i = 0; i < 4; i++)
        {
            geometry_msgs::Pose pose_;
            pose_.position.x = boatSide[i].x();
            pose_.position.y = boatSide[i].y();
            msg_boatside.poses.push_back(pose_);
        }

        boatside_pub.publish(msg_boatside);

        savePublishCurrentMission();
    }
}

QPoint CameraPanel::normalizePoint(QPoint point_)
{
    QPoint norm_pon;
    norm_pon.setX(point_.x() / image_width);
    norm_pon.setY(point_.y() / image_height);
    return norm_pon;
}

void CameraPanel::savePublishCurrentMission()
{
    boatSide_all[current_mission * 4] = *(ui_->camera->getBoatSide())[0];
    boatSide_all[current_mission * 4 + 1] = *(ui_->camera->getBoatSide())[1];
    boatSide_all[current_mission * 4 + 2] = *(ui_->camera->getBoatSide())[2];
    boatSide_all[current_mission * 4 + 3] = *(ui_->camera->getBoatSide())[3];

    crit_lines[current_mission] = ui_->camera->getCritLine(1);
    horizon[current_mission] = ui_->camera->getCritLine(0);

    rviz_plugin::BoatsideService boatside_ser;
    boatside_ser.request.back_camera_x = (float)boatSide_all[current_mission * 4 + 3].x() / image_width;
    boatside_ser.request.back_camera_y = (float)boatSide_all[current_mission * 4 + 3].y() / image_height;
    boatside_ser.request.front_camera_x = (float)boatSide_all[current_mission * 4 + 2].x() / image_width;
    boatside_ser.request.front_camera_y = (float)boatSide_all[current_mission * 4 + 2].y() / image_height;

    rviz_plugin::FloatService critline_ser;
    critline_ser.request.data = (float)crit_lines[current_mission] / image_height;

    rviz_plugin::FloatService horizon_ser;
    horizon_ser.request.data = (float)horizon[current_mission] / image_height;

    switch (current_mission)
    {
    case (0):
        if (ros::service::call("/mission/navigationchannel/boatside", boatside_ser))
            std::cout << "GOT RESPONSE navigationchannel boatside " << boatside_ser.response.status << "\n";
        if (ros::service::call("/mission/navigationchannel/critline", critline_ser))
            std::cout << "GOT RESPONSE navigationchannel critline " << critline_ser.response.status << "\n";
        if (ros::service::call("/mission/navigationchannel/obstacleline", horizon_ser))
            std::cout << "GOT RESPONSE navigationchannel obstacleline " << horizon_ser.response.status << "\n";

        break;

    case (1):
        if (ros::service::call("/mission/snackrun/boatside", boatside_ser))
            std::cout << "GOT RESPONSE snackrun boatside " << boatside_ser.response.status << "\n";
        if (ros::service::call("/mission/snackrun/critline", critline_ser))
            std::cout << "GOT RESPONSE snackrun critline " << critline_ser.response.status << "\n";
        if (ros::service::call("/mission/snackrun/obstacleline", horizon_ser))
            std::cout << "GOT RESPONSE snackrun obstacleline " << horizon_ser.response.status << "\n";

        break;

    case (2):
        if (ros::service::call("/mission/avoidthecrouds/boatside", boatside_ser))
            std::cout << "GOT RESPONSE avoidthecrouds boatside " << boatside_ser.response.status << "\n";
        if (ros::service::call("/mission/avoidthecrouds/critline", critline_ser))
            std::cout << "GOT RESPONSE avoidthecrouds critline " << critline_ser.response.status << "\n";
        if (ros::service::call("/mission/avoidthecrouds/obstacleline", horizon_ser))
            std::cout << "GOT RESPONSE avoidthecrouds obstacleline " << horizon_ser.response.status << "\n";

        break;

    case (3):
        if (ros::service::call("/mission/findaseatattheshow/boatside", boatside_ser))
            std::cout << "GOT RESPONSE findaseatattheshow boatside " << boatside_ser.response.status << "\n";
        if (ros::service::call("/mission/findaseatattheshow/critline_upper", critline_ser))
            std::cout << "GOT RESPONSE findaseatattheshow critline_upper " << critline_ser.response.status << "\n";
        if (ros::service::call("/mission/findaseatattheshow/critline_lower", horizon_ser))
            std::cout << "GOT RESPONSE findaseatattheshow critline_lower " << horizon_ser.response.status << "\n";

        break;

    case (4):
        if (ros::service::call("/mission/waterblast/boatside", boatside_ser))
            std::cout << "GOT RESPONSE waterblast boatside " << boatside_ser.response.status << "\n";
        if (ros::service::call("/mission/waterblast/critline_lower", critline_ser))
            std::cout << "GOT RESPONSE waterblast critline_lower " << critline_ser.response.status << "\n";
        // if (ros::service::call("/mission/waterblast/critline_lower", horizon_ser))
        //     std::cout << "GOT RESPONSE waterblast critline_lower " << horizon_ser.response.status << "\n";

        break;

    case (5):
        if (ros::service::call("/mission/skeeball/boatside", boatside_ser))
            std::cout << "GOT RESPONSE skeeball boatside " << boatside_ser.response.status << "\n";
        if (ros::service::call("/mission/skeeball/critline_lower", critline_ser))
            std::cout << "GOT RESPONSE skeeball critline_lower " << critline_ser.response.status << "\n";
        // if (ros::service::call("/mission/skeeball/obstacleline", horizon_ser))
        //     std::cout << "GOT RESPONSE skeeball obstacleline " << horizon_ser.response.status << "\n";

        break;

    case (6):
        if (ros::service::call("/mission/obstacleavoidance/camera/boatside", boatside_ser))
            std::cout << "GOT RESPONSE obstacleavoidance boatside " << boatside_ser.response.status << "\n";
        if (ros::service::call("/mission/obstacleavoidance/camera/critline", critline_ser))
            std::cout << "GOT RESPONSE obstacleavoidance critline " << critline_ser.response.status << "\n";
        if (ros::service::call("/mission/obstacleavoidance/camera/obstacleline", horizon_ser))
            std::cout << "GOT RESPONSE obstacleavoidance obstacleline " << horizon_ser.response.status << "\n";

        break;
    }
}

void CameraPanel::setLeftBoat2()
{
    if (ui_->buttonLeftBoat_2->text() == "L")
    {
        ui_->camera->changeSetMode(5);
        ui_->buttonLeftBoat_2->setText("Done");
        ui_->buttonRightBoat_2->setDisabled(true);
    }
    else
    {
        ui_->camera->changeSetMode(0);
        ui_->buttonLeftBoat_2->setText("L");
        ui_->buttonRightBoat_2->setDisabled(false);
        Q_EMIT boatSideChanged2(ui_->camera->getBoatSide2());
    }
}

void CameraPanel::setRightBoat2()
{
    if (ui_->buttonRightBoat_2->text() == "R")
    {
        ui_->camera->changeSetMode(6);
        ui_->buttonRightBoat_2->setText("Done");
        ui_->buttonLeftBoat_2->setDisabled(true);
    }
    else
    {
        ui_->camera->changeSetMode(0);
        ui_->buttonRightBoat_2->setText("R");
        ui_->buttonLeftBoat_2->setDisabled(false);
        Q_EMIT boatSideChanged2(ui_->camera->getBoatSide2());
    }
}

void CameraPanel::setCritLine()
{
    if (ui_->buttonCritLine->text() == "CritLine")
    {
        ui_->camera->changeSetMode(1);
        ui_->buttonCritLine->setText("Done");
    }
    else
    {
        ui_->camera->changeSetMode(0);
        ui_->buttonCritLine->setText("CritLine");
        Q_EMIT critLineChanged(0, ui_->camera->getCritLine(0));
        Q_EMIT critLineChanged(1, ui_->camera->getCritLine(1));

        std_msgs::Int32MultiArray msg_crt;
        msg_crt.data.push_back(0);
        msg_crt.data.push_back(ui_->camera->getCritLine(1));
        msg_crt.data.push_back(ui_->camera->getCritLine(0));
        critline_pub.publish(msg_crt);

        savePublishCurrentMission();
    }
}

void CameraPanel::changeCritLine(int i, int crit)
{
    ui_->camera->changeCritLine(i, crit);
}

void CameraPanel::missionChanged(int cur_mis)
{
    char* mission_name[] = {"Navigation Channel", "Snack Run", "Avoid The Crouds", "Find The Seat", "Water Blast", "Skeebal", "Obstacle Avoidance"};
    current_mission = cur_mis - 1;
    std::cout << "CHANGE MISSION " << current_mission << "\n";

    ui_->label_name_mission->setText(mission_name[current_mission]);

    ui_->camera->setBoatSide(boatSide_all[current_mission * 4 + 2], boatSide_all[current_mission * 4 + 3]);
    ui_->camera->changeCritLine(1, crit_lines[current_mission]);
    ui_->camera->changeCritLine(0, horizon[current_mission]);
}

void CameraPanel::loadFile(std::string filename_)
{
    std::cout << "OPEN FILE " << filename_ << std::endl;

    QFile file(filename_.c_str());

    if (!file.open(QIODevice::ReadWrite))
        QMessageBox::information(this, tr("Can't Open File"), file.errorString());

    QByteArray saveData = file.readAll();
    QJsonDocument document(QJsonDocument::fromJson(saveData));
    QJsonObject json = document.object();

    //    json["P1"].toDouble()
    QJsonDocument misi_;
    QJsonObject json_misi_, json_sub_misi_, boatside_;
    const char* mission_name[5]= { "NavigationChannel", "SnackRun", "AvoidTheCrouds", "FindASeatAtTheShow", "WaterBlast" };
    
    for(int i = 0; i < 5; i++){
        if (json.contains(mission_name[i]))
        {
            int current_mission_ = i;
            json_misi_ = QJsonDocument::fromJson(json[mission_name[i]].toString().toUtf8()).object();

            boatside_ = QJsonDocument::fromJson(json_misi_["boatside"].toString().toUtf8()).object();

            boatSide_all[current_mission_ * 4 + 3].setX(boatside_["back_cam_x"].toDouble() * image_width);
            boatSide_all[current_mission_ * 4 + 3].setY(boatside_["back_cam_y"].toDouble() * image_height);
            boatSide_all[current_mission_ * 4 + 2].setX(boatside_["front_cam_x"].toDouble() * image_width);
            boatSide_all[current_mission_ * 4 + 2].setY(boatside_["front_cam_y"].toDouble() * image_height);

            boatSide_all[current_mission_ * 4 + 0].setY(boatSide_all[current_mission_ * 4 + 2].y());
            boatSide_all[current_mission_ * 4 + 1].setY(boatSide_all[current_mission_ * 4 + 3].y());
            boatSide_all[current_mission_ * 4 + 0].setX(image_width - boatSide_all[current_mission_ * 4 + 2].x());
            boatSide_all[current_mission_ * 4 + 1].setX(image_width - boatSide_all[current_mission_ * 4 + 3].x());

            crit_lines[current_mission_] = json_misi_["critical_line"].toDouble() * image_height;
            horizon[current_mission_] = json_misi_["obstacle_line"].toDouble() * image_height;
        }
        missionChanged(i);
    }

    missionChanged(ui_->spinBoxMission->value());

    

    
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(CameraPanel, rviz::Panel)