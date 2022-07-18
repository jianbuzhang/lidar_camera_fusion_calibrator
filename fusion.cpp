#include "fusion.h"
#include "./ui_fusion.h"

#include <QtGui>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <fstream>

#include <stdio.h>
#include <dirent.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/core/eigen.hpp"

#include "yaml-cpp/parser.h"
#include "yaml-cpp/eventhandler.h"
#include "yaml-cpp/yaml.h"
#ifdef __cplusplus
extern "C"{
#endif
#include "elog.h"
#include "elog_file.h"
#ifdef __cplusplus
}
#endif

using namespace cv;
using namespace Eigen;

enum LOG_LEVEL {
    QLOGA = 0,
    QLOGE,
    QLOGW,
    QLOGI,
    QLOGD,
    QLOGV,
};

//#define LOG_SAVE_TO_FILE

#define INIT_PARAM_PATH "/opt/CalibratorTool/initparam.yaml"

#define X_MIN           0
#define X_MAX           1000
#define Y_MIN           0
#define Y_MAX           1000
#define Z_MIN           0
#define Z_MAX           1000
#define ROLL_MIN        0
#define ROLL_MAX        1000
#define PITCH_MIN       0
#define PITCH_MAX       6000
#define YAW_MIN         0
#define YAW_MAX         1000

typedef struct _CamInitCfg_S
{
    int X_Init;
    int Y_Init;
    int Z_Init;
    int Roll_Init;
    int Pitch_Init;
    int Yaw_Init;
} CamInitCfg_S;

QString CameraString = "左后广角, 左前广角, 右前广角, 右后广角, 左长焦, 右长焦";
QString CameraStringEn = "LeftBackWideAngle,LeftFrontWideAngle,RightFrontWideAngle,RightBackWideAngle,LeftLongFocus,RightLongFocus";
QStringList CamStrList;
QStringList CamStrListEn;
CamInitCfg_S CamInitCfg[] = {
    {500, 500, 500, 500, 3000, 500},
    {500, 500, 500, 500, 3000, 500},
    {500, 500, 500, 500, 3000, 500},
    {500, 500, 500, 500, 3000, 500},
    {500, 500, 500, 500, 3000, 500},
    {500, 500, 500, 500, 3000, 500},
};

Eigen::Matrix3d rotation_matrix(3, 3);
Eigen::MatrixXd calibration_mtx_final(4, 4);
struct EulerAngles {
    double roll, pitch, yaw;
};
EulerAngles angles;

std::vector<cv::Mat> Imgs;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PcdPtrs;
cv::Mat Reprojected;

Fusion::Fusion(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Fusion)
{
    ui->setupUi(this);

    CamStrList = CameraString.split(",");
    CamStrListEn = CameraStringEn.split(",");

    this->FolderReady = 0;
    this->CamIndex = 0;
    this->ImgNums = 0;
    this->No = 0;

    this->InitWinStyle();

#ifdef LOG_SAVE_TO_FILE
    EasyloggerInit("./data/log", "funsion");
#endif

    ImgShow = new ImgPointShow;
}

Fusion::~Fusion()
{
    delete ui;
}

void Fusion::InitWinStyle()
{
    //this->setWindowIcon(QIcon(":/image/movex3.png"));
    this->setWindowTitle("lidar-camera-fusion-calibrator");
    QDesktopWidget *desktop = QApplication::desktop();
    move((desktop->width()-this->width())/2,(desktop->height()-this->height())/2);
}

void Fusion::InitWinForm()
{
    {
        /* 初始化组件----SelCamCfg_comboBox */
        ui->SelCamCfg_comboBox->clear();
        for (QStringList::iterator it = CamStrList.begin(); it != CamStrList.end(); ++it)
        {
            ui->SelCamCfg_comboBox->addItem(*it);
        }
        ui->SelCamCfg_comboBox->setCurrentIndex(0);
        connect(ui->SelCamCfg_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(Slot_SelCamCfg_ComboBoxChanged(int)));
    }

    {
        /* 初始化组件----No_SpinBox No_Slider */
        ui->No_SpinBox->setMinimum(0);
        ui->No_SpinBox->setMaximum(0);
        ui->No_SpinBox->setSingleStep(1);
        ui->No_Slider->setOrientation(Qt::Horizontal); // 水平方向
        ui->No_Slider->setMinimum(0);
        ui->No_Slider->setMaximum(0);
        ui->No_Slider->setSingleStep(1);
        ui->No_Slider->setPageStep(1);
        connect(ui->No_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(Slot_No_SpinBox_SetValue_clicked(int)));
        connect(ui->No_Slider, SIGNAL(valueChanged(int)), this, SLOT(Slot_No_Slider_SetValue_clicked(int)));
    }

    {
        /* 初始化组件----X_SpinBox X_Slider */
        ui->X_SpinBox->setMinimum(X_MIN);
        ui->X_SpinBox->setMaximum(X_MAX);
        ui->X_SpinBox->setValue(this->X);
        ui->X_SpinBox->setSingleStep(1);
        ui->X_Slider->setOrientation(Qt::Horizontal); // 水平方向
        ui->X_Slider->setMinimum(X_MIN);
        ui->X_Slider->setMaximum(X_MAX);
        ui->X_Slider->setValue(this->X);
        ui->X_Slider->setSingleStep(1);
        ui->X_Slider->setPageStep(1);
        connect(ui->X_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(Slot_X_SpinBox_SetValue_clicked(int)));
        connect(ui->X_Slider, SIGNAL(valueChanged(int)), this, SLOT(Slot_X_Slider_SetValue_clicked(int)));
    }

    {
        /* 初始化组件----Y_SpinBox Y_Slider */
        ui->Y_SpinBox->setMinimum(Y_MIN);
        ui->Y_SpinBox->setMaximum(Y_MAX);
        ui->Y_SpinBox->setValue(this->Y);
        ui->Y_SpinBox->setSingleStep(1);
        ui->Y_Slider->setOrientation(Qt::Horizontal); // 水平方向
        ui->Y_Slider->setMinimum(Y_MIN);
        ui->Y_Slider->setMaximum(Y_MAX);
        ui->Y_Slider->setValue(this->Y);
        ui->Y_Slider->setSingleStep(1);
        ui->Y_Slider->setPageStep(1);
        connect(ui->Y_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(Slot_Y_SpinBox_SetValue_clicked(int)));
        connect(ui->Y_Slider, SIGNAL(valueChanged(int)), this, SLOT(Slot_Y_Slider_SetValue_clicked(int)));
    }

    {
        /* 初始化组件----Z_SpinBox Z_Slider */
        ui->Z_SpinBox->setMinimum(Z_MIN);
        ui->Z_SpinBox->setMaximum(Z_MAX);
        ui->Z_SpinBox->setValue(this->Z);
        ui->Z_SpinBox->setSingleStep(1);
        ui->Z_Slider->setOrientation(Qt::Horizontal); // 水平方向
        ui->Z_Slider->setMinimum(Z_MIN);
        ui->Z_Slider->setMaximum(Z_MAX);
        ui->Z_Slider->setValue(this->Z);
        ui->Z_Slider->setSingleStep(1);
        ui->Z_Slider->setPageStep(1);
        connect(ui->Z_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(Slot_Z_SpinBox_SetValue_clicked(int)));
        connect(ui->Z_Slider, SIGNAL(valueChanged(int)), this, SLOT(Slot_Z_Slider_SetValue_clicked(int)));
    }

    {
        /* 初始化组件----Roll_SpinBox Roll_Slider */
        ui->Roll_SpinBox->setMinimum(ROLL_MIN);
        ui->Roll_SpinBox->setMaximum(ROLL_MAX);
        ui->Roll_SpinBox->setValue(this->Roll);
        ui->Roll_SpinBox->setSingleStep(1);
        ui->Roll_Slider->setOrientation(Qt::Horizontal); // 水平方向
        ui->Roll_Slider->setMinimum(ROLL_MIN);
        ui->Roll_Slider->setMaximum(ROLL_MAX);
        ui->Roll_Slider->setValue(this->Roll);
        ui->Roll_Slider->setSingleStep(1);
        ui->Roll_Slider->setPageStep(1);
        connect(ui->Roll_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(Slot_Roll_SpinBox_SetValue_clicked(int)));
        connect(ui->Roll_Slider, SIGNAL(valueChanged(int)), this, SLOT(Slot_Roll_Slider_SetValue_clicked(int)));
    }

    {
        /* 初始化组件----Pitch_SpinBox Pitch_Slider */
        ui->Pitch_SpinBox->setMinimum(PITCH_MIN);
        ui->Pitch_SpinBox->setMaximum(PITCH_MAX);
        ui->Pitch_SpinBox->setValue(this->Pitch);
        ui->Pitch_SpinBox->setSingleStep(1);
        ui->Pitch_Slider->setOrientation(Qt::Horizontal); // 水平方向
        ui->Pitch_Slider->setMinimum(PITCH_MIN);
        ui->Pitch_Slider->setMaximum(PITCH_MAX);
        ui->Pitch_Slider->setValue(this->Pitch);
        ui->Pitch_Slider->setSingleStep(1);
        ui->Pitch_Slider->setPageStep(1);
        connect(ui->Pitch_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(Slot_Pitch_SpinBox_SetValue_clicked(int)));
        connect(ui->Pitch_Slider, SIGNAL(valueChanged(int)), this, SLOT(Slot_Pitch_Slider_SetValue_clicked(int)));
    }

    {
        /* 初始化组件----Yaw_SpinBox Yaw_Slider */
        ui->Yaw_SpinBox->setMinimum(YAW_MIN);
        ui->Yaw_SpinBox->setMaximum(YAW_MAX);
        ui->Yaw_SpinBox->setValue(this->Yaw);
        ui->Yaw_SpinBox->setSingleStep(1);
        ui->Yaw_Slider->setOrientation(Qt::Horizontal); // 水平方向
        ui->Yaw_Slider->setMinimum(YAW_MIN);
        ui->Yaw_Slider->setMaximum(YAW_MAX);
        ui->Yaw_Slider->setValue(this->Yaw);
        ui->Yaw_Slider->setSingleStep(1);
        ui->Yaw_Slider->setPageStep(1);
        connect(ui->Yaw_SpinBox, SIGNAL(valueChanged(int)), this, SLOT(Slot_Yaw_SpinBox_SetValue_clicked(int)));
        connect(ui->Yaw_Slider, SIGNAL(valueChanged(int)), this, SLOT(Slot_Yaw_Slider_SetValue_clicked(int)));
    }

    /* 初始化组件----选择目录 选择文件 */
    connect(ui->OpenPcd_Button, SIGNAL(clicked()), this, SLOT(Slot_Btn_OpenPath_clicked()));
    connect(ui->OpenCamCfg_Button, SIGNAL(clicked()), this, SLOT(Slot_Btn_OpenFile_clicked()));

    /* 初始化组件----fx fy cx cy k1 k2 k3 p1 p1 */
    connect(ui->fx_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->fy_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->cx_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->cy_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->k1_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->k2_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->k3_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->p1_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));
    connect(ui->p2_lineEdit, SIGNAL(editingFinished()), this, SLOT(Slot_CamInternalParam_editingFinished()));

    /* 初始化组件----保存参数,保存日志,配置参数更新 */
    connect(ui->SaveParam_Button, SIGNAL(clicked()), this, SLOT(Slot_Btn_SaveParam_clicked()));
    connect(ui->SaveLog_Button, SIGNAL(clicked()), this, SLOT(Slot_Btn_SaveLog_clicked()));
    connect(ui->InitParamUpdate_Button, SIGNAL(clicked()), this, SLOT(Slot_Btn_InitParamUpdate_clicked()));

#if 0
    /* thread */
    m_objThread= new QThread();
    connect(m_objThread, &QThread::finished, m_objThread, &QObject::deleteLater);
    connect(this, &Fusion::StartReprojectSignal, m_objThread, &Fusion::doSomething);
    m_objThread->start();
#endif
}

void Fusion::InitParam()
{
    QFileInfo file(INIT_PARAM_PATH);

    /* 如果初始参数通过argv[1]传入则使用传入参数，否则使用默认参数 */
    if (this->InitParamFlag) {
        ReadInitParamFile(this->InitParamFile);
    } else if (file.exists()) {
        ReadInitParamFile(INIT_PARAM_PATH);
    } else {
        LogToText("未传入初始配置文件，使用默认配置！", 1, 1, QLOGI);
    }
    this->X = CamInitCfg[0].X_Init;
    this->Y = CamInitCfg[0].Y_Init;
    this->Z = CamInitCfg[0].Z_Init;
    this->Roll = CamInitCfg[0].Roll_Init;
    this->Pitch = CamInitCfg[0].Pitch_Init;
    this->Yaw = CamInitCfg[0].Yaw_Init;
}

void Fusion::ReadInitParamFile(QString filename)
{
    QString logmsg;
    YAML::Node node = YAML::LoadFile(filename.toStdString());

    for (unsigned long i = 0; i < sizeof(CamInitCfg) / sizeof(CamInitCfg[0]); i++) {
        CamInitCfg[i].X_Init = node[CamStrListEn[i].toStdString()]["X"].as<int>();
        CamInitCfg[i].Y_Init = node[CamStrListEn[i].toStdString()]["Y"].as<int>();
        CamInitCfg[i].Z_Init = node[CamStrListEn[i].toStdString()]["Z"].as<int>();
        CamInitCfg[i].Roll_Init = node[CamStrListEn[i].toStdString()]["Roll"].as<int>();
        CamInitCfg[i].Pitch_Init = node[CamStrListEn[i].toStdString()]["Pitch"].as<int>();
        CamInitCfg[i].Yaw_Init = node[CamStrListEn[i].toStdString()]["Yaw"].as<int>();

        if (i == 0) {
            LogToText("解析初始参数成功！", 1, 1, QLOGI);
        }
        logmsg = QString("%1: %2,%3,%4,%5,%6，%7")
                .arg(CamStrListEn[i])
                .arg(CamInitCfg[i].X_Init)
                .arg(CamInitCfg[i].Y_Init)
                .arg(CamInitCfg[i].Z_Init)
                .arg(CamInitCfg[i].Roll_Init)
                .arg(CamInitCfg[i].Pitch_Init)
                .arg(CamInitCfg[i].Yaw_Init);
        LogToText(logmsg, 1, 1, QLOGI);
    }
}

void Fusion::EasyloggerInit(const char *path, const char *module)
{
    static char logpath[1024] = {0};

    sprintf(logpath, "%s/%s.log", path, module);

    elog_deinit(); /* close printf buffer */
    /* logpath must be static , because there internal no memory copy*/
    elog_file_setopts(logpath, 20 * 1024 * 1024, 10);
    setbuf(stdout, NULL);
    /* initialize EasyLogger */
    elog_init();
    /* set EasyLogger log format */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL & ~ELOG_FMT_P_INFO & ~ELOG_FMT_T_INFO);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_ALL & ~ELOG_FMT_P_INFO & ~ELOG_FMT_T_INFO);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME | ELOG_FMT_DIR | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME | ELOG_FMT_DIR | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME | ELOG_FMT_DIR | ELOG_FMT_FUNC | ELOG_FMT_LINE);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME | ELOG_FMT_DIR | ELOG_FMT_FUNC | ELOG_FMT_LINE);
#ifdef ELOG_COLOR_ENABLE
    elog_set_text_color_enabled(true);
#endif
    /* start EasyLogger */
    elog_start();
}

void Fusion::LogToText(QString msg, int isouttext, int isoutfile, int loglevel)
{
    if (isouttext) {
        QString logmsg;
        QTextCharFormat fmt;
        fmt.setFontPointSize(10);
        if (loglevel == QLOGA) {
            fmt.setForeground(QBrush("red"));
            logmsg = QString("[ASSERT]%1").arg(msg);
        } else if (loglevel == QLOGE) {
            fmt.setForeground(QBrush("red"));
            logmsg = QString("[ERROR]%1").arg(msg);
        } else if (loglevel == QLOGW) {
            fmt.setForeground(QBrush("yellow"));
            logmsg = QString("[WARN]%1").arg(msg);
        } else if (loglevel == QLOGI) {
            fmt.setForeground(QBrush("black"));
            logmsg = QString("[INFO]%1").arg(msg);
        } else if (loglevel == QLOGD) {
            fmt.setForeground(QBrush("black"));
            logmsg = QString("[DEBUG]%1").arg(msg);
        } else if (loglevel == QLOGV) {
            fmt.setForeground(QBrush("black"));
            logmsg = QString("[VERBOSE]%1").arg(msg);
        }

        ui->Log_textEdit->mergeCurrentCharFormat(fmt);
        ui->Log_textEdit->append(logmsg);
    }
    if (isoutfile) {
#ifdef LOG_SAVE_TO_FILE
        QByteArray ba = msg.toLocal8Bit();

        if (loglevel == QLOGA) {
            log_a(ba.data());
        } else if (loglevel == QLOGE) {
            log_e(ba.data());
        } else if (loglevel == QLOGW) {
            log_w(ba.data());
        } else if (loglevel == QLOGI) {
            log_i(ba.data());
        } else if (loglevel == QLOGD) {
            log_d(ba.data());
        } else if (loglevel == QLOGV) {
            log_v(ba.data());
        }
#endif
    }
}

int Fusion::Min(int v1, int v2)
{
    return v1 < v2 ? v1 : v2;
}

int Fusion::ReadFolder(std::string &folder)
{
    DIR *dir;
    struct dirent *diread;
    std::set<std::string> file_names;

    if ((dir = opendir(folder.c_str())) != nullptr)
    {
           while ((diread = readdir(dir)) != nullptr)
           {
               file_names.insert(diread->d_name);
           }
           closedir(dir);
    }
    else
    {
        LogToText("Invalid folder path!", 1, 1, QLOGE);
        return 1;
    }

    bool size_is_initialized = false;
    /* Load image and pcd */
    for (auto it = file_names.begin(); it != file_names.end(); it++)
    {
        try
        {
            std::string str = *it;
            size_t found = str.find(".png");
            if (found == std::string::npos)
            {
                throw 1;
            }

            std::string name = str.substr(0, found);
            std::string img_path = folder + "/" + name + ".png";
            cv::Mat img = cv::imread(img_path);

            std::string pcd_path = folder + "/" + name + ".pcd";
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
            {
                throw 1;
            }

            for (int i = 0; i < cloud->points.size(); i++)
            {
                // Assign position for camera coordinates
                double x = -cloud->points[i].y;
                double y = -cloud->points[i].z;
                double z = cloud->points[i].x;

                cloud->points[i].x = x;
                cloud->points[i].y = y;
                cloud->points[i].z = z;
            }

            Imgs.emplace_back(img);
            PcdPtrs.emplace_back(cloud);

            ImgHeight = img.rows;
            ImgWidth = img.cols;

            qDebug() << "ImgHeight=" << ImgHeight;
            qDebug() << "ImgWidth=" << ImgWidth;

            size_is_initialized = true;
        }
        catch (int e)
        {

        }
    }
    if (PcdPtrs.size() == 0)
    {
        LogToText("No data is found!", 1, 1, QLOGE);
        return 1;
    }
    return 0;
}

void Fusion::DisplayMat(cv::Mat image)
{
    cv::Mat rgb;
    QImage img;

    if(image.channels() == 3)
    {
        cvtColor(image, rgb, CV_BGR2RGB);
        img = QImage((const unsigned char*)(rgb.data),
            rgb.cols, rgb.rows, rgb.cols*rgb.channels(),//rgb.cols*rgb.channels()可以替换为image.step
            QImage::Format_RGB888);
    }
    else
    {
        img = QImage((const unsigned char*)(image.data),
            image.cols, image.rows, rgb.cols*image.channels(),
            QImage::Format_RGB888);
    }

    /* Lable窗口显示图片，图片*/
    ui->PcdImg_Lable->setPixmap(QPixmap::fromImage(img));
    ui->PcdImg_Lable->resize(QSize(img.width(),img.height()));
    ui->PcdImg_Lable->setScaledContents(true);

    ImgShow->DisplayMat(img);
}

void Fusion::doSomething()
{
    doReproject();
}

void Fusion::StartReproject()
{
    //emit StartReprojectSignal();
    if (FolderReady) {
        doReproject();
    }
}

void Fusion::doReproject()
{
    for (int i = 0; i < ImgHeight; i++) {
        for (int j = 0; j < ImgWidth; j++) {
            Reprojected.at<cv::Vec3b>(i, j) = Imgs[this->No].at<cv::Vec3b>(i, j);
        }
    }

    double rollVal = (this->Roll - (ROLL_MAX / 2)) / 1000.0;
    double pitchVal = (this->Pitch - (PITCH_MAX / 2)) / 1000.0;
    double yawVal = (this->Yaw - (YAW_MAX / 2)) / 1000.0;
    Eigen::MatrixXd calibration_mtx(4, 4);
    calibration_mtx << cos(yawVal) * cos(pitchVal), cos(yawVal) * sin(pitchVal) * sin(rollVal) - sin(yawVal) * cos(rollVal), cos(yawVal) * sin(pitchVal) * cos(rollVal) + sin(yawVal) * sin(rollVal), (this->X - (X_MAX / 2)) / 500.0,
        sin(yawVal) * cos(pitchVal), sin(yawVal) * sin(pitchVal) * sin(rollVal) + cos(yawVal) * cos(rollVal), sin(yawVal) * sin(pitchVal) * cos(rollVal) - cos(yawVal) * sin(rollVal), (this->Y - (Y_MAX / 2)) / 500.0,
        -sin(pitchVal), cos(pitchVal) * sin(rollVal), cos(pitchVal) * cos(rollVal), (this->Z - (Z_MAX / 2)) / 500.0,
        0, 0, 0, 1;
    Eigen::MatrixXd coordinates_mtx(4, 4);
    coordinates_mtx << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    //Eigen::MatrixXd calibration_mtx_final(4, 4);
    calibration_mtx_final = calibration_mtx * coordinates_mtx;
    calibration_mtx_final =calibration_mtx_final.inverse();
    //Eigen::Matrix3d rotation_matrix(3, 3);
    rotation_matrix << calibration_mtx_final(0, 0), calibration_mtx_final(0, 1), calibration_mtx_final(0, 2), calibration_mtx_final(1, 0), calibration_mtx_final(1, 1), calibration_mtx_final(1, 2), calibration_mtx_final(2, 0), calibration_mtx_final(2, 1), calibration_mtx_final(2, 2);

    LogToText("-----------参数输出开始------------", 1, 1, QLOGI);

    QString logmsg = QString("calibration_mtx: \n%1 %2 %3 %4 \n%5 %6 %7 %8 \n%9 %10 %11 %12 \n%13 %14 %15 %16")
            .arg(calibration_mtx_final(0, 0)).arg(calibration_mtx_final(0, 1)).arg(calibration_mtx_final(0, 2)).arg(calibration_mtx_final(0, 3))
            .arg(calibration_mtx_final(1, 0)).arg(calibration_mtx_final(1, 1)).arg(calibration_mtx_final(1, 2)).arg(calibration_mtx_final(1, 3))
            .arg(calibration_mtx_final(2, 0)).arg(calibration_mtx_final(2, 1)).arg(calibration_mtx_final(2, 2)).arg(calibration_mtx_final(2, 3))
            .arg(calibration_mtx_final(3, 0)).arg(calibration_mtx_final(3, 1)).arg(calibration_mtx_final(3, 2)).arg(calibration_mtx_final(3, 3));
    LogToText(logmsg, 1, 1, QLOGI);
    logmsg = QString("rotation_matrix: \n%1 %2 %3 \n%4 %5 %6 \n%7 %8 %9")
            .arg(calibration_mtx_final(0, 0)).arg(calibration_mtx_final(0, 1)).arg(calibration_mtx_final(0, 2))
            .arg(calibration_mtx_final(1, 0)).arg(calibration_mtx_final(1, 1)).arg(calibration_mtx_final(1, 2))
            .arg(calibration_mtx_final(2, 0)).arg(calibration_mtx_final(2, 1)).arg(calibration_mtx_final(2, 2));
    LogToText(logmsg, 1, 1, QLOGI);

    Eigen::Quaterniond q(rotation_matrix);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1) {
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        angles.pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    logmsg = QString("内参: fx=%1 fy=%2 cx=%3 cy=%4").arg(this->fx).arg(this->fy).arg(this->cx).arg(this->cy);
    LogToText(logmsg, 1, 1, QLOGI);
    logmsg = QString("径向畸变: k1=%1 k2=%2 k3=%3").arg(this->k1).arg(this->k2).arg(this->k3);
    LogToText(logmsg, 1, 1, QLOGI);
    logmsg = QString("切向畸变: p1=%1 p2=%2").arg(this->p1).arg(this->p2);
    LogToText(logmsg, 1, 1, QLOGI);
    logmsg = QString("平移向量: x=%1 y=%2 z=%3").arg(calibration_mtx_final(0, 3)).arg( calibration_mtx_final(1, 3))
            .arg( calibration_mtx_final(2, 3));
    LogToText(logmsg, 1, 1, QLOGI);
    logmsg = QString("欧拉角: roll=%1 pitch=%2 yaw=%3").arg(angles.roll).arg(angles.pitch).arg(angles.yaw);
    LogToText(logmsg, 1, 1, QLOGI);

    LogToText("-----------参数输出结束------------", 1, 1, QLOGI);

    for (int i = 0; i < PcdPtrs[this->No]->points.size(); i++)
    {
        double rawX = PcdPtrs[this->No]->points[i].x;
        double rawY = PcdPtrs[this->No]->points[i].y;
        double rawZ = PcdPtrs[this->No]->points[i].z;

        double r = sqrt(rawX * rawX + rawZ * rawZ);
        double x = calibration_mtx(0, 0) * rawX + calibration_mtx(0, 1) * rawY + calibration_mtx(0, 2) * rawZ + calibration_mtx(0, 3);
        double y = calibration_mtx(1, 0) * rawX + calibration_mtx(1, 1) * rawY + calibration_mtx(1, 2) * rawZ + calibration_mtx(1, 3);
        double z = calibration_mtx(2, 0) * rawX + calibration_mtx(2, 1) * rawY + calibration_mtx(2, 2) * rawZ + calibration_mtx(2, 3);

        if (z > 0)
        {
            //计算点云畸变后投影
            double xi = x / z;
            double yi = y / z;
            double r2i = xi * xi + yi * yi;
            double r4i = r2i * r2i;
            double r6i = r2i * r2i * r2i;
            double fi = 1 + this->k1 * r2i + this->k2 * r4i + this->k3 * r6i;
            xi = xi * fi;
            yi = yi * fi;
            double dx = xi + 2 * this->p1 * xi * yi + this->p2 * (r2i + 2 * xi * xi);
            double dy = yi + 2 * this->p2 * xi * yi + this->p1 * (r2i + 2 * yi * yi);
            int u = (int)(this->cx + this->fx * dx);
            int v = (int)(this->cy + this->fy * dy);
            //int u = (int)(c_x + f_x * x / z);
            //int v = (int)(c_y + f_x * y / z);
            if (0 <= u && u < ImgWidth && 0 <= v && v < ImgHeight)
            {
                uchar colorR = (uchar)Min(0 + z * 40, 255.0);
                uchar colorG = (uchar)(Min(0 + z * 15, 255.0));
                uchar colorB = (uchar)(Min(0 + z * 5, 255.0));
                cv::circle(Reprojected, cv::Point(u, v), 2, cv::Scalar(colorR, colorG, colorB));
            }
        }
    }

    DisplayMat(Reprojected);
}

void Fusion::CreateOneFolder(QString dir)
{
    QString logmsg;

    if(!(boost::filesystem::exists(dir.toStdString()))) {
        logmsg = QString("%1文件不存在!").arg(dir);
        LogToText(logmsg, 1, 1, QLOGD);
        if (boost::filesystem::create_directory(dir.toStdString())) {
            logmsg = QString("创建%1成功!").arg(dir);
            LogToText(logmsg, 1, 1, QLOGD);
        }
    }
}

void Fusion::CreateFolders()
{
    QString dir;

    CreateOneFolder("./data");
    CreateOneFolder("./data/log");
    CreateOneFolder("./data/internal_param");

    for (int i = 0; i < CamStrListEn.size(); i++) {
        dir.clear();
        dir.append("./data/internal_param/");
        dir.append(CamStrListEn[i]);
        CreateOneFolder(dir);
    }
}

void Fusion::SaveParam(QString filename)
{
    cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
    fs << "internal_param" << "{";
        fs << "fx" << this->fx;
        fs << "fy" << this->fy;
        fs << "cx" << this->cx;
        fs << "cy" << this->cy;
        fs << "k1" << this->k1;
        fs << "k2" << this->k2;
        fs << "k3" << this->k3;
        fs << "p1" << this->p1;
        fs << "p2" << this->p2;
        fs << "angle" << "{";
            fs << "roll" << angles.roll;
            fs << "pitch" << angles.pitch;
            fs << "yaw" << angles.yaw;
        fs << "}";
    fs << "}";

    cv::Mat tempMat;
    cv::eigen2cv(calibration_mtx_final, tempMat);
    fs << "calibrationMatrix" << tempMat;
    cv::eigen2cv(rotation_matrix, tempMat);
    fs << "rotationMatrix" << tempMat;
    fs.release();

    QString logmsg = QString("保存参数成功: %1").arg(filename);
    LogToText(logmsg, 1, 1, QLOGI);
}

void Fusion::SaveLog(QString filename)
{
    QFile file(filename);//文件命名
    if (!file.open(QFile::WriteOnly | QFile::Text))		//检测文件是否打开
    {
        QMessageBox::information(this, "Error Message", "Open log file fail!");
        return;
    }
    QTextStream out(&file);
    out << ui->Log_textEdit->toPlainText();
    file.close();

    LogToText("保存日志成功！", 1, 1, QLOGI);
}

void Fusion::Slot_Btn_OpenPath_clicked()
{
    std::string folderpath;
    QString path = QFileDialog::getExistingDirectory(this, "选择目录", "./",  QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks | QFileDialog::DontUseNativeDialog);
    if (!path.isEmpty()) {
        ui->PcdPath_lineEdit->setText(path);
        qDebug() << path;
        folderpath = path.toStdString();
        int ret = ReadFolder(folderpath);

        ImgNums = Imgs.size();
        if (0 == ret && ImgNums > 0) {
            ui->No_Slider->setMaximum(ImgNums - 1);
            ui->No_SpinBox->setMaximum(ImgNums - 1);

            Reprojected = cv::Mat::zeros(ImgHeight, ImgWidth, CV_8UC3);
            ImgShow->showMaximized();

            FolderReady = 1;
            StartReproject();
        } else {
            FolderReady = 0;
        }
    }
}

void Fusion::Slot_Btn_OpenFile_clicked()
{
    QString file = QFileDialog::getOpenFileName(this, "选择文件", "./", "yaml files(*.yaml)", 0, QFileDialog::DontUseNativeDialog);
    if (!file.isEmpty()) {
        ui->CamCfg_lineEdit->setText(file);

        QString logmsg = QString("导入内参文件: %1").arg(file);
        LogToText(logmsg, 1, 1, QLOGI);

        YAML::Node node = YAML::LoadFile(file.toStdString());

        this->fx = node["f_x"].as<double>();
        this->fy = node["f_y"].as<double>();
        this->cx = node["c_x"].as<double>();
        this->cy = node["c_y"].as<double>();
        this->k1 = node["k_1"].as<double>();
        this->k2 = node["k_2"].as<double>();
        this->k3 = node["k_3"].as<double>();
        this->p1 = node["p_1"].as<double>();
        this->p2 = node["p_2"].as<double>();

        ui->fx_lineEdit->setText(QString("%1").arg(this->fx));
        ui->fy_lineEdit->setText(QString("%1").arg(this->fy));
        ui->cx_lineEdit->setText(QString("%1").arg(this->cx));
        ui->cy_lineEdit->setText(QString("%1").arg(this->cy));
        ui->k1_lineEdit->setText(QString("%1").arg(this->k1));
        ui->k2_lineEdit->setText(QString("%1").arg(this->k2));
        ui->k3_lineEdit->setText(QString("%1").arg(this->k3));
        ui->p1_lineEdit->setText(QString("%1").arg(this->p1));
        ui->p2_lineEdit->setText(QString("%1").arg(this->p2));

        StartReproject();
    }
}

void Fusion::Slot_SelCamCfg_ComboBoxChanged(int index)
{
    this->CamIndex = index;

    ui->X_Slider->setValue(CamInitCfg[index].X_Init);
    ui->X_SpinBox->setValue(CamInitCfg[index].X_Init);
    ui->Y_Slider->setValue(CamInitCfg[index].Y_Init);
    ui->Y_SpinBox->setValue(CamInitCfg[index].Y_Init);
    ui->Z_Slider->setValue(CamInitCfg[index].Z_Init);
    ui->Z_SpinBox->setValue(CamInitCfg[index].Z_Init);
    ui->Roll_Slider->setValue(CamInitCfg[index].Roll_Init);
    ui->Roll_SpinBox->setValue(CamInitCfg[index].Roll_Init);
    ui->Pitch_Slider->setValue(CamInitCfg[index].Pitch_Init);
    ui->Pitch_SpinBox->setValue(CamInitCfg[index].Pitch_Init);
    ui->Yaw_Slider->setValue(CamInitCfg[index].Yaw_Init);
    ui->Yaw_SpinBox->setValue(CamInitCfg[index].Yaw_Init);

    this->X = CamInitCfg[index].X_Init;
    this->Y = CamInitCfg[index].Y_Init;
    this->Z = CamInitCfg[index].Z_Init;
    this->Roll = CamInitCfg[index].Roll_Init;
    this->Pitch = CamInitCfg[index].Pitch_Init;
    this->Yaw = CamInitCfg[index].Yaw_Init;

    StartReproject();

    QString logmsg = QString("%1: X=%2,Y=%3,Z=%4,Roll=%5,Pitch=%6,Yaw=%7")
            .arg(CamStrList[index]).arg(X).arg(Y).arg(Z).arg(Roll).arg(Pitch).arg(Yaw);
    LogToText(logmsg, 1, 1, QLOGI);
}

void Fusion::Slot_No_SpinBox_SetValue_clicked(int value)
{
    ui->No_Slider->setValue(value);
    No = value;

    StartReproject();

    QString logmsg = QString("修改No成功: No=%1").arg(No);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_No_Slider_SetValue_clicked(int value)
{
    ui->No_SpinBox->setValue(value);
    No = value;
}

void Fusion::Slot_X_SpinBox_SetValue_clicked(int value)
{
    ui->X_Slider->setValue(value);
    X = value;
    CamInitCfg[this->CamIndex].X_Init = X;

    StartReproject();

    QString logmsg = QString("修改X成功: X=%1").arg(X);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_X_Slider_SetValue_clicked(int value)
{
    ui->X_SpinBox->setValue(value);
    X = value;
    CamInitCfg[this->CamIndex].X_Init = X;
}

void Fusion::Slot_Y_SpinBox_SetValue_clicked(int value)
{
    ui->Y_Slider->setValue(value);
    Y = value;
    CamInitCfg[this->CamIndex].Y_Init = Y;

    StartReproject();

    QString logmsg = QString("修改Y成功: Y=%1").arg(Y);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_Y_Slider_SetValue_clicked(int value)
{
    ui->Y_SpinBox->setValue(value);
    Y = value;
    CamInitCfg[this->CamIndex].Y_Init = Y;
}

void Fusion::Slot_Z_SpinBox_SetValue_clicked(int value)
{
    ui->Z_Slider->setValue(value);
    Z = value;
    CamInitCfg[this->CamIndex].Z_Init = Z;

    StartReproject();

    QString logmsg = QString("修改Z成功: Z=%1").arg(Z);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_Z_Slider_SetValue_clicked(int value)
{
    ui->Z_SpinBox->setValue(value);
    Z = value;
    CamInitCfg[this->CamIndex].Z_Init = Z;
}

void Fusion::Slot_Roll_SpinBox_SetValue_clicked(int value)
{
    ui->Roll_Slider->setValue(value);
    Roll = value;
    CamInitCfg[this->CamIndex].Roll_Init = Roll;

    StartReproject();

    QString logmsg = QString("修改Roll成功: Roll=%1").arg(Roll);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_Roll_Slider_SetValue_clicked(int value)
{
    ui->Roll_SpinBox->setValue(value);
    Roll = value;
    CamInitCfg[this->CamIndex].Roll_Init = Roll;
}

void Fusion::Slot_Pitch_SpinBox_SetValue_clicked(int value)
{
    ui->Pitch_Slider->setValue(value);
    Pitch = value;
    CamInitCfg[this->CamIndex].Pitch_Init = Pitch;

    StartReproject();

    QString logmsg = QString("修改Pitch成功: Pitch=%1").arg(Pitch);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_Pitch_Slider_SetValue_clicked(int value)
{
    ui->Pitch_SpinBox->setValue(value);
    Pitch = value;
    CamInitCfg[this->CamIndex].Pitch_Init = Pitch;
}

void Fusion::Slot_Yaw_SpinBox_SetValue_clicked(int value)
{
    ui->Yaw_Slider->setValue(value);
    Yaw = value;
    CamInitCfg[this->CamIndex].Yaw_Init = Yaw;

    StartReproject();

    QString logmsg = QString("修改Yaw成功: Yaw=%1").arg(Yaw);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_Yaw_Slider_SetValue_clicked(int value)
{
    ui->Yaw_SpinBox->setValue(value);
    Yaw = value;
    CamInitCfg[this->CamIndex].Yaw_Init = Yaw;
}

void Fusion::Slot_CamInternalParam_editingFinished(void)
{
    this->fx = ui->fx_lineEdit->text().toDouble();
    this->fy = ui->fy_lineEdit->text().toDouble();
    this->cx = ui->cx_lineEdit->text().toDouble();
    this->cy = ui->cy_lineEdit->text().toDouble();
    this->k1 = ui->k1_lineEdit->text().toDouble();
    this->k2 = ui->k2_lineEdit->text().toDouble();
    this->k3 = ui->k3_lineEdit->text().toDouble();
    this->p1 = ui->p1_lineEdit->text().toDouble();
    this->p2 = ui->p2_lineEdit->text().toDouble();

    StartReproject();

    QString logmsg = QString("内参修改: fx=%1,fy=%2,cx=%3,cy=%4,k1=%5,k2=%6,k3=%7,p1=%8,p2=%9")
            .arg(fx).arg(fy).arg(cx).arg(cy).arg(k1).arg(k2).arg(k3)
            .arg(p1).arg(p2);
    LogToText(logmsg, 1, 1, QLOGD);
}

void Fusion::Slot_Btn_SaveParam_clicked(void)
{
    QString file;
    file = QFileDialog::getSaveFileName(this, tr("Save file"), "test.yaml", tr("Param Files (*.yaml)"), 0,
            QFileDialog::DontConfirmOverwrite);
    if (!file.isEmpty()) {
        SaveParam(file);
    } else {
        QString logmsg = QString("保存参数失败!: %1").arg(file);
        LogToText(logmsg, 1, 1, QLOGE);
    }
}

void Fusion::Slot_Btn_SaveLog_clicked(void)
{
    QString file;
    file = QFileDialog::getSaveFileName(this, tr("Save file"), "test.log", tr("Log Files (*.log)"), 0,
            QFileDialog::DontConfirmOverwrite);
    if (!file.isEmpty()) {
        SaveLog(file);
    } else {
        QString logmsg = QString("保存日志失败!: %1").arg(file);
        LogToText(logmsg, 1, 1, QLOGE);
    }
}

void Fusion::Slot_Btn_InitParamUpdate_clicked(void)
{
    QString logmsg;
    QString file;
    file.append(INIT_PARAM_PATH);

    cv::FileStorage fs(file.toStdString(), cv::FileStorage::WRITE);

    for (unsigned long i = 0; i < sizeof(CamInitCfg) / sizeof(CamInitCfg[0]); i++) {
        fs << CamStrListEn[i].toStdString() << "{";
            fs << "X" << CamInitCfg[i].X_Init;
            fs << "Y" << CamInitCfg[i].Y_Init;
            fs << "Z" << CamInitCfg[i].Z_Init;
            fs << "Roll" << CamInitCfg[i].Roll_Init;
            fs << "Pitch" << CamInitCfg[i].Pitch_Init;
            fs << "Yaw" << CamInitCfg[i].Yaw_Init;
        fs << "}";

        if (i == 0) {
            LogToText("更新初始参数成功！", 1, 1, QLOGI);
        }
        logmsg = QString("%1: %2,%3,%4,%5,%6，%7")
                .arg(CamStrListEn[i])
                .arg(CamInitCfg[i].X_Init)
                .arg(CamInitCfg[i].Y_Init)
                .arg(CamInitCfg[i].Z_Init)
                .arg(CamInitCfg[i].Roll_Init)
                .arg(CamInitCfg[i].Pitch_Init)
                .arg(CamInitCfg[i].Yaw_Init);
        LogToText(logmsg, 1, 1, QLOGI);
    }

    fs.release();
}
