#ifndef FUSION_H
#define FUSION_H

#include <QWidget>
#include "opencv2/opencv.hpp"
#include "imgpointshow.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Fusion; }
QT_END_NAMESPACE

class Fusion : public QWidget
{
    Q_OBJECT

public:
    Fusion(QWidget *parent = nullptr);
    ~Fusion();

    void InitWinStyle();
    void InitWinForm();
    void CreateOneFolder(QString dir);
    void CreateFolders();
    void InitParam();
    void ReadInitParamFile(QString filename);
    void EasyloggerInit(const char *path, const char *module);
    void LogToText(QString msg, int isouttext, int isoutfile, int loglevel);

    int Min(int v1, int v2);
    int ReadFolder(std::string &folder);
    void DisplayMat(cv::Mat image);
    void StartReproject();
    void doReproject();
    void SaveParam(QString filename);
    void SaveLog(QString filename);

signals:
    void StartReprojectSignal();

private slots:
    void doSomething();

private slots:
    void Slot_Btn_OpenFile_clicked();
    void Slot_Btn_OpenPath_clicked();
    void Slot_SelCamCfg_ComboBoxChanged(int index);
    void Slot_No_SpinBox_SetValue_clicked(int value);
    void Slot_No_Slider_SetValue_clicked(int value);
    void Slot_X_SpinBox_SetValue_clicked(int value);
    void Slot_X_Slider_SetValue_clicked(int value);
    void Slot_Y_SpinBox_SetValue_clicked(int value);
    void Slot_Y_Slider_SetValue_clicked(int value);
    void Slot_Z_SpinBox_SetValue_clicked(int value);
    void Slot_Z_Slider_SetValue_clicked(int value);
    void Slot_Roll_SpinBox_SetValue_clicked(int value);
    void Slot_Roll_Slider_SetValue_clicked(int value);
    void Slot_Pitch_SpinBox_SetValue_clicked(int value);
    void Slot_Pitch_Slider_SetValue_clicked(int value);
    void Slot_Yaw_SpinBox_SetValue_clicked(int value);
    void Slot_Yaw_Slider_SetValue_clicked(int value);
    void Slot_CamInternalParam_editingFinished(void);
    void Slot_Btn_SaveParam_clicked(void);
    void Slot_Btn_SaveLog_clicked(void);
    void Slot_Btn_InitParamUpdate_clicked(void);

public:
    int InitParamFlag;
    QString InitParamFile;

private:
    Ui::Fusion *ui;

    int FolderReady;/*图片点云数据目录导入成功*/

    int ImgWidth;
    int ImgHeight;
    int ImgNums;

    int CamIndex;/*相机索引*/

    int No;/*图片点云数据编号*/
    int X;
    int Y;
    int Z;
    int Roll;
    int Pitch;
    int Yaw;

    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double k3;
    double p1;
    double p2;

    QThread *m_objThread;
    ImgPointShow *ImgShow;
};

#endif // FUSION_H
