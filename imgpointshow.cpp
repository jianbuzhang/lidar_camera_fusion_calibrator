#include "imgpointshow.h"
#include "ui_imgpointshow.h"

#include <QtGui>
#include <QDesktopWidget>
#include <QFileDialog>

ImgPointShow::ImgPointShow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImgPointShow)
{
    ui->setupUi(this);

    this->setWindowTitle("MOVE-X融合标定工具");
    QDesktopWidget *desktop = QApplication::desktop();
    move((desktop->width()-this->width())/2,(desktop->height()-this->height())/2);
}

ImgPointShow::~ImgPointShow()
{
    delete ui;
}

void ImgPointShow::DisplayMat(QImage img)
{
    ui->PcdImg_Lable->setPixmap(QPixmap::fromImage(img));
    ui->PcdImg_Lable->resize(QSize(img.width(),img.height()));

    //ui->PcdImg_Lable->setPixmap(QPixmap::fromImage(img).scaled(ui->PcdImg_Lable->size()));
    ui->PcdImg_Lable->setScaledContents(true);
}
