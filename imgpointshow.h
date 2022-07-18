#ifndef IMGPOINTSHOW_H
#define IMGPOINTSHOW_H

#include <QWidget>

namespace Ui {
class ImgPointShow;
}

class ImgPointShow : public QWidget
{
    Q_OBJECT

public:
    explicit ImgPointShow(QWidget *parent = nullptr);
    ~ImgPointShow();

    void DisplayMat(QImage img);

public:
    Ui::ImgPointShow *ui;
};

#endif // IMGPOINTSHOW_H
