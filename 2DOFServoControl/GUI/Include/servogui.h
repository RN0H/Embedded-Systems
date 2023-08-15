#ifndef SERVOGUI_H
#define SERVOGUI_H

#include "communication.h"
#include <QMainWindow>
#include <QElapsedTimer>
#include <QThread>

QT_BEGIN_NAMESPACE
namespace Ui { class ServoGUI; }
QT_END_NAMESPACE

class ServoGUI : public QMainWindow, Communication
{
    Q_OBJECT

public:
    ServoGUI(QWidget *parent = nullptr);
    ~ServoGUI();
    void ServoThread();


protected:
    void keyPressEvent(QKeyEvent* ) override;
    void keyReleaseEvent(QKeyEvent* ) override;

private:
    Ui::ServoGUI *ui;

    float ServoYaw = 0.f;
    float ServoPitch = 0.f;

    char Output[3];
    QElapsedTimer timer;


private slots:
    void MouseClick();
    void MouseRelease();
    void ServoConnect();

};
#endif // SERVOGUI_H
