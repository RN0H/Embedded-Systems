#include "servogui.h"
#include "communication.h"
#include <QApplication>
#include <QThread>
#include <stdio.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ServoGUI w;
    w.show();
    return a.exec();
}
