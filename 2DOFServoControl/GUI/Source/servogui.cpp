#include "servogui.h"
#include "ui_servogui.h"
#include "communication.h"
#include <stdio.h>
#include <QDateTime>
#include <QObject>
#include <QKeyEvent>
#include <QThread>
using namespace std;

static Communication* sptr;

ServoGUI::ServoGUI(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::ServoGUI)
{


    ui->setupUi(this);
    unsigned const idx = 9;

    bool connections [idx] = {
                              (bool)QObject::connect(ui->UpButton, SIGNAL(pressed()),  this, SLOT(MouseClick())),
                              (bool)QObject::connect(ui->UpButton, SIGNAL(released()),  this, SLOT(MouseRelease())),
                              (bool)QObject::connect(ui->DownButton, SIGNAL(pressed()),  this, SLOT(MouseClick())),
                              (bool)QObject::connect(ui->DownButton, SIGNAL(released()),  this, SLOT(MouseRelease())),
                              (bool)QObject::connect(ui->LeftButton, SIGNAL(pressed()),  this, SLOT(MouseClick())),
                              (bool)QObject::connect(ui->LeftButton, SIGNAL(released()),  this, SLOT(MouseRelease())),
                              (bool)QObject::connect(ui->RightButton, SIGNAL(pressed()),  this, SLOT(MouseClick())),
                              (bool)QObject::connect(ui->RightButton, SIGNAL(released()),  this, SLOT(MouseRelease())),
                              (bool)QObject::connect(ui->Connect, SIGNAL(clicked()), this, SLOT(ServoConnect())),

    };

    sptr = Communication::GetSerialPortInstance();

    ServoYaw = 0.f;
    ServoPitch = 0.f;
    memset(Output, 0x0, strlen(Output));

    for (unsigned i = 0; i < idx; ++i)
        qDebug() << "Connection : "<<connections[i];

}

ServoGUI::~ServoGUI()
{
    delete ui;
}

void ServoGUI::keyPressEvent(QKeyEvent* event)
{
    if (!timer.isValid())
        timer.start();

    if (event->key() == Qt::Key_D){
        sptr->WritePort("R");
        ServoYaw+= 1;
        ui->AzimuthDisplay->setText(QString::number(ServoYaw));
    }
    if (event->key() == Qt::Key_A){
        sptr->WritePort("L");
        ServoYaw-= 1;
        ui->AzimuthDisplay->setText(QString::number(ServoYaw));
    }
    if (event->key() == Qt::Key_W){
        sptr->WritePort("U");
        ServoPitch+= 1;
        ui->ElevationDisplay->setText(QString::number(ServoPitch));
    }
    if (event->key() == Qt::Key_S){
        sptr->WritePort("D");
        ServoPitch-= 1;
        ui->ElevationDisplay->setText(QString::number(ServoPitch));
    }

}

void ServoGUI::keyReleaseEvent(QKeyEvent* event)
{
    sptr->WritePort("S");
    timer.invalidate();
}

void ServoGUI::MouseClick()
{
    if (!timer.isValid())
        timer.start();
    QPushButton* button = qobject_cast<QPushButton*>(sender());

    if (button->text() == "Up")
        sptr->WritePort("U");
    else if (button->text() == "Down")
        sptr->WritePort("D");
    else if (button->text() == "Left")
        sptr->WritePort("L");
    else if (button->text() == "Right")
        sptr->WritePort("R");
}

void ServoGUI::MouseRelease()
{
    sptr->WritePort("S");
    ui->DisplayBox->setText(QString::number(timer.elapsed()));
    timer.invalidate();
}

void ServoGUI::ServoConnect()
{
    const char* COM;
    unsigned BaudRate, ByteSize, StopBit, Parity, Count;
    bool IsConnected = false;
    Count = 0;
    qDebug()<<sptr->GetCommStatus();


    if (ui->PortInput->text() == ""){
        ui->DisplayBox->setText("Enter port!\n");
    }
    else{
        QString qs = QString("\\\\.\\%1").arg(ui->PortInput->text());

        COM =   "\\\\.\\COM5"  ;//qs.toStdString().c_str();

        Count++;
    }
    if (ui->BaudrateInput->text() == ""){
        ui->DisplayBox->setText("Enter Baudrate!\n");
    }

    else{
        BaudRate = ui->BaudrateInput->text().toUInt();
        Count++;
    }

    if (ui->ByteSizeInput->text() == ""){
        ui->DisplayBox->setText("Enter Byte size!\n");
    }
    else{
        ByteSize = ui->ByteSizeInput->text().toUInt();
        Count++;
    }
    if (ui->StopBitInput->text() == ""){
        ui->DisplayBox->setText("Enter stop bit!\n");
    }
    else{
        StopBit = ui->StopBitInput->text().toUInt();
        Count++;
    }

    if (ui->ParityInput->text() == ""){
        ui->DisplayBox->setText("Enter parity bit!\n");
    }
    else{
        Parity = ui->ParityInput->text().toUInt();
        Count++;
    }
    qDebug()<<COM<<BaudRate<<ByteSize<<StopBit<<Parity;
    if (Count == 5)
        IsConnected = sptr->OpenPort(COM, BaudRate, ByteSize, StopBit, Parity);

    if (IsConnected)
        ui->DisplayBox->setText("Connected!");
    else
         ui->DisplayBox->setText("Cannot Connected!");

}





