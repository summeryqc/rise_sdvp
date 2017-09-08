/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "utility.h"
#include <QSerialPortInfo>
#include <QDebug>
#include <cmath>
#include <QMessageBox>
#include <QFileDialog>
#include <QHostInfo>
#include <QInputDialog>

#ifdef HAS_SBS
#include "networkinterface.h"
#include "nmeawidget.h"
#include "basestation.h"
#endif

namespace {
void stepTowards(double &value, double goal, double step) {
    if (value < goal) {
        if ((value + step) < goal) {
            value += step;
        } else {
            value = goal;
        }
    } else if (value > goal) {
        if ((value - step) > goal) {
            value -= step;
        } else {
            value = goal;
        }
    }
}

#ifdef HAS_JOYSTICK
void deadband(double &value, double tres, double max) {
    if (fabs(value) < tres) {
        value = 0.0;
    } else {
        double k = max / (max - tres);
        if (value > 0.0) {
            value = k * value + max * (1.0 - k);
        } else {
            value = -(k * -value + max * (1.0 - k));
        }

    }
}
#endif
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mVersion = "0.5";

    qRegisterMetaType<LocPoint>("LocPoint");

    mTimer = new QTimer(this);
    mTimer->start(ui->pollIntervalBox->value());
    mStatusLabel = new QLabel(this);
    ui->statusBar->addPermanentWidget(mStatusLabel);
    mStatusInfoTime = 0;
    mPacketInterface = new PacketInterface(this);
    mSerialPort = new QSerialPort(this);
    mThrottle = 0.0;
    mSteering = 0.0;

#ifdef HAS_JOYSTICK
    mJoystick = new Joystick(this);
    mJsType = JS_TYPE_HK;
#endif

    mPing = new Ping(this);
    mNmea = new NmeaServer(this);
    mUdpSocket = new QUdpSocket(this);

    mKeyUp = false;
    mKeyDown = false;
    mKeyLeft = false;
    mKeyRight = false;

    ui->mapWidget->setRoutePointSpeed(ui->mapRouteSpeedBox->value() / 3.6);
    ui->networkLoggerWidget->setMap(ui->mapWidget);
    ui->networkInterface->setMap(ui->mapWidget);
    ui->networkInterface->setPacketInterface(mPacketInterface);
    ui->moteWidget->setPacketInterface(mPacketInterface);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mSerialPort, SIGNAL(readyRead()),
            this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mPacketInterface, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(packetDataToSend(QByteArray&)));
    connect(mPacketInterface, SIGNAL(stateReceived(quint8,CAR_STATE)),
            this, SLOT(stateReceived(quint8,CAR_STATE)));
    connect(mPacketInterface, SIGNAL(mrStateReceived(quint8,MULTIROTOR_STATE)),
            this, SLOT(mrStateReceived(quint8,MULTIROTOR_STATE)));
    connect(ui->mapWidget, SIGNAL(posSet(quint8,LocPoint)),
            this, SLOT(mapPosSet(quint8,LocPoint)));
    connect(mPacketInterface, SIGNAL(ackReceived(quint8,CMD_PACKET,QString)),
            this, SLOT(ackReceived(quint8,CMD_PACKET,QString)));
    connect(ui->rtcmWidget, SIGNAL(rtcmReceived(QByteArray)),
            this, SLOT(rtcmReceived(QByteArray)));
    connect(ui->baseStationWidget, SIGNAL(rtcmOut(QByteArray)),
            this, SLOT(rtcmReceived(QByteArray)));
    connect(ui->rtcmWidget, SIGNAL(refPosGet()), this, SLOT(rtcmRefPosGet()));
    connect(mPing, SIGNAL(pingRx(int,QString)), this, SLOT(pingRx(int,QString)));
    connect(mPing, SIGNAL(pingError(QString,QString)), this, SLOT(pingError(QString,QString)));
    connect(mPacketInterface, SIGNAL(enuRefReceived(quint8,double,double,double)),
            this, SLOT(enuRx(quint8,double,double,double)));
    connect(mNmea, SIGNAL(clientGgaRx(int,NmeaServer::nmea_gga_info_t)),
            this, SLOT(nmeaGgaRx(int,NmeaServer::nmea_gga_info_t)));
    connect(ui->mapWidget, SIGNAL(routePointAdded(LocPoint)),
            this, SLOT(routePointAdded(LocPoint)));
    connect(ui->mapWidget, SIGNAL(infoTraceChanged(int)),
            this, SLOT(infoTraceChanged(int)));

    connect(ui->actionAboutQt, SIGNAL(triggered(bool)),
            qApp, SLOT(aboutQt()));

    on_serialRefreshButton_clicked();

    qApp->installEventFilter(this);

#ifdef HAS_SBS
    setWindowTitle("RControlStation-SBS");
    setWindowIcon(QIcon(":/Icons/Car-96_white.png"));

    // Add button for xbox controller
    xbox = new QCheckBox(ui->groupBox_3);
    xbox->setObjectName("xboxButton");
    xbox->setFixedWidth(150);
    xbox->setText("Xbox controller");
    ui->gridLayout_3->addWidget(xbox, 0, 0, Qt::AlignTop);
    connect(xbox, SIGNAL(toggled(bool)),
            this, SLOT(xboxButtonToggled(bool)));

    mXbox = false;
    mBrakeValue = 0.0;
    mSpeed = 0.0;
    mLimit = 0.0;

    mL1 = 0.0;
    mR1 = 0.0;
    mL2 = 0.0;
    mR2 = 0.0;
    mAxisLeftX = 0.0;
    mAxisLeftY = 0.0;
    mA = false;
    mY = false;

    // Connect UDP and expose mainwidow to networkinterface
    ui->networkInterface->setMainWindow(this);
    ui->networkInterface->connectUDP();

#endif
}

MainWindow::~MainWindow()
{
    // Remove all vehicles before this window is destroyed to not get segfaults
    // in their destructors.
    while (mCars.size() > 0 || mCopters.size() > 0) {
        QWidget *w = ui->carsWidget->currentWidget();

        if (dynamic_cast<CarInterface*>(w) != NULL) {
            CarInterface *car = (CarInterface*)w;

            ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
            mCars.removeOne(car);
            delete car;
        } else if (dynamic_cast<CopterInterface*>(w) != NULL) {
            CopterInterface *copter = (CopterInterface*)w;

            ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
            mCopters.removeOne(copter);
            delete copter;
        }
    }

    delete ui;
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    Q_UNUSED(object);

    // Emergency stop on escape
    if (e->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        if (keyEvent->key() == Qt::Key_Escape) {
            on_stopButton_clicked();
            return true;
        }
    }

#ifdef HAS_JOYSTICK
    if (mJoystick->isConnected()) {
        return false;
    }
#endif

    if (e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        bool isPress = e->type() == QEvent::KeyPress;

        switch(keyEvent->key()) {
        case Qt::Key_Up:
        case Qt::Key_Down:
        case Qt::Key_Left:
        case Qt::Key_Right:
            break;

        default:
            return false;
        }

        switch(keyEvent->key()) {
        case Qt::Key_Up: mKeyUp = isPress; break;
        case Qt::Key_Down: mKeyDown = isPress; break;
        case Qt::Key_Left: mKeyLeft = isPress; break;
        case Qt::Key_Right: mKeyRight = isPress; break;

        default:
            break;
        }

        // Return true to not pass the key event on
        return true;
    }

    return false;
}

void MainWindow::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();
        mPacketInterface->processData(data);
    }
}

void MainWindow::serialPortError(QSerialPort::SerialPortError error)
{
    QString message;
    switch (error) {
    case QSerialPort::NoError:
        break;

    default:
        message = "Serial port error: " + mSerialPort->errorString();
        break;
    }

    if(!message.isEmpty()) {
        showStatusInfo(message, false);

        if(mSerialPort->isOpen()) {
            mSerialPort->close();
        }
    }
}

void MainWindow::timerSlot()
{
    bool js_connected = false;
    double js_mr_thr = 0.0;
    double js_mr_roll = 0.0;
    double js_mr_pitch = 0.0;
    double js_mr_yaw = 0.0;

#ifdef HAS_JOYSTICK
    js_connected = mJoystick->isConnected();
#endif

    // Update throttle and steering from keys.
    if (js_connected) {
#ifdef HAS_JOYSTICK
        if (mJsType == JS_TYPE_HK) {
            mThrottle = -(double)mJoystick->getAxis(4) / 32768.0;
            deadband(mThrottle,0.1, 1.0);
            mSteering = -(double)mJoystick->getAxis(0) / 32768.0;

            js_mr_thr = (((double)mJoystick->getAxis(2) / 32768.0) + 0.85) / 1.7;
            js_mr_roll = -(double)mJoystick->getAxis(0) / 32768.0;
            js_mr_pitch = -(double)mJoystick->getAxis(1) / 32768.0;
            js_mr_yaw = -(double)mJoystick->getAxis(4) / 32768.0;
            utility::truncate_number(&js_mr_thr, 0.0, 1.0);
            utility::truncate_number_abs(&js_mr_roll, 1.0);
            utility::truncate_number_abs(&js_mr_pitch, 1.0);
            utility::truncate_number_abs(&js_mr_yaw, 1.0);
        } else if (mJsType == JS_TYPE_PS4) {
            mThrottle = -(double)mJoystick->getAxis(1) / 32768.0;
            deadband(mThrottle,0.1, 1.0);
            mSteering = (double)mJoystick->getAxis(2) / 32768.0;
        }

        //mSteering /= 2.0;
#endif

#ifdef HAS_SBS
    } else if (mXbox && mCars.size() > 0){

        CarInterface *mSelectedCar = mCars[ui->mapCarBox->value()];

        // mSpeed is between 1 and 0, procentage of the currently set max speed
         mSpeed = ((mSelectedCar->getSpeed()) / (21 * ui->throttleMaxBox->value() + 0.1));

        // for testing:
//        mSpeed = ((mThrottle * ui->throttleMaxBox->value() * 21) / (21 * ui->throttleMaxBox->value() + 0.1));




        // Button R2 -> Throttle
        if (!mA && mL2 == 0.0) {
            mBrakeValue = 0.0;
            mThrottle = mR2;
        }

        // Button L2 -> Brake + -Throttle
//        if (!mA && mL2 > 0.0) {
//            if (mSelectedCar->getSpeed() > 0.1) {
//                mThrottle = 0.0;
//                stepTowards(mBrakeValue,
//                            25.0 * (0.1 + (0.9 * mL2)),
//                            0.3306 * ((1 - mSpeed) + 0.1) * (0.1 + (0.9 * mL2)));
//                mPacketInterface->setRcControlCurrentBrake(255, mBrakeValue, 0.0);
//                // stepTowards(brakeValue, 20.0, 0.4);
//            } else {
//                mBrakeValue = 0.0;
//                mThrottle = -1 * mL2;
//            }
//        }

        if (!mA && mL2 > 0.0) {
            if (mSelectedCar->getSpeed() > 0.1) {
                mThrottle = 0.0;
                stepTowards(mBrakeValue, 25, 0.2 * mL2);
                mPacketInterface->setRcControlCurrentBrake(255, mBrakeValue, 0.0);
            } else {
                mBrakeValue = 0.0;
                mThrottle = -1 * mL2;
            }
        }

        // Button AxisLeftX -> Steering
        // Adjuting the value cuz the stick is very sensitive around 0
        if (mAxisLeftX < 0.0) {
            mSteering = (mAxisLeftX + 0.20) * 1.251;
        } else if (mAxisLeftX > 0.0) {
            mSteering = (mAxisLeftX - 0.20) * 1.25;
        } else {
            mSteering = 0.0;
        }

//        if (mAxisLeftY > 0.15) {
//            mSteering = mAxisLeftX;
//        } else {
//            if (mAxisLeftX < 0.0) {
//                mSteering = (mAxisLeftX + 0.20) * 1.1;
//            } else if (mAxisLeftX > 0.0) {
//                mSteering = (mAxisLeftX - 0.20) * 1.1;
//            } else {
//                mSteering = 0.0;
//            }
//        }


//        if (mAxisLeftX < 0.0) {
//            if (mAxisLeftY < 0.5) {
//                mSteering = mAxisLeftX + (0.2 * (0.5 - mAxisLeftY)) ;
//            } else {
//                mSteering = mAxisLeftX +  0.2;
//            }
//        } else if (mAxisLeftX > 0.0) {
//            if (mAxisLeftY < 0.5) {
//                mSteering = mAxisLeftX + (-0.2 * (0.5 - mAxisLeftY)) ;
//            } else {
//                mSteering = mAxisLeftX - 0.2;
//            }
//        } else {
//            mSteering = 0.0;
//        }



        // Button A -> Handbrake
        if (mA) {
            mThrottle = 0.0;
            mLimit = mSpeed;
            stepTowards(mBrakeValue, 20.0, 0.3306 * ((1 - mSpeed) + 0.1)); //not tested
//            stepTowards(brakeValue, 20.0, 0.4);
            mPacketInterface->setRcControlCurrentBrake(255, mBrakeValue, 0.0);
        }

        // Button Y -> Emergency Stop
        if (mY) {
            mThrottle = 0.0;
            mSteering = 0.0;
            mBrakeValue = 0.0;
            mLimit = 0.0;
            for (int i = 0;i < mCars.size();i++) {
                mCars[i]->emergencyStop();
            }
            mXbox = false;
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
            mPacketInterface->setRcControlCurrentBrake(255, 20.0, 0.0);
        }

        // Button L1 -> Select Previous Car
        if (mL1 && ui->carsWidget->count() > 1) {
            if (ui->carsWidget->currentIndex() == 0) {
                ui->carsWidget->setCurrentIndex(ui->carsWidget->count() - 1);
            } else {
                ui->carsWidget->setCurrentIndex(ui->carsWidget->currentIndex() - 1);
            }
            ui->mapCarBox->setValue(mCars[ui->carsWidget->currentIndex()]->getId());
            ui->mapWidget->setSelectedCar(mCars[ui->carsWidget->currentIndex()]->getId());
        }

        // Button R1 -> Select Next Car
        if (mR1 && ui->carsWidget->count() > 1) {
            if (ui->carsWidget->currentIndex() == ui->carsWidget->count() - 1) {
                ui->carsWidget->setCurrentIndex(0);
            } else {
                ui->carsWidget->setCurrentIndex(ui->carsWidget->currentIndex() + 1);
            }
            ui->mapCarBox->setValue(mCars[ui->carsWidget->currentIndex()]->getId());
            ui->mapWidget->setSelectedCar(mCars[ui->carsWidget->currentIndex()]->getId());
        }

        // Limit Acceleration
        if (!mA && mL2 == 0.0 && mSpeed >= 0.0) {
            stepTowards(mLimit, mThrottle, ((mSpeed + 0.01) * (0.02 / (ui->throttleMaxBox->value() + 0.01)))
                        - ((0.013 / (ui->throttleMaxBox->value() + 0.01)) * (mSpeed * mSpeed * mSpeed)));
            if (mLimit < 0.0) {
                mLimit = 0.0;
            }

            if (mLimit < mThrottle && mR2 > 0.0) {
                mThrottle = mLimit;
            }

            if (mLimit > mThrottle) {
                mLimit = mSpeed;
            }
        }

        // Limit -Acceleration, -Throttle
        if (!mA && mL2 > 0.0 && mSpeed <= 0.0) {
            stepTowards(mLimit, 0.4 * mThrottle, (((-1 * mSpeed) + 0.01) * (0.02 / (ui->throttleMaxBox->value() + 0.01))));
            if (mLimit > 0.0) {
                mLimit = 0.0;
            }

            if (mLimit > mThrottle) {
                mThrottle = mLimit;
            } else {
                mLimit = mSpeed;
            }
        }

        // Set toggable
        if (mL1) {
            mL1 = false;
        }

        if (mR1) {
            mR1 = false;
        }
#endif

    } else {
        if (mKeyUp) {
            stepTowards(mThrottle, 1.0, ui->throttleGainBox->value());
        } else if (mKeyDown) {
            stepTowards(mThrottle, -1.0, ui->throttleGainBox->value());
        } else {
            stepTowards(mThrottle, 0.0, ui->throttleGainBox->value());
        }

        if (mKeyRight) {
            stepTowards(mSteering, 1.0, ui->steeringGainBox->value());
        } else if (mKeyLeft) {
            stepTowards(mSteering, -1.0, ui->steeringGainBox->value());
        } else {
            stepTowards(mSteering, 0.0, ui->steeringGainBox->value());
        }
    }

    // Prevent car from keep going when focus is lost
    if (!qApp->focusWidget()) {
        mThrottle = 0;
        mKeyUp = 0;
        mKeyDown = 0;
    }

    ui->mrThrottleBar->setValue(js_mr_thr * 100.0);
    ui->mrRollBar->setValue(js_mr_roll * 100.0);
    ui->mrPitchBar->setValue(js_mr_pitch * 100.0);
    ui->mrYawBar->setValue(js_mr_yaw * 100.0);

    ui->throttleBar->setValue(mThrottle * 100.0);
    ui->steeringBar->setValue(mSteering * 100.0);        

    // Notify about key events
    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        car->setControlValues(mThrottle, mSteering, ui->throttleMaxBox->value(), ui->throttleCurrentButton->isChecked());
    }

    // Notify about joystick events
    for(QList<CopterInterface*>::Iterator it_copter = mCopters.begin();it_copter < mCopters.end();it_copter++) {
        CopterInterface *copter = *it_copter;
        copter->setControlValues(js_mr_thr, js_mr_roll, js_mr_pitch, js_mr_yaw);
    }

    // Update status label
    if (mStatusInfoTime) {
        mStatusInfoTime--;
        if (!mStatusInfoTime) {
            mStatusLabel->setStyleSheet(qApp->styleSheet());
        }
    } else {
        if (mSerialPort->isOpen() || mPacketInterface->isUdpConnected()) {
            mStatusLabel->setText("Connected");
        } else {
            mStatusLabel->setText("Not connected");
        }
    }

    // Poll data (one vehicle per timeslot)
    static int next_car = 0;
    int ind = 0;
    int largest = 0;
    bool polled = false;

    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        if (car->pollData() && ind >= next_car && !polled) {
            mPacketInterface->getState(car->getId());
            next_car = ind + 1;
            polled = true;
        }

        if (car->pollData() && ind > largest) {
            largest = ind;
        }

        ind++;
    }

    for(QList<CopterInterface*>::Iterator it_copter = mCopters.begin();it_copter < mCopters.end();it_copter++) {
        CopterInterface *copter = *it_copter;
        if (copter->pollData() && ind >= next_car && !polled) {
            mPacketInterface->getMrState(copter->getId());
            next_car = ind + 1;
            polled = true;
        }

        if (copter->pollData() && ind > largest) {
            largest = ind;
        }

        ind++;
    }

    if (next_car > largest) {
        next_car = 0;
    }

    // Update map settings
    if (ui->mapFollowBox->isChecked()) {
        ui->mapWidget->setFollowCar(ui->mapCarBox->value());
    } else {
        ui->mapWidget->setFollowCar(-1);
    }
    if (ui->mapTraceBox->isChecked()) {
        ui->mapWidget->setTraceCar(ui->mapCarBox->value());
    } else {
        ui->mapWidget->setTraceCar(-1);
    }
    ui->mapWidget->setSelectedCar(ui->mapCarBox->value());

    // Joystick connected
#ifdef HAS_JOYSTICK
    static bool jsWasconn = false;
    if (mJoystick->isConnected() != jsWasconn) {
        jsWasconn = mJoystick->isConnected();

        if (jsWasconn) {
            ui->jsConnectedLabel->setText("Connected");
        } else {
            ui->jsConnectedLabel->setText("Not connected");
        }
    }
#endif

    // Update nmea stream connected label
    static bool wasNmeaStreamConnected = false;
    if (wasNmeaStreamConnected != mNmea->isClientTcpConnected()) {
        wasNmeaStreamConnected = mNmea->isClientTcpConnected();

        if (wasNmeaStreamConnected) {
            ui->mapStreamNmeaConnectedLabel->setText("Connected");
        } else {
            ui->mapStreamNmeaConnectedLabel->setText("Not connected");
        }
    }
}

void MainWindow::showStatusInfo(QString info, bool isGood)
{
    if (isGood) {
        mStatusLabel->setStyleSheet("QLabel { background-color : lightgreen; color : black; }");
    } else {
        mStatusLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }

    mStatusInfoTime = 80;
    mStatusLabel->setText(info);
}

void MainWindow::packetDataToSend(QByteArray &data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->write(data);
    }
}

void MainWindow::stateReceived(quint8 id, CAR_STATE state)
{
    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        if (car->getId() == id) {
            car->setStateData(state);
        }
    }
}

void MainWindow::mrStateReceived(quint8 id, MULTIROTOR_STATE state)
{
    for(QList<CopterInterface*>::Iterator it_copter = mCopters.begin();it_copter < mCopters.end();it_copter++) {
        CopterInterface *copter = *it_copter;
        if (copter->getId() == id) {
            copter->setStateData(state);
        }
    }
}

void MainWindow::mapPosSet(quint8 id, LocPoint pos)
{
    mPacketInterface->setPos(id, pos.getX(), pos.getY(), pos.getYaw() * 180.0 / M_PI);
}

void MainWindow::ackReceived(quint8 id, CMD_PACKET cmd, QString msg)
{
    (void)cmd;
    QString str;
    str.sprintf("Vehicle %d ack: ", id);
    str += msg;
    showStatusInfo(str, true);
}

void MainWindow::rtcmReceived(QByteArray data)
{
    mPacketInterface->sendRtcmUsb(255, data);
}

void MainWindow::rtcmRefPosGet()
{
    double lat, lon, height;
    if (ui->baseStationWidget->getAvgPosLlh(lat, lon, height) > 0) {
        ui->rtcmWidget->setRefPos(lat, lon, height);
    } else {
        QMessageBox::warning(this, "Reference Position",
                             "No samples collected yet.");
    }
}

void MainWindow::pingRx(int time, QString msg)
{
    QString str;
    str.sprintf("ping response time: %.3f ms", (double)time / 1000.0);
    QMessageBox::information(this, "Ping " + msg, str);
}

void MainWindow::pingError(QString msg, QString error)
{
    QMessageBox::warning(this, "Error ping " + msg, error);
}

void MainWindow::enuRx(quint8 id, double lat, double lon, double height)
{
    (void)id;
    ui->mapWidget->setEnuRef(lat, lon, height);
}

void MainWindow::nmeaGgaRx(int fields, NmeaServer::nmea_gga_info_t gga)
{
    if (fields >= 5) {
        if (gga.fix_type == 4 || gga.fix_type == 5 ||
                (gga.fix_type == 1 && !ui->mapStreamNmeaRtkOnlyBox->isChecked())) {
            double i_llh[3];

            if (ui->mapStreamNmeaZeroEnuBox->isChecked()) {
                i_llh[0] = gga.lat;
                i_llh[1] = gga.lon;
                i_llh[2] = gga.height;
                ui->mapWidget->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                ui->mapStreamNmeaZeroEnuBox->setChecked(false);
            } else {
                ui->mapWidget->getEnuRef(i_llh);
            }

            double llh[3];
            double xyz[3];

            llh[0] = gga.lat;
            llh[1] = gga.lon;
            llh[2] = gga.height;
            utility::llhToEnu(i_llh, llh, xyz);

            LocPoint p;
            p.setXY(xyz[0], xyz[1]);
            QString info;

            QString fix_t = "Unknown";
            if (gga.fix_type == 4) {
                fix_t = "RTK fix";
                p.setColor(Qt::green);
            } else if (gga.fix_type == 5) {
                fix_t = "RTK float";
                p.setColor(Qt::yellow);
            } else if (gga.fix_type == 1) {
                fix_t = "Single";
                p.setColor(Qt::red);
            }

            info.sprintf("Fix type: %s\n"
                         "Height  : %.2f",
                         fix_t.toLocal8Bit().data(),
                         gga.height);

            p.setInfo(info);
            ui->mapWidget->addInfoPoint(p);

            // Optionally stream the data over UDP
            if (ui->mapStreamNmeaForwardUdpBox->isChecked()) {
                QString hostString = ui->mapStreamNmeaForwardUdpHostEdit->text();
                QHostAddress host;

                host.setAddress(hostString);

                // In case setting the address failed try DNS lookup. Notice
                // that the lookup is stored in a static QHostInfo as long as
                // the host line does not change. This is to avoid some delay.
                if (host.isNull()) {
                    static QString hostStringBefore;
                    static QHostInfo hostBefore;

                    QList<QHostAddress> addresses = hostBefore.addresses();

                    // Make a new lookup if the address has changed or the old one is invalid.
                    if (hostString != hostStringBefore || addresses.isEmpty()) {
                        hostBefore = QHostInfo::fromName(hostString);
                        hostStringBefore = hostString;
                    }

                    if (!addresses.isEmpty()) {
                        host.setAddress(addresses.first().toString());
                    }
                }

                if (!host.isNull()) {
                    static int seq = 0;
                    QByteArray datagram;
                    QTextStream out(&datagram);
                    QString str;

                    utility::llhToXyz(llh[0], llh[1], llh[2],
                            &xyz[0], &xyz[1], &xyz[2]);

                    out << str.sprintf("%d\n", seq);          // Seq
                    out << str.sprintf("%05f\n", xyz[0]);     // X
                    out << str.sprintf("%05f\n", xyz[1]);     // Y
                    out << str.sprintf("%05f\n", xyz[2]);     // Height
                    out << str.sprintf("%05f\n", gga.t_tow);  // GPS time of week
                    out << str.sprintf("%d\n", 2);            // Vehicle ID
                    out.flush();

                    mUdpSocket->writeDatagram(datagram,
                                              host,
                                              ui->mapStreamNmeaForwardUdpPortBox->value());

                    seq++;
                } else {
                    QMessageBox::warning(this,
                                         tr("Host not found"),
                                         tr("Could not find %1").arg(hostString));
                    ui->mapStreamNmeaForwardUdpBox->setChecked(false);
                }
            }
        }
    }
}

void MainWindow::routePointAdded(LocPoint pos)
{
    (void)pos;
    QTime t = ui->mapRouteTimeEdit->time();
    t = t.addMSecs(ui->mapRouteAddTimeEdit->time().msecsSinceStartOfDay());
    ui->mapRouteTimeEdit->setTime(t);
}

void MainWindow::infoTraceChanged(int traceNow)
{
    ui->mapInfoTraceBox->setValue(traceNow);
}

void MainWindow::on_carAddButton_clicked()
{
    CarInterface *car = new CarInterface(this);
    int id = mCars.size() + mCopters.size();
    mCars.append(car);
    QString name;
    name.sprintf("Car %d", id);
    car->setID(id);
    ui->carsWidget->addTab(car, name);
    car->setMap(ui->mapWidget);
    car->setPacketInterface(mPacketInterface);

    connect(car, SIGNAL(showStatusInfo(QString,bool)), this, SLOT(showStatusInfo(QString,bool)));
}

void MainWindow::on_copterAddButton_clicked()
{
    CopterInterface *copter = new CopterInterface(this);
    int id = mCars.size() + mCopters.size();
    mCopters.append(copter);
    QString name;
    name.sprintf("Copter %d", id);
    copter->setID(id);
    ui->carsWidget->addTab(copter, name);
    copter->setMap(ui->mapWidget);
    copter->setPacketInterface(mPacketInterface);

    connect(copter, SIGNAL(showStatusInfo(QString,bool)), this, SLOT(showStatusInfo(QString,bool)));
}

void MainWindow::on_serialConnectButton_clicked()
{
    if(mSerialPort->isOpen()) {
        return;
    }

    mSerialPort->setPortName(ui->serialPortBox->currentData().toString());
    mSerialPort->open(QIODevice::ReadWrite);

    if(!mSerialPort->isOpen()) {
        return;
    }

    mSerialPort->setBaudRate(QSerialPort::Baud115200);
    mSerialPort->setDataBits(QSerialPort::Data8);
    mSerialPort->setParity(QSerialPort::NoParity);
    mSerialPort->setStopBits(QSerialPort::OneStop);
    mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

    mPacketInterface->stopUdpConnection();
}

void MainWindow::on_serialRefreshButton_clicked()
{
    ui->serialPortBox->clear();
    bool found = false;

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        QString name = port.portName();
        int index = ui->serialPortBox->count();
        // put STMicroelectronics device first in list and add prefix
        if(port.manufacturer() == "STMicroelectronics") {
            name.insert(0, "IF - ");
            index = 0;
            found = true;
        }
        ui->serialPortBox->insertItem(index, name, port.systemLocation());
    }

    ui->serialPortBox->setCurrentIndex(0);

    if (found && !mSerialPort->isOpen()) {
        on_serialConnectButton_clicked();
    }
}

void MainWindow::on_disconnectButton_clicked()
{
    if (mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    if (mPacketInterface->isUdpConnected()) {
        mPacketInterface->stopUdpConnection();
    }
}

void MainWindow::on_mapRemoveTraceButton_clicked()
{
    ui->mapWidget->clearTrace();
}

void MainWindow::on_MapRemovePixmapsButton_clicked()
{
    ui->mapWidget->clearPerspectivePixmaps();
}

void MainWindow::on_udpConnectButton_clicked()
{
    QHostAddress ip;

    if (ip.setAddress(ui->udpIpEdit->text().trimmed())) {
        if (mSerialPort->isOpen()) {
            mSerialPort->close();
        }

        mPacketInterface->startUdpConnection(ip, ui->udpPortBox->value());
    } else {
        showStatusInfo("Invalid IP address", false);
    }
}

void MainWindow::on_mapZeroButton_clicked()
{
    ui->mapWidget->setXOffset(0);
    ui->mapWidget->setYOffset(0);
}

void MainWindow::on_mapRemoveRouteButton_clicked()
{
    ui->mapWidget->clearRoute();
}

void MainWindow::on_mapRouteSpeedBox_valueChanged(double arg1)
{
    ui->mapWidget->setRoutePointSpeed(arg1 / 3.6);
}

void MainWindow::on_jsConnectButton_clicked()
{
#ifdef HAS_JOYSTICK
    if (mJoystick->init(ui->jsPortEdit->text()) == 0) {
        qDebug() << "Axes:" << mJoystick->numAxes();
        qDebug() << "Buttons:" << mJoystick->numButtons();
        qDebug() << "Name:" << mJoystick->getName();

        if (mJoystick->getName().contains("sony", Qt::CaseInsensitive)) {
            mJsType = JS_TYPE_PS4;
            qDebug() << "Treating joystick as PS4 USB controller.";
            showStatusInfo("PS4 USB joystick connected!", true);
        } else {
            mJsType = JS_TYPE_HK;
            qDebug() << "Treating joystick as hobbyking simulator.";
            showStatusInfo("HK joystick connected!", true);
        }
    } else {
        qWarning() << "Opening joystick failed.";
        showStatusInfo("Opening joystick failed.", false);
    }
#else
    QMessageBox::warning(this, "Joystick",
                         "This build does not have joystick support.");
#endif
}

void MainWindow::on_jsDisconnectButton_clicked()
{
#ifdef HAS_JOYSTICK
    mJoystick->stop();
#endif
}

void MainWindow::on_mapAntialiasBox_toggled(bool checked)
{
    ui->mapWidget->setAntialiasDrawings(checked);
}

void MainWindow::on_carsWidget_tabCloseRequested(int index)
{
    QWidget *w = ui->carsWidget->widget(index);
    ui->carsWidget->removeTab(index);

    if (dynamic_cast<CarInterface*>(w) != NULL) {
        CarInterface *car = (CarInterface*)w;
        mCars.removeOne(car);
        delete car;
    } else if (dynamic_cast<CopterInterface*>(w) != NULL) {
        CopterInterface *copter = (CopterInterface*)w;
        mCopters.removeOne(copter);
        delete copter;
    }
}

void MainWindow::on_genCircButton_clicked()
{
    double rad = ui->genCircRadBox->value();
    double speed = ui->mapRouteSpeedBox->value() / 3.6;
    double ang_ofs = M_PI;
    double cx = 0;
    double cy = 0;
    int points = ui->genCircPointsBox->value();
    int type = ui->genCircCenterBox->currentIndex();

    if (type == 1 || type == 2) {
        CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
        if (car) {
            LocPoint p = car->getLocation();
            double ang = p.getYaw();

            cx = p.getX();
            cy = p.getY();

            if (ui->genCircCenterBox->currentIndex() == 1) {
                cx += rad * sin(ang);
                cy += rad * cos(ang);
                ang_ofs = ang + M_PI;
            }
        }
    }

    if (type == 3) {
        cx = ui->genCircXBox->value();
        cy = ui->genCircYBox->value();
    }

    for (int i = 1;i <= points;i++) {
        int ind = i;

        if (rad < 0.0) {
            ind = points - i;
        }

        double ang = -((double)ind * 2.0 * M_PI) / (double)points + ang_ofs;

        double px = sin(ang) * rad;
        double py = cos(ang) * rad;

        // Move up
        px += cx;
        py += cy;

        ui->mapWidget->addRoutePoint(px, py, speed);

        bool res = true;
        LocPoint pos;
        pos.setXY(px, py);
        pos.setSpeed(speed);

        QList<LocPoint> points;
        points.append(pos);

        for (int i = 0;i < mCars.size();i++) {
            if (mCars[i]->updateRouteFromMap()) {
                res = mPacketInterface->setRoutePoints(mCars[i]->getId(), points);
            }
        }

        if (!res) {
            QMessageBox::warning(this, "Generate Cirlce",
                                 "No ack from car when uploading point.");
            break;
        }
    }
}

void MainWindow::on_mapSetAbsYawButton_clicked()
{
    CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
    if (car) {
        if (mSerialPort->isOpen() || mPacketInterface->isUdpConnected()) {
            ui->mapSetAbsYawButton->setEnabled(false);
            ui->mapAbsYawSlider->setEnabled(false);
            bool ok = mPacketInterface->setYawOffsetAck(car->getId(), (double)ui->mapAbsYawSlider->value());
            ui->mapSetAbsYawButton->setEnabled(true);
            ui->mapAbsYawSlider->setEnabled(true);

            if (!ok) {
                qDebug() << "No pos ack received";
            }
        }
    }
}

void MainWindow::on_mapAbsYawSlider_valueChanged(int value)
{
    (void)value;
    CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
    if (car) {
        mPacketInterface->setYawOffset(car->getId(), (double)ui->mapAbsYawSlider->value());
    }
}

void MainWindow::on_mapAbsYawSlider_sliderReleased()
{
    on_mapSetAbsYawButton_clicked();
}

void MainWindow::on_stopButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        mCars[i]->emergencyStop();
    }

    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
}

void MainWindow::on_mapUploadRouteButton_clicked()
{
    if (!mSerialPort->isOpen() && !mPacketInterface->isUdpConnected()) {
        QMessageBox::warning(this, "Upload route",
                             "Serial port not connected.");
        return;
    }

    QList<LocPoint> route = ui->mapWidget->getRoute();
    int len = route.size();
    int car = ui->mapCarBox->value();
    bool ok = true;

    if (len <= 0) {
        QMessageBox::warning(this, "Upload route",
                             "No route on map.");
        return;
    }

    ui->mapUploadRouteButton->setEnabled(false);

    // Stop car
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == car) {
            ok = mCars[i]->setAp(false);
            break;
        }
    }

    // Clear previous route
    if (ok) {
        ok = mPacketInterface->clearRoute(car);
    }

    if (ok) {
        int ind = 0;
        for (ind = 0;ind < len;ind += 5) {
            QList<LocPoint> tmpList;
            for (int j = ind;j < (ind + 5);j++) {
                if (j < len) {
                    tmpList.append(route.at(j));
                }
            }

            ok = mPacketInterface->setRoutePoints(car, tmpList);

            if (!ok) {
                break;
            }

            ui->mapUploadRouteProgressBar->setValue((100 * (ind + 5)) / len);
        }
    }

    if (!ok) {
        QMessageBox::warning(this, "Upload route",
                             "No response when uploading route.");
    } else {
        ui->mapUploadRouteProgressBar->setValue(100);
    }

    ui->mapUploadRouteButton->setEnabled(true);
}

void MainWindow::on_mapApButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->setCtrlAp();
        }
    }
}

void MainWindow::on_mapKbButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->setCtrlKb();
        }
    }
}

void MainWindow::on_mapOffButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->emergencyStop();
        }
    }
}

void MainWindow::on_mapUpdateSpeedButton_clicked()
{
    QList<LocPoint> route = ui->mapWidget->getRoute();

    for (int i = 0;i < route.size();i++) {
        route[i].setSpeed(ui->mapRouteSpeedBox->value() / 3.6);
    }

    ui->mapWidget->setRoute(route);
}

void MainWindow::on_udpPingButton_clicked()
{
    mPing->pingHost(ui->udpIpEdit->text(), 64, "UDP Host");
}

void MainWindow::on_mapOpenStreetMapBox_toggled(bool checked)
{
    ui->mapWidget->setDrawOpenStreetmap(checked);
    ui->mapWidget->update();
}

void MainWindow::on_mapAntialiasOsmBox_toggled(bool checked)
{
    ui->mapWidget->setAntialiasOsm(checked);
}

void MainWindow::on_mapOsmResSlider_valueChanged(int value)
{
    ui->mapWidget->setOsmRes((double)value / 100.0);
}

void MainWindow::on_mapChooseNmeaButton_clicked()
{
    QString path;
    path = QFileDialog::getOpenFileName(this, tr("Choose log file to open"));
    if (path.isNull()) {
        return;
    }

    ui->mapImportNmeaEdit->setText(path);
}

void MainWindow::on_mapImportNmeaButton_clicked()
{
    QFile file;
    file.setFileName(ui->mapImportNmeaEdit->text());
    bool mapUpdated = false;

    if (file.exists()) {
        bool ok = file.open(QIODevice::ReadOnly | QIODevice::Text);

        if (ok) {
            QTextStream in(&file);

            double i_llh[3];
            bool i_llh_set = false;

            while(!in.atEnd()) {
                QString line = in.readLine();

                NmeaServer::nmea_gga_info_t gga;
                int res = NmeaServer::decodeNmeaGGA(line.toLocal8Bit(), gga);

                if (res > 5) {
                    if (!i_llh_set) {
                        if (ui->mapImportNmeaZeroEnuBox->isChecked()) {
                            i_llh[0] = gga.lat;
                            i_llh[1] = gga.lon;
                            i_llh[2] = gga.height;
                            ui->mapWidget->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                        } else {
                            ui->mapWidget->getEnuRef(i_llh);
                        }

                        i_llh_set = true;
                    }

                    double llh[3];
                    double xyz[3];

                    llh[0] = gga.lat;
                    llh[1] = gga.lon;
                    llh[2] = gga.height;
                    utility::llhToEnu(i_llh, llh, xyz);

                    LocPoint p;
                    p.setXY(xyz[0], xyz[1]);
                    QString info;

                    QString fix_t = "Unknown";
                    if (gga.fix_type == 4) {
                        fix_t = "RTK fix";
                        p.setColor(Qt::green);
                    } else if (gga.fix_type == 5) {
                        fix_t = "RTK float";
                        p.setColor(Qt::yellow);
                    } else if (gga.fix_type == 1) {
                        fix_t = "Single";
                        p.setColor(Qt::red);
                    }

                    info.sprintf("Fix type: %s\n"
                                 "Height  : %.2f",
                                 fix_t.toLocal8Bit().data(),
                                 gga.height);

                    p.setInfo(info);

                    if (!mapUpdated) {
                        mapUpdated = true;
                        ui->mapWidget->setNextEmptyOrCreateNewInfoTrace();
                    }

                    ui->mapWidget->addInfoPoint(p);
                }
            }
        } else {
            QMessageBox::warning(this, "Open Error", "Could not open " + file.fileName());
        }

    } else {
        QMessageBox::warning(this, "Open Error", "Please select a valid log file");
    }
}

void MainWindow::on_mapRemoveInfoAllButton_clicked()
{
    ui->mapWidget->clearAllInfoTraces();
}

void MainWindow::on_traceInfoMinZoomBox_valueChanged(double arg1)
{
    ui->mapWidget->setInfoTraceTextZoom(arg1);
}

void MainWindow::on_removeRouteExtraButton_clicked()
{
    on_mapRemoveRouteButton_clicked();
}

void MainWindow::on_mapOsmClearCacheButton_clicked()
{
    ui->mapWidget->osmClient()->clearCache();
    ui->mapWidget->update();
}

void MainWindow::on_mapOsmServerOsmButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://tile.openstreetmap.org");
    }
}

void MainWindow::on_mapOsmServerHiResButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://c.osm.rrze.fau.de/osmhd"); // Also https
    }
}

void MainWindow::on_mapOsmServerVedderButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://tiles.vedder.se/osm_tiles");
    }
}

void MainWindow::on_mapOsmServerVedderHdButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://tiles.vedder.se/osm_tiles_hd");
    }
}

void MainWindow::on_mapOsmMaxZoomBox_valueChanged(int arg1)
{
    ui->mapWidget->setOsmMaxZoomLevel(arg1);
}

void MainWindow::on_mapDrawGridBox_toggled(bool checked)
{
    ui->mapWidget->setDrawGrid(checked);
}

void MainWindow::on_mapGetEnuButton_clicked()
{
    mPacketInterface->getEnuRef(ui->mapCarBox->value());
}

void MainWindow::on_mapSetEnuButton_clicked()
{
    double llh[3];
    ui->mapWidget->getEnuRef(llh);
    mPacketInterface->setEnuRef(ui->mapCarBox->value(), llh);
}

void MainWindow::on_mapOsmStatsBox_toggled(bool checked)
{
    ui->mapWidget->setDrawOsmStats(checked);
}

void MainWindow::on_removeTraceExtraButton_clicked()
{
    ui->mapWidget->clearTrace();
}

void MainWindow::on_mapEditHelpButton_clicked()
{
    QMessageBox::information(this, tr("Keyboard shortcuts"),
                             tr("<b>CTRL + Left click:</b> Move selected car<br>"
                                "<b>CTRL + Right click:</b> Update route point speed<br>"
                                "<b>Shift + Left click:</b> Add route point<br>"
                                "<b>Shift + Left drag:</b> Move route point<br>"
                                "<b>Shift + right click:</b> Delete route point<br>"
                                "<b>CTRL + SHIFT + Left click:</b> Zero map ENU coordinates<br>"));
}

void MainWindow::on_mapStreamNmeaConnectButton_clicked()
{
    mNmea->connectClientTcp(ui->mapStreamNmeaServerEdit->text(),
                            ui->mapStreamNmeaPortBox->value());
}

void MainWindow::on_mapStreamNmeaDisconnectButton_clicked()
{
    mNmea->disconnectClientTcp();
}

void MainWindow::on_mapStreamNmeaClearTraceButton_clicked()
{
    ui->mapWidget->clearInfoTrace();
}

void MainWindow::on_mapRouteBox_valueChanged(int arg1)
{
    ui->mapWidget->setRouteNow(arg1);
}

void MainWindow::on_mapRemoveRouteAllButton_clicked()
{
    ui->mapWidget->clearAllRoutes();
}

void MainWindow::on_mapUpdateTimeButton_clicked()
{
    bool ok;
    int res = QInputDialog::getInt(this,
                                   tr("Set new route start time"),
                                   tr("Seconds from now"), 30, 0, 60000, 1, &ok);

    if (ok) {
        QList<LocPoint> route = ui->mapWidget->getRoute();
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());
        qint32 now = current.msecsSinceStartOfDay() + res * 1000;
        qint32 start_diff = 0;

        for (int i = 0;i < route.size();i++) {
            if (i == 0) {
                start_diff = now - route[i].getTime();
            }

            route[i].setTime(route[i].getTime() + start_diff);
        }

        ui->mapWidget->setRoute(route);
    }
}

void MainWindow::on_mapRouteTimeEdit_timeChanged(const QTime &time)
{
    ui->mapWidget->setRoutePointTime(time.msecsSinceStartOfDay());
}

void MainWindow::on_mapTraceMinSpaceCarBox_valueChanged(double arg1)
{
    ui->mapWidget->setTraceMinSpaceCar(arg1 / 1000.0);
}

void MainWindow::on_mapTraceMinSpaceGpsBox_valueChanged(double arg1)
{
    ui->mapWidget->setTraceMinSpaceGps(arg1 / 1000.0);
}

void MainWindow::on_mapInfoTraceBox_valueChanged(int arg1)
{
    ui->mapWidget->setInfoTraceNow(arg1);
}

void MainWindow::on_removeInfoTraceExtraButton_clicked()
{
    ui->mapWidget->clearInfoTrace();
}

void MainWindow::on_pollIntervalBox_valueChanged(int arg1)
{
    mTimer->setInterval(arg1);
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, "RControlStation",
                       tr("<b>RControlStation %1</b><br>"
                          "&copy; Benjamin Vedder 2016 - 2017<br>"
                          "<a href=\"mailto:benjamin@vedder.se\">benjamin@vedder.se</a><br>").
                       arg(mVersion));
}

void MainWindow::on_actionAboutLibrariesUsed_triggered()
{
    QMessageBox::about(this, "Libraries Used",
                       tr("<b>Icons<br>"
                          "<a href=\"https://icons8.com/\">https://icons8.com/</a><br><br>"
                          "<b>Plotting<br>"
                          "<a href=\"http://qcustomplot.com/\">http://qcustomplot.com/</a><br><br>"
                          "<b>Linear Algebra<br>"
                          "<a href=\"http://eigen.tuxfamily.org\">http://eigen.tuxfamily.org</a>"));
}

void MainWindow::on_actionExit_triggered()
{
    qApp->exit();
}

#ifdef HAS_SBS
bool MainWindow::initializeCar(quint8 id, bool hasCar, bool hasBase)
{
    bool carExists = false;

    if (hasCar) {
        // Connect serial, MOTE
        if(!mSerialPort->isOpen()) {
            ui->serialRefreshButton->click();

            for (int i=0; i<ui->serialPortBox->count(); ++i) {
                ui->serialPortBox->setCurrentIndex(i);
                if (ui->serialPortBox->currentText() == "COM3") {
                    ui->serialConnectButton->click();
                    break;
                }
            }

            if(!mSerialPort->isOpen()) {
                ui->networkInterface->sendError(
                            "Is the MOTE connected? It should appear as COM 3."
                            , "Couldn't find MOTE"
                            );
                return 0;
            }
        }
    }


    // Add car
    for (int i=0; i<mCars.size(); ++i) {
        if (mCars[i]->getId() == id) {
            carExists = true;
        }
    }

    if (!carExists) {
        CarInterface *car = new CarInterface(this);
        mCars.append(car);
        QString name;
        name.sprintf("Car %d", id);
        car->setID(id);
        ui->carsWidget->addTab(car, name);
        car->setMap(ui->mapWidget);
        car->setPacketInterface(mPacketInterface);
        connect(car, SIGNAL(showStatusInfo(QString,bool)), this, SLOT(showStatusInfo(QString,bool)));

        // Expose base station and nmeawidget to networkinterface
        BaseStation *pBase = ui->baseStationWidget;
        ui->networkInterface->setBaseStation(pBase);

        NmeaWidget *pNmea = car->getNmeaWidget();
        pNmea->setId(id);
        ui->networkInterface->setNmeaWidget(pNmea);
    }


    if (hasBase) {
        // Initialize base station
        if (!ui->baseStationWidget->initialize()) {
            ui->networkInterface->sendError(
                        "Is the ublox connected? It should appear as COM (No.)."
                        , "Couldn't find/connect to ublox"
                        );
            return 0;
        }
    }


    // Clear route
    on_mapRemoveRouteButton_clicked();

    return 1;
}

void MainWindow::setRPiClock(int id)
{
    for (int i=0; i<mCars.size(); ++i) {
        if (mCars[i]->getId() == id) {
            mCars[i]->setRPiClock();
        }
    }
}

void MainWindow::xboxButtonToggled(bool checked)
{
    mXbox = checked;
    ui->throttleGainBox->setEnabled(!checked);
    ui->steeringGainBox->setEnabled(!checked);
    ui->throttleDutyButton->setEnabled(!checked);
    ui->throttleCurrentButton->setEnabled(!checked);
}

void MainWindow::setUpdateRouteFromMap(int id, bool enabled)
{
    for (int i=0; i<mCars.size(); ++i) {
        if (mCars[i]->getId() == id) {
            mCars[i]->setUpdateRouteFromMap(enabled);
        }
    }
}

void MainWindow::setL1(bool L1) {
    mL1 = L1;
}

void MainWindow::setR1(bool R1) {
    mR1 = R1;
}

void MainWindow::setL2(double L2) {
    mL2 = L2;
}

void MainWindow::setR2(double R2) {
    mR2 = R2;
}

void MainWindow::setAxisLeftX(double AxisLeftX) {
    mAxisLeftX = AxisLeftX;
}

void MainWindow::setAxisLeftY(double AxisLeftY) {
    mAxisLeftY = AxisLeftY;
}

void MainWindow::setY(bool Y) {
    mY = Y;
}

void MainWindow::setA(bool A) {
    mA = A;
}
#endif













