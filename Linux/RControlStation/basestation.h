/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef BASESTATION_H
#define BASESTATION_H

#include <QWidget>
#include <QTcpSocket>
#include <QTimer>
#include "ublox.h"
#include "tcpbroadcast.h"

namespace Ui {
class BaseStation;
}

class BaseStation : public QWidget
{
    Q_OBJECT

public:
    explicit BaseStation(QWidget *parent = 0);
    ~BaseStation();
    int getAvgPosLlh(double &lat, double &lon, double &height);

#ifdef HAS_SBS
    bool initialize();
    bool isValid();
    double getSamples();
//    double getPos(int i);
#endif

signals:
    void rtcmOut(QByteArray data);

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void timerSlot();
    void rxGga(int fields, NmeaServer::nmea_gga_info_t gga);
    void rxRawx(ubx_rxm_rawx rawx);

    void on_nmeaConnectButton_clicked();
    void on_nmeaSampleClearButton_clicked();
    void on_ubxSerialRefreshButton_clicked();
    void on_ubxSerialDisconnectButton_clicked();
    void on_ubxSerialConnectButton_clicked();
    void on_refGetButton_clicked();
    void on_tcpServerBox_toggled(bool checked);

#ifdef HAS_SBS
    void configTimerSlot();
#endif

private:
    Ui::BaseStation *ui;
    QTcpSocket *mTcpSocket;
    bool mTcpConnected;
    QTimer *mTimer;
    Ublox *mUblox;
    TcpBroadcast *mTcpServer;
    int mBasePosCnt;

    double mXNow;
    double mYNow;
    double mZNow;
    double mXAvg;
    double mYAvg;
    double mZAvg;
    double mAvgSamples;

    QString mFixNowStr;
    QString mSatNowStr;

    void updateNmeaText();

#ifdef HAS_SBS
    QTimer *configTimer;
    bool proceed;
    bool valid; // hack

    double lastOkPosX;
    double lastOkPosY;
    double lastOkPosZ;

    double xMaxNow;
    double yMaxNow;
    double zMaxNow;

    double xMaxAvg;
    double yMaxAvg;
    double zMaxAvg;
#endif

};

#endif // BASESTATION_H

