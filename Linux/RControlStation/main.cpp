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
#include <QApplication>
#include <QStyleFactory>

#ifdef HAS_SBS
#include <QQmlApplicationEngine>
#include <QQmlContext>
#endif

int main(int argc, char *argv[])
{
    //qputenv("QT_SCALE_FACTOR", QByteArray("1.5"));

    QApplication a(argc, argv);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    // Style
    a.setStyleSheet("");
    a.setStyle(QStyleFactory::create("Fusion"));

    // Settings
    a.setOrganizationName("RISE");
    a.setOrganizationDomain("ri.se");
    a.setApplicationName("RC Car Tool");

    MainWindow w;

#ifdef HAS_SBS
    // Has to be done in main, for some reason
    QQmlApplicationEngine engine;
    QQmlContext* ctx = engine.rootContext();
    ctx->setContextProperty("mController", &w);
    engine.load(QUrl(QStringLiteral("qrc:///xbox.qml")));

    w.showMinimized();

#else
    w.show();
#endif

    return a.exec();
}
