#include <QApplication>
#include "gui/MainWindow.h"

int main(int argc, char *argv[]) {
    // 1. Initialize the Qt Application
    QApplication app(argc, argv);

    // 2. Instantiate and show the main window
    MainWindow w;
    w.setWindowTitle("AeroGrid - Drone Telemetry Center");
    w.resize(800, 600);
    w.show();

    // 3. Enter the event loop
    return app.exec();
}