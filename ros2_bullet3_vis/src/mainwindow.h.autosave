#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtWidgets>

namespace Ui {
class MainWindow;
}
typedef enum{
    JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
}displayMode_;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    int displayMode=JOINT_POSITION;
    ~MainWindow();

private slots:
    void RealtimeDataSlot();

    void on_displayTypeComboBox_currentIndexChanged(const QString &arg1);

    void on_displayTypeComboBox_currentIndexChanged(int index);



private:
    Ui::MainWindow *ui;
    QTimer DataTimer;

    void GraphInit();

};

#endif // MAINWINDOW_H
