#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "main.h"
#include <QImage>
#include <QPixmap>
QImage matToQImage(const cv::Mat &mat)
{
    switch (mat.type())
    {
    case CV_8UC1:
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    case CV_8UC3:
        {
            cv::Mat rgb;
            cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
            return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
        }
    case CV_8UC4:
        {
            cv::Mat rgba;
            //cv::cvtColor(mat, rgba, cv::COLOR_BGRA2RGBA);
            cv::cvtColor(mat, rgba, cv::COLOR_BGRA2RGBA);
            return QImage(rgba.data, rgba.cols, rgba.rows, rgba.step, QImage::Format_RGBA8888);
        }
    default:
        throw std::runtime_error("The provided matrix format is not supported in this example");
    }
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GraphInit();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::GraphInit()
{
    
    
    // include this section to fully disable antialiasing for higher performance:
    /*
    customPlot->setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    customPlot->xAxis->setTickLabelFont(font);
    customPlot->yAxis->setTickLabelFont(font);
    customPlot->legend->setFont(font);
    */
    
    for(int i =0;i<JOINTNUM ;i++){
        ui->widget_RealTimeGraph->addGraph(); // red line
        ui->widget_RealTimeGraph->graph(i)->setPen(QPen(QColor(40, 110, 255)));
        QString s1 = "Joint ";
        QString s2 = QString::number(i+1);
        QString legend_name = s1+s2;
        ui->widget_RealTimeGraph->graph(i)->setName(legend_name);
    }
    
  
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    ui->widget_RealTimeGraph->xAxis->setTicker(timeTicker);
    ui->widget_RealTimeGraph->axisRect()->setupFullAxesBox();
    ui->widget_RealTimeGraph->yAxis->setRange(-3.141592*2, 3.141592*2);   
    ui->widget_RealTimeGraph->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);    
    ui->widget_RealTimeGraph->legend->setVisible(true);
    ui->widget_RealTimeGraph->legend->setBrush(QBrush(QColor(255,255,255,150)));
    ui->widget_RealTimeGraph->axisRect()->insetLayout()->setInsetAlignment(0,Qt::AlignLeft|Qt::AlignTop);
    
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->widget_RealTimeGraph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->widget_RealTimeGraph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph->yAxis2, SLOT(setRange(QCPRange)));
    
    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&DataTimer, SIGNAL(timeout()), this, SLOT(RealtimeDataSlot()));
    DataTimer.start(10); // Interval 0 means to refresh as fast as possible
}

void MainWindow::RealtimeDataSlot()
{
    QImage qImage = matToQImage(img);

    ui->graphic_label->setPixmap(QPixmap::fromImage(qImage));
    ui->graphic_label->show();    
    static QTime time(QTime::currentTime());
    qsrand(QTime::currentTime().msecsSinceStartOfDay());

      // calculate two new data points:
      double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
      try{
        for(int i =0;i<JOINTNUM;i++){
            switch(this->displayMode){
                case JOINT_POSITION:ui->widget_RealTimeGraph->graph(i)->addData(key, robot_states_msg.robot_joint_position[i]);break;
                case JOINT_VELOCITY:ui->widget_RealTimeGraph->graph(i)->addData(key, robot_states_msg.robot_joint_velocity[i]);break;
                case JOINT_TORQUE:ui->widget_RealTimeGraph->graph(i)->addData(key, robot_states_msg.robot_joint_torque[i]);break;

            }


        }
      }
      
      catch(int err){
        for(int i =0;i<JOINTNUM;i++){
        ui->widget_RealTimeGraph->graph(i)->addData(key, 0);
        }
      }
        

     // if (key-lastPointKey > 0.002) // at most add point every 2 ms
     // {
        // add data to lines:

        // rescale value (vertical) axis to fit the current data:
    //    ui->customPlot->graph(0)->rescaleValueAxis();
    //    ui->widget_RealTimeGraph->graph(1)->rescaleValueAxis(true);
    //    lastPointKey = key;
    //  }
      // make key axis range scroll with the data (at a constant range size of 8):

      ui->widget_RealTimeGraph->xAxis->setRange(key,8,Qt::AlignRight);
      ui->widget_RealTimeGraph->replot();

      // calculate frames per second:
      static double lastFpsKey;
      static int frameCount;
      ++frameCount;
      if (key-lastFpsKey > 1) // average fps over 2 seconds
      {
        ui->statusBar->showMessage(
              QString("%1 FPS, Total Data points: %2")
              .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
              .arg(ui->widget_RealTimeGraph->graph(0)->data()->size()+ui->widget_RealTimeGraph->graph(1)->data()->size())
              , 0);
        lastFpsKey = key;
        frameCount = 0;
      }
}



void MainWindow::on_displayTypeComboBox_currentIndexChanged(const QString &arg1)
{

}

void MainWindow::on_displayTypeComboBox_currentIndexChanged(int index)
{
     //ui->label->setText(arg1);
    this->displayMode=index;
}

