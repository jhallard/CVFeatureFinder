#ifndef CVFF_MAINWINDOW_H_
#define CVFF_MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>

// ROS includes
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

 
namespace Ui {
    class CVFF_MainWindow;
}
 
class CVFF_MainWindow : public QMainWindow
{
    Q_OBJECT
 
public:
    explicit CVFF_MainWindow(QWidget *parent = 0);
    
    ~CVFF_MainWindow()
    {
    	int x = 0;
    }
 
private slots:
    void handleButton();
 
private:
    QPushButton *m_button;
};

#endif