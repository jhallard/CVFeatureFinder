#ifndef CVFF_MAINWINDOW_H_
#define CVFF_MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
 
namespace Ui {
    class CVFF_MainWindow;
}
 
class CVFF_MainWindow : public QMainWindow
{
    Q_OBJECT
 
public:
    explicit CVFF_MainWindow(QWidget *parent = 0);
 
private slots:
    void handleButton();
 
private:
    QPushButton *m_button;
};

#endif