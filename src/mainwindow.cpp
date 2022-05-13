#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_toggled(bool checked)
{
    //ui->view->m_sim.toggleVOXELS();
    ui->view->m_smoke->toggleVOXELS();
}


void MainWindow::on_pushButton_2_toggled(bool checked)
{
//    ui->view->m_sim.toggleDENSITY();
    ui->view->m_smoke->toggleDENSITY();
}


void MainWindow::on_pushButton_3_toggled(bool checked)
{
//    ui->view->m_sim.toggleARROWS();
    ui->view->m_smoke->toggleARROWS();
}


void MainWindow::on_pushButton_clicked()
{
//    ui->view->m_sim.toggleVOXELS();
    ui->view->m_smoke->toggleVOXELS();
}


void MainWindow::on_pushButton_2_clicked()
{
//    ui->view->m_sim.toggleDENSITY();
    ui->view->m_smoke->toggleDENSITY();

}


void MainWindow::on_pushButton_3_clicked()
{
//    ui->view->m_sim.toggleARROWS();
    ui->view->m_smoke->toggleARROWS();
}

