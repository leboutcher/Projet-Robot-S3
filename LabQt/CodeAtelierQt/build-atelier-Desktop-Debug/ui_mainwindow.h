/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>
#include "QtCharts"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QTextEdit *msgEditor;
    QPushButton *msgButton;
    QLabel *label_5;
    QChartView *graphicsView;
    QPushButton *lightButton;
    QComboBox *comboBoxPort;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(770, 366);
        MainWindow->setAcceptDrops(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        msgEditor = new QTextEdit(centralWidget);
        msgEditor->setObjectName(QString::fromUtf8("msgEditor"));

        gridLayout->addWidget(msgEditor, 4, 1, 1, 1);

        msgButton = new QPushButton(centralWidget);
        msgButton->setObjectName(QString::fromUtf8("msgButton"));

        gridLayout->addWidget(msgButton, 5, 1, 1, 1);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 0, 0, 1, 1);

        graphicsView = new QChartView(centralWidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));

        gridLayout->addWidget(graphicsView, 4, 0, 1, 1);

        lightButton = new QPushButton(centralWidget);
        lightButton->setObjectName(QString::fromUtf8("lightButton"));

        gridLayout->addWidget(lightButton, 5, 0, 1, 1);

        comboBoxPort = new QComboBox(centralWidget);
        comboBoxPort->setObjectName(QString::fromUtf8("comboBoxPort"));

        gridLayout->addWidget(comboBoxPort, 1, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Interface Identification", nullptr));
        msgButton->setText(QCoreApplication::translate("MainWindow", "Envoyer message", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Port:", nullptr));
        lightButton->setText(QCoreApplication::translate("MainWindow", "Allumer lumiere", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
