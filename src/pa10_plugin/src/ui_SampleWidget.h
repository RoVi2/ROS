/********************************************************************************
** Form generated from reading UI file 'SampleWidget.ui'
**
** Created: Tue Feb 25 21:32:48 2014
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEWIDGET_H
#define UI_SAMPLEWIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SampleWidgetClass
{
public:
    QVBoxLayout *verticalLayout;
    QPushButton *_sendJointBtn;
    QPushButton *_getJointBtn;
    QCheckBox *_activateRobotChb;

    void setupUi(QWidget *SampleWidgetClass)
    {
        if (SampleWidgetClass->objectName().isEmpty())
            SampleWidgetClass->setObjectName(QString::fromUtf8("SampleWidgetClass"));
        SampleWidgetClass->resize(168, 202);
        verticalLayout = new QVBoxLayout(SampleWidgetClass);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        _sendJointBtn = new QPushButton(SampleWidgetClass);
        _sendJointBtn->setObjectName(QString::fromUtf8("_sendJointBtn"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Ignored);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(_sendJointBtn->sizePolicy().hasHeightForWidth());
        _sendJointBtn->setSizePolicy(sizePolicy);
        _sendJointBtn->setMaximumSize(QSize(16777215, 40));

        verticalLayout->addWidget(_sendJointBtn);

        _getJointBtn = new QPushButton(SampleWidgetClass);
        _getJointBtn->setObjectName(QString::fromUtf8("_getJointBtn"));
        sizePolicy.setHeightForWidth(_getJointBtn->sizePolicy().hasHeightForWidth());
        _getJointBtn->setSizePolicy(sizePolicy);
        _getJointBtn->setMaximumSize(QSize(16777215, 40));

        verticalLayout->addWidget(_getJointBtn);

        _activateRobotChb = new QCheckBox(SampleWidgetClass);
        _activateRobotChb->setObjectName(QString::fromUtf8("_activateRobotChb"));

        verticalLayout->addWidget(_activateRobotChb);


        retranslateUi(SampleWidgetClass);

        QMetaObject::connectSlotsByName(SampleWidgetClass);
    } // setupUi

    void retranslateUi(QWidget *SampleWidgetClass)
    {
        SampleWidgetClass->setWindowTitle(QApplication::translate("SampleWidgetClass", "Rovi2 Plugin", 0, QApplication::UnicodeUTF8));
        _sendJointBtn->setText(QApplication::translate("SampleWidgetClass", "sendJointBtn", 0, QApplication::UnicodeUTF8));
        _getJointBtn->setText(QApplication::translate("SampleWidgetClass", "getJointBtn", 0, QApplication::UnicodeUTF8));
        _activateRobotChb->setText(QApplication::translate("SampleWidgetClass", "Activate Robot", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SampleWidgetClass: public Ui_SampleWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEWIDGET_H
