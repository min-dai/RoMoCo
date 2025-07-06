#include "biped_command/radio_slider_gui.hpp"

RadioSliderGUI::RadioSliderGUI(QWidget *parent) : QWidget(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    sliderSB = new QSlider(Qt::Horizontal);
    sliderSB->setMinimum(0);
    sliderSB->setMaximum(3);
    sliderSB->setValue(2);
    layout->addWidget(sliderSB);
    labelSB = new QLabel("SB Value");
    layout->addWidget(labelSB);

    sliderLV = new QSlider(Qt::Horizontal);
    sliderLV->setMinimum(0);
    sliderLV->setMaximum(100);
    sliderLV->setValue(50);
    layout->addWidget(sliderLV);
    labelLV = new QLabel("LV Value");
    layout->addWidget(labelLV);

    sliderLH = new QSlider(Qt::Horizontal);
    sliderLH->setMinimum(0);
    sliderLH->setMaximum(100);
    sliderLH->setValue(50);
    layout->addWidget(sliderLH);
    labelLH = new QLabel("LH Value");
    layout->addWidget(labelLH);

    sliderRV = new QSlider(Qt::Horizontal);
    sliderRV->setMinimum(0);
    sliderRV->setMaximum(100);
    sliderRV->setValue(50);
    layout->addWidget(sliderRV);
    labelRV = new QLabel("RV Value");
    layout->addWidget(labelRV);

    sliderRH = new QSlider(Qt::Horizontal);
    sliderRH->setMinimum(0);
    sliderRH->setMaximum(100);
    sliderRH->setValue(50);
    layout->addWidget(sliderRH);
    labelRH = new QLabel("RH Value");
    layout->addWidget(labelRH);

    sliderS1 = new QSlider(Qt::Horizontal);
    sliderS1->setMinimum(0);
    sliderS1->setMaximum(100);
    sliderS1->setValue(50);
    layout->addWidget(sliderS1);
    labelS1 = new QLabel("S1 Value");
    layout->addWidget(labelS1);

    sliderS2 = new QSlider(Qt::Horizontal);
    sliderS2->setMinimum(0);
    sliderS2->setMaximum(100);
    sliderS2->setValue(50);
    layout->addWidget(sliderS2);
    labelS2 = new QLabel("S2 Value");
    layout->addWidget(labelS2);

    sliderLS = new QSlider(Qt::Horizontal);
    sliderLS->setMinimum(0);
    sliderLS->setMaximum(100);
    sliderLS->setValue(50);
    layout->addWidget(sliderLS);
    labelLS = new QLabel("LS Value");
    layout->addWidget(labelLS);

    sliderRS = new QSlider(Qt::Horizontal);
    sliderRS->setMinimum(0);
    sliderRS->setMaximum(100);
    sliderRS->setValue(50);
    layout->addWidget(sliderRS);
    labelRS = new QLabel("RS Value");
    layout->addWidget(labelRS);

    connect(sliderLV, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderLH, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderRV, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderRH, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderS1, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderS2, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderLS, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderRS, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);

    connect(sliderSB, &QSlider::valueChanged, this, &RadioSliderGUI::handleDTSliderChange);

    // lineEdit = new QLineEdit();
    // lineEdit->setValidator(new QIntValidator(0, 100, this));
    // layout->addWidget(lineEdit);
    // success = connect(lineEdit, &QLineEdit::textChanged, this, &RadioSliderGUI::handleInputChange);

    setLayout(layout);
}

Eigen::VectorXd RadioSliderGUI::getSliderValues() const
{
    Eigen::VectorXd slider_values(9);
    slider_values << sliderSB_value, sliderLV_value, sliderLH_value, sliderRV_value, sliderRH_value, sliderS1_value, sliderS2_value, sliderLS_value, sliderRS_value;
    return slider_values;
}

void RadioSliderGUI::handleDTSliderChange(int value)
{
    sliderSB_value = value - 2;
    labelSB->setText("SB Value: " + QString::number(sliderSB_value));
}

void RadioSliderGUI::handleCTSliderChange(int value)
{
    QSlider *slider = qobject_cast<QSlider *>(sender());
    if (slider == sliderLV)
    {
        sliderLV_value = (value - 50) / 50.0;
        labelLV->setText("LV Value: " + QString::number(sliderLV_value));
    }
    else if (slider == sliderLH)
    {
        sliderLH_value = (value - 50) / 50.0;
        labelLH->setText("LH Value: " + QString::number(sliderLH_value));
    }
    else if (slider == sliderRV)
    {
        sliderRV_value = (value - 50) / 50.0;
        labelRV->setText("RV Value: " + QString::number(sliderRV_value));
    }
    else if (slider == sliderRH)
    {
        sliderRH_value = (value - 50) / 50.0;
        labelRH->setText("RH Value: " + QString::number(sliderRH_value));
    }
    else if (slider == sliderS1)
    {
        sliderS1_value = (value - 50) / 50.0;
        labelS1->setText("S1 Value: " + QString::number(sliderS1_value));
    }
    else if (slider == sliderS2)
    {
        sliderS2_value = (value - 50) / 50.0;
        labelS2->setText("S2 Value: " + QString::number(sliderS2_value));
    }
    else if (slider == sliderLS)
    {
        sliderLS_value = (value - 50) / 50.0;
        labelLS->setText("LS Value: " + QString::number(sliderLS_value));
    }
    else if (slider == sliderRS)
    {
        sliderRS_value = (value - 50) / 50.0;
        labelRS->setText("RS Value: " + QString::number(sliderRS_value));
    }
}

void RadioSliderGUI::handleInputChange(const QString &text)
{
    input_value = text.toInt();
}
