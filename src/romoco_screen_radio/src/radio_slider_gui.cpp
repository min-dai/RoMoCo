#include "romoco_screen_radio/radio_slider_gui.hpp"

namespace romoco
{
RadioSliderGUI::RadioSliderGUI(QWidget *parent) : QWidget(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    sliderMode = new QSlider(Qt::Horizontal);
    sliderMode->setMinimum(0);
    sliderMode->setMaximum(3);
    sliderMode->setValue(1);
    layout->addWidget(sliderMode);
    labelMode = new QLabel("Mode Value: 0");
    layout->addWidget(labelMode);

    sliderX = new QSlider(Qt::Horizontal);
    sliderX->setMinimum(0);
    sliderX->setMaximum(100);
    sliderX->setValue(50);
    layout->addWidget(sliderX);
    labelX = new QLabel("X Value: 0");
    layout->addWidget(labelX);

    sliderY = new QSlider(Qt::Horizontal);
    sliderY->setMinimum(0);
    sliderY->setMaximum(100);
    sliderY->setValue(50);
    layout->addWidget(sliderY);
    labelY = new QLabel("Y Value: 0");
    layout->addWidget(labelY);

    sliderZ = new QSlider(Qt::Horizontal);
    sliderZ->setMinimum(0);
    sliderZ->setMaximum(100);
    sliderZ->setValue(50);
    layout->addWidget(sliderZ);
    labelZ = new QLabel("Z Value: 0");
    layout->addWidget(labelZ);

    sliderRoll = new QSlider(Qt::Horizontal);
    sliderRoll->setMinimum(0);
    sliderRoll->setMaximum(100);
    sliderRoll->setValue(50);
    layout->addWidget(sliderRoll);
    labelRoll = new QLabel("Roll Value: 0");
    layout->addWidget(labelRoll);

    sliderPitch = new QSlider(Qt::Horizontal);
    sliderPitch->setMinimum(0);
    sliderPitch->setMaximum(100);
    sliderPitch->setValue(50);
    layout->addWidget(sliderPitch);
    labelPitch = new QLabel("Pitch Value: 0");
    layout->addWidget(labelPitch);

    sliderYaw = new QSlider(Qt::Horizontal);
    sliderYaw->setMinimum(0);
    sliderYaw->setMaximum(100);
    sliderYaw->setValue(50);
    layout->addWidget(sliderYaw);
    labelYaw = new QLabel("Yaw Value: 0");
    layout->addWidget(labelYaw);

    sliderStepTime = new QSlider(Qt::Horizontal);
    sliderStepTime->setMinimum(0);
    sliderStepTime->setMaximum(100);
    sliderStepTime->setValue(50);
    layout->addWidget(sliderStepTime);
    labelStepTime = new QLabel("StepTime Value: 0");
    layout->addWidget(labelStepTime);

    sliderStepWidth = new QSlider(Qt::Horizontal);
    sliderStepWidth->setMinimum(0);
    sliderStepWidth->setMaximum(100);
    sliderStepWidth->setValue(50);
    layout->addWidget(sliderStepWidth);
    labelStepWidth = new QLabel("StepWidth Value: 0");
    layout->addWidget(labelStepWidth);

    connect(sliderMode, &QSlider::valueChanged, this, &RadioSliderGUI::handleDTSliderChange);

    connect(sliderX, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderY, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderZ, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderRoll, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderPitch, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderYaw, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderStepTime, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    connect(sliderStepWidth, &QSlider::valueChanged, this, &RadioSliderGUI::handleCTSliderChange);
    
    setLayout(layout);
}

Eigen::VectorXd RadioSliderGUI::getSliderValues() const
{
    Eigen::VectorXd slider_values(9);
    slider_values << sliderMode_value, sliderX_value, sliderY_value, sliderZ_value, sliderRoll_value, sliderPitch_value, sliderYaw_value, sliderStepTime_value, sliderStepWidth_value;
    return slider_values;
}

void RadioSliderGUI::handleDTSliderChange(int value)
{
    sliderMode_value = value - 2;
    labelMode->setText("Mode Value: " + QString::number(sliderMode_value));
}

void RadioSliderGUI::handleCTSliderChange(int value)
{
    QSlider *slider = qobject_cast<QSlider *>(sender());
    if (slider == sliderX)
    {
        sliderX_value = (value - 50) / 50.0;
        labelX->setText("X Value: " + QString::number(sliderX_value));
    }
    else if (slider == sliderY)
    {
        sliderY_value = (value - 50) / 50.0;
        labelY->setText("Y Value: " + QString::number(sliderY_value));
    }
    else if (slider == sliderZ)
    {
        sliderZ_value = (value - 50) / 50.0;
        labelZ->setText("Z Value: " + QString::number(sliderZ_value));
    }
    else if (slider == sliderRoll)
    {
        sliderRoll_value = (value - 50) / 50.0;
        labelRoll->setText("Roll Value: " + QString::number(sliderRoll_value));
    }
    else if (slider == sliderPitch)
    {
        sliderPitch_value = (value - 50) / 50.0;
        labelPitch->setText("Pitch Value: " + QString::number(sliderPitch_value));
    }
    else if (slider == sliderYaw)
    {
        sliderYaw_value = (value - 50) / 50.0;
        labelYaw->setText("Yaw Value: " + QString::number(sliderYaw_value));
    }
    else if (slider == sliderStepTime)
    {
        sliderStepTime_value = (value - 50) / 50.0;
        labelStepTime->setText("StepTime Value: " + QString::number(sliderStepTime_value));
    }
    else if (slider == sliderStepWidth)
    {
        sliderStepWidth_value = (value - 50) / 50.0;
        labelStepWidth->setText("StepWidth Value: " + QString::number(sliderStepWidth_value));
    }
    
}

void RadioSliderGUI::handleInputChange(const QString &text)
{
    input_value = text.toInt();
}
} // namespace romoco