#ifndef SLIDER_GUI_HPP
#define SLIDER_GUI_HPP

#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QIntValidator>
#include <QDebug>

#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace romoco
{
    /**
     * @class RadioSliderGUI
     * @ingroup group_ui
     * @brief A GUI class for controlling radio sliders using Qt.
     *
     * This class provides a graphical user interface (GUI) for controlling radio sliders.
     * It includes sliders for various parameters and a line edit for input values.
     * The GUI is built using the Qt framework and allows users to interactively adjust
     * slider values and input text.
     *
     * Main Features:
     * - Sliders for Mode, X, Y, Z, Roll, Pitch, Yaw, StepTime, StepWidth parameters.
     * - Line edit for user input with integer validation.
     * - Signal-slot connections for handling slider changes and input updates.
     *
     * Usage:
     * 1. Create an instance of RadioSliderGUI.
     * 2. Use getSliderValues() to retrieve the current slider values as an Eigen::VectorXd.
     *
     * Note: This class requires the Qt framework to be properly set up in the development environment.
     */
    class RadioSliderGUI : public QWidget
    {
        Q_OBJECT

    public:
        RadioSliderGUI(QWidget *parent = nullptr);
        Eigen::VectorXd getSliderValues() const;

    private slots:
        void handleCTSliderChange(int value);
        void handleDTSliderChange(int value);
        // void handleSliderChange(int value) ;
        void handleInputChange(const QString &text);

    private:
        QSlider *sliderMode;
        QSlider *sliderX;
        QSlider *sliderY;
        QSlider *sliderZ;
        QSlider *sliderRoll;
        QSlider *sliderPitch;
        QSlider *sliderYaw;
        QSlider *sliderStepTime;
        QSlider *sliderStepWidth;

        QLabel *labelMode;
        QLabel *labelX;
        QLabel *labelY;
        QLabel *labelZ;
        QLabel *labelRoll;
        QLabel *labelPitch;
        QLabel *labelYaw;
        QLabel *labelStepTime;
        QLabel *labelStepWidth;

        QLineEdit *lineEdit;

        int sliderMode_value = -1;

        double sliderX_value = 0;
        double sliderY_value = 0;
        double sliderZ_value = 0;
        double sliderRoll_value = 0;
        double sliderPitch_value = 0;
        double sliderYaw_value = 0;
        double sliderStepTime_value = 0;
        double sliderStepWidth_value = 0;

        int input_value;
    };

} // namespace romoco

#endif // SLIDER_GUI_HPP
