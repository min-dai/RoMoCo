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
     * - Sliders for SB, LV, LH, RV, RH, S1, S2, LS, RS parameters.
     * - Line edit for user input with integer validation.
     * - Signal-slot connections for handling slider changes and input updates.
     *
     * Usage:
     * 1. Create an instance of RadioSliderGUI.
     * 2. Use getSliderValues() to retrieve the current slider values as an Eigen::VectorXd.
     *
     * Note: This class requires the Qt framework to be properly set up in the development environment.
     */
class RadioSliderGUI : public QWidget {
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
    QSlider *sliderSB;
    QSlider *sliderLV;
    QSlider *sliderLH;
    QSlider *sliderRV;
    QSlider *sliderRH;
    QSlider *sliderS1;
    QSlider *sliderS2;
    QSlider *sliderLS;
    QSlider *sliderRS;
    
    QLabel *labelSB;
    QLabel *labelLV;
    QLabel *labelLH;
    QLabel *labelRV;
    QLabel *labelRH;
    QLabel *labelS1;
    QLabel *labelS2;
    QLabel *labelLS;
    QLabel *labelRS;
    

    QLineEdit *lineEdit;
    // QTimer *rosTimer;


    int sliderSB_value = -1;


    double sliderLV_value = 0;
    double sliderLH_value = 0;
    double sliderRV_value = 0;
    double sliderRH_value = 0;
    double sliderS1_value = 0;
    double sliderS2_value = 0;
    double sliderLS_value = 0;
    double sliderRS_value = 0;

    

    int input_value;
};

} // namespace romoco

#endif // SLIDER_GUI_HPP
