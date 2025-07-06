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


    int sliderSB_value = 0;


    double sliderLV_value = 0;
    double sliderLH_value = 0;
    double sliderRV_value = 0;
    double sliderRH_value = 0;
    double sliderS1_value = 0;
    double sliderS2_value = 0;
    double sliderLS_value = 0;
    double sliderRS_value = 0;

    

    int input_value;
};;

#endif // SLIDER_GUI_HPP
