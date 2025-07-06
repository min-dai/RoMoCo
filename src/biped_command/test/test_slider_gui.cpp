#include <gtest/gtest.h>
#include <QApplication>

#include "biped_command/radio_slider_gui.hpp"

// Needed because QApplication requires argc/argv.
int argc = 0;
char **argv = nullptr;
QApplication *app = nullptr;

class RadioSliderGUITest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      if (!app)
      {
         app = new QApplication(argc, argv);
      }
      gui_ = new RadioSliderGUI();
   }

   void TearDown() override
   {
      delete gui_;
   }

   RadioSliderGUI *gui_;
};

TEST_F(RadioSliderGUITest, InitialValuesAreCorrect)
{
   Eigen::VectorXd values = gui_->getSliderValues();
   ASSERT_EQ(values.size(), 9);

   EXPECT_EQ(values(0), 0); // SB slider initial: 2 - 2
   for (int i = 1; i < 9; ++i)
   {
      EXPECT_NEAR(values(i), 0.0, 1e-6); // all others: (50-50)/50 = 0
   }
}

TEST_F(RadioSliderGUITest, SliderValueChangesAreReflected)
{
   // Programmatically change sliders and check output
   gui_->findChild<QSlider *>()->setValue(3); // SB slider
   for (int i = 1; i < 9; ++i)
   {
      auto sliders = gui_->findChildren<QSlider *>();
      sliders[i]->setValue(100); // max
   }

   app->processEvents(); // Let Qt process signals

   Eigen::VectorXd values = gui_->getSliderValues();

   EXPECT_EQ(values(0), 1); // 3 - 2
   for (int i = 1; i < 9; ++i)
   {
      EXPECT_NEAR(values(i), 1.0, 1e-6); // (100-50)/50 = 1
   }
}
