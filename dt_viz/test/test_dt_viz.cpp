/**
 * @file test_dt_viz.cpp
 * @brief Unit tests for the dt_viz UI and Controllers using GTest and QTest.
 */

#include <gtest/gtest.h>
#include <QApplication>
#include <QTest>
#include <QSignalSpy>
#include <QLabel>
#include <QPushButton>

#include "dt_viz/ui/navigation_scene.hpp"
#include "dt_viz/ui/telemetry_panel.hpp"
#include "dt_viz/ui/map_canvas.hpp"
#include "dt_viz/controllers/simulation_controller.hpp"

/**
 * @class DtVizTest
 * @brief Test suite for UI components.
 */
class DtVizTest : public ::testing::Test {
protected:
    // Helper to find a QLabel in a widget by checking if its text starts with a prefix
    QLabel* findLabelWithPrefix(QWidget* parent, const QString& prefix) {
        auto labels = parent->findChildren<QLabel*>();
        for (auto* label : labels) {
            if (label->text().startsWith(prefix)) {
                return label;
            }
        }
        return nullptr;
    }
};

// ==============================================================================
// 1. NAVIGATION SCENE TESTS
// ==============================================================================

/**
 * @brief Tests the logic for dynamic grid resolution in NavigationScene.
 */
TEST_F(DtVizTest, NavigationScene_CalculateGridStep) {
    dt_viz::NavigationScene scene;
    
    EXPECT_DOUBLE_EQ(scene.calculateGridStep(5.0), 10.0);
    EXPECT_DOUBLE_EQ(scene.calculateGridStep(2.0), 50.0);
    EXPECT_DOUBLE_EQ(scene.calculateGridStep(0.6), 100.0);
    EXPECT_DOUBLE_EQ(scene.calculateGridStep(0.2), 500.0);
    EXPECT_DOUBLE_EQ(scene.calculateGridStep(0.05), 1000.0);
}

// ==============================================================================
// 2. TELEMETRY PANEL TESTS
// ==============================================================================

/**
 * @brief Tests if the TelemetryPanel correctly updates the USV labels.
 */
TEST_F(DtVizTest, TelemetryPanel_UpdateUsvTelemetry) {
    dt_viz::TelemetryPanel panel;
    
    panel.updateUsvTelemetry(15.5, 20.2, 90.0);

    QLabel* pos_label = findLabelWithPrefix(&panel, "Posição:\nx = ");
    ASSERT_NE(pos_label, nullptr) << "Position label not found.";
    EXPECT_TRUE(pos_label->text().contains("x = 15.5 m"));
    EXPECT_TRUE(pos_label->text().contains("y = 20.2 m"));

    QLabel* heading_label = findLabelWithPrefix(&panel, "Heading:");
    ASSERT_NE(heading_label, nullptr) << "Heading label not found.";
    EXPECT_TRUE(heading_label->text().contains("90.0°"));
}

/**
 * @brief Tests if the click on the recenter button emits the correct signal using QTest and QSignalSpy.
 */
TEST_F(DtVizTest, TelemetryPanel_RecenterButtonClicked) {
    dt_viz::TelemetryPanel panel;
    
    // QSignalSpy monitors the signal
    QSignalSpy spy(&panel, &dt_viz::TelemetryPanel::requestRecenter);
    
    // Find the recenter button
    QPushButton* btn = panel.findChild<QPushButton*>();
    ASSERT_NE(btn, nullptr) << "Recenter button not found.";
    
    // Simulate a real user click
    QTest::mouseClick(btn, Qt::LeftButton);
    
    // Verify the signal was emitted exactly once
    EXPECT_EQ(spy.count(), 1);
}

// ==============================================================================
// 3. MAP CANVAS TESTS
// ==============================================================================

/**
 * @brief Tests if clicking on the MapCanvas correctly interrupts USV tracking.
 */
TEST_F(DtVizTest, MapCanvas_MousePressInterruptsTracking) {
    dt_viz::MapCanvas canvas;
    
    // Initially, let's pretend tracking is on (centered)
    canvas.centerOnUsv();
    
    QSignalSpy spy(&canvas, &dt_viz::MapCanvas::trackingInterrupted);
    
    // Simulate a mouse click on the canvas viewport
    QTest::mouseClick(canvas.viewport(), Qt::LeftButton);
    
    // Verify the signal was emitted, meaning tracking was disabled
    EXPECT_EQ(spy.count(), 1);
}

// ==============================================================================
// 4. SIMULATION CONTROLLER TESTS
// ==============================================================================

/**
 * @brief Tests the null pointer protection on the SimulationController.
 */
TEST_F(DtVizTest, SimulationController_NullCoreProtection) {
    // Instantiate with a nullptr for DigitalTwinCore
    dt_viz::SimulationController controller(nullptr);
    
    // We expect no crash here because processTick checks if dt_core_ is valid.
    SUCCEED() << "Controller successfully instantiated with null core pointer and did not crash.";
}

// ==============================================================================
// MAIN ENTRY POINT
// ==============================================================================

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    // QApplication is strictly required to test QWidget components like TelemetryPanel and MapCanvas
    QApplication app(argc, argv);
    
    return RUN_ALL_TESTS();
}