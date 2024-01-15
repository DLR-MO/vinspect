// SPDX-FileCopyrightText: 2023 Lewe Christiansen <lewe.christiansen@dlr.de>
// SPDX-FileCopyrightText: 2023 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef SETTINGS_PANEL_HPP_
#define SETTINGS_PANEL_HPP_

#include <QButtonGroup>
#include <QDebug>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include "vinspect_msgs/msg/settings.hpp"
#include "vinspect_msgs/srv/start_reconstruction.hpp"

class string;
class QButtonGroup;
class QRadioButton;
class QHBoxLayout;
class QVBoxLayout;
class QWidget;
class QDebug;
class QSlider;
class VisParams;
class QLineEdit;

namespace plugins
{
class SettingsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SettingsPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const;
  void load(const rviz_common::Config & conf);

  virtual ~SettingsPanel();

protected:
  QRadioButton * radioButton1_;
  QRadioButton * radioButton2_;
  QSlider * transparency_slider_;
  QLineEdit * dot_size_input_;
  QLineEdit * sphere_radius_;
  QButtonGroup * button_group_object_;
  QButtonGroup * button_group_mean_;
  QButtonGroup * button_group_color_;

  QLineEdit * voxel_length_;
  QLineEdit * sdf_trunc_;
  QLineEdit * depth_scale_;
  QLineEdit * depth_trunc_;

  QLineEdit * file_path_;

  std::shared_ptr<rclcpp::Node> requester_node_;
  rclcpp::Publisher<vinspect_msgs::msg::Settings>::SharedPtr requester_publisher_;
  rclcpp::Client<vinspect_msgs::srv::StartReconstruction>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_client_;
  vinspect_msgs::msg::Settings settings_msg_;

protected Q_SLOTS:
  void onObjectButtonClick(const QString & text);
  void onMeanButtonClick(const QString & text);
  void onColorButtonClick(const QString & text);
  void onDotSizeChange(const std::string & text);
  void onSphereSizeChange(const std::string & text);
  void onTransparencyChange(int value);
  void pauseClick();
  void resumeClick();
  void clearSparseClick();
  void setDefaultClick(const std::reference_wrapper<QLineEdit *> input_objects[]);
  void clearClick(const std::reference_wrapper<QLineEdit *> input_objects[]);
  void requestStartClick();
  void requestStopClick();
};

}  // namespace plugins

#endif  // SETTINGS_PANEL_HPP_
