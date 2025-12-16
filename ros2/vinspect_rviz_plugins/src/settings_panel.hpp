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
#include <QtWidgets> 

#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>

#include "vinspect_msgs/msg/settings.hpp"
#include "vinspect_msgs/srv/start_reconstruction.hpp"
#include "vinspect_msgs/srv/save_diconde.hpp"

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
  QLineEdit * depth_scale_;
  QLineEdit * depth_trunc_;

  QLineEdit * file_path_;

  QSlider * multi_pose_percentage;

  std::shared_ptr<rclcpp::Node> plugin_node_;
  rclcpp::Publisher<vinspect_msgs::msg::Settings>::SharedPtr settings_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dense_req_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dense_available_poses_publisher_;
  rclcpp::Client<vinspect_msgs::srv::StartReconstruction>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_client_;
  rclcpp::Client<vinspect_msgs::srv::SaveDICONDE>::SharedPtr save_diconde_client_;
  vinspect_msgs::msg::Settings sparse_settings_msg_;

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
  void requestStartClick();
  void requestStopClick();
  void denseReqClick();
  void denseAvailablePosesClick();
  void saveDicondeClick();
};

}  // namespace plugins

#endif  // SETTINGS_PANEL_HPP_
