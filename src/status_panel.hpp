// SPDX-FileCopyrightText: 2023 Lewe Christiansen <lewe.christiansen@dlr.de>
// SPDX-FileCopyrightText: 2023 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef STATUS_PANEL_HPP_
#define STATUS_PANEL_HPP_

#include <QDebug>
#include <QLabel>
#include <QVBoxLayout>

#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include "vinspect_msgs/msg/sparse.hpp"
#include "vinspect_msgs/msg/status.hpp"

class string;
class QRadioButton;
class QHBoxLayout;
class QVBoxLayout;
class QWidget;
class QLabel;
class QSlider;
class VisParams;

namespace plugins
{
class StatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit StatusPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const;
  void load(const rviz_common::Config & conf);
  void sparseCb(const vinspect_msgs::msg::Sparse & msg);
  void sparseIncorrectCb(const vinspect_msgs::msg::Sparse & msg);
  void statusCb(const vinspect_msgs::msg::Status & msg);
  virtual ~StatusPanel();

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<vinspect_msgs::msg::Sparse>::SharedPtr sparse_data_subscriber_;
  rclcpp::Subscription<vinspect_msgs::msg::Sparse>::SharedPtr sparse_incorrect_data_subscriber_;
  rclcpp::Subscription<vinspect_msgs::msg::Status>::SharedPtr status_subscriber_;

  QLabel * last_position_x_;
  QLabel * last_position_y_;
  QLabel * last_position_z_;
  QLabel * last_value_;
  QLabel * last_position_in_x_;
  QLabel * last_position_in_y_;
  QLabel * last_position_in_z_;
  QLabel * last_value_in_;
  QLabel * recorded_values_;
  QLabel * status_text_;
  QLabel * dense_status_text_;
  QLabel * integrated_images_;

protected Q_SLOTS:
};

}  // namespace plugins

#endif  // STATUS_PANEL_HPP_
