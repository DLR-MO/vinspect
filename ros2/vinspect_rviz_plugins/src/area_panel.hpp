// SPDX-FileCopyrightText: 2023 Lewe Christiansen <lewe.christiansen@dlr.de>
// SPDX-FileCopyrightText: 2023 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#ifndef AREA_PANEL_HPP_
#define AREA_PANEL_HPP_

#include <QDebug>
#include <QLabel>
#include <QVBoxLayout>

#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include "vinspect_msgs/msg/area_data.hpp"

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
class AreaPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AreaPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const;
  void load(const rviz_common::Config & conf);
  void topicCallback(const vinspect_msgs::msg::AreaData & msg);
  virtual ~AreaPanel();

private:
  std::shared_ptr<rclcpp::Node> display_data_node_;
  rclcpp::Subscription<vinspect_msgs::msg::AreaData>::SharedPtr data_subscriber_;

  QLabel * show_in_area_;
  QLabel * show_direct_neighbor_;
  QLabel * mean_value_;
  QLabel * min_value_;
  QLabel * max_value_;

protected Q_SLOTS:
};

}  // namespace plugins

#endif  // AREA_PANEL_HPP_
