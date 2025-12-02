// SPDX-FileCopyrightText: 2023 Lewe Christiansen <lewe.christiansen@dlr.de>
// SPDX-FileCopyrightText: 2023 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "area_panel.hpp"

using std::placeholders::_1;

namespace plugins
{

AreaPanel::AreaPanel(QWidget * parent)
: Panel(parent)
{
  // Creates a vertical box layout
  QVBoxLayout * display_layout = new QVBoxLayout(this);
  // adding a headline to this box
  display_layout->addWidget(new QLabel("Number of neighbors in area:"));
  show_in_area_ = new QLabel();
  show_in_area_->setText("Move interactive marker");
  display_layout->addWidget(show_in_area_);

  display_layout->addWidget(new QLabel("Direct neighbor ID:"));
  show_direct_neighbor_ = new QLabel();
  show_direct_neighbor_->setText("Move interactive marker");
  display_layout->addWidget(show_direct_neighbor_);

  display_layout->addWidget(new QLabel("Mean value:"));
  mean_value_ = new QLabel();
  mean_value_->setText("Move interactive marker");
  display_layout->addWidget(mean_value_);

  display_layout->addWidget(new QLabel("Min value:"));
  min_value_ = new QLabel();
  min_value_->setText("Move interactive marker");
  display_layout->addWidget(min_value_);

  display_layout->addWidget(new QLabel("Max value:"));
  max_value_ = new QLabel();
  max_value_->setText("Move interactive marker");
  display_layout->addWidget(max_value_);
}

void AreaPanel::save(const rviz_common::Config conf) const {rviz_common::Panel::save(conf);}

void AreaPanel::load(const rviz_common::Config & conf) {rviz_common::Panel::load(conf);}

void AreaPanel::onInitialize()
{
  /*
  Create a subscriber from a RosNodeAbstraction.
  This way this Plugin does not need to handle spinning the node for the subscriber to work.
  */
  display_data_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  data_subscriber_ = display_data_node_->create_subscription<vinspect_msgs::msg::AreaData>(
    "display_info", 10, std::bind(&AreaPanel::topicCallback, this, _1));
}

void AreaPanel::topicCallback(const vinspect_msgs::msg::AreaData & msg)
{
  /*
  Callback for the data_subscriber.
  Setting to QStrings to display recieved msg.
  */
  QString area_string = QString::fromStdString(msg.in_area);
  QString neighbor_string = QString::fromStdString(msg.next_neighbor);
  QString mean_string = QString::fromStdString(msg.mean);
  QString min_string = QString::fromStdString(msg.min);
  QString max_string = QString::fromStdString(msg.max);

  show_in_area_->setText(area_string);
  show_direct_neighbor_->setText(neighbor_string);
  mean_value_->setText(mean_string);
  min_value_->setText(min_string);
  max_value_->setText(max_string);
}

AreaPanel::~AreaPanel()
{
  // empty
}

}  // namespace plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(plugins::AreaPanel, rviz_common::Panel)
