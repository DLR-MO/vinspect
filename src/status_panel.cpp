// SPDX-FileCopyrightText: 2023 Lewe Christiansen <lewe.christiansen@dlr.de>
// SPDX-FileCopyrightText: 2023 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "status_panel.hpp"

using std::placeholders::_1;

namespace plugins
{

StatusPanel::StatusPanel(QWidget * parent)
: Panel(parent)
{
  // Creates a vertical box layout
  QVBoxLayout * display_layout = new QVBoxLayout(this);
  QHBoxLayout * hlayoutlabel = new QHBoxLayout(this);
  display_layout->addLayout(hlayoutlabel);
  hlayoutlabel->addWidget(new QLabel(""));
  hlayoutlabel->addWidget(new QLabel("Last valid:"));
  hlayoutlabel->addWidget(new QLabel("Last:"));
  // horizontal layout to make number be in same row as label
  QHBoxLayout * hlayoutx = new QHBoxLayout(this);
  display_layout->addLayout(hlayoutx);
  hlayoutx->addWidget(new QLabel("X:"));
  last_position_x_ = new QLabel();
  last_position_x_->setText("No data received yet");
  hlayoutx->addWidget(last_position_x_);
  last_position_in_x_ = new QLabel();
  last_position_in_x_->setText("No data received yet");
  hlayoutx->addWidget(last_position_in_x_);

  QHBoxLayout * hlayouty = new QHBoxLayout(this);
  display_layout->addLayout(hlayouty);
  hlayouty->addWidget(new QLabel("Y:"));
  last_position_y_ = new QLabel();
  last_position_y_->setText("No data received yet");
  hlayouty->addWidget(last_position_y_);
  last_position_in_y_ = new QLabel();
  last_position_in_y_->setText("No data received yet");
  hlayouty->addWidget(last_position_in_y_);

  QHBoxLayout * hlayoutz = new QHBoxLayout(this);
  display_layout->addLayout(hlayoutz);
  hlayoutz->addWidget(new QLabel("Z:"));
  last_position_z_ = new QLabel();
  last_position_z_->setText("No data received yet");
  hlayoutz->addWidget(last_position_z_);
  last_position_in_z_ = new QLabel();
  last_position_in_z_->setText("No data received yet");
  hlayoutz->addWidget(last_position_in_z_);

  QHBoxLayout * hlayoutv = new QHBoxLayout(this);
  display_layout->addLayout(hlayoutv);
  hlayoutv->addWidget(new QLabel("Last value:"));
  last_value_ = new QLabel();
  last_value_->setText("No data received yet");
  hlayoutv->addWidget(last_value_);
  last_value_in_ = new QLabel();
  last_value_in_->setText("No data received yet");
  hlayoutv->addWidget(last_value_in_);

  // horizontal divider line
  QWidget * horizontalLineWidget = new QWidget;
  horizontalLineWidget->setFixedHeight(2);
  horizontalLineWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  horizontalLineWidget->setStyleSheet(QString("background-color: #c0c0c0;"));
  display_layout->addWidget(horizontalLineWidget);

  QHBoxLayout * hlayoutn = new QHBoxLayout(this);
  display_layout->addLayout(hlayoutn);
  hlayoutn->addWidget(new QLabel("Number of recorded values:"));
  recorded_values_ = new QLabel();
  recorded_values_->setText("0");
  hlayoutn->addWidget(recorded_values_);

  QHBoxLayout * hlayouts = new QHBoxLayout(this);
  display_layout->addLayout(hlayouts);
  hlayouts->addWidget(new QLabel("Status:"));
  status_text_ = new QLabel();
  status_text_->setText("Unknown");
  hlayouts->addWidget(status_text_);

  // horizontal divider line
  QWidget * horizontalLineWidget2 = new QWidget;
  horizontalLineWidget2->setFixedHeight(2);
  horizontalLineWidget2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  horizontalLineWidget2->setStyleSheet(QString("background-color: #c0c0c0;"));
  display_layout->addWidget(horizontalLineWidget2);

  QHBoxLayout * hlayoutnd = new QHBoxLayout(this);
  display_layout->addLayout(hlayoutnd);
  hlayoutnd->addWidget(new QLabel("Number of integrated images:"));
  integrated_images_ = new QLabel();
  integrated_images_->setText("0");
  hlayoutnd->addWidget(integrated_images_);

  QHBoxLayout * hlayoutsd = new QHBoxLayout(this);
  display_layout->addLayout(hlayoutsd);
  hlayoutsd->addWidget(new QLabel("Status:"));
  dense_status_text_ = new QLabel();
  dense_status_text_->setText("Unknown");
  hlayoutsd->addWidget(dense_status_text_);
}

void StatusPanel::save(const rviz_common::Config conf) const {rviz_common::Panel::save(conf);}

void StatusPanel::load(const rviz_common::Config & conf) {rviz_common::Panel::load(conf);}

void StatusPanel::onInitialize()
{
  /*
  Create a subscriber from a RosNodeAbstraction.
  This way this Plugin does not need to handle spinning the node for the subscriber to work.
  */
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  sparse_data_subscriber_ = node_->create_subscription<vinspect_msgs::msg::Sparse>(
    "sparse", 10, std::bind(&StatusPanel::sparseCb, this, _1));
  sparse_incorrect_data_subscriber_ = node_->create_subscription<vinspect_msgs::msg::Sparse>(
    "sparse_incorrect", 10, std::bind(&StatusPanel::sparseIncorrectCb, this, _1));
  status_subscriber_ = node_->create_subscription<vinspect_msgs::msg::Status>(
    "vinspect/status", 10, std::bind(&StatusPanel::statusCb, this, std::placeholders::_1));
}

std::string round_string(double value, int precision = 2)
{
  std::stringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

void StatusPanel::sparseCb(const vinspect_msgs::msg::Sparse & msg)
{
  /*
  Callback for the data_subscriber.
  Status to QStrings to display recieved msg.
  */
  QString last_position_x =
    QString::fromStdString(round_string(msg.pose.position.x * 1000.0) + " mm");
  last_position_x_->setText(last_position_x);
  QString last_position_y =
    QString::fromStdString(round_string(msg.pose.position.y * 1000.0) + " mm");
  last_position_y_->setText(last_position_y);
  QString last_position_z =
    QString::fromStdString(round_string(msg.pose.position.z * 1000.0) + " mm");
  last_position_z_->setText(last_position_z);
  // todo this should show all data entries or be selectable which one to display
  // todo the unit should also be loaded dynamically, maybe add a service in
  // vinspect to querry the units once, so that they donÄt need to be send each time
  QString last_value = QString::fromStdString(round_string(msg.data[0]) + " mm");
  last_value_->setText(last_value);

  last_position_in_x_->setText(
    QString::fromStdString(round_string(msg.pose.position.x * 1000.0) + " mm"));
  last_position_in_y_->setText(
    QString::fromStdString(round_string(msg.pose.position.y * 1000.0) + " mm"));
  last_position_in_z_->setText(
    QString::fromStdString(round_string(msg.pose.position.z * 1000.0) + " mm"));
  last_value_in_->setText(QString::fromStdString(round_string(msg.data[0]) + " mm"));
}

void StatusPanel::sparseIncorrectCb(const vinspect_msgs::msg::Sparse & msg)
{
  /*
  Callback for the data_subscriber.
  Status to QStrings to display recieved msg.
  */
  last_position_in_x_->setText(
    QString::fromStdString(round_string(msg.pose.position.x * 1000.0) + " mm"));
  last_position_in_y_->setText(
    QString::fromStdString(round_string(msg.pose.position.y * 1000.0) + " mm"));
  last_position_in_z_->setText(
    QString::fromStdString(round_string(msg.pose.position.z * 1000.0) + " mm"));
  last_value_in_->setText(QString::fromStdString(round_string(msg.data[0]) + " mm"));
}

void StatusPanel::statusCb(const vinspect_msgs::msg::Status & msg)
{
  QString recorded_values_string = QString::fromStdString(std::to_string(msg.recorded_values));
  recorded_values_->setText(recorded_values_string);
  status_text_->setText(QString::fromStdString(msg.status));
  integrated_images_->setText(QString::fromStdString(std::to_string(msg.integrated_images)));
  dense_status_text_->setText(QString::fromStdString(msg.dense_status));
  // todo if we don't recieve a status after some time, we should stop displaying running
}

StatusPanel::~StatusPanel()
{
  // empty
}

}  // namespace plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(plugins::StatusPanel, rviz_common::Panel)
