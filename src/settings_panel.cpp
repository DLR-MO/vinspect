// SPDX-FileCopyrightText: 2023 Lewe Christiansen <lewe.christiansen@dlr.de>
// SPDX-FileCopyrightText: 2023 Marc Bestmann <marc.bestmann@dlr.de>
//
// SPDX-License-Identifier: MIT

#include "settings_panel.hpp"

namespace plugins
{

SettingsPanel::SettingsPanel(QWidget * parent)
: Panel(parent),
  button_group_object_(new QButtonGroup()),
  button_group_mean_(new QButtonGroup()),
  button_group_color_(new QButtonGroup())
{
  float start_dot_size = 0.1;
  float start_sphere_size = 0.2;

  sparse_settings_msg_ = vinspect_msgs::msg::Settings();
  sparse_settings_msg_.transparency = 0.5;
  sparse_settings_msg_.dot_size = start_dot_size;
  sparse_settings_msg_.sphere_radius = start_sphere_size;
  sparse_settings_msg_.mean_min_max = vinspect_msgs::msg::Settings::MEAN;
  sparse_settings_msg_.custom_color = false;
  sparse_settings_msg_.resume = false;
  sparse_settings_msg_.pause = false;
  sparse_settings_msg_.clear = false;

  // Creates a vertical box layout
  QVBoxLayout * sparse_layout = new QVBoxLayout(this);

  // Transparency slider
  sparse_layout->addWidget(new QLabel("Transparency:"));
  QSlider * transparency_slider_ = new QSlider(Qt::Horizontal);
  transparency_slider_->setFocusPolicy(Qt::StrongFocus);
  transparency_slider_->setTickPosition(QSlider::TicksBothSides);
  transparency_slider_->setTickInterval(10);
  transparency_slider_->setSingleStep(1);
  transparency_slider_->setTracking(false);
  // slider scale is in 0-100 not 0-1
  transparency_slider_->setValue(sparse_settings_msg_.transparency * 100);
  sparse_layout->addWidget(transparency_slider_);
  connect(transparency_slider_, SIGNAL(valueChanged(int)), this, SLOT(onTransparencyChange(int)));

  // horizontal divider line
  QWidget * horizontal_line_widget_1 = new QWidget;
  horizontal_line_widget_1->setFixedHeight(2);
  horizontal_line_widget_1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  horizontal_line_widget_1->setStyleSheet(QString("background-color: #c0c0c0;"));
  sparse_layout->addWidget(horizontal_line_widget_1);

  // dot size
  sparse_layout->addWidget(new QLabel("Dot radius:"));
  dot_size_input_ = new QLineEdit(QString::fromStdString(std::to_string(start_dot_size)));
  dot_size_input_->setPlaceholderText("Set new dot radius");
  connect(
    dot_size_input_, &QLineEdit::returnPressed, this, [this] {
      onDotSizeChange(dot_size_input_->text().toStdString());
    });
  sparse_layout->addWidget(dot_size_input_);

  // sphere radius
  sparse_layout->addWidget(new QLabel("Sphere radius:"));
  sphere_radius_ = new QLineEdit(QString::fromStdString(std::to_string(start_sphere_size)));
  sphere_radius_->setPlaceholderText("Set new selection sphere radius");
  connect(
    sphere_radius_, &QLineEdit::returnPressed, this, [this] {
      onSphereSizeChange(sphere_radius_->text().toStdString());
    });
  sparse_layout->addWidget(sphere_radius_);

  // mean/min/max
  sparse_layout->addWidget(new QLabel("Merging method:"));
  QStringList button_names_mean = {"Mean", "Min", "Max"};
  for (int i = 0; i < button_names_mean.size(); ++i) {
    QString name = button_names_mean[i];
    // creating radioButton ojects
    QRadioButton * button = new QRadioButton(name);
    button_group_mean_->addButton(button, i);
    // signal/slot connection with lambda function as slot to handle
    // all buttons with one "onButtonClick"
    connect(button, &QRadioButton::clicked, [this, name] {onMeanButtonClick(name);});
    if (i == 0) {
      button->setChecked(true);
    }
    // adding the buttons to the box
    sparse_layout->addWidget(button);
  }

  // custom color
  sparse_layout->addWidget(new QLabel("Use custom color:"));
  QStringList button_names_colo = {"Off", "On"};
  for (int i = 0; i < button_names_colo.size(); ++i) {
    QString name = button_names_colo[i];
    QRadioButton * button = new QRadioButton(name);
    button_group_color_->addButton(button, i);
    connect(button, &QRadioButton::clicked, [this, name] {onColorButtonClick(name);});
    if (i == 0) {
      button->setChecked(true);
    }
    sparse_layout->addWidget(button);
  }
  sparse_layout->addWidget(new QLabel("Note: Remeshing might take a while"));

  // horizontal divider line
  QWidget * horizontal_line_widget_2 = new QWidget;
  horizontal_line_widget_2->setFixedHeight(2);
  horizontal_line_widget_2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  horizontal_line_widget_2->setStyleSheet(QString("background-color: #c0c0c0;"));
  sparse_layout->addWidget(horizontal_line_widget_2);

  // horizontal layout for buttons
  QHBoxLayout * hlayout = new QHBoxLayout(this);
  sparse_layout->addLayout(hlayout);

  QPushButton * pause_button = new QPushButton("Pause");
  connect(pause_button, &QPushButton::clicked, [this] {pauseClick();});
  hlayout->addWidget(pause_button);

  QPushButton * resume_button = new QPushButton("Resume");
  connect(resume_button, &QPushButton::clicked, [this] {resumeClick();});
  hlayout->addWidget(resume_button);

  QPushButton * clear_sparse_button = new QPushButton("Clear");
  connect(clear_sparse_button, &QPushButton::clicked, [this] {clearSparseClick();});
  hlayout->addWidget(clear_sparse_button);

  // dense settings
  QGridLayout * dense_layout = new QGridLayout(this);
  QStringList input_names = {"voxel_length_", "sdf_trunc_", "depth_scale_", "depth_trunc_"};
  std::reference_wrapper<QLineEdit *> input_objects[] = {
    voxel_length_, sdf_trunc_, depth_scale_, depth_trunc_};
  int row = 0;
  int column = 0;
  for (int i = 0; i < 4; i++) {
    input_objects[i].get() = new QLineEdit("");
    input_objects[i].get()->setPlaceholderText("Set " + input_names[i]);
    dense_layout->addWidget(input_objects[i], row, column);
    if (row < 5) {
      row++;
    } else {
      column++;
      row = 0;
    }
  }
  QPushButton * service_start_button = new QPushButton("Send start request");
  connect(service_start_button, &QPushButton::clicked, [this] {requestStartClick();});
  dense_layout->addWidget(service_start_button, 7, 0, 1, 2);

  QPushButton * default_button = new QPushButton("Set default values");
  connect(
    default_button, &QPushButton::clicked, [this, input_objects] {
      setDefaultClick(input_objects);
    });
  dense_layout->addWidget(default_button, 6, 0);

  QPushButton * clear_button = new QPushButton("Clear");
  connect(
    clear_button, &QPushButton::clicked, [this, input_objects] {clearClick(input_objects);});
  dense_layout->addWidget(clear_button, 6, 1);

  QWidget * horizontal_line_widget_3 = new QWidget;
  horizontal_line_widget_3->setFixedHeight(2);
  horizontal_line_widget_3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  horizontal_line_widget_3->setStyleSheet(QString("background-color: #c0c0c0;"));
  dense_layout->addWidget(horizontal_line_widget_3, 9, 0, 1, 2);

  QPushButton * service_stop_button = new QPushButton("Send stop request");
  connect(service_stop_button, &QPushButton::clicked, [this] {requestStopClick();});
  dense_layout->addWidget(service_stop_button, 12, 0, 1, 2);

  QWidget * horizontal_line_widget_4 = new QWidget;
  horizontal_line_widget_4->setFixedHeight(2);
  horizontal_line_widget_4->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  horizontal_line_widget_4->setStyleSheet(QString("background-color: #c0c0c0;"));
  dense_layout->addWidget(horizontal_line_widget_4, 13, 0, 1, 2);

  QPushButton * dense_req_button = new QPushButton("Get dense data at pose");
  connect(dense_req_button, &QPushButton::clicked, [this] {denseReqClick();});
  dense_layout->addWidget(dense_req_button, 15, 0, 1, 2);

  QPushButton * dense_available_poses_button = new QPushButton("Show available dense poses");
  connect(dense_available_poses_button, &QPushButton::clicked, [this] {denseAvailablePosesClick();});
  dense_layout->addWidget(dense_available_poses_button, 16, 0);

  multi_pose_percentage = new QLineEdit("");
  multi_pose_percentage->setToolTip("Enter an intger from 0 between 100");
  multi_pose_percentage->setText("50");
  dense_layout->addWidget(multi_pose_percentage, 16, 1);

  // make tab layout of sparse and dense
  QTabWidget * tab_widget = new QTabWidget(this);
  auto sparse_widget = new QWidget;
  auto dense_widget = new QWidget;
  sparse_widget->setLayout(sparse_layout);
  dense_widget->setLayout(dense_layout);
  tab_widget->addTab(sparse_widget, "Sparse Data");
  tab_widget->addTab(dense_widget, "Dense Data");
}

void SettingsPanel::save(const rviz_common::Config conf) const {rviz_common::Panel::save(conf);}

void SettingsPanel::load(const rviz_common::Config & conf) {rviz_common::Panel::load(conf);}

void SettingsPanel::onInitialize()
{
  plugin_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  rclcpp::QoS latching_qos = rclcpp::QoS(1).transient_local();
  settings_publisher_ = plugin_node_->create_publisher<vinspect_msgs::msg::Settings>(
    "vinspect/settings", latching_qos);
  // publish settings message once with latching
  settings_publisher_->publish(sparse_settings_msg_);
  start_client_ = plugin_node_->create_client<vinspect_msgs::srv::StartReconstruction>(
    "/vinspect/start_reconstruction");
  stop_client_ =
    plugin_node_->create_client<std_srvs::srv::Empty>("/vinspect/stop_reconstruction");
  dense_req_publisher_ = plugin_node_->create_publisher<std_msgs::msg::String>(
    "vinspect/dense_data_req", latching_qos);
  dense_available_poses_publisher_ = plugin_node_->create_publisher<std_msgs::msg::Int32>(
    "vinspect/multi_dense_data_req", latching_qos);
}


void SettingsPanel::onMeanButtonClick(const QString & text)
{
  /*
    If the merge method is changed, we publish an update.
  */
  uint8_t new_mean_min_max;
  if (text.toStdString() == "Mean") {
    new_mean_min_max = vinspect_msgs::msg::Settings::MEAN;
  } else if (text.toStdString() == "Min") {
    new_mean_min_max = vinspect_msgs::msg::Settings::MIN;
  } else if (text.toStdString() == "Max") {
    new_mean_min_max = vinspect_msgs::msg::Settings::MAX;
  } else {
    RCLCPP_ERROR(plugin_node_->get_logger(), "Mean/Min/Max not recognized");
    return;
  }
  if (new_mean_min_max != sparse_settings_msg_.mean_min_max) {
    sparse_settings_msg_.mean_min_max = new_mean_min_max;
    settings_publisher_->publish(sparse_settings_msg_);
  }
}

void SettingsPanel::onColorButtonClick(const QString & text)
{
  bool new_color;
  if (text.toStdString() == "Off") {
    new_color = false;
  } else if (text.toStdString() == "On") {
    new_color = true;
  } else {
    RCLCPP_ERROR(plugin_node_->get_logger(), "Color not recognized");
    return;
  }
  if (new_color != sparse_settings_msg_.custom_color) {
    sparse_settings_msg_.custom_color = new_color;
    settings_publisher_->publish(sparse_settings_msg_);
  }
}

void SettingsPanel::onDotSizeChange(const std::string & text)
{
  std::cout << "Dot size changed to " << text << std::endl;
  double value = std::stod(text);
  if (value != sparse_settings_msg_.dot_size) {
    sparse_settings_msg_.dot_size = value;
    settings_publisher_->publish(sparse_settings_msg_);
  }
}

void SettingsPanel::onSphereSizeChange(const std::string & text)
{
  std::cout << "Sphere size changed to " << text << std::endl;
  double value = std::stod(text);
  if (value != sparse_settings_msg_.sphere_radius) {
    sparse_settings_msg_.sphere_radius = value;
    settings_publisher_->publish(sparse_settings_msg_);
  }
}

void SettingsPanel::onTransparencyChange(int value)
{
  /*
  Is called when the slider value changes.
  */
  float value_f = static_cast<float>(value);
  // Not allowed to have a Value of 0, value is needed between 0 and 1
  float transparency = ((value_f + 1) / 100);
  if (transparency != sparse_settings_msg_.transparency) {
    sparse_settings_msg_.transparency = transparency;
    settings_publisher_->publish(sparse_settings_msg_);
  }
}

void SettingsPanel::pauseClick()
{
  sparse_settings_msg_.pause = true;
  settings_publisher_->publish(sparse_settings_msg_);
  sparse_settings_msg_.pause = false;
}

void SettingsPanel::resumeClick()
{
  sparse_settings_msg_.resume = true;
  settings_publisher_->publish(sparse_settings_msg_);
  sparse_settings_msg_.resume = false;
}

void SettingsPanel::clearSparseClick()
{
  sparse_settings_msg_.clear = true;
  settings_publisher_->publish(sparse_settings_msg_);
  sparse_settings_msg_.clear = false;
}

void SettingsPanel::requestStartClick()
{
  auto request = std::make_shared<vinspect_msgs::srv::StartReconstruction::Request>();

  request->voxel_length = voxel_length_->text().toFloat();
  request->sdf_trunc = sdf_trunc_->text().toFloat();

  request->depth_scale = depth_scale_->text().toFloat();
  request->depth_trunc = depth_trunc_->text().toFloat();

  using ServiceResponseFutureStart =
    rclcpp::Client<vinspect_msgs::srv::StartReconstruction>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFutureStart future) {
      auto result = future.get();
      auto x = result ? "true" : "false";
      std::cout << x << std::endl;
      RCLCPP_INFO(plugin_node_->get_logger(), x);
    };
  auto future_result = start_client_->async_send_request(request, response_received_callback);
}

void SettingsPanel::requestStopClick()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  using ServiceResponseFutureStop = rclcpp::Client<std_srvs::srv::Empty>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFutureStop future) {
      auto result = future.get();
      auto x = result ? "true" : "false";
      std::cout << x << std::endl;
      RCLCPP_INFO(plugin_node_->get_logger(), x);
    };
  auto future_result = stop_client_->async_send_request(request, response_received_callback);
}

void SettingsPanel::setDefaultClick(const std::reference_wrapper<QLineEdit *> input_objects[])
{
  QStringList default_input = {"0.02", "0.04", "1000.0", "0.75"};
  for (int i = 0; i < 4; i++) {
    input_objects[i].get()->setText(default_input[i]);
  }
}

void SettingsPanel::clearClick(const std::reference_wrapper<QLineEdit *> input_objects[])
{
  for (int i = 0; i < 4; i++) {
    input_objects[i].get()->setText("");
  }
}

void SettingsPanel::denseReqClick()
{
  auto message = std_msgs::msg::String();
  message.data = "request";
  dense_req_publisher_->publish(message);
}

void SettingsPanel::denseAvailablePosesClick()
{
  /*
  Potential ToDo:
  Create new ROS msg with more options like percentage of poses to show, color, heatmap etc. 
  */
  auto message = std_msgs::msg::Int32();
  int percentage = multi_pose_percentage->text().toInt();
  if (percentage > 0 && percentage <= 100)
  {
    message.data = percentage;
    dense_available_poses_publisher_->publish(message);
  }
  else
  {
    RCLCPP_ERROR(plugin_node_->get_logger(), "Multiple pose request value not > 0 and >= 100");
  }
}

SettingsPanel::~SettingsPanel()
{
  // empty
}

}  // namespace plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(plugins::SettingsPanel, rviz_common::Panel)
