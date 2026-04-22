#include "drive_widget.h"
#include "teleop_panel.h"
#include <rclcpp/time.hpp>

namespace teleop_rviz_plugin
{

TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz_common::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , mouse_pressed_( false )
  , mouse_pressed_sent_( false )
{
  node_ = rclcpp::Node::make_shared("teleop_panel_node");

  QVBoxLayout* layout = new QVBoxLayout;
  check_box_1_ = new QCheckBox( "Planning Attemptable", this );
  check_box_1_->setCheckState( Qt::Checked );
  layout->addWidget( check_box_1_ );
  check_box_2_ = new QCheckBox( "Update Visibility Graph", this );
  // Start frozen by default so any auto-loaded / read-from-file V-graph remains unchanged
  // until the user explicitly enables updates.
  check_box_2_->setCheckState( Qt::Unchecked );
  layout->addWidget( check_box_2_ );
  push_button_1_ = new QPushButton( "Reset Visibility Graph", this );
  layout->addWidget( push_button_1_ );
  push_button_2_ = new QPushButton( "Resume Navigation to Goal", this );
  layout->addWidget( push_button_2_ );
  push_button_3_ = new QPushButton( "Read", this );
  layout->addWidget( push_button_3_ );
  push_button_4_ = new QPushButton( "Save", this );
  layout->addWidget( push_button_4_ );
  drive_widget_ = new DriveWidget;
  layout->addWidget( drive_widget_ );
  setLayout( layout );

  QTimer* output_timer = new QTimer( this );

  connect( push_button_1_, SIGNAL( pressed() ), this, SLOT( pressButton1() ));
  connect( push_button_2_, SIGNAL( pressed() ), this, SLOT( pressButton2() ));
  connect( push_button_3_, SIGNAL( pressed() ), this, SLOT( pressButton3() ));
  connect( push_button_4_, SIGNAL( pressed() ), this, SLOT( pressButton4() ));
  connect( check_box_1_, SIGNAL( stateChanged(int) ), this, SLOT( clickBox1(int) ));
  connect( check_box_2_, SIGNAL( stateChanged(int) ), this, SLOT( clickBox2(int) ));
  connect( drive_widget_, SIGNAL( outputVelocity( float, float, bool )), this, SLOT( setVel( float, float, bool )));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  output_timer->start( 100 );

  velocity_publisher_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
  attemptable_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/planning_attemptable", 5);
  update_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/update_visibility_graph", 5);
  reset_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("/reset_visibility_graph", 5);
  read_publisher_ = node_->create_publisher<std_msgs::msg::String>("/read_file_dir", 5);
  save_publisher_ = node_->create_publisher<std_msgs::msg::String>("/save_file_dir", 5);

  resume_vgraph_client_ = node_->create_client<std_srvs::srv::Trigger>("/resume_visibility_graph_update");
  stop_vgraph_client_ = node_->create_client<std_srvs::srv::Trigger>("/stop_visibility_graph_update");

  load_vgraph_client_ = node_->create_client<std_srvs::srv::Trigger>("/load_visibility_graph");
  save_vgraph_client_ = node_->create_client<std_srvs::srv::Trigger>("/save_visibility_graph");


  drive_widget_->setEnabled( true );
}

void TeleopPanel::pressButton1()
{
  if (rclcpp::ok() && velocity_publisher_->get_subscription_count() > 0)
  {
    std_msgs::msg::Empty msg;
    reset_publisher_->publish(msg);
  }
}

void TeleopPanel::pressButton2()
{
  if (rclcpp::ok() && velocity_publisher_->get_subscription_count() > 0)
  {
    sensor_msgs::msg::Joy joy;
    
    joy.axes.push_back(0);
    joy.axes.push_back(0);
    joy.axes.push_back(-1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(0);
    joy.axes.push_back(0);

    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(1);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    joy.buttons.push_back(0);
    
    joy.header.stamp = node_->now();  // ROS2 uses node_->now() to get the current time
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_->publish(joy);
  }
}

void TeleopPanel::pressButton3()
{
  if (rclcpp::ok())
  {
    QString qFilename = QFileDialog::getOpenFileName(this, tr("Read File"), "/", tr("VGH - Visibility Graph Files (*.vgh)"));
    std::string filename = qFilename.toStdString();

    if (filename.empty()) {
      return;
    }

    // Preferred path: set FAR parameter and call service.
    node_->set_parameter(rclcpp::Parameter("vgraph_file_path", filename));
    if (load_vgraph_client_ && load_vgraph_client_->service_is_ready())
    {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      (void)load_vgraph_client_->async_send_request(req);
      return;
    }

    // Fallback: publish filename for older FARMaster versions.
    if (read_publisher_ && read_publisher_->get_subscription_count() > 0) {
      std_msgs::msg::String msg;
      msg.data = filename;
      read_publisher_->publish(msg);
    }
  }
}

void TeleopPanel::pressButton4()
{
  if (rclcpp::ok())
  {
    QString qFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "/", tr("VGH - Visibility Graph Files (*.vgh)"));

    std::string filename = qFilename.toStdString();
    if (filename != "") {
      int length = filename.length();
      if (length < 4) {
        filename += ".vgh";
      } else if (filename[length - 4] != '.' || filename[length - 3] != 'v' || filename[length - 2] != 'g' || filename[length - 1] != 'h') {
        filename += ".vgh";
      }
    }
    std_msgs::msg::String msg;
    if (filename.empty()) {
      return;
    }

    // Preferred path: set FAR parameter and call service.
    node_->set_parameter(rclcpp::Parameter("vgraph_file_path", filename));
    if (save_vgraph_client_ && save_vgraph_client_->service_is_ready())
    {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      (void)save_vgraph_client_->async_send_request(req);
      return;
    }

    // Fallback: publish filename for older FARMaster versions.
    if (save_publisher_ && save_publisher_->get_subscription_count() > 0) {
      msg.data = filename;
      save_publisher_->publish(msg);
    }
  } 
}

void TeleopPanel::clickBox1(int val)
{
  if (rclcpp::ok())
  {
    std_msgs::msg::Bool msg;
    // Qt::CheckState: 0=Unchecked, 2=Checked. Treat any non-zero as true.
    msg.data = (val != 0);
    attemptable_publisher_->publish(msg);
  }
}

void TeleopPanel::clickBox2(int val)
{
  if (rclcpp::ok())
  {
    // Qt::CheckState: 0=Unchecked, 2=Checked. Treat any non-zero as true.
    // checked  -> resume updates
    // unchecked-> freeze updates
    const bool enable_updates = (val != 0);

    // Prefer service semantics (one-shot command) if the FARMaster service servers exist.
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      enable_updates ? resume_vgraph_client_ : stop_vgraph_client_;

    if (client && client->service_is_ready())
    {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      // Fire-and-forget to avoid blocking the RViz UI.
      (void)client->async_send_request(req);
    }
    else
    {
      // Backward-compatible fallback: publish checkbox state on the existing topic.
      std_msgs::msg::Bool msg;
      msg.data = enable_updates;
      update_publisher_->publish(msg);
    }
  }
}

void TeleopPanel::setVel(float lin, float ang, bool pre)
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
  mouse_pressed_ = pre;
}

void TeleopPanel::sendVel()
{
  if (rclcpp::ok() && velocity_publisher_->get_subscription_count() > 0 && (mouse_pressed_ || mouse_pressed_sent_))
  {
    sensor_msgs::msg::Joy joy;

    joy.axes.push_back( 0 );
    joy.axes.push_back( 0 );
    joy.axes.push_back( 1.0 );
    joy.axes.push_back( angular_velocity_ );
    joy.axes.push_back( linear_velocity_ );
    joy.axes.push_back( 1.0 );
    joy.axes.push_back( 0 );
    joy.axes.push_back( 0 );

    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 1 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );
    joy.buttons.push_back( 0 );

    joy.header.stamp = node_->now();
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_->publish(joy);

    mouse_pressed_sent_ = mouse_pressed_;
  }
}

void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void TeleopPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

} // end namespace teleop_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(teleop_rviz_plugin::TeleopPanel, rviz_common::Panel)