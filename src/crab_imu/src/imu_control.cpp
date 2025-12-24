#include "imu_control.hpp"

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

rclcpp::Time timer_val;
rclcpp::Time timer_old;

int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {1,0,0},
  {0,1,0},
  {0,0,1}
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

float Temporary_Matrix[3][3]={
  {0,0,0},
  {0,0,0},
  {0,0,0}
};

class ImuControlNode : public rclcpp::Node
{
public:
  ImuControlNode() : Node("imu_control"), imu_on_(false)
  {
    this->declare_parameter<bool>("auto_start", false);

    body_state_.z = -0.08;
    body_state_.leg_radius = 0.11;

    body_cmd_sub_ = this->create_subscription<crab_msgs::msg::BodyCommand>(
      "/teleop/body_command", 1,
      std::bind(&ImuControlNode::teleopBodyCmd, this, std::placeholders::_1));

    move_body_pub_ = this->create_publisher<crab_msgs::msg::BodyState>("/teleop/move_body", 1);

    // Timer for main loop at 45Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(22),
      std::bind(&ImuControlNode::mainLoop, this));

    timer_val = this->now();
    timer_old = this->now();

    if (this->get_parameter("auto_start").as_bool()) {
      startImuControl();
    }
  }

private:
  void startImuControl()
  {
    if (imu_on_) {
      return;
    }
    setup_IMU();
    rclcpp::Rate r(25);
    while (body_state_.z >= -0.08) {
      body_state_.z -= 0.0025;
      r.sleep();
      move_body_pub_->publish(body_state_);
    }
    imu_on_ = true;
  }

  void stopImuControl()
  {
    if (!imu_on_) {
      return;
    }
    rclcpp::Rate r(25);
    body_state_.roll = 0;
    body_state_.pitch = 0;
    body_state_.yaw = 0;
    body_state_.x = 0;
    body_state_.y = 0;
    while (body_state_.z <= -0.016) {
      body_state_.z += 0.0025;
      r.sleep();
      move_body_pub_->publish(body_state_);
    }
    imu_on_ = false;
  }

  void setup_IMU()
  {
    RCLCPP_INFO(this->get_logger(), "Start initialization IMU");
    IMU_Init();
    usleep(20000);
    for(int i=0; i<32; i++)    // We take some readings...
    {
      Read_Gyro();
      Read_Accel();
      for(int y=0; y<6; y++)   // Cumulate values
        AN_OFFSET[y] += AN[y];
      usleep(20000);
    }

    for(int y=0; y<6; y++)
      AN_OFFSET[y] = AN_OFFSET[y]/32;

    AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

    timer_val = this->now();
    usleep(20000);
    counter=0;
  }

  void teleopBodyCmd(const crab_msgs::msg::BodyCommand::SharedPtr body_cmd)
  {
    if (body_cmd->cmd == crab_msgs::msg::BodyCommand::IMU_START_CMD) {
      startImuControl();
    }
    if (body_cmd->cmd == crab_msgs::msg::BodyCommand::IMU_STOP_CMD) {
      stopImuControl();
    }
  }

  void mainLoop()
  {
    if (!imu_on_) return;

#if CALIBRMODE == 0
    counter++;
    timer_old = timer_val;
    timer_val = this->now();
    if (timer_val > timer_old)
      G_Dt = (timer_val.seconds() - timer_old.seconds());
    else
      G_Dt = 0;

    // *** DCM algorithm
    // Data acquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();  // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();

    if (roll > 0.015) body_state_.roll = body_state_.roll + 0.1 * roll;
    if (roll < -0.015) body_state_.roll = body_state_.roll + 0.1 * roll;
    if (pitch > 0.015) body_state_.pitch = body_state_.pitch - 0.1 * pitch;
    if (pitch < -0.015) body_state_.pitch = body_state_.pitch - 0.1 * pitch;

    if (body_state_.roll > 0.35) body_state_.roll = 0.35;
    if (body_state_.roll < -0.35) body_state_.roll = -0.35;
    if (body_state_.pitch > 0.35) body_state_.pitch = 0.35;
    if (body_state_.pitch < -0.35) body_state_.pitch = -0.35;

    move_body_pub_->publish(body_state_);
#endif
  }

  bool imu_on_;
  crab_msgs::msg::BodyState body_state_;
  rclcpp::Publisher<crab_msgs::msg::BodyState>::SharedPtr move_body_pub_;
  rclcpp::Subscription<crab_msgs::msg::BodyCommand>::SharedPtr body_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
