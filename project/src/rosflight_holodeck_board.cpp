#include "rosflight_holodeck/rosflight_holodeck_board.h"

RFHBoard::RFHBoard() :
normal_distribution_(0,1.0),
uniform_distribution_(-1, 1),
gravity_{0, 0, 9.80665},
time_us_(0)
{ 
}

void RFHBoard::init_board(ros::NodeHandle *nh, State *x)
{
  nh_ = nh;
  x_ = x;

  // get mav model parameters
  mav_type_ = nh_->param<std::string>("mav_type", "multirotor");
  
  gyro_stdev_ = nh_->param<double>("gyro_stdev", 0.13);
  gyro_bias_range_ = nh_->param<double>("gyro_bias_range", 0.15);
  gyro_bias_walk_stdev_ = nh_->param<double>("gyro_bias_walk_stdev", 0.001);

  acc_stdev_ = nh_->param<double>("acc_stdev", 1.15);
  acc_bias_range_ = nh_->param<double>("acc_bias_range", 0.15);
  acc_bias_walk_stdev_ = nh_->param<double>("acc_bias_walk_stdev", 0.001);

  mag_stdev_ = nh_->param<double>("mag_stdev", 1.15);
  mag_bias_range_ = nh_->param<double>("mag_bias_range", 0.15);
  mag_bias_walk_stdev_ = nh_->param<double>("mag_bias_walk_stdev", 0.001);

  baro_stdev_ = nh_->param<double>("baro_stdev", 1.15);
  baro_bias_range_ = nh_->param<double>("baro_bias_range", 0.15);
  baro_bias_walk_stdev_ = nh_->param<double>("baro_bias_walk_stdev", 0.001);

  airspeed_stdev_ = nh_->param<double>("airspeed_stdev", 1.15);
  airspeed_bias_range_ = nh_->param<double>("airspeed_bias_range", 0.15);
  airspeed_bias_walk_stdev_ = nh_->param<double>("airspeed_bias_walk_stdev", 0.001);

  sonar_stdev_ = nh_->param<double>("sonar_stdev", 1.15);
  sonar_min_range_ = nh_->param<double>("sonar_min_range", 0.25);
  sonar_max_range_ = nh_->param<double>("sonar_max_range", 8.0);

  imu_update_rate_ = nh_->param<double>("imu_update_rate", 1000.0);
  imu_update_period_us_ = (uint64_t)(1e6/imu_update_rate_);

  ground_altitude_ = nh_->param<double>("ground_altitude", 1387.0);

  double inclination = nh_->param<double>("inclination", 1.14316156541);
  double declination = nh_->param<double>("declination", 0.198584539676);
  inertial_magnetic_field_.x() = sin(-inclination);
  inertial_magnetic_field_.y() = cos(-inclination)*cos(-declination);
  inertial_magnetic_field_.z() = cos(-inclination)*sin(-declination);
}

void RFHBoard::board_reset(bool bootloader)
{
}

void RFHBoard::clock_delay(uint32_t milliseconds)
{
}

// sensors
void RFHBoard::sensors_init()
{
  next_imu_update_time_us_ = 0;

  gyro_bias_ = gyro_bias_range_ * randomUniform<double, 3, 1>(uniform_distribution_, random_generator_);
  acc_bias_  = acc_bias_range_ * randomUniform<double, 3, 1>(uniform_distribution_, random_generator_);
  mag_bias_ = mag_bias_range_ * randomUniform<double, 3, 1>(uniform_distribution_, random_generator_);
  baro_bias_ = baro_bias_range_*uniform_distribution_(random_generator_);
}

uint16_t RFHBoard::num_sensor_errors(void)
{
  return 0;
}

bool RFHBoard::new_imu_data()
{
  uint64_t now_us = clock_micros();
  if (now_us >= next_imu_update_time_us_)
  {
    next_imu_update_time_us_ = now_us + imu_update_period_us_;
    return true;
  }
  else
  {
    return false;
  }
}

bool RFHBoard::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  // Simulate imu measurements from truth

  Eigen::Vector3d y_acc;

  // this is James' egregious hack to overcome wild imu while sitting on the ground
  if (x_->vel.norm() < 0.05)
    y_acc = x_->pose.q_.rotp(-gravity_);
  else
    y_acc = x_->pose.q_.rotp(x_->acc - gravity_);

  if (motors_spinning())
    y_acc += acc_stdev_ * randomNormal<double, 3, 1>(normal_distribution_, random_generator_);
  acc_bias_ += acc_bias_walk_stdev_ * randomNormal<double, 3, 1>(normal_distribution_, random_generator_);
  y_acc += acc_bias_;

  accel[0] = y_acc.x();
  accel[1] = y_acc.y();
  accel[2] = y_acc.z();

  Eigen::Vector3d y_gyro = x_->omega;
  if (motors_spinning())
    y_gyro += gyro_stdev_ * randomNormal<double, 3, 1>(normal_distribution_, random_generator_);
  gyro_bias_ += gyro_bias_walk_stdev_ * randomNormal<double, 3, 1>(normal_distribution_, random_generator_);
  y_gyro += gyro_bias_;

  gyro[0] = y_gyro.x();
  gyro[1] = y_gyro.y();
  gyro[2] = y_gyro.z();

  (*temperature) = 27.0;
  (*time_us) = clock_micros();
  return true;
}

void RFHBoard::imu_not_responding_error(void)
{
  ROS_ERROR("[rosflight_holodeck_board] imu not responding");
}

void RFHBoard::mag_read(float mag[3])
{
  // Simulate magnetometer measurements from truth
  Vector3d noise = randomNormal<double, 3, 1>(normal_distribution_, random_generator_);
  mag_bias_ += mag_bias_walk_stdev_ * randomNormal<double, 3, 1>(normal_distribution_, random_generator_);
  Vector3d y_mag = x_->pose.q_.rotp(inertial_magnetic_field_) + mag_bias_ + noise;
  mag[0] = y_mag.x();
  mag[1] = y_mag.y();
  mag[2] = y_mag.z();
}

bool RFHBoard::mag_present(void)
{
  return true;
}

bool RFHBoard::baro_present()
{
  return true;
}

void RFHBoard::baro_read(float *pressure, float *temperature)
{
  double alt = -x_->pose.t_.z();
  double y_baro = 101325.0f*(float)pow((1-2.25694e-5 * alt), 5.2553);
  y_baro += baro_stdev_*normal_distribution_(random_generator_);
  baro_bias_ += baro_bias_walk_stdev_*normal_distribution_(random_generator_);

  (*pressure) = (float)y_baro;
  (*temperature) = 27.0f;
}

bool RFHBoard::diff_pressure_present(void)
{
  if(mav_type_ == "fixedwing")
    return true;
  else
    return false;
}

void RFHBoard::diff_pressure_read(float *diff_pressure, float *temperature)
{
  static double rho_ = 1.225;
  // Calculate Airspeed
  double Va = x_->vel.norm();
  // Invert Airpseed to get sensor measurement
  double y_as = rho_*Va*Va/2.0; // Page 130 in the UAV Book

  // Add noise
  y_as += airspeed_stdev_*normal_distribution_(random_generator_);
  airspeed_bias_ += airspeed_bias_walk_stdev_*normal_distribution_(random_generator_);
  y_as += airspeed_bias_;

  *diff_pressure = y_as;
  *temperature = 27.0;
}

bool RFHBoard::sonar_present(void)
{
  return true;
}

float RFHBoard::sonar_read(void)
{
  double alt = -x_->pose.t_.z();

  if (alt < sonar_min_range_)
  {
    return sonar_min_range_;
  }
  else if (alt > sonar_max_range_)
  {
    return sonar_max_range_;
  }
  else
  {
    return alt + sonar_stdev_*normal_distribution_(random_generator_);
  }
}

bool RFHBoard::battery_voltage_present() const
{
  return true;
}

float RFHBoard::battery_voltage_read() const
{
  return 15 * battery_voltage_multiplier;
}

void RFHBoard::battery_voltage_set_multiplier(double multiplier)
{
  battery_voltage_multiplier = multiplier;
}

bool RFHBoard::battery_current_present() const
{
  return true;
}

float RFHBoard::battery_current_read() const
{
  return 1 * battery_current_multiplier;
}

void RFHBoard::battery_current_set_multiplier(double multiplier)
{
  battery_current_multiplier = multiplier;
}

// PWM
void RFHBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  rc_received_ = false;
  latestRC_.values[0] = 1500; // x
  latestRC_.values[1] = 1500; // y
  latestRC_.values[3] = 1500; // z
  latestRC_.values[2] = 1000; // F
  latestRC_.values[4] = 1000; // attitude override
  latestRC_.values[5] = 1000; // arm

  for (size_t i = 0; i < 14; i++)
    pwm_outputs_[i] = 1000;

  rc_sub_ = nh_->subscribe("RC", 1, &RFHBoard::RCCallback, this);
}

float RFHBoard::rc_read(uint8_t channel)
{
  if(rc_sub_.getNumPublishers() > 0)
  {
    return static_cast<float>(latestRC_.values[channel]-1000)/1000.0;
  }

  //no publishers, set throttle low and center everything else
  if(channel == 2)
    return 0.0;

  return 0.5;
}

void RFHBoard::pwm_write(uint8_t channel, float value)
{
  pwm_outputs_[channel] = 1000+(uint16_t)(1000*value);
}

void RFHBoard::pwm_disable()
{
  for(int i=0;i<14;i++)
    pwm_write(i,0);
}

bool RFHBoard::rc_lost(void)
{
  return !rc_received_;
}

void RFHBoard::rc_init(rc_type_t rc_type) {}

// non-volatile memory
void RFHBoard::memory_init(void) {}

bool RFHBoard::memory_read(void *dest, size_t len)
{
  std::string directory = "rosflight_memory" + nh_->getNamespace();
  std::ifstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);

  if(!memory_file.is_open())
  {
    ROS_ERROR("Unable to load rosflight memory file %s/mem.bin", directory.c_str());
    return false;
  }

  memory_file.read((char*) dest, len);
  memory_file.close();
  return true;
}

bool RFHBoard::memory_write(const void *src, size_t len)
{
  std::string directory = "rosflight_memory" + nh_->getNamespace();
  std::string mkdir_command = "mkdir -p " + directory;
  const int dir_err = system(mkdir_command.c_str());

  if (dir_err == -1)
  {
    ROS_ERROR("Unable to write rosflight memory file %s/mem.bin", directory.c_str());
    return false;
  }

  std::ofstream memory_file;
  memory_file.open(directory + "/mem.bin", std::ios::binary);
  memory_file.write((char*) src, len);
  memory_file.close();
  return true;
}

bool RFHBoard::motors_spinning()
{
  if(pwm_outputs_[2] > 1100)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// LED

void RFHBoard::led0_on(void) { }
void RFHBoard::led0_off(void) { }
void RFHBoard::led0_toggle(void) { }

void RFHBoard::led1_on(void) { }
void RFHBoard::led1_off(void) { }
void RFHBoard::led1_toggle(void) { }

void RFHBoard::backup_memory_init()
{
}

bool RFHBoard::backup_memory_read(void *dest, size_t len)
{
  if(len <= BACKUP_SRAM_SIZE)
  {
    memcpy(dest, backup_memory_, len);
    return true;
  }
  else
    return false;
}

void RFHBoard::backup_memory_write(const void *src, size_t len)
{
  if (len < BACKUP_SRAM_SIZE)
    memcpy(backup_memory_, src, len);
}

void RFHBoard::backup_memory_clear(size_t len)
{
  if(len< BACKUP_SRAM_SIZE)
    memset(backup_memory_, 0, len);
}

void RFHBoard::RCCallback(const rosflight_msgs::RCRawConstPtr& msg)
{
  rc_received_ = true;
  latestRC_ = *msg;
}

bool RFHBoard::gnss_present() { return false; }
void RFHBoard::gnss_update() {}

rosflight_firmware::GNSSData RFHBoard::gnss_read()
{
  rosflight_firmware::GNSSData out;
  
  // TODO: Fill this crap in with good simulated GPS values
  out.lat = 0;
  out.lon = 0;
  out.height = 0;

  out.vel_n = 0;
  out.vel_e = 0;
  out.vel_d = 0;

  out.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_FIX;
  out.time_of_week = 0;
  out.time = 0;
  out.nanos = 0;

  out.h_acc = 0;
  out.v_acc = 0;

  out.ecef.x = 0;
  out.ecef.y = 0;
  out.ecef.z = 0;
  out.ecef.p_acc = 0;
  out.ecef.vx = 0;
  out.ecef.vy = 0;
  out.ecef.vz = 0;
  out.ecef.s_acc = 0;

  out.rosflight_timestamp = clock_micros();

  return out;
}

bool RFHBoard::gnss_has_new_data() { return false; }

rosflight_firmware::GNSSRaw RFHBoard::gnss_raw_read()
{
  rosflight_firmware::GNSSRaw out;
  
  out.lat = 0;
  out.lon = 0;
  out.height = 0;
  out.height_msl = out.height + ground_altitude_;

  out.vel_n = 0;
  out.vel_e = 0;
  out.vel_d = 0;

  out.fix_type = rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_FIX;
  out.time_of_week = 0;
  out.num_sat = 15;
  
  out.year = 0;
  out.month = 0;
  out.day = 0;
  out.hour = 0;
  out.min = 0;
  out.sec = 0;
  out.valid = 0;
  out.t_acc = 0;
  out.nano = 0;

  out.h_acc = 0;
  out.v_acc = 0;
  
  out.g_speed = 0;

  out.head_mot = 0;
  out.p_dop = 0;
  out.rosflight_timestamp = clock_micros();

  return out;
}