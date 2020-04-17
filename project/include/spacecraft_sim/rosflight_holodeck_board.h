#pragma once

#include <fstream>

#include "ros/ros.h"
#include "rosflight_firmware/udp_board.h"
#include "rosflight_msgs/RCRaw.h"
#include "geometry/support.h"

#include "rosflight_holodeck/state.h"

class RFHBoard : public rosflight_firmware::UDPBoard
{
public:
  RFHBoard();
  ~RFHBoard() {};

  // setup
  void init_board() override {};
  void init_board(ros::NodeHandle *nh, State *x);  // FIXME: document this
  void board_reset(bool bootloader) override;

  // clock
  uint32_t clock_millis() override { return time_us_ * 1e-3; }
  uint64_t clock_micros() override { return time_us_; }
  uint64_t clock_nanos()           { return time_us_ * 1e3; } // FIXME: document this
  void clock_delay(uint32_t milliseconds) override;
  // FIXME: Document these next two 
  uint64_t increment_clock() { return time_us_ += imu_update_period_us_; };
  void set_clock_time(double time_sec) { time_us_ = (uint64_t) (time_sec*1e6); }
  
  // sensors
  void sensors_init() override;
  uint16_t num_sensor_errors(void) override;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us) override;
  void imu_not_responding_error() override;

  bool mag_present(void) override;
  void mag_read(float mag[3]) override;
  void mag_update(void) override {};

  bool baro_present(void) override;
  void baro_read(float *pressure, float *temperature) override;
  void baro_update(void) override {};

  bool diff_pressure_present(void) override;
  void diff_pressure_read(float *diff_pressure, float *temperature) override;
  void diff_pressure_update(void) override {};

  bool sonar_present(void) override;
  float sonar_read(void) override;
  void sonar_update(void) override {};

  // PWM
  // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override;
  void pwm_write(uint8_t channel, float value) override;
  void pwm_disable(void) override;
  int* get_outputs() { return pwm_outputs_; }

  // RC
  float rc_read(uint8_t channel) override;
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost(void) override;


  // non-volatile memory
  void memory_init(void) override;
  bool memory_read(void *dest, size_t len) override;
  bool memory_write(const void *src, size_t len) override;

  // LEDs
  void led0_on(void) override;
  void led0_off(void) override;
  void led0_toggle(void) override;

  void led1_on(void) override;
  void led1_off(void) override;
  void led1_toggle(void) override;
  
  //Backup Memory
  void backup_memory_init() override;
  bool backup_memory_read(void *dest, size_t len) override;
  void backup_memory_write(const void *src, size_t len) override;
  void backup_memory_clear(size_t len) override;

  bool gnss_present() override;
  void gnss_update() override;

  rosflight_firmware::GNSSData gnss_read() override;
  bool gnss_has_new_data() override;
  rosflight_firmware::GNSSRaw gnss_raw_read() override;

  bool battery_voltage_present() const override;
  float battery_voltage_read() const override;
  void battery_voltage_set_multiplier(double multiplier) override;
  
  bool battery_current_present() const override;
  float battery_current_read() const override;
  void battery_current_set_multiplier(double multiplier) override;
  
private:
  // rosflight_holodeck keeps track of the mav's state
  State *x_;

  // sensors
  Eigen::Vector3d inertial_magnetic_field_;
  
  double gyro_stdev_;
  double gyro_bias_walk_stdev_;
  double gyro_bias_range_;

  double acc_stdev_;
  double acc_bias_range_;
  double acc_bias_walk_stdev_;

  double baro_bias_walk_stdev_;
  double baro_stdev_;
  double baro_bias_range_;

  double mag_bias_walk_stdev_;
  double mag_stdev_;
  double mag_bias_range_;

  double airspeed_bias_walk_stdev_;
  double airspeed_stdev_;
  double airspeed_bias_range_;

  double sonar_stdev_;
  double sonar_max_range_;
  double sonar_min_range_;

  double horizontal_gps_stdev_;
  double vertical_gps_stdev_;
  double gps_velocity_stdev_;

  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d mag_bias_;
  double baro_bias_;
  double airspeed_bias_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;
  
  Eigen::Vector3d gravity_;
  double ground_altitude_; // TODO: Replaced by origin_altitude_ in SIL_Board class
  double origin_latitude_;
  double origin_longitude_;
  // double origin_altitude_;
  
  ros::NodeHandle* nh_;
  ros::Subscriber rc_sub_;
  rosflight_msgs::RCRaw latestRC_;
  bool rc_received_;
  
  std::string mav_type_;
  int pwm_outputs_[14];  //assumes maximum of 14 channels [1000-2000]
  
  // Time variables
  uint64_t time_us_;  // simulated clock time (in microseconds); called 'now_us' in SIL_Board
  double boot_time_;
  uint64_t next_imu_update_time_us_;
  double imu_update_rate_;
  uint64_t imu_update_period_us_;
  
  void RCCallback(const rosflight_msgs::RCRawConstPtr& msg);
  bool motors_spinning();

  float battery_voltage_multiplier{1.0};
  float battery_current_multiplier{1.0};
  static constexpr size_t BACKUP_SRAM_SIZE{1024};
  uint8_t backup_memory_[BACKUP_SRAM_SIZE];

};
