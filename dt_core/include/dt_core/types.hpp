#pragma once
#include <cstdint>
#include <string>
#include <stdexcept>
#include <vector>

namespace types{
  class Point{
    private:
      double _x;
      double _y;
      double _z;
    public:
      Point(const double initial_x = 0.0, const double initial_y = 0.0, const double initial_z = 0.0) : _x(initial_x), _y(initial_y), _z(initial_z){
      }

      void set_x(const double new_x){
        _x = new_x;
      }
      void set_y(const double new_y){
        _y = new_y;
      }
      void set_z(const double new_z){
        _z = new_z;
      }

      double get_x() const{
        return _x;
      }
      double get_y() const{
        return _y;
      }
      double get_z() const{
        return _z;
      }
  };

  class Orientation{
    private:
      double _roll;
      double _pitch;
      double _yaw;
    public:
      Orientation(const double initial_roll = 0.0, const double initial_pitch = 0.0, const double initial_yaw = 0.0) : _roll(initial_roll), _pitch(initial_pitch), _yaw(initial_yaw){
      }

      void set_roll(const double new_roll){
        _roll = new_roll;
      }
      void set_pitch(const double new_pitch){
        _pitch = new_pitch;
      }
      void set_yaw(const double new_yaw){
        _yaw = new_yaw;
      }

      double get_roll() const{
        return _roll;
      }
      double get_pitch() const{
        return _pitch;
      }
      double get_yaw() const{
        return _yaw;
      }
  };

  class Pose{
    private:
      Point _position;
      Orientation _orientation;
    public:
      Pose(const Point& initial_position, const Orientation& initial_orientation) : _position(initial_position), _orientation(initial_orientation){
      }
      Pose(const double initial_x = 0.0, const double initial_y = 0.0, const double initial_z = 0.0,
           const double initial_roll = 0.0, const double initial_pitch = 0.0, const double initial_yaw = 0.0) : _position(initial_x, initial_y, initial_z), _orientation(initial_roll, initial_pitch, initial_yaw){
      }
      
      void set_position(const Point& new_position){
        _position = new_position;
      }
      void set_position(const double new_x, const double new_y, const double new_z){
        _position.set_x(new_x);
        _position.set_y(new_y);
        _position.set_z(new_z);
      }

      void set_orientation(const Orientation& new_orientation){
        _orientation = new_orientation;
      }
      void set_orientation(const double new_roll, const double new_pitch, const double new_yaw){
        _orientation.set_roll(new_roll);
        _orientation.set_pitch(new_pitch);
        _orientation.set_yaw(new_yaw);
      }

      const Point& get_position() const{
        return _position;
      }
      const Orientation& get_orientation() const{
        return _orientation;
      }
      
      double get_x() const{
        return _position.get_x();
      }
      double get_y() const{
        return _position.get_y();
      }
      double get_z() const{
        return _position.get_z();
      }

      double get_roll() const{
        return _orientation.get_roll();
      }
      double get_pitch() const{
        return _orientation.get_pitch();
      }
      double get_yaw() const{
        return _orientation.get_yaw();
      }
  };

  class Velocity{
    private:
      double _vx;
      double _vy;
      double _vz;
    public:
      Velocity(const double initial_vx = 0.0, const double initial_vy = 0.0, const double initial_vz = 0.0) : _vx(initial_vx), _vy(initial_vy), _vz(initial_vz){
      }

      void set_vx(const double new_vx){
        _vx = new_vx;
      }
      void set_vy(const double new_vy){
        _vy = new_vy;
      }
      void set_vz(const double new_vz){
        _vz = new_vz;
      }

      double get_vx() const{
        return _vx;
      }
      double get_vy() const{
        return _vy;
      }
      double get_vz() const{
        return _vz;
      }
  };

  class Kinematics{
    private:
      Velocity _velocity;
    public:
      explicit Kinematics(const Velocity& initial_velocity) : _velocity(initial_velocity){
      }

      void set_velocity(const Velocity& new_velocity){
        _velocity = new_velocity;
      }

      const Velocity& get_velocity() const{
        return _velocity;
      }

      double get_vx() const{
        return _velocity.get_vx();
      }

      double get_vy() const{
        return _velocity.get_vy();
      }

      double get_vz() const{
        return _velocity.get_vz();
      }
  };

  class Covariance{
    private:
      double _xx;
      double _xy;
      double _yy;
    public:
  };

  class Entity{
    private:
      Pose _pose;
      Kinematics _kinematics;
    public:
      Entity(const Pose& initial_pose, const Kinematics& initial_kinematics) : _pose(initial_pose), _kinematics(initial_kinematics){
      }

      virtual ~Entity() = default;

      void set_pose(const Pose& new_pose){
        _pose = new_pose;
      }

      void set_kinematics(const Kinematics& new_kinematics){
        _kinematics = new_kinematics;
      }

      void set_velocity(const Velocity& new_velocity){
        _kinematics.set_velocity(new_velocity);
      }

      const Pose& get_pose() const{
        return _pose;
      }

      const Kinematics& get_kinematics() const{
        return _kinematics;
      }

      const Velocity& get_velocity() const{
        return _kinematics.get_velocity();
      }
  };

  class Target : public Entity{
    private:
      uint32_t _id;
      std::string _description;
    public:
      Target(const int32_t id, const std::string& description, const Pose& initial_pose, const Kinematics& initial_kinematics) : _description(description), Entity(initial_pose, initial_kinematics){
        this->set_id(id);
      }

      void set_description(const std::string& new_description){
        _description = new_description;
      }

      void set_id(const int32_t new_id){
        if (new_id < 0) {
          throw std::invalid_argument("Error: Target ID must be positive!");
        } else {
          _id = static_cast<uint32_t>(new_id);
        }
      }

      uint32_t get_id() const{
        return _id;
      }

      const std::string& get_description() const{
        return _description;
      }
  };

  class Trajectory {
    private:
      std::vector<Pose> poses;
    public:
      Trajectory() = default;

      explicit Trajectory(const std::vector<Pose>& poses) : poses(poses) {}

      void add_pose(const Pose& pose) {
          poses.push_back(pose);
      }

      size_t size() const {
          return poses.size();
      }

      bool empty() const {
          return poses.empty();
      }

      const Pose& get_pose(size_t index) const {
          if (index >= poses.size() || index < static_cast<size_t>(0)) {
              throw std::out_of_range("Error: Index out of range");
          }
          return poses[index];
      }

      const std::vector<Pose>& get_poses() const {
          return poses;
      }

      void clear() {
          poses.clear();
      }
  };

  class Report{
    private:
      std::string _msg;
    public:
      Report(const std::string& msg = "") : _msg(msg){
      }

      void set_msg(const std::string& new_msg){
        _msg = new_msg;
      }

      const std::string& get_msg() const{
        return _msg;
      }
  };

  class CollisionReport : public Report{
    private:
      Pose _usv_cpa;
      Pose _obstacle_cpa;
      double _dcpa;
      double _tcpa;
      double _risk;
    public:
      CollisionReport(const Pose& usv_cpa = Pose(), const Pose& obstacle_cpa = Pose(), const double dcpa = -1.0, const double tcpa = -1.0, const double risk = -1.0, const std::string& msg = "") : 
      _usv_cpa(usv_cpa), _obstacle_cpa(obstacle_cpa), _dcpa(dcpa), _tcpa(tcpa), _risk(risk), Report(msg){
      }

      void set_usv_cpa(const Pose& new_cpa){
        _usv_cpa = new_cpa;
      }

      void set_obstacle_cpa(const Pose& new_cpa){
        _obstacle_cpa = new_cpa;
      }

      void set_dcpa(const double new_dcpa){
        _dcpa = new_dcpa;
      }

      void set_tcpa(const double new_tcpa){
        _tcpa = new_tcpa;
      }

      void set_risk(const double new_risk){
        _risk = new_risk;
      }

      const Pose& get_usv_cpa() const{
        return _usv_cpa;
      }

      const Pose& get_obstacle_cpa() const{
        return _obstacle_cpa;
      }

      double get_dcpa() const{
        return _dcpa;
      }

      double get_tcpa() const{
        return _tcpa;
      }

      double get_risk() const{
        return _risk;
      }
  };

  class TargetCollisionReport : public CollisionReport{
    private:
      uint32_t _id;
    public:
      TargetCollisionReport(const int32_t id, const Pose& usv_cpa = Pose(), const Pose& obstacle_cpa = Pose(), const double dcpa = -1.0, const double tcpa = -1.0, const double risk = -1.0, const std::string& msg = "") : 
      CollisionReport(usv_cpa, obstacle_cpa, dcpa, tcpa, risk, msg){
        this->set_id(id);
      }

      void set_id(const int32_t new_id){
        if (new_id < 0) {
          throw std::invalid_argument("Error: Target ID must be positive!");
        } else {
          _id = static_cast<uint32_t>(new_id);
        }
      }

      uint32_t get_id() const{
        return _id;
      }
  };
} // namespace types