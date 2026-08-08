/**
 * @file types.hpp
 * @brief Core data structures and domain models for the Digital Twin.
 */

#pragma once

#include <cstdint>
#include <string>
#include <stdexcept>
#include <vector>

namespace types {

/**
 * @class Point
 * @brief Represents a 3D spatial point with optional geographic coordinates.
 */
class Point {
private:
    double _x;
    double _y;
    double _z;
    double _lat;
    double _lon;
    std::string _frame_id;

public:
    Point(double initial_x = 0.0, double initial_y = 0.0, double initial_z = 0.0) 
        : _x(initial_x), _y(initial_y), _z(initial_z), _lat(0.0), _lon(0.0), _frame_id("map") {}

    void set_x(double x) { _x = x; }
    void set_y(double y) { _y = y; }
    void set_z(double z) { _z = z; }
    
    void set_lat_lon(double lat, double lon) {
        _lat = lat;
        _lon = lon;
    }

    void set_frame_id(const std::string& frame) { _frame_id = frame; }

    double get_x() const { return _x; }
    double get_y() const { return _y; }
    double get_z() const { return _z; }
    double get_lat() const { return _lat; }
    double get_lon() const { return _lon; }
    const std::string& get_frame_id() const { return _frame_id; }
};

/**
 * @class Orientation
 * @brief Represents the attitude of an entity using Euler angles.
 */
class Orientation {
private:
    double _roll;
    double _pitch;
    double _yaw;

public:
    Orientation(double initial_roll = 0.0, double initial_pitch = 0.0, double initial_yaw = 0.0) 
        : _roll(initial_roll), _pitch(initial_pitch), _yaw(initial_yaw) {}

    void set_roll(double roll) { _roll = roll; }
    void set_pitch(double pitch) { _pitch = pitch; }
    void set_yaw(double yaw) { _yaw = yaw; }

    double get_roll() const { return _roll; }
    double get_pitch() const { return _pitch; }
    double get_yaw() const { return _yaw; }
};

/**
 * @class Pose
 * @brief Combines position and orientation to fully describe an object's spatial state.
 */
class Pose {
private:
    Point _position;
    Orientation _orientation;

public:
    Pose(const Point& initial_position, const Orientation& initial_orientation) 
        : _position(initial_position), _orientation(initial_orientation) {}

    Pose(double initial_x = 0.0, double initial_y = 0.0, double initial_z = 0.0,
         double initial_roll = 0.0, double initial_pitch = 0.0, double initial_yaw = 0.0) 
        : _position(initial_x, initial_y, initial_z), _orientation(initial_roll, initial_pitch, initial_yaw) {}
    
    void set_position(const Point& position) { _position = position; }
    void set_position(double x, double y, double z) {
        _position.set_x(x);
        _position.set_y(y);
        _position.set_z(z);
    }

    void set_orientation(const Orientation& orientation) { _orientation = orientation; }
    void set_orientation(double roll, double pitch, double yaw) {
        _orientation.set_roll(roll);
        _orientation.set_pitch(pitch);
        _orientation.set_yaw(yaw);
    }

    const Point& get_position() const { return _position; }
    const Orientation& get_orientation() const { return _orientation; }
    
    double get_x() const { return _position.get_x(); }
    double get_y() const { return _position.get_y(); }
    double get_z() const { return _position.get_z(); }
    double get_roll() const { return _orientation.get_roll(); }
    double get_pitch() const { return _orientation.get_pitch(); }
    double get_yaw() const { return _orientation.get_yaw(); }
};

/**
 * @class Velocity
 * @brief Represents 3D linear velocity.
 */
class Velocity {
private:
    double _vx;
    double _vy;
    double _vz;

public:
    Velocity(double initial_vx = 0.0, double initial_vy = 0.0, double initial_vz = 0.0) 
        : _vx(initial_vx), _vy(initial_vy), _vz(initial_vz) {}

    void set_vx(double vx) { _vx = vx; }
    void set_vy(double vy) { _vy = vy; }
    void set_vz(double vz) { _vz = vz; }

    double get_vx() const { return _vx; }
    double get_vy() const { return _vy; }
    double get_vz() const { return _vz; }
};

/**
 * @class Kinematics
 * @brief Container for velocity and potential higher-order motion derivatives.
 */
class Kinematics {
private:
    Velocity _velocity;

public:
    explicit Kinematics(const Velocity& initial_velocity) : _velocity(initial_velocity) {}

    void set_velocity(const Velocity& velocity) { _velocity = velocity; }
    const Velocity& get_velocity() const { return _velocity; }

    double get_vx() const { return _velocity.get_vx(); }
    double get_vy() const { return _velocity.get_vy(); }
    double get_vz() const { return _velocity.get_vz(); }
};

/**
 * @class Covariance
 * @brief Represents a simplified 2D covariance matrix for uncertainty estimation.
 */
class Covariance {
private:
    double _xx;
    double _xy;
    double _yy;

public:
    Covariance(double xx = 0.0, double xy = 0.0, double yy = 0.0) 
        : _xx(xx), _xy(xy), _yy(yy) {}

    void set_xx(double xx) { _xx = xx; }
    void set_xy(double xy) { _xy = xy; }
    void set_yy(double yy) { _yy = yy; }

    double get_xx() const { return _xx; }
    double get_xy() const { return _xy; }
    double get_yy() const { return _yy; }
};

/**
 * @class Entity
 * @brief Base class for any physical object in the Digital Twin environment.
 */
class Entity {
private:
    Pose _pose;
    Kinematics _kinematics;
    Covariance _covariance;

public:
    Entity(const Pose& initial_pose, const Kinematics& initial_kinematics, const Covariance& initial_covariance = Covariance()) 
        : _pose(initial_pose), _kinematics(initial_kinematics), _covariance(initial_covariance) {}

    virtual ~Entity() = default;

    void set_pose(const Pose& pose) { _pose = pose; }
    void set_kinematics(const Kinematics& kinematics) { _kinematics = kinematics; }
    void set_covariance(const Covariance& covariance) { _covariance = covariance; }
    void set_velocity(const Velocity& velocity) { _kinematics.set_velocity(velocity); }

    const Pose& get_pose() const { return _pose; }
    const Kinematics& get_kinematics() const { return _kinematics; }
    const Covariance& get_covariance() const { return _covariance; }
    const Velocity& get_velocity() const { return _kinematics.get_velocity(); }
};

/**
 * @class Target
 * @brief Represents a tracked dynamic object, such as an AIS vessel.
 */
class Target : public Entity {
private:
    uint32_t _id;
    std::string _description;
    double _radius;

public:
    Target(int32_t id, const std::string& description, const Pose& initial_pose, 
           const Kinematics& initial_kinematics, const Covariance& initial_covariance = Covariance(), 
           double radius = 1.0) 
        : Entity(initial_pose, initial_kinematics, initial_covariance), _description(description), _radius(radius) {
        this->set_id(id);
    }

    void set_description(const std::string& description) { _description = description; }
    
    void set_id(int32_t id) {
        if (id < 0) {
            throw std::invalid_argument("Error: Target ID must be positive.");
        }
        _id = static_cast<uint32_t>(id);
    }

    void set_radius(double radius) { _radius = radius; }

    uint32_t get_id() const { return _id; }
    const std::string& get_description() const { return _description; }
    double get_radius() const { return _radius; }
};

/**
 * @class Trajectory
 * @brief Defines a planned path or historical route as a sequence of poses.
 */
class Trajectory {
private:
    std::vector<Pose> _poses;

public:
    Trajectory() = default;
    explicit Trajectory(const std::vector<Pose>& poses) : _poses(poses) {}

    void add_pose(const Pose& pose) { _poses.push_back(pose); }
    size_t size() const { return _poses.size(); }
    bool empty() const { return _poses.empty(); }
    void clear() { _poses.clear(); }

    const Pose& get_pose_by_index(size_t index) const {
        if (index >= _poses.size()) {
            throw std::out_of_range("Error: Index out of range.");
        }
        return _poses[index];
    }

    const std::vector<Pose>& get_poses() const { return _poses; }
};

/**
 * @class MapData
 * @brief Container for geospatial map references.
 */
class MapData {
private:
    std::string _map_name;
    std::string _shapefile_path;
    bool _is_loaded;

public:
    MapData(const std::string& map_name = "", const std::string& shapefile_path = "") 
        : _map_name(map_name), _shapefile_path(shapefile_path), _is_loaded(false) {}

    void set_map_name(const std::string& name) { _map_name = name; }
    void set_shapefile_path(const std::string& path) { _shapefile_path = path; }
    void set_loaded(bool loaded) { _is_loaded = loaded; }

    const std::string& get_map_name() const { return _map_name; }
    const std::string& get_shapefile_path() const { return _shapefile_path; }
    bool is_loaded() const { return _is_loaded; }
};

/**
 * @class Report
 * @brief Base class for system-generated analysis reports.
 */
class Report {
private:
    std::string _msg;

public:
    explicit Report(const std::string& msg = "") : _msg(msg) {}

    void set_msg(const std::string& msg) { _msg = msg; }
    const std::string& get_msg() const { return _msg; }
};

/**
 * @class CollisionReport
 * @brief Contains Closest Point of Approach (CPA) calculations for collision risk analysis.
 */
class CollisionReport : public Report {
private:
    Pose _usv_cpa;
    Pose _obstacle_cpa;
    double _dcpa;
    double _tcpa;
    double _risk;
    bool _safe;

public:
    CollisionReport(const Pose& usv_cpa = Pose(), const Pose& obstacle_cpa = Pose(), 
                    double dcpa = -1.0, double tcpa = -1.0, double risk = -1.0, 
                    const std::string& msg = "", bool safe = true) 
        : Report(msg), _usv_cpa(usv_cpa), _obstacle_cpa(obstacle_cpa), 
          _dcpa(dcpa), _tcpa(tcpa), _risk(risk), _safe(safe) {}

    void set_usv_cpa(const Pose& cpa) { _usv_cpa = cpa; }
    void set_obstacle_cpa(const Pose& cpa) { _obstacle_cpa = cpa; }
    void set_dcpa(double dcpa) { _dcpa = dcpa; }
    void set_tcpa(double tcpa) { _tcpa = tcpa; }
    void set_risk(double risk) { _risk = risk; }
    void set_safety(bool safe) { _safe = safe; }

    const Pose& get_usv_cpa() const { return _usv_cpa; }
    const Pose& get_obstacle_cpa() const { return _obstacle_cpa; }
    double get_dcpa() const { return _dcpa; }
    double get_tcpa() const { return _tcpa; }
    double get_risk() const { return _risk; }
    bool is_safe() const { return _safe; }
};

/**
 * @class TargetCollisionReport
 * @brief Specific collision report associated with a tracked target.
 */
class TargetCollisionReport : public CollisionReport {
private:
    uint32_t _id;

public:
    TargetCollisionReport(int32_t id = 0, const Pose& usv_cpa = Pose(), const Pose& obstacle_cpa = Pose(), 
                          double dcpa = -1.0, double tcpa = -1.0, double risk = -1.0, 
                          const std::string& msg = "") 
        : CollisionReport(usv_cpa, obstacle_cpa, dcpa, tcpa, risk, msg) {
        this->set_id(id);
    }

    void set_id(int32_t id) {
        if (id < 0) {
            throw std::invalid_argument("Error: Target ID must be positive.");
        }
        _id = static_cast<uint32_t>(id);
    }

    uint32_t get_id() const { return _id; }
};

} // namespace types