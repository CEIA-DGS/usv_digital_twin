/**
 * @file twin_interface.hpp
 * @brief Interfaces for the Digital Twin Core and state representation.
 */

#pragma once

#include "dt_core/types.hpp"
#include <vector>
#include <memory>

namespace dt {

/**
 * @class WorldStateSnapshot
 * @brief Represents an immutable snapshot of the world state at a specific instant.
 * 
 * This interface is designed for high-speed, thread-safe access by the navigation 
 * and prediction algorithms. Since it is immutable, it requires no locks to read.
 */
class WorldStateSnapshot {
public:
    virtual ~WorldStateSnapshot() = default;

    // --- Environment Representation ---

    /**
     * @brief Calculates the distance to the nearest static obstacle.
     * @param pos The position to calculate the distance from.
     * @return Distance in meters, or a negative value if unavailable.
     */
    virtual float get_closest_static_obstacle_distance(const types::Point& pos) const = 0;

    /**
     * @brief Verifies if a given point lies within a restricted/non-navigable zone.
     * @param pos The spatial coordinate to check.
     * @return True if the point is in a restricted zone, false otherwise.
     */
    virtual bool is_inside_restricted_zone(const types::Point& pos) const = 0;

    /**
     * @brief Retrieves dynamic targets within a specific radius from a center point.
     * @param center The central point for the search.
     * @param radius The search radius in meters.
     * @return Vector of active targets found within the radius.
     */
    virtual std::vector<types::Target> get_active_local_targets(const types::Point& center, float radius) const = 0;

    // --- Planning Support ---

    /**
     * @brief Predicts the trajectory of a specific target over a given time horizon.
     * @param id The ID of the target to predict.
     * @param targets Current list of context targets.
     * @param time_horizon Total prediction time in seconds.
     * @param time_step Time increment between prediction steps.
     * @return Predicted trajectory for the specified target.
     */
    virtual types::Trajectory predict_trajectory_by_id(int32_t id, const std::vector<types::Target>& targets, double time_horizon, double time_step) const = 0;

    /**
     * @brief Evaluates potential collisions along a candidate trajectory against known targets.
     * @param candidate_trajectory The planned route to evaluate.
     * @param usv_state Current kinematics and pose of the Ego Vehicle.
     * @param speed_profile Assumed speed profile along the route.
     * @param targets Surrounding dynamic targets.
     * @param start_time Evaluation start offset.
     * @return Detailed report containing CPA (Closest Point of Approach) metrics.
     */
    virtual types::TargetCollisionReport check_collisions_on_trajectory(const types::Trajectory& candidate_trajectory, const types::Entity& usv_state, double speed_profile, const std::vector<types::Target>& targets, double start_time = 0.0) const = 0;

    /**
     * @brief Calculates a dynamic risk field value for a given position based on target kinematics.
     * @param position Point in space to evaluate.
     * @param timestamp Time offset for the risk evaluation.
     * @param usv_state Ego vehicle state.
     * @param targets Dynamic environmental targets.
     * @return Risk factor (higher indicates greater collision probability).
     */
    virtual double get_dynamic_risk_field(const types::Point& position, double timestamp, const types::Entity& usv_state, const std::vector<types::Target>& targets) const = 0;
    
    // --- Raw State Access (For GUI / Visualization) ---

    /**
     * @brief Retrieves the ego vehicle's pose at the time of the snapshot.
     * @return Current 3D pose and orientation.
     */
    virtual types::Pose get_vehicle_pose() const = 0;

    /**
     * @brief Retrieves the complete list of tracked targets.
     * @return Vector of all known dynamic targets.
     */
    virtual std::vector<types::Target> get_all_targets() const = 0;

    /**
     * @brief Retrieves the planned trajectory at the time of the snapshot.
     * @return The planned trajectory structure.
     */
    virtual types::Trajectory get_planned_trajectory() const = 0;
};

/**
 * @class DigitalTwinCore
 * @brief Primary interface and state manager for the Digital Twin.
 * 
 * Implements a Double-Buffering / Lock-Free paradigm for consumers. 
 * Asynchronous data writers update the internal state, while high-frequency readers 
 * pull the latest immutable snapshot.
 */
class DigitalTwinCore {
public:
    virtual ~DigitalTwinCore() = default;

    /**
     * @brief Asynchronously updates the static map data.
     * @param map The new map configuration.
     */
    void update_static_map(const types::MapData& map);

    /**
     * @brief Asynchronously updates the ego vehicle pose.
     * @param pose The latest pose estimation.
     */
    void update_vehicle_pose(const types::Pose& pose);

    /**
     * @brief Asynchronously updates the tracked dynamic targets.
     * @param targets The latest target list (e.g., from AIS/Radar).
     */
    void update_dynamic_targets(const std::vector<types::Target>& targets);

    /**
     * @brief Asynchronously updates the currently planned trajectory.
     * @param traj The planned route.
     */
    void update_planned_trajectory(const types::Trajectory& traj);

    /**
     * @brief Synchronously fetches the most recent thread-safe state snapshot.
     * @return A shared pointer to an immutable WorldStateSnapshot.
     */
    std::shared_ptr<const WorldStateSnapshot> get_latest_state() const;
};

} // namespace dt