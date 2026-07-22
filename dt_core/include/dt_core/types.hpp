namespace types{
  class Point{
    private:
      double x;
      double y;
      double z;
    public:
      Point(const double x_i = 0.0, const double y_i = 0.0, const double z_i = 0.0);

      void set_x(const double new_x);
      void set_y(const double new_y);
      void set_z(const double new_z);

      double get_x() const;
      double get_y() const;
      double get_z() const;
  };

  class Orientation{
    private:
      double roll;
      double pitch;
      double yaw;
    public:
      Orientation(const double roll_i = 0.0, const double pitch_i = 0.0, const double yaw_i = 0.0);

      void set_roll(const double new_roll);
      void set_pitch(const double new_pitch);
      void set_yaw(const double new_yaw);

      double get_roll() const;
      double get_pitch() const;
      double get_yaw() const;
  };

  class Pose{
    private:
      Point position;
      Orientation orientation;
    public:
      Pose(const Point& position_i, const Orientation& orientation_i);
      Pose(const double x_i = 0.0, const double y_i = 0.0, const double z_i = 0.0,
           const double roll_i = 0.0, const double pitch_i = 0.0, const double yaw_i = 0.0);
      
      void set_position(const Point& new_position);
      void set_position(const double new_x, const double new_y, const double new_z);

      void set_orientation(const Orientation& new_orientation);
      void set_orientation(const double new_roll, const double new_pitch, const double new_yaw);

      const Point& get_position() const;
      const Orientation& get_orientation() const;
      
      double get_x() const;
      double get_y() const;
      double get_z() const;

      double get_roll() const;
      double get_pitch() const;
      double get_yaw() const;
  };

  class Entity{
    private:
      Pose pose;
    public:
  };

} // namespace types