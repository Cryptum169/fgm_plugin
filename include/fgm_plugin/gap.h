#ifndef GAP
#define GAP

#include <math.h>
#include <cmath>
class Gap
{
  private:
    int start_angle;
    int size;
    float left_dist;
    float right_dist;
    int end_angle;
    float angle_increment;
    float angle_min;
    float gap_score;
    float goal_angle;
    float gap_angle;
    bool score;
    float _robot_ori;
    double _map_score;

  public:
    Gap();
    Gap(int _start, int _end, int _size, float l_dist, float r_dist, float goal, bool score, double map_score);

    void setGoalAngle(float _goal_angle);

    void setSensorModel(float _increment, float _min);

    float getAngle();

    float getScore() const;
    float traversable();

    int getSize() const;

    void recordOdom(float robot_orientation);
    float getOdom();

    void setScore(double);
};
#else
#endif