#ifndef GAP
#define GAP

#include <math.h>
#include <cmath>
class Gap
{
    int start_angle;
    int size;
    float left_dist;
    float right_dist;
    int end_angle;
    float angle_increment;
    float angle_min;
    static float goal_angle;

  public:
    Gap(int _start, int _end, int _size, float l_dist, float r_dist)
    {
        start_angle = _start;
        end_angle = _end;
        left_dist = l_dist;
        right_dist = r_dist;
    }

    void setGoalAngle(float _goal_angle) {
        goal_angle = _goal_angle;
    }

    void setSensorModel(float _increment, float _min) {
        angle_increment = _increment;
        angle_min = _min;
    }

    float getAngle() {
        float gap_size = (start_angle + end_angle) / 2 * angle_increment + angle_min;
        return gap_size;
    }
    int getSize() const { return size; }
};
#else
#endif