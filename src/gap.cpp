#include <fgm_plugin/gap.h>
// #ifndef
#define PI 3.1415926
// #endif

Gap::Gap() {
    // place holder to return worst scenario
    goal_angle = 0;
    gap_angle = -3.1415926;
}

Gap::Gap(int _start, int _end, int _size, float l_dist, float r_dist, float goal)
    {
        start_angle = _start;
        end_angle = _end;
        left_dist = l_dist;
        right_dist = r_dist;
        size = _size;
        goal_angle = goal;
        gap_angle = this->getAngle();
    }

    void Gap::setGoalAngle(float _goal_angle) {
        goal_angle = _goal_angle;
    }

    void Gap::setSensorModel(float _increment, float _min) {
        angle_increment = _increment;
        angle_min = _min;
    }

    float Gap::getAngle() {
        angle_increment = 0.0122718466446;
        angle_min = -3.14159274101;
        float gap_dir = (start_angle + end_angle) / 2 * angle_increment + angle_min;
        return gap_dir;
    }

    float Gap::getScore() const {
        float val = goal_angle - gap_angle;
        val = val > 0 ? fmod(val + 2 * PI, 2 * PI) : fmod(val - 2 * PI, 2 * PI);
        return fabs(val);
    }

    float Gap::traversable() {
        float dist = fmin(left_dist, right_dist);
        float clearance = 2 * dist * sin((start_angle - end_angle) * angle_increment / 2);
        if (clearance > 0.1) {
            return 1;
        } else {
            return 0;
        }
    }

    int Gap::getSize() const { return size; }