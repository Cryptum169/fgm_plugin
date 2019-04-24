#include <fgm_plugin/gap.h>

#ifndef PI
#define PI 3.1415926
#endif

Gap::Gap() {
    // place holder to return worst scenario
    goal_angle = 0;
    gap_angle = -3.1415926;
    score = false;
}

Gap::Gap(int _start, int _end, int _size, float l_dist, float r_dist, float goal, bool score, double map_score)
    {
        start_angle = _start;
        end_angle = _end;
        left_dist = l_dist;
        right_dist = r_dist;
        size = _size;
        goal_angle = goal;
        gap_angle = this->getAngle();
        score = score;
        _map_score = map_score;
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
        angle_min = -3.14159274101; // Returned by sensor message
        float gap_dir = (start_angle + end_angle) / 2 * angle_increment + angle_min;
        return gap_dir;
    }

    float Gap::getScore() const {
        float val = goal_angle - gap_angle;
        if (val > PI) {
            val -= 2 * PI;
        }

        if (val < -PI) {
            val += 2 * PI;
        }
        // val = val > 0 ? fmod(val + 2 * PI, 2 * PI) - PI: fmod(val - 2 * PI, 2 * PI) - PI;
        return score ? fabs(val) : 0 - size;
        // return - _map_score;
    }

    float Gap::traversable() {
        float dist = fmin(left_dist, right_dist);
        float clearance = 2 * dist * sin((end_angle - start_angle) * angle_increment / 2);
        // TODO: Set for rectangular path
        if (clearance > 0.4) {
            return 1;
        } else {
            return 0;
        }
    }

    int Gap::getSize() const { return size; }

    void Gap::recordOdom(float robot_orientation) {
        _robot_ori = robot_orientation;
    }
    
    float Gap::getOdom() {
        return _robot_ori;
    }

    void Gap::setScore(double map_score) {
        _map_score = map_score;
    }
