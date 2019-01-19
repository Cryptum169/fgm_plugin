#ifndef GAP
#define GAP
class Gap
{
    int start_angle;
    int size;
    float dist;

  public:
    Gap(int _start, int _size, float _dist)
    {
        start_angle = _start;
        size = _size;
        dist = _dist;
    }
    void setSize(int val) { size = val; }
    void setStart(int val) { start_angle = val; }
    float getDistance() const { return dist; }
    int getStartAngle() const { return start_angle; }
    int getSize() const { return size; }
};
#else
#endif