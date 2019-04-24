#include <fgm_plugin/gap.h>
class gapComparator
{
  public:
    int operator()(const Gap &g1, const Gap &g2)
    {
        return g2.getScore() < g1.getScore();
    }
};