#include <fgm_plugin/gap.h>
class gapComparator
{
  public:
    int operator()(const Gap &g1, const Gap &g2)
    {
        return g1.getSize() < g2.getSize();
    }
};