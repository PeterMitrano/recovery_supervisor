#include <pcl/register_point_struct.h>

namespace recovery_supervisor
{
struct RecoveryPoint
{
  float x;
  float y;
  float z;
  float ox;
  float oy;
  float oz;
  float ow;
  int index;
};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(recovery_supervisor::RecoveryPoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, ox, ox)(float, oy, oy)(float, oz, oz)(
                                      float, ow, ow)(int, index, index));
