// GS-3 negative control: a module outside the codec that includes a MAVLink header and names an in-scope message.
// tools/ci/check_gs3.py, given this file explicitly, must reject it. Not built.
#include <mavlink/common/mavlink.h>

int gs3_planted_mavlink() {
  return MAVLINK_MSG_ID_HEARTBEAT;
}
