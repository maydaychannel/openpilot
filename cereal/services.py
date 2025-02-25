#!/usr/bin/env python3
import os
from typing import Optional

EON = os.path.isfile('/EON')

class Service:
  def __init__(self, port: int, should_log: bool, frequency: float, decimation: Optional[int] = None):
    self.port = port
    self.should_log = should_log
    self.frequency = frequency
    self.decimation = decimation

service_list = {
  "roadCameraState": Service(8002, True, 20., 1),
  "sensorEvents": Service(8003, True, 100., 100),
  "gpsNMEA": Service(8004, True, 9.),
  "deviceState": Service(8005, True, 2., 1),
  "can": Service(8006, True, 100.),
  "controlsState": Service(8007, True, 100., 100),
  "features": Service(8010, True, 0.),
  "pandaState": Service(8011, True, 2., 1),
  "radarState": Service(8012, True, 20., 5),
  "roadEncodeIdx": Service(8015, True, 20., 1),
  "liveTracks": Service(8016, True, 20.),
  "sendcan": Service(8017, True, 100.),
  "logMessage": Service(8018, True, 0.),
  "liveCalibration": Service(8019, True, 4., 4),
  "androidLog": Service(8020, True, 0., 1),
  "carState": Service(8021, True, 100., 10),
  "carControl": Service(8023, True, 100., 10),
  "longitudinalPlan": Service(8024, True, 20., 2),
  "liveLocation": Service(8025, True, 0., 1),
  "gpsLocation": Service(8026, True, 1., 1),
  "procLog": Service(8031, True, 0.5),
  "gpsLocationExternal": Service(8032, True, 10., 1),
  "ubloxGnss": Service(8033, True, 10.),
  "clocks": Service(8034, True, 1., 1),
  "liveMpc": Service(8035, True, 20.),
  "liveLongitudinalMpc": Service(8036, True, 20.),
  "ubloxRaw": Service(8042, True, 20.),
  "liveLocationKalman": Service(8054, True, 20., 2),
  "uiLayoutState": Service(8060, True, 0.),
  "liveParameters": Service(8064, True, 20., 2),
  "cameraOdometry": Service(8066, True, 20., 5),
  "lateralPlan": Service(8067, True, 20., 2),
  "thumbnail": Service(8069, True, 0.2, 1),
  "carEvents": Service(8070, True, 1., 1),
  "carParams": Service(8071, True, 0.02, 1),
  "driverCameraState": Service(8072, True, 10. if EON else 20., 1),
  "driverEncodeIdx": Service(8061, True, 10. if EON else 20., 1),
  "driverState": Service(8063, True, 10. if EON else 20., 1),
  "driverMonitoringState": Service(8073, True, 10. if EON else 20., 1),
  "offroadLayout": Service(8074, False, 0.),
  "wideRoadEncodeIdx": Service(8075, True, 20., 1),
  "wideRoadCameraState": Service(8076, True, 20., 1),
  "modelV2": Service(8077, True, 20., 20),
  "managerState": Service(8078, True, 2., 1),

  "dynamicFollowData": Service(8079, True, 20.),
  "dynamicFollowButton": Service(8081, False, 0.),  # 8080 is reverved
  "laneSpeed": Service(8082, False, 0.),
  "laneSpeedButton": Service(8083, False, 0.),
  "dynamicCameraOffset": Service(8084, False, 0.),
  "modelLongButton": Service(8085, False, 0.),

  "testModel": Service(8040, False, 0.),
  "testLiveLocation": Service(8045, False, 0.),
  "testJoystick": Service(8056, False, 0.),
}


def build_header():
  h = ""
  h += "/* THIS IS AN AUTOGENERATED FILE, PLEASE EDIT services.py */\n"
  h += "#ifndef __SERVICES_H\n"
  h += "#define __SERVICES_H\n"
  h += "struct service { char name[0x100]; int port; bool should_log; int frequency; int decimation; };\n"
  h += "static struct service services[] = {\n"
  for k, v in service_list.items():
    should_log = "true" if v.should_log else "false"
    decimation = -1 if v.decimation is None else v.decimation
    h += '  { .name = "%s", .port = %d, .should_log = %s, .frequency = %d, .decimation = %d },\n' % \
         (k, v.port, should_log, v.frequency, decimation)
  h += "};\n"
  h += "#endif\n"
  return h


if __name__ == "__main__":
  print(build_header())
