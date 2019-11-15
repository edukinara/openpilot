#!/usr/bin/env python
import time
import zmq

from common.params import Params
from selfdrive.messaging import SubMaster

def dashboard_thread(rate=100):

  sm = SubMaster(['controlsState','pathPlan','liveParameters','gpsLocation'])

  vEgo = 0.0
  frame_count = 0

  server = "tcp://opdashboard.jevenet.com:8599"
  context = zmq.Context()
  pub_sock = context.socket(zmq.PUSH)
  pub_sock.connect(server)

  params = Params()
  user_id = params.get("DongleId", encoding='utf8')

  lateral_type = ""
  gpsFormatString = "location,user=" + user_id + " latitude=%s,longitude=%s,altitude=%s,speed=%s,bearing=%s %s\n"
  pathFormatString = "pathPlan,user=" + user_id + " l0=%s,l1=%s,l2=%s,l3=%s,r0=%s,r1=%s,r2=%s,r3=%s,d0=%s,d1=%s,d2=%s,d3=%s,l_prob=%s,r_prob=%s,lane_width=%s,rate_steers_des=%s %s\n"
  pathDataFormatString = "%0.2f,%0.2f,%0.2f,%0.2f,%d|"
  polyDataString = "%.10f,%0.8f,%0.6f,%0.4f,"
  pathDataString = ""
  liveParamsFormatString = "liveParameters,user=" + user_id + " yaw_rate=%s,gyro_bias=%s,angle_offset=%s,angle_offset_avg=%s,tire_stiffness=%s,steer_ratio=%s %s\n"
  liveParamsString = "%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%d|"
  liveParamsDataString = ""
  influxDataString = ""
  gpsDataString = ""
  insertString = "40bcc0"

  monoTimeOffset = 0
  receiveTime = 0
  active = False

  while 1:
    sm.update(0)
    if sm.updated['liveParameters']:
      _liveParams = sm['liveParameters']
#      print('2')
      for _lp in _liveParams:
        lp = _lp.liveParameters
        if vEgo > 0 and active:
          liveParamsDataString += (liveParamsString % (lp.yawRate, lp.gyroBias, lp.angleOffset, lp.angleOffsetAverage, lp.stiffnessFactor, lp.steerRatio, receiveTime))

    if sm.updated['gpsLocation']:
#      print('3')
      _gps = sm['gpsLocation']
      for _g in _gps:
        receiveTime = int(monoTimeOffset + _g.logMonoTime)
        if (abs(receiveTime - int(time.time() * 0.001)) > 10000):
          monoTimeOffset = (time.time() * 1000000000) - _g.logMonoTime
          receiveTime = int(monoTimeOffset + _g.logMonoTime)
        lg = _g.gpsLocation
        gpsDataString += ("%f,%f,%f,%f,%f,%d|" %
              (lg.latitude ,lg.longitude ,lg.altitude ,lg.speed ,lg.bearing, receiveTime))
        frame_count += 100

    if sm.updated['pathPlan']:
#      print('4')
      _pathPlan = sm['pathPlan']
      for _pp in _pathPlan:
        pp = _pp.pathPlan
        if vEgo > 0 and active:
          pathDataString += polyDataString % tuple(map(float, pp.lPoly))
          pathDataString += polyDataString % tuple(map(float, pp.rPoly))
          pathDataString += polyDataString % tuple(map(float, pp.dPoly))
          pathDataString +=  (pathDataFormatString % (pp.lProb, pp.rProb, pp.laneWidth, pp.rateSteers, int(monoTimeOffset + _pp.logMonoTime)))

    if sm.updated['controlsState']:
#      print('5')
      _controlsState = sm['controlsState']
      for l100 in _controlsState:
        if lateral_type == "":
          if l100.controlsState.lateralControlState.which == "pidState":
            lateral_type = "pid"
            influxFormatString = user_id + ",sources=capnp angle_steers_des=%s,angle_steers=%s,steer_override=%s,v_ego=%s,p=%s,i=%s,f=%s,output=%s %s\n"
          else:
            lateral_type = "indi"
            influxFormatString = user_id + ",sources=capnp angle_steers_des=%s,angle_steers=%s,steer_override=%s,v_ego=%s,output=%s,indi_angle=%s,indi_rate=%s,indi_rate_des=%s,indi_accel=%s,indi_accel_des=%s,accel_error=%s,delayed_output=%s,indi_delta=%s %s\n"
        vEgo = l100.controlsState.vEgo
        active = l100.controlsState.active
        receiveTime = int(monoTimeOffset + l100.logMonoTime)
        if (abs(receiveTime - int(time.time() * 0.001)) > 10000):
          monoTimeOffset = (time.time() * 1000000000) - l100.logMonoTime
          receiveTime = int(monoTimeOffset + l100.logMonoTime)
        if vEgo > 0 and active:
          dat = l100.controlsState

          if lateral_type == "pid":
            influxDataString += ("%0.3f,%0.2f,%d,%0.1f,%0.4f,%0.4f,%0.4f,%0.4f,%d|" %
                (dat.angleSteersDes, dat.angleSteers, dat.steerOverride, vEgo,
                dat.lateralControlState.pidState.p, dat.lateralControlState.pidState.i, dat.lateralControlState.pidState.f,dat.lateralControlState.pidState.output, receiveTime))
          else:
            s = dat.lateralControlState.indiState
            influxDataString += ("%0.3f,%0.2f,%d,%0.1f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%d|" %
                (dat.angleSteersDes, dat.angleSteers,  dat.steerOverride, vEgo,
                s.output, s.steerAngle, s.steerRate, s.rateSetPoint, s.steerAccel, s.accelSetPoint, s.accelError, s.delayedOutput, s.delta, receiveTime))

          frame_count += 1

    if frame_count >= 100:
      if influxDataString != "":
        insertString += influxFormatString + "~" + influxDataString + "!"
      if pathDataString != "":
        insertString += pathFormatString + "~" + pathDataString + "!"
      if liveParamsDataString  != "":
        insertString += liveParamsFormatString + "~" + liveParamsDataString + "!"
      if gpsDataString != "":
        insertString += gpsFormatString + "~" + gpsDataString + "!"
      if insertString != "40bcc0":
#        pub_sock.send_string(insertString)
        print(len(insertString))
        frame_count = 0
        influxDataString = ""
        gpsDataString = ""
        liveParamsDataString = ""
        pathDataString = ""
        insertString = "40bcc0"
    else:
      time.sleep(0.01)

def main(rate=200):
  dashboard_thread(rate)

if __name__ == "__main__":
  main()
