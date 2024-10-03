import communication show Communicator WsCommunication

import ..led
import ..motors
import ..pinout
import ..utilities


left-motor := Motor LEFT-MOTOR-DIR-PIN LEFT-MOTOR-PWM-PIN
right-motor := Motor RIGHT-MOTOR-DIR-PIN RIGHT-MOTOR-PWM-PIN

main:
  heartbeat-handler := HeartbeatHandler
  comm := WsCommunication heartbeat-handler --heartbeat-ms=1000

  sleep --ms=1000
  left-motor.set-pwm-duty-factor 1.0
  right-motor.set-pwm-duty-factor 0.98
  
  sleep --ms=5000 

  left-motor.stop
  right-motor.stop
  print "DONE"

  // uploading program causes wheel to spin
  // goes into floating state
  // no longer setting pin to specific values when uploading the prgraom
  // same thing when we dont have a program running at all
  while true:
    sleep --ms=1000
