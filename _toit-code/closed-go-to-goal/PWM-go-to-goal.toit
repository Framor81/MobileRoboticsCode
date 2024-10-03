import communication show Communicator WsCommunication

import ..led
import ..motors
import ..pinout
import ..utilities


left-motor := Motor LEFT-MOTOR-DIR-PIN LEFT-MOTOR-PWM-PIN
right-motor := Motor RIGHT-MOTOR-DIR-PIN RIGHT-MOTOR-PWM-PIN

main:
  motor-speed := 0.0

  while motor-speed <= 1:

    print "$motor-speed"
    left-motor.set-pwm-duty-factor motor-speed
    right-motor.set-pwm-duty-factor motor-speed

    sleep --ms=1000

    motor-speed += 0.01


  left-motor.stop
  right-motor.stop
  print "DONE"
