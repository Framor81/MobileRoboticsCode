import ..communication
import ..led
import ..display
import ..motors

class ForwardMover implements Communicator:
  state := Communicator.DISABLED

  led/Led := Led
  display/Display := Display --inverted=true
  motors/Motors := Motors
    
  constructor:
    led.off
    motors.stop

  on-start address port: 
    display.add-text "$address"
    display.add-text --y=32 "$port"
    display.draw

    
  on-open: enable
  on-close: disable
  on-message message:
    print "Received message: $message"
    enable

  is-enabled:
    return state == Communicator.ENABLED

  enable:
    if state == Communicator.ENABLED: return
    print "Enabling"
    state = Communicator.ENABLED
    led.on

  disable:
    if state == Communicator.DISABLED: return
    print "Disabling"
    state = Communicator.DISABLED
    led.off
    motors.stop

main:
  forward-mover := ForwardMover
  comm := WsCommunication forward-mover --heartbeat-ms=1000
  forward-time := 5_000    

  while true:

    if forward-mover.is-enabled:

      // The motors might turn off if forward-mover is disabled
      // by the communication model
      forward-mover.motors.set-speed-forward -0.5
      
      sleep --ms=forward-time
      forward-mover.motors.stop
      break
    
    else:
      sleep --ms=1000

 