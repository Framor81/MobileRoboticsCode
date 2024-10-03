import ..communication
import ..led
import ..display
import ..motors

class IPDisplay implements Communicator:
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
    motors.set-speed-forward 0.25

  disable:
    if state == Communicator.DISABLED: return
    print "Disabling"
    state = Communicator.DISABLED
    led.off
    motors.stop

main:
  ip-display := IPDisplay
  comm := WsCommunication ip-display --heartbeat-ms=1000


 