import ..motors

main: 

  motors := Motors
  updates := 4

  while true:

    print "Turning in Place..."

    motors.set-left-speed 0.5
    motors.set-right-speed -0.5

    updates.repeat:
      print "Left: $(%.2f (motors.left-encoder.get-speed*100)), Right: $(%.2f (motors.right-encoder.get-speed*100))"
      sleep --ms=500

    print "Stopping..."
    motors.stop
    sleep --ms=1_000

  motors.close