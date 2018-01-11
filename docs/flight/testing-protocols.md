# Flight - Testing Protocols

This section covers procedures and protocols for testing your flight code on a real vehicle. It also includes details on preparing for a competition run.

### General Proverbs

1. Simulator first, vehicle second.
   1. **Any line of code that makes a motor move should be tested in a simulator first. **
      1. Even if you're doing tuning outside and only changing values by fractions of a decimal you should test first in a simulator. I often pre-test my testing values in the simulator. If, for instance I'm changing a value from 0.10 to 0.15 and I want to test every step in between, I test that in the simulator first. 
      2. **While I develop code in a simulator, I try to anticipate test cases and changes I may want to make while running on a real vehicle.**
2. Poke the hell out of your code.
   1. How does the simulated vehicle react if I put an obstacle here? Or, how does it react if I change this particular value? This goes hand in hand with first point. Poke holes in your code in the simulator to determine it's robustness and so that you won't be surprised when something abberant happens on a real vehicle.
3. Your code's effects should be reproducible.
   1. **The same code on different machines should produce the same output.**
   2. **The same code on different vehicles should produce the same output.**
   3. **You should be able to run your code multiple times on the same vehicle and get the same output.**
   4. As with any code we write, we care about whether or not the solution works but the most important thing to remember when dealing with systems that could seriously injure someone and cost a lot of money, is whether or not the solution is reliable and safe.

### Testing Locations

#### Outdoor

Typical outdoors testing tips:

* Ensure that it's a clear day with no rain in the forecast and light wind. 
* Test on grass or some other semi-soft surface outdoors, even if the vehicle has landing gear. 
* Make sure that the grass or other surface is not wet from recent rain.
* Make sure that you are in an open area, at least **25-50 meters **from other people, buildings, etc.
* Ensure that you are not in the vehicle's flight path.

Typical outdoor testing locations

* Lion's Club Park or any other park.
* IM Fields
* Football Fields
* Havener Lawn \(After 5PM or usually anytime on the weekends.\)
* Field of Dreams \(Between RC1 and RC2.\)
* For simple flight maneuvers, there is a small space directly next to the design center near the Rec center. It's directly across from the garage door as you leave the SDELC.

#### Indoor

Typical indoors testing tips:

* Make sure that you are in a large open area, with walls at least **5-10 meters away.**
* Ensure that you are not in the vehicle's flight path.

Typical indoor testing locations

* Gymnasium \(A single 1/3rd of the gym will suffice\)
* Black Box in Leach Theatre

### Autonomous flight run procedures for testing and competition

1. Boot the vehicle and connect to it via SSH over WiFi.

   1. Verify that optical flow is enabled. \(FLOW\_ENABLE is set to 1.\)

   2. Verify that the battery voltage is above the failsafe for that particular vehicle.

2. Copy your code over to the vehicle with the following commands. \(The inclusion of the **-r** flag assumes that you are copying a directory.\)

   ```
   scp -r ~/Desktop/MyFlightCodeDirectory mrr@192.168.12.1:~/MyFlightCodeDirectory
   ```

3. Stop the service that bridges the PixHawk to WiFi with either of the following commands:

   ```
   drone_console
   (Then hit CTRL-C)

   OR 

   sudo service mavlinkbridge stop
   ```

4. Start with the vehicle in Stabilize mode and throttle up until you see the motors move **but do not takeoff yet**. Ensure that the kill-switch is functional by stopping the motors.

   1. If you have changed the vehicle's structure or an important parameter and are unsure of the vehicle's flight readi-ness, you can perform a **1m** flight in Stabilize in addition to the below instructions. **Flying in Stabilize will not use the OF sensor.**

5. Switch to LOITER mode or whichever flight mode you intend to test with and do a transmitter flight at around 1m.

   1. Ensure that the vehicle hovers normally with no abberant behavior.

6. Ensure that someone has a transmitter and that the Pilot is aware of:

   1. What to expect from your code.

   2. The kill-switch location/function.

   3. If possible, how to manually take over the vehicle.

7. Test your code.

   1. Any change you make to your code during testing should be done on your local machine and then copied via the scp command above. Feel free to copy the entire directory. **Do not make a quick edit while SSH'd \(even for trivial changes\) on the companion computer itself. This ensures that the code that you are looking at in your editor on your laptop is in sync with what is actually running on the vehicle.**
   2. **After every 2 or so tests, you should re-enable the service that bridges the PixHawk to WiFi to check the battery voltage of the vehicle in a GCS.** If the PixHawk is connect to the companion computer via USB then the battery failsafe will not trigger, and so we must manually check the battery voltage.
      ```
      sudo service mavlinkbridge restart
      ```



