🚨 Read before installing! 🚨
------
To increase the probability that you have an excellent experience and choose the right branch for your car, it is recommended to read this before proceeding.

[![](https://i.imgur.com/JmHgT9D.png)](#)
  
🚗 Installation  
------
* Install via URL: smiskol.com/fork/aragon7777/0.8.9-devel-combo-honda
* Install via SSH: `cd /data; cp -rf ./openpilot ./openpilot.bak; rm -rf ./openpilot; git clone https://github.com/Aragon7777/openpilot.git openpilot; cd openpilot; git checkout 0.8.9-devel-combo-honda && reboot`
* Don't forget to check out the new toggles via the settings!
  
↪️ Spektor56 behavior explained:
------
* Honda and Toyota: Behavior like stock Honda or Toyota Sensing, thanks to Spektor56. 
*      LKAS and ACC are two separate functions that can be used independently.
*           LKAS: Lane-keeping-assist-system.
*           LKAS is activated using the LKAS button the steering wheel.
*           LKAS is active when the built in HUD lanelines are solid. 
*           LKAS is inactive when the built in HUD lanelines are outlined (Honda) or do not exist (Toyota).
*           LKAS will disengage on brake, but automatically come back.
*           LKAS will disengage below the Auto Lane Change (ALC) speed with blinker.
*           LKAS will stay disengaged briefly after blinkers, this helps driver recenter wheel.
*           LKAS will stay disengaged if seatbelt unlatched, door open, or unsupported gear.
*           ACC: Adaptive cruise control.
*           ACC is activated using the SET or RES(ume) button on the steering wheel.
*           ACC will disengage on brake, and never automatically come back until reset by the driver.
*           ACC can be adjusted in increments of 1MPH or +5MPH by holding, even with a (Honda) comma pedal.
*           ACC will not engage if seatbelt unlatched, door open, or unsupported gear.
*       Pedal: Speeds can now be set in increments of +/- 1 and hold down for +/- 5, just like stock behavior.
*       Driver Monitoring: Driver monitoring remains exactly the same as Comma's policy.  
  Note: Due to the separation of LKAS/ACC, driver monitoring requires fully disengaging and disabling both ACC and LKAS via their respective buttons, or pressing MAIN to clear a disengagment required alert. Failing to do so quickly enough might get you marked as too distracted and locked out until restarting the vehicle. Driver monitoring as a whole has been unchanged but this is a side effect to take into consideration.  
  
🏆 Special Thanks  
------  
[Spektor56](https://github.com/spektor56/openpilot)   
[eisenheim](https://github.com/eyezenheim/openpilot)  
[ShaneSmiskol](https://github.com/ShaneSmiskol/openpilot)    
[wirelessnet2](https://github.com/wirelessnet2/openpilot)    
[kegman](https://github.com/kegman/openpilot)    
[cfranhonda](https://github.com/cfranhonda/openpilot)    
[doktor](https://github.com/doktorsleepelss)    
[qadmus](https://github.com/qadmus/openpilot)  
[reddn](https://github.com/reddn)

📬 Contact  
------  
If you'd like to reach out to me, message `Aragon#7777` on Discord, or tag me in #custom-forks on the official Comma server regarding this branch.  