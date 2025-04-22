## Steps for Simple Operation

### Setup:
1. Plug in Stepper Motor and Stepper Encoder cables  
2. Plug in a Cat6 (Ethernet) cable to the control and power unit and the pendulum cart  
3. Plug in a C13 cable into the control and power unit  
4. Ensure emergency stop button is plugged into GND and pin 52 (unplugged will cause fault)  

### Operation:
1. Place the cart in the center (line up the two black lines on the front)  
2. Let the pendulum arm hang down freely  
3. Flip on power switch on control and power unit  
4. Once red light on Arduino turns off, flip pendulum arm vertical  
5. Once arm is 180 degrees from its hanging position, the balancing control will activate  

### On Fault:  
(*If angle > 15 degrees, position exceeds safe limits, or emergency stop button is pressed*)  
1. Cart will return to center automatically  
2. Let the pendulum arm hang down freely  
3. Once red light on Arduino turns off, flip pendulum arm vertical  
4. Once arm is 180 degrees from its hanging position, the balancing control will activate  
