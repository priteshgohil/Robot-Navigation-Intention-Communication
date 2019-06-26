# Message Framework:

The message sent trough serial port was defined as a string with a definite composition, 
which allows us to parse the data in an easy way in our Circuit Playground



`<RIC##LED:0387:FFFF00;PAT:1000;BUZ:2000:50:2001#RIC>`

<RIC##: Represent the init of our string

 RIC>:  Represent the end of our string

Each input parameter is separated by semi colon
1. **LED:0387:FFFF00;**

  - LED: Word to identify parameters related to LEDs
  
  - XXXX: Hexadecimal representation of the desired leds to be turned on
  
  - XXXXXX; Hexadecimal representation of the desired colour which is RRGGBB
             Red, Green, Blue

2. **PAT:1000**

  - PAT: Word to identify parameters related to Pattern
 
  - XXXX; Desired pattern to be executed by the Circuit Playground. Example: all leds blinking, sspecified sequence, gradient of colors


3. **BUZ:2000:50:2001**

  - BUZ: Word to identify parameters related to Buzzer
 
  - XXXX: Time in miliseconds
 
  - XX: Desired frequency
 
  - XXXX: pattern for sound tone
