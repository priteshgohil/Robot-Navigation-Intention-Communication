Message Framework:

The message sent trough serial port was defined as a string with a definite composition, 
which allows us to parse the data in an easy way in our Circuit Playground



<RIC##LED:0387:FFFF00;PAT:1000;BUZ:2000:50:2001#RIC>

<RIC##: Represent the init of our string

 RIC>:  Represent the end of our string

Each input parameter is separated by semi colon
* LED:0387:FFFF00;

  ** LED: Word to identify parameters ralated to LEDs
  
  ** XXXX: Hexadecimal representation of the desired leds to be turned on
  
  ** XXXXXX; Hexadecimal representation of the desired colour being RRGGBB
             Red, Green, Blue

* PAT:1000

  ** PAT: Word to identify parameters ralated to Pattern
 
  ** XXXX; Desired pattern to be executed by the Circuit Playground. Example: all leds blinking, sspecified sequence, gradient of colors


* BUZ:2000:50:2001

  ** BUZ: Word to identify parameters ralated to Buzzer
 
  ** XXXX: Time in miliseconds
 
  ** XX: Desired frequency
 
  ** XXXX; 
