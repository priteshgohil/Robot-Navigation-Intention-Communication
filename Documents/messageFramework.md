Message Framework:

The message sent trough serial port was defined as a string with a definite composition, 
which allows us to parse the data in an easy way in our Circuit Playground



<RIC##LED:0387:FFFF00;PAT:1000;BUZ:2000;50;2001#RIC>

<RIC##: Represent the init of our string
 RIC>:  Represent the end of our string

The data contained in the string can be visualzied in the next table

LED         :XXXX  Material      Color
----------- -----  ------------  ------------
LED command desired led  leather       brown

Table: Shoes sizes, materials, and colors.
