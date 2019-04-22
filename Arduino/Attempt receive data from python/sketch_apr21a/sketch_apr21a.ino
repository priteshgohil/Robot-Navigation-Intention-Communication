#include <Adafruit_CircuitPlayground.h>
const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data
bool newData = false;
bool execute = false;
char data_1[32]={0};
char data_2[32]={0};
int data_3=0;

void setup() {
 CircuitPlayground.begin();
 Serial.begin(9600);
 Serial.println("<Arduino is ready>");
}

void loop() {
 is_data_available();
 parsed_Data();
 execute_action();
 
}

void execute_action(){
  if (execute == true) 
  {
    int i=0;
    String str_1 = data_1;
    Serial.println(str_1);
    if (str_1.equals("Goto")){
          for(i=0;i<=data_3;i++){
              CircuitPlayground.setPixelColor(i,0,0,255);
              delay(500);
              }
    }
    else if (str_1.equals("Dock")){
      for(i=data_3;i>=0;i--){
          CircuitPlayground.setPixelColor(i,0,0,0);
          delay(500);
        }
    }//end else if
    execute = false;
  }//end_new_data
}//end execute_action
  

void parsed_Data()
{
  if (newData == true) {
  char * str_idx;
  str_idx = strtok(receivedChars,":");
  strcpy(data_1,str_idx);
  Serial.println(data_1);
  str_idx = strtok(NULL,":");
  strcpy(data_2,str_idx);
  Serial.println(data_2);
  str_idx = strtok(NULL,":");
  data_3=atoi(str_idx);
  Serial.println(data_3);
  newData = false;
  execute = true;
  }
}

void is_data_available() {
 static byte idx = 0;
 char endline = '\n';
 char rc;
 
  if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
 rc = Serial.read(); //Also Serial.readString(); is available but parsing the data should be implemented in a different way

 if (rc != endline) 
 {
   receivedChars[idx] = rc;
   idx++;
    if (idx >= numChars) 
      {idx = numChars - 1;}
 }//end if endline
 else 
 {
   receivedChars[idx] = '\0'; // terminate the string
   idx = 0;
   newData = true;
 }
 }
}
}
