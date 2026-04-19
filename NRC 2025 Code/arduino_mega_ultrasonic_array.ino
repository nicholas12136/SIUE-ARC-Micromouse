/***************************************************************************************
Reading Four HC-SR04 Sensors on Arduino Mega
A wiring diagram is available at the bottom of this code.
***************************************************************************************/

//Defining constant integers for the GPIOs. The constant portion of these means they will not change[1]. 
//They are an integer, so they are simply a whole number. Our Arduino stores integers using 16 bits giving a possible range of +/- 32,767 as the stored value[2].
const int trigPin_1 = 6; //trigger pin for first sensor on Pin 6
const int trigPin_2 = 7; //trigger pin for second sensor on Pin 7
const int trigPin_3 = 8; //trigger pin for second sensor on Pin 8
const int trigPin_4 = 9; //trigger pin for second sensor on Pin 9
const int echoPin_1 = 10;//trigger pin for second sensor on Pin 10
const int echoPin_2 = 11;//trigger pin for second sensor on Pin 11
const int echoPin_3 = 12;//trigger pin for second sensor on Pin 12
const int echoPin_4 = 13;//trigger pin for second sensor on Pin 13

//Defining floating point numbers for our duration and distance variables. 
//The 'float' means these variables can be decimal numbers. Arduino stores these as 32-bit values which can hold a decimal value of +/-  3.4028235E+38 [3]. 
float duration,duration_1,duration_2,duration_3,duration_4,distance_1,distance_2,distance_3, distance_4;

//This is our setup function. We could have initialized our variables here as well instead of above[4]. 
//This code runs once to set the groundwork for the rest of what our code will be doing.
void setup() {
  pinMode(trigPin_1, OUTPUT);//set the Arduino pin that we have connected to the trigger pin on our sensor 1 to output[5].
  pinMode(trigPin_2, OUTPUT);
  pinMode(trigPin_3, OUTPUT);
  pinMode(trigPin_4, OUTPUT);
  pinMode(echoPin_1, INPUT); //set the Arduino pin that we have connected to the echo pin on our sensor 1 to input[5].
  pinMode(echoPin_2, INPUT);
  pinMode(echoPin_3, INPUT);
  pinMode(echoPin_4, INPUT);
  Serial.begin(9600);        //begin serial communication[6]. 
}

//cdefine a function to read a sensor[7]. this function is essentially just the loop of the original code we were working with.
//the data this function returns is a float value so we start with that
//read_distance is the name of the function and we are passing it two arguments as integers: int trig (the trigger pin of the sensor we want to read) and int echo. 
float read_distance(int trig, int echo){
  digitalWrite(trig, LOW);                //set the pin that corresponds with the value of the argument 'trig' to LOW.
  delayMicroseconds(2);                   
  digitalWrite(trig, HIGH);               //set that pin HIGH.
  delayMicroseconds(10);                  //delay is 10 microseconds because that is how long the sensor pulse is[8, section 4. Hardware Information].
  digitalWrite(trig, LOW);                
  duration = pulseIn(echo, HIGH);         //pulseIn function: starts timing when the 'echo' pin goes HIGH. stops timing when the echo pin goes LOW[9].
  return(duration);                       //return the duration value
}

void loop() {
  duration_1 = read_distance(trigPin_1, echoPin_1);//call the read_distance function for sensor 1. save the value it returns as duration_1.
  delay(25);                                       //wait a lil bit to make sure our second sensor doesn't accidentally read the pulse from sensor 1.  
  duration_2 = read_distance(trigPin_2, echoPin_2);
  delay(25);
  duration_3 = read_distance(trigPin_3, echoPin_3);
  delay(25);
  duration_4 = read_distance(trigPin_4, echoPin_4);
  delay(25);
  
  distance_1 = (duration_1*.01350)/2;             //calculate distance_1. (time in microseconds) * (velocity of sound in inches/microsecond) / (2)
  distance_2 = (duration_2*.01350)/2;
  distance_3 = (duration_3*.01350)/2;
  distance_4 = (duration_4*.01350)/2;
  Serial.println("Distance1: ");                  //serial print line[10].
  Serial.println(distance_1);
  Serial.println("Distance 2: ");
  Serial.println(distance_2);
  Serial.println("Distance 3: ");
  Serial.println(distance_3);
  Serial.println("Distance 4: ");
  Serial.println(distance_4);                     //Access your Serial Monitor to see this data using: Tools > Serial Monitor.
  
 
  delay(1000);                                    //one second delay so we can actually look at the data as it comes in[11].
}
//wiring diagram[12]

/***************************************************************************************
Sources:

[1]https://docs.arduino.cc/language-reference/en/variables/variable-scope-qualifiers/const/
[2]https://docs.arduino.cc/language-reference/en/variables/data-types/int/
[3]https://docs.arduino.cc/language-reference/en/variables/data-types/float/
[4]https://docs.arduino.cc/language-reference/en/structure/sketch/setup/
[5]https://docs.arduino.cc/language-reference/en/functions/digital-io/pinMode/
[6]https://docs.arduino.cc/language-reference/en/functions/communication/serial/begin/
[7]https://docs.arduino.cc/learn/programming/functions/
[8]https://www.handsontec.com/dataspecs/HC-SR04-Ultrasonic.pdf
[9]https://docs.arduino.cc/language-reference/en/functions/advanced-io/pulseIn/
[10]https://docs.arduino.cc/language-reference/en/functions/communication/serial/println/
[11]https://docs.arduino.cc/language-reference/en/functions/time/delay/
[12]https://app.cirkitdesigner.com/project/5233f618-ebff-4be5-a1e8-7c40f4ec8d1d

***************************************************************************************/