#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include<stdio.h>
#include<string.h>

//LCD and buttons
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int btn_key;                                        //records button selected
#define SID 13214489
#define BTNRIGHT  0
#define BTNUP     1
#define BTNDOWN   2
#define BTNLEFT   3
#define BTNSELECT 4
#define BTNNONE   5

#define pi 3.14159265359

#define WATER 1
#define FIRE 2

//compass
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

#define LEFT 12
#define RIGHT 9
#define STRAIGHT 69
#define BENTCLOSE 420
#define BENTFAR 506
int compass = EAST, x = 2, y = 4;
int robot_x = 2, robot_y = 4;
int goal_x = 2, goal_y = 4;

//Delete
int movement = 0, taskToComplete = 0, goalObtained = 0; //movement records how many units robot moved, task records if at wall follow, finding water or fire
double distFromGoal = 0.0;
int goalID = 0;
//timer
unsigned volatile long dispTimerCurrentTime = 0, dispTimerPreviousTime = 0;
volatile int seconds = 0, minutes = 0;
int tempFlag = 1;
boolean timer = 1;

//Main menu
int menu = 0;
//Timer and ADC
volatile long unsigned int milliseconds = 0;

//Flags to stay in functions
int controlFlag = 1;

//Serial functions
char terminator = 13;

//PID
double rad = (pi / 180), degree = (180 / pi); //alpha is the angle between d1 and d2 (i.e. angle difference)
double alpha = (45.0 * rad), theta = 0.0, frontDist = 0.0;
//theta is the angle difference between ideal direction (parralel to the wall)and real direction
double distWall = 0.0; //d1, d2, dw,x d is local
int arrDRSize = 25; //MUST CHANGE 2/2, THIS AND BELOW; remember to change array size below in String distanceReadings[int]
String distanceReadings[25], medianDistance;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  lcd.begin(16, 2);                                            //initialise LCD, must always be done or LCD will be unpredictable
  Serial.begin(9600);
  //initialise, timer, adc and ports
  set_up_adc();
  //set_up_timer();

  lcd.setCursor(0, 0);
  lcd.print(SID);
  lcd.setCursor(0, 1);
  lcd.print("00:00");
  //update_compass(0);
}

void loop()
{
  btn_key = read_lcd_buttons();
  switch (btn_key)
  {
    case BTNSELECT:

      PrintMessage("CMD_START");
      set_up_timer();
      my_delay(100);
      update_lcd_screen();
      wall_follow_mode();

      break;
  }
}

void display_timer() //attempt 2
{
  if (timer == 1)
  {
    dispTimerCurrentTime = my_millis(); //increment seconds and then minutes

    if ((dispTimerCurrentTime - dispTimerPreviousTime) >= 1000) // 1s = 1000ms
    {
      seconds ++;
      Serial.println(seconds);

      if (seconds < 10)
      {
        lcd.setCursor(4, 1);
        lcd.print(seconds);
      }
      else if ((seconds >= 10) && (seconds < 60))
      {
        lcd.setCursor(3, 1);
        lcd.print(seconds);
      }
      else if (seconds >= 60)
      {
        seconds = 0;
        minutes ++;
        lcd.setCursor(3, 1);
        lcd.print("00");

        if (minutes < 10)
        {
          lcd.setCursor(1, 1);
          lcd.print(minutes);
        }
        else if (minutes > 10)
        {
          lcd.setCursor(0, 1);
          lcd.print(minutes);
        }
      }

      dispTimerPreviousTime = dispTimerCurrentTime;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Wall Follow Tab
void wall_follow_mode()
{
  int tempFlag = 1;
  int movementCountingTurns = 0;
  movement = 0;

  /*while (tempFlag == 1)
    {

    }*/
  while (goalObtained != 2)
  {
    distWall = find_dw(); //gives dw, theta, frontDist

    my_delay(1000);

    robotDecideMovement();

    my_delay(2000);

    checkForGoal();

    my_delay(1000);

    goalObtained = CMD_SEN_GOAL();
    if(goalObtained == 2)
    {
      break;
    }
    
    update_lcd_screen();

    my_delay(1000);
  }

  PrintMessage("CMD_ACT_ROT_1_180");


  for (int reversingMovement = 0; reversingMovement <= movement; reversingMovement++)
  {
    lcd.setCursor(14,1);
    lcd.print("H");
    distWall = go_home_dw();

    my_delay(1000);

    robotDecideHowToGoHome();

    my_delay(1000);    
  }

  end_robot();
  
  PrintMessage("CMD_CLOSE");
  
}

void update_lcd_screen()
{
  if (goalObtained == 0)
  {
    lcd.setCursor(14, 1);
    lcd.print("W");
  }
  else if (goalObtained == 1)
  {
    lcd.setCursor (14, 1);
    lcd.print("F");
  }
  else if (goalObtained == 2)
  {
    lcd.setCursor(14, 1);
    lcd.print("H");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Wall Follow Functions Tab
void robotDecideMovement()
{
  if ((frontDist > 0) && (frontDist < 1.89)) //wall is too close, turn
  {
    PrintMessage("Wall too close");
    if ((distWall > 0) && (distWall < 5)) //wall on the right, turn left
    {
      PrintMessage("Turn left");
      adjust_angle();//adjust angle so turn is correct
      PrintMessage("CMD_ACT_ROT_0_88");
      update_compass(LEFT);
      my_delay(250);
    }
    else //there's no wall on the right, turn right
    {
      PrintMessage("Turn right");
      adjust_angle();//adjust angle so turn is correct
      PrintMessage("CMD_ACT_ROT_1_90");
      update_compass(RIGHT);
      my_delay(250);
    }
  }

  else//no wall, far enough
  {
    adjust_angle();
    adjust_distance();
    movement ++;
  }
}
void adjust_distance()
{
  PrintMessage("Adjust distance");
  double error = 0.0, errorDegree = 0.0;

  if ((distWall > 0.0) && (distWall <= 5.0)) //avoid NaN values
  {
    if ((distWall > 1.89) && (distWall < 2.10)) //distance is accurate and is between 1.90 - 2.1
    {
      PrintMessage("Distance Accurate");
      PrintMessage("CMD_ACT_LAT_1_1");
      update_robot_location(STRAIGHT);
      my_delay(600);
    }
    else if ((distWall > 0.0) && (distWall < 1.90)) // 0.00 - 1.90
    {
      PrintMessage("Too close");
      error = 2.0 - distWall;
      errorDegree = (atan(error));//no need to multiply by rad
      errorDegree = (errorDegree * degree); //gives values in rad

      CMD_ACT_ROT_(0, errorDegree);
      PrintMessage("CMD_ACT_LAT_1_1");
      CMD_ACT_ROT_(1, errorDegree);
      my_delay(600);
      //update_robot_location(BENTFAR);
    }
    else if ((distWall > 2.10) && (distWall <= 5.0)) //2.11 - 5.0
    {
      PrintMessage("Too far");
      error = distWall - 2.0;
      errorDegree = (atan(error));
      errorDegree = (errorDegree * degree);

      CMD_ACT_ROT_(1, errorDegree);
      PrintMessage("CMD_ACT_LAT_1_1");
      CMD_ACT_ROT_(0, errorDegree);
      my_delay(600);
    }
  }
  else
  {
    PrintMessage("CMD_ACT_LAT_1_1");
    my_delay(600);
  }
}

void adjust_angle()
{
  PrintMessage("Adjust angle");
  if ((theta >= 0.2) && (theta < 16.0)) //+ve theta, facing away from wall, so correct by moving robot direction towards wall (CW)
  {
    CMD_ACT_ROT_(1, theta);
    //PrintMessage("CMD_ACT_LAT_1_1");
  }
  else if ((theta > -16.0) && (theta <= -0.2)) //-ve theta, facing wall, correct by moving robot CCW
  {
    theta = ((-1.0) * theta);
    CMD_ACT_ROT_(0, theta);
    //PrintMessage("CMD_ACT_LAT_1_1");
  }
  my_delay(250);
}

double find_dw()
{
  //check if robot is parallel using PID controls
  PrintMessage("CMD_SEN_ROT_0");
  //PrintMessage("CMD_SEN_IR");
  for (int readingForFrontWall = 0; readingForFrontWall < arrDRSize; readingForFrontWall++) //enter values into an array so that values can be organised in ascending order then the median found
  {
    PrintMessage("CMD_SEN_IR"); //read the value of the IR for the first time
    distanceReadings[readingForFrontWall] = Serial.readStringUntil(terminator); //read incoming value from sensor and store in array
  }
  sort_ascending(distanceReadings); //order values of array in ascending order
  medianDistance = find_median(distanceReadings);  //find the median of the array
  //distanceReadings[0] = Serial.readStringUntil(terminator);
  frontDist = medianDistance.toFloat();

  //my_delay(150);

  PrintMessage("CMD_SEN_ROT_270");//check d1 at 270 degrees, check d2 at 315

  for (int x = 0; x < arrDRSize; x++) //enter values into an array so that values can be organised in ascending order then the median found
  {
    PrintMessage("CMD_SEN_IR"); //read the value of the IR for the first time
    distanceReadings[x] = Serial.readStringUntil(terminator); //read incoming value from sensor and store in array
  }
  sort_ascending(distanceReadings); //order values of array in ascending order
  medianDistance = find_median(distanceReadings);  //find the median of the array

  double d1 = medianDistance.toFloat();//convert to a float and store the distance

  my_delay(250);

  PrintMessage("CMD_SEN_ROT_315");//check d2 at 315

  for (int x = 0; x < arrDRSize; x++)
  {
    PrintMessage("CMD_SEN_IR");
    distanceReadings[x] = Serial.readStringUntil(terminator);
  }
  sort_ascending(distanceReadings);
  medianDistance = find_median(distanceReadings);

  double d2 = medianDistance.toFloat();//convert to a float and store the distance

  my_delay(250);

  //do calculations to find dw and theta
  double d1sq = pow(d1, 2), d2sq = pow(d2, 2), annoying = (2 * d1 * d2 * cos(alpha));
  //cos 45 = 0.5253, having cos in my equation fucked things up
  double dxsq = ((d1sq + d2sq) - annoying), dx = sqrt(dxsq);
  //dw
  double dw = ((d2 * d1 * (sin(alpha))) / dx);

  //theta
  double thetarad = (acos(dw / d2));
  theta = ((thetarad * degree) - 45); //if theta is +ve = facing away from wall, -ve = facing towards the wall

  Serial.print("theta = ");
  Serial.println(theta);
  endSerial();

  return dw;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Goal Finder Tab
void exploreNeighbour()
{
  PrintMessage("CMD_SEN_IR");
  String checkForCollision = Serial.readStringUntil(terminator);
  double distFromCollision = checkForCollision.toFloat();
  double goToGoal = distFromGoal - 0.3, safeMovement = distFromCollision - 0.3;
  my_delay(150);
  if ((distFromCollision > distFromGoal) && (distFromCollision > 0)) //0.1 is room for error, should robot move to fucken much. If wall < distFromGoal by 0.1m then move
  {
    CMD_ACT_LAT_(1, goToGoal);
    my_delay(700);
    CMD_ACT_LAT_(0, goToGoal);
    my_delay(700);
  }
  else if ((distFromCollision < distFromGoal) && (distFromCollision > 0))
  {
    CMD_ACT_LAT_(1, safeMovement);
    my_delay(600);
    CMD_ACT_LAT_(0, safeMovement);
    my_delay(600);
  }
  else // NaN
  {
    CMD_ACT_LAT_(1, distFromGoal);
    my_delay(600);
    CMD_ACT_LAT_(0, distFromGoal);
    my_delay(600);
  }
}

void checkForGoal()
{
  goalObtained = CMD_SEN_GOAL(); //check if goal is obtained
  my_delay(150);
  PrintMessage("CMD_SEN_ID");
  my_delay(100);
  String checkFire = Serial.readStringUntil(terminator);
  goalID = checkFire.toInt();

  if (goalObtained == 0) //finding water
  {
    PrintMessage("CMD_SEN_PING");
    String inputGoal = Serial.readStringUntil(terminator);
    distFromGoal = inputGoal.toFloat();
    my_delay(100);
    findGoalNearby();
  }
  else if (goalObtained == 1) //finding fire
  {
    PrintMessage("CMD_SEN_PING");
    String inputGoal = Serial.readStringUntil(terminator);
    distFromGoal = inputGoal.toFloat();

    my_delay(100);
    if (goalID == 2);
    {
      my_delay(100);
      findGoalNearby();
    }
  }
  else if (goalObtained == 2)
  {
    goalObtained = 2;
  }
  my_delay(500);
}

void findGoalNearby()
{

  if ((distFromGoal > 0) && (distFromGoal < 2.1))
  {

    PrintMessage("Finding Goal");
    //move forward
    PrintMessage("CMD_SEN_ROT_0");//check IR sensor for Wall
    exploreNeighbour();

    //if else, do nothing

    //turn left
    for (int increaseAngleSlowly = 0; increaseAngleSlowly < 3; increaseAngleSlowly++)
    {
      CMD_ACT_ROT_(1, 45); //rotate to left, 90 + 45
    }
    my_delay(100);
    exploreNeighbour();

    PrintMessage("CMD_ACT_ROT_0_45");//turnback to face 90
    my_delay(100);
    exploreNeighbour();

    PrintMessage("CMD_ACT_ROT_0_45");//turnback to face 45
    my_delay(100);
    exploreNeighbour();

    //turn right
    PrintMessage("CMD_ACT_ROT_0_90");//turn back to place position at 45 degrees to right
    my_delay(100);
    exploreNeighbour();

    PrintMessage("CMD_ACT_ROT_0_45");//turn to face 90
    my_delay(100);
    exploreNeighbour();

    PrintMessage("CMD_ACT_ROT_0_45");//turn to face 135
    my_delay(100);
    exploreNeighbour();

    //PrintMessage("CMD_ACT_ROT_1_135");//turn back to face front
    for (int increaseAngleSlowly = 0; increaseAngleSlowly < 3; increaseAngleSlowly++)
    {
      CMD_ACT_ROT_(1, 45); //rotate to left, 90 + 45
    }
    my_delay(100);
  }



  //else{}

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Go Home Tab
void end_robot()
{
  distWall = find_dw();
  double endingDist = 0.0;
  if ((frontDist > 0) && (frontDist < 1.89)) //wall is too close, turn
  {

    CMD_ACT_LAT_(0, (2 - frontDist));
  }

  else if ((frontDist > 0) && (frontDist > 2.1))
  {
    CMD_ACT_LAT_(1, (frontDist - 2));
  }

  my_delay(500);

  if ((distWall > 0) && (distWall < 2.1))
  {
    endingDist = (2 - distWall);
    PrintMessage("CMD_ACT_ROT_0_90");
    CMD_ACT_LAT_(0, endingDist);
  }
  else //there's no wall on the right, turn right
  {
    endingDist = (distWall - 2);
    PrintMessage("CMD_ACT_ROT_0_90");
    CMD_ACT_LAT_(0, endingDist);
  }
  
  my_delay(500);
  lcd.setCursor(14, 1);
  lcd.print("C");
  timer = 0;
  
}
void robotDecideHowToGoHome()
{
  if ((frontDist > 0) && (frontDist < 2)) //wall is too close, turn
  {
    PrintMessage("Wall too close");
    if ((distWall > 0) && (distWall < 5)) //wall on the right, turn left
    {
      PrintMessage("Turn right");
      homie_adjust_angle();//adjust angle so turn is correct
      PrintMessage("CMD_ACT_ROT_1_90");
      //update_compass(RIGHT);

    }
    else //there's no wall on the right, turn right
    {
      PrintMessage("Turn left");
      homie_adjust_angle();//adjust angle so turn is correct
      PrintMessage("CMD_ACT_ROT_0_88");
      //update_compass(LEFT);
    }
  }

  else//no wall, far enough
  {
    if ((distWall > 0) && (distWall < 5)) 
    {
    homie_adjust_angle();
    homie_adjust_distance();
    //movement ++;
    }
    else //no reading
    {
      PrintMessage("CMD_ACT_ROT_0_88");
      PrintMessage("CMD_ACT_LAT_1_1");   
      my_delay(100);   
    }
  }
}
double go_home_dw()
{
  //check if robot is parallel using PID controls
  PrintMessage("CMD_SEN_ROT_0");
  //PrintMessage("CMD_SEN_IR");
  for (int readingForFrontWall = 0; readingForFrontWall < arrDRSize; readingForFrontWall++) //enter values into an array so that values can be organised in ascending order then the median found
  {
    PrintMessage("CMD_SEN_IR"); //read the value of the IR for the first time
    distanceReadings[readingForFrontWall] = Serial.readStringUntil(terminator); //read incoming value from sensor and store in array
  }
  sort_ascending(distanceReadings); //order values of array in ascending order
  medianDistance = find_median(distanceReadings);  //find the median of the array
  //distanceReadings[0] = Serial.readStringUntil(terminator);
  frontDist = medianDistance.toFloat();

  //my_delay(150);

  PrintMessage("CMD_SEN_ROT_90");//check d1 at 270 degrees, check d2 at 315

  for (int x = 0; x < arrDRSize; x++) //enter values into an array so that values can be organised in ascending order then the median found
  {
    PrintMessage("CMD_SEN_IR"); //read the value of the IR for the first time
    distanceReadings[x] = Serial.readStringUntil(terminator); //read incoming value from sensor and store in array
  }
  sort_ascending(distanceReadings); //order values of array in ascending order
  medianDistance = find_median(distanceReadings);  //find the median of the array

  double d1 = medianDistance.toFloat();//convert to a float and store the distance



  PrintMessage("CMD_SEN_ROT_45");//check d2 at 315

  for (int x = 0; x < arrDRSize; x++)
  {
    PrintMessage("CMD_SEN_IR");
    distanceReadings[x] = Serial.readStringUntil(terminator);
  }
  sort_ascending(distanceReadings);
  medianDistance = find_median(distanceReadings);

  double d2 = medianDistance.toFloat();//convert to a float and store the distance



  //do calculations to find dw and theta
  double d1sq = pow(d1, 2), d2sq = pow(d2, 2), annoying = (2 * d1 * d2 * cos(alpha));
  //cos 45 = 0.5253, having cos in my equation fucked things up
  double dxsq = ((d1sq + d2sq) - annoying), dx = sqrt(dxsq);
  //dw
  double dw = ((d2 * d1 * (sin(alpha))) / dx);

  //theta
  double thetarad = (acos(dw / d2));
  theta = ((thetarad * degree) - 45); //if theta is +ve = facing away from wall, -ve = facing towards the wall

  return dw;
}
void homie_adjust_distance()
{
  PrintMessage("Adjust distance");
  double error = 0.0, errorDegree = 0.0;

  if ((distWall > 0.0) && (distWall <= 5.0)) //avoid NaN values
  {
    if ((distWall > 1.89) && (distWall < 2.10)) //distance is accurate and is between 1.90 - 2.1
    {
      PrintMessage("Distance Accurate");
      PrintMessage("CMD_ACT_LAT_1_1");
      update_robot_location(STRAIGHT);

    }
    else if ((distWall > 0.0) && (distWall < 1.90)) // 0.00 - 1.90
    {
      PrintMessage("Too close");
      error = 2.0 - distWall;
      errorDegree = (atan(error));//no need to multiply by rad
      errorDegree = (errorDegree * degree); //gives values in rad

      CMD_ACT_ROT_(1, errorDegree);
      PrintMessage("CMD_ACT_LAT_1_1");
      CMD_ACT_ROT_(0, errorDegree);

      //update_robot_location(BENTFAR);
    }
    else if ((distWall > 2.10) && (distWall <= 5.0)) //2.11 - 5.0
    {
      PrintMessage("Too far");
      error = distWall - 2.0;
      errorDegree = (atan(error));
      errorDegree = (errorDegree * degree);

      CMD_ACT_ROT_(0, errorDegree);
      PrintMessage("CMD_ACT_LAT_1_1");
      CMD_ACT_ROT_(1, errorDegree);

    }
  }
  else
  {
    PrintMessage("CMD_ACT_LAT_1_1");

  }
}

void homie_adjust_angle()
{
  PrintMessage("Adjust angle");
  if ((theta >= 0.2) && (theta < 30.0)) //+ve theta, facing away from wall, so correct by moving robot direction towards wall (CW)
  {
    CMD_ACT_ROT_(0, theta);
    //PrintMessage("CMD_ACT_LAT_1_1");
  }
  else if ((theta > -30.0) && (theta <= -0.2)) //-ve theta, facing wall, correct by moving robot CCW
  {
    theta = ((-1.0) * theta);
    CMD_ACT_ROT_(1, theta);
    //PrintMessage("CMD_ACT_LAT_1_1");
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Coordinates mapping tab
void update_robot_location(int movingCellDirection)
{
  if (movingCellDirection == STRAIGHT)
  {
    switch (compass)
    {
      case NORTH:
        {
          robot_y++;
          break;
        }

      case EAST:
        {
          robot_x++;
          break;
        }

      case SOUTH:
        {
          robot_y--;
          break;
        }

      case WEST:
        {
          robot_x--;
          break;
        }
    }
  }
  else if (movingCellDirection == BENTCLOSE)
  {
    switch (compass)
    {
      case NORTH:
        {
          robot_x++;
          robot_y++;
          break;
        }

      case EAST:
        {
          robot_x++;
          robot_y--;
          break;
        }

      case SOUTH:
        {
          robot_x--;
          robot_y--;
          break;
        }

      case WEST:
        {
          robot_x--;
          robot_y++;
          break;
        }
    }
  }

  else if (movingCellDirection == BENTFAR)
  {
    switch (compass)
    {
      case NORTH:
        {
          robot_x--;
          robot_y++;
          break;
        }

      case EAST:
        {
          robot_x++;
          robot_y++;
          break;
        }

      case SOUTH:
        {
          robot_y--;
          robot_x++;
          break;
        }

      case WEST:
        {
          robot_x--;
          robot_y--;
          break;
        }

    }
  }
  Serial.print("robot location");
  Serial.print(robot_x);
  Serial.print(",");
  Serial.println(robot_y);
  endSerial();

  my_delay(250);
}

void update_compass(int turningDirection)
{
  if (turningDirection == LEFT)
  {
    switch (compass)
    {
      case NORTH:
        {
          compass = WEST;
          break;
        }

      case EAST:
        {
          compass = NORTH;
          break;
        }

      case SOUTH:
        {
          compass = EAST;
          break;
        }

      case WEST:
        {
          compass = SOUTH;
          break;
        }
    }
  }

  else if (turningDirection == RIGHT)
  {
    switch (compass)
    {
      case NORTH:
        {
          compass = EAST;
          break;
        }

      case EAST:
        {
          compass = SOUTH;
          break;
        }

      case SOUTH:
        {
          compass = WEST;
          break;
        }

      case WEST:
        {
          compass = NORTH;
          break;
        }
    }
  }

  else
  {
    compass = compass;
  }
  /*lcd.setCursor(11, 0);
  lcd.print(compass);*/
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//constant functions tab
void swap(String *p, String *q) //used for sorting an array into ascending order
{
  String t;                                       //declare a replacement variable
  t = *p;                                         //the value of p is stored in t
  * p = *q;                                       //value in p is replaced by value in q
  * q = t;                                        //finally the value  in q is replaced by value in t (which was previously p)
}

void sort_ascending(String distanceReadings[]) //used for finding the median of an array by first sorting it into asceding order using this function
{
  int x = 0, y = 0; //x and y are the position of the values in the array, not the values themselves, they are used to compare the first value then the next value
  String a;

  for (x = 0; x < arrDRSize; x++)
  {
    for (y = x + 1; y < arrDRSize; y++)
    {
      if (distanceReadings[x] > distanceReadings[y])//if the current variable (y) is less than previous variable (x) then swap their positions
      {
        swap(&distanceReadings[x], &distanceReadings[y]);
      }
    }
  }

}

//finding median
String find_median(String inputArrayForMedian[])//input an array which has been organised in ascending order
{
  String median;
  int positionOfMedian = ((arrDRSize + 1)/2);
  median = inputArrayForMedian[positionOfMedian];//since all my arrays have 11 values, then the median will be 6 but array starts at 0 therefore 5, this function can be more flexible by having two inputs "String find_media (*string, *int), where int
  //is the array size
  return median;
}

//constant_functions tab : assessment 2 functions, these are adc, millis, delays etc.
//Buttons
int read_lcd_buttons()                      //this function determines which button is being pressed by getting the ADC reading from A0
{
  set_up_adc_buttons();

  if (ADC <= 0) return BTNRIGHT;

  if ((ADC <= 207) && (ADC >= 200)) return BTNUP;

  if ((ADC <= 408) && (ADC >= 308)) return BTNDOWN;

  if ((ADC <= 626) && (ADC >= 526)) return BTNLEFT;

  if ((ADC <= 826) && (ADC >= 820)) return BTNSELECT;

  return BTNNONE;  // when all others fail, return this...
}

//Timer, ISR, my_millis function
void set_up_timer()
{
  TCCR2A = 0x00;
  TCCR2B = 0x00;
  TCCR2B |= (1 << CS22);           //prescaler 64, approximately equal to 1ms overflow

  TIMSK2 = (1 << TOIE2);
  sei();
  TIFR2 |= (1 << TOV2);
}

ISR(TIMER2_OVF_vect)
{
  milliseconds++;
  display_timer();
}

void my_delay(int delaytime)
{
  volatile int TimerCounter = 0;
  while (TimerCounter <= delaytime)
  {
    if (TCNT2 >= 254)
    {
      TimerCounter++;
    }
  }
}

int my_millis()
{
  return milliseconds;//millisecond;
}

//ADC
void set_up_adc()
{
  ADCSRA = (1 << ADEN);     //set ADC enable on
  ADCSRA |= (1 << ADPS2);   //set ADC prescaler select
  ADCSRA |= (1 << ADPS1);
  ADCSRA |= (1 << ADPS0);  //prescaler 128, ADC clock to 125kHz
  ADMUX = (1 << REFS0);     //decide which pin you're reading from, uses internal vcc
  //set reference with vcc, left/right justified (it doesn't matter)
  //set MUX when you're reading the pin as you need to change it when you need it
  //prescaler factor: frequency speed of sampling, always be half a speed of your clock
  //if you're not checking often then you don't need a fast frequency

}

void set_up_adc_buttons()
{
  ADMUX &= 0b11110000;                      //resets pins, allowing it to be set to 0, therefore can interchange between IR analog sensor and buttons
  ADMUX |= (0 << MUX0);                     //which pin you're writing to
  ADCSRA |= (1 << ADSC);                    //start conversion
  while (!(ADCSRA & (1 << ADIF)));          //wait for conversion to complete
  ADCSRA |= (1 << ADIF);                    //clears interrupt flag
  /*ADCSRA |= (1 << ADPS2);                   //sets Prescaler
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS0);*/
}

//AVR pins and registers
void my_pin_mode(int Pin)                 //Port D and Port B only, doesn't access analog
{
  volatile uint8_t *port;
  if (Pin <= 7)
  {
    port = &DDRD;
  }
  //portd
  else if (Pin <= 13)

  {
    port = &DDRB;
  }
  if (Pin <= 7)
  {
    *port |= (1 << Pin);
  }
  else if (Pin <= 13)
  {
    *port |= (1 << (Pin - 8));
  }   //Portb
}

void my_digital_write(int Pin, int ON)
{
  volatile uint8_t *port;
  if (Pin <= 7)
  {
    port = &PORTD;
  }
  else if (Pin <= 13)

  {
    port = &PORTB;
  }
  if (Pin <= 7)
  {
    if (ON == 1)
    {
      *port |= (1 << Pin);
    }
    else
    {
      *port &= ~(1 << Pin);
    }
  }
  else if (Pin <= 13)
  {
    if (ON == 1)
    {
      *port |= (1 << (Pin - 8));
    }
    else
    {
      *port &= ~(1 << (Pin - 8));
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Serial Functions tab
//serial_functions tab
/*void CMD_ACT_ROT_(double direction2Rotate, double angleRotation)
{
  for(int angle = 0; angle < angleRotation; angle++)
  {
  Serial.print("CMD_ACT_ROT_"); //0==rotate CCW,
  Serial.print(direction2Rotate);
  Serial.print("_");
  Serial.print(1);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
  }
}*/


int CMD_SEN_GOAL()
{
  PrintMessage("CMD_SEN_GOAL");
  String incomingByte = Serial.readStringUntil(terminator);
  double obtainedGoal = incomingByte.toFloat();
  Serial.print("obtainedGoal= ");
  Serial.println(obtainedGoal);
  if (obtainedGoal == 1.0)
  {
    return WATER;
  }
  else if (obtainedGoal == 2.0)
  {
    return FIRE;
  }
  else
  {
    return 0;
  }
}
void PrintMessage(String message)
{
  Serial.print(message);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
}

void endSerial()
{
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
}


void CMD_SEN_ROT_(double angleRotation)
{
  Serial.print("CMD_SEN_ROT_");
  Serial.print(angleRotation);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
}

void CMD_ACT_ROT_(double direction2Rotate, double angleRotation)
{
  //for(i
  //Serial.print("CMD_ACT_LAT_0_0.01");  //dont need this since we just need to turn motor then move it forward is all
  Serial.print("CMD_ACT_ROT_"); //0==rotate CCW,
  Serial.print(direction2Rotate);
  Serial.print("_");
  Serial.print(angleRotation);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
}

void CMD_ACT_LAT_(double directionToRotate, double displacement)
{
  Serial.print("CMD_ACT_LAT_"); //0==rotate CCW,
  Serial.print(directionToRotate);
  Serial.print("_");
  Serial.print(displacement);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
}
