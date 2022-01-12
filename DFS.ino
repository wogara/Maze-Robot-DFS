#include <Wire.h> //including libraries
#include <Adafruit_MotorShield.h> //this is what controls our motors
#include "Adafruit_VCNL4010.h" //this is our sensor
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "utility/twi.h"


extern "C" {
    Room_t *get_neighbor(Room_t *room, Direction dir, Room_t maze[][16]);
    Direction oppositeDir(Direction dir);
    int isWallDir(Direction dir);
    void move1Forward();
    void move1Backward();
    int turnDir(Direction dir);
    void moveDir(Direction dir);
    void chooseDirection(Direction dir[4], int row, int col);
    Direction prevDir(int row, int col, int prev_row, int prev_col);
    int dfs(int row, int col, Direction back_dir, int maze[][16]);
}

#define TCAADDR 0x70 


Adafruit_VCNL4010 front = Adafruit_VCNL4010(); //defining the sensors
Adafruit_VCNL4010 left = Adafruit_VCNL4010();
Adafruit_VCNL4010 right = Adafruit_VCNL4010();
Adafruit_VCNL4010 back = Adafruit_VCNL4010();

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //motorshield controls the two motors
Adafruit_StepperMotor *myLeftMotor = AFMS.getStepper(200,1);// two motors instantiated
Adafruit_StepperMotor *myRightMotor = AFMS.getStepper(200,2);

typedef enum {
  NORTH = 0,
  SOUTH = 1,
  WEST = 2,
  EAST = 3
} Direction;


typedef struct Room_t {
  int row;
  int col;
  int visited;
  struct Room_t *next;
  struct Room_t *prev;
} Room_t;



void tcaseselect(uint8_t i) { //this method is called to get another sensor.
  if (i>7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void turnRight() {

  int turnRandom = random(21);
  int turnNumber = 0;

  if (turnRandom <= 19){
  turnNumber = 92;
  }else{
    turnNumber = 93;
  }
  Serial.println(turnNumber);
  turnNumber = 92;
  //92 is too much tardo



  for(int s = 0; s<turnNumber;s++){
    myRightMotor->step(1,BACKWARD,DOUBLE);
    myLeftMotor->step(1,FORWARD,DOUBLE);
  }

}

void turnLeft() {

  int turnRandom = random(21);
  int turnNumber = 0;

  if (turnRandom <= 15){
    turnNumber = 85;
  }else{
    turnNumber = 86;
  }

  for(int s = 0; s<turnNumber;s++){
    myRightMotor->step(1,FORWARD,DOUBLE);
    myLeftMotor->step(1,BACKWARD,DOUBLE);
  }

}

void moveForward() {

  myRightMotor->step(1,FORWARD,DOUBLE);
  myLeftMotor->step(1,FORWARD,DOUBLE);

}

void moveBackward() {

  myRightMotor->step(1,BACKWARD,DOUBLE);
  myLeftMotor->step(1,BACKWARD,DOUBLE);

}

boolean isWallRight(){ //each of these isWallWherever methods return true if there is a wall and false if there isn't
  tcaseselect(4);
  if (right.readProximity() < 5000){
    return true;
  }else{
    return false;
  }
}

boolean isWallLeft(){
  tcaseselect(5);
  if (left.readProximity() < 5000){
    return true;
  }else{
    return false;
  }
}

boolean isWallFront(){
  tcaseselect(7);
  if (front.readProximity() < 5000){
    return true;
  } else{
    return false;
  }
}

boolean isWallBack(){
  tcaseselect(3);
  if (back.readProximity() < 5000){
    return true;
  }else{
    return false;
  }
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  AFMS.begin();

  myLeftMotor->setSpeed(10);
  myRightMotor->setSpeed(10);
  delay(1000);



  Serial.begin(115200);
  Serial.println("\nTCAScanner ready!");

  for (uint8_t t=0; t<8; t++){
    tcaseselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++){
      if (addr == TCAADDR) continue;

      uint8_t data;
      if (! twi_writeTo(addr,&data,0,1,1)){
        Serial.print("Found 12C 0x"); Serial.println(addr,HEX);

      }
    }
  }
  Serial.println("done");
  tcaseselect(7);
  if (!front.begin()){
    Serial.println("front not found");
    while(1);
  }
  Serial.println("found front");

  tcaseselect(4);
  if (!right.begin()){
    Serial.println("right not found");
    while(1);
  }
  Serial.println("found right");
  tcaseselect(5);
  if (!left.begin()){
    Serial.println("left not found");
    while(1);
  }
  Serial.println("left found");
  tcaseselect(3);
  if (!back.begin()){
    Serial.println("back not found");
    while(1);
  }
  Serial.println("back found");

}

Direction robot_dir = NORTH;

Room_t *get_neighbor(Room_t *room, Direction dir, Room_t maze[][16]) {
    int new_row = room->row;
    int new_col = room->col;
    if (dir == NORTH) {
      new_row -= 1;
    } else if (dir == SOUTH) {
      new_row += 1;
    } else if (dir == EAST) {
      new_col += 1;
    } else {
      new_col -= 1;
    }
   return &maze[new_row][new_col];
}

Direction oppositeDir(Direction dir) {
   if (dir == NORTH) {
    return SOUTH;
  } else if (dir == SOUTH) {
    return NORTH;
  } else if (dir == EAST) {
    return WEST;
  } else if (dir == WEST) {
    return EAST;
  }
}

int isWallDir(Direction dir) {
    if (dir == NORTH) {
      return (isWallBack() && robot_dir == SOUTH) ||
             (isWallFront() && robot_dir == NORTH) ||
             (isWallRight() && robot_dir == WEST) ||
             (isWallLeft() && robot_dir == EAST);
    } else if (dir == SOUTH) {
      return (isWallBack() && robot_dir == NORTH) ||
             (isWallFront() && robot_dir == SOUTH) ||
             (isWallRight() && robot_dir == EAST) ||
             (isWallLeft() && robot_dir == WEST);
    } else if ( dir == EAST) {
        return (isWallBack() && robot_dir == WEST) ||
             (isWallFront() && robot_dir == EAST) ||
             (isWallRight() && robot_dir == NORTH) ||
             (isWallLeft() && robot_dir == SOUTH)
    } else if (dir == WEST) {
         return (isWallBack() && robot_dir == EAST) ||
             (isWallFront() && robot_dir == WEST) ||
             (isWallRight() && robot_dir == SOUTH) ||
             (isWallLeft() && robot_dir == NORTH)
    }
}

/*
 * c = circumference of the wheel
 * l = side of the block = 16.8
 */
void move1Forward() {
  double c = 3; // to be specified
  int num_go_forward = (int) 200*16.8/c;
  for (int i = 1; i <= num_go_forward; i++) {
     moveForward();
  }
}


void move1Backward() {
  double c = 3; // to be specified
  int num_go_forward = (int) 200*16.8/c;
  for (int i = 1; i <= num_go_forward; i++) {
     moveBackward();
  }
}

int turnDir(Direction dir) {
    if (dir == robot_dir) return 0;
    if (dir == NORTH) {
      if (robot_dir == SOUTH) return 1;
      else if (robot_dir == EAST) {turnLeft(); return 0;}
      else if (robot_dir == WEST) {turnRight(); return 0;}
    } else if (dir == SOUTH) {
        if (robot_dir == NORTH) return 1;
        else if (robot_dir == WEST) {turnLeft(); return 0;}
        else if (robot_dir == EAST) {turnRight(); return 0;}
    } else if ( dir == EAST) {
        if (robot_dir == WEST) return 1;
        else if (robot_dir == SOUTH) {turnLeft(); return 0;}
        else if (robot_dir == NORTH) {turnRight(); return 0;}
    } else if (dir == WEST) {
        if (robot_dir == EAST) return 1;
        else if (robot_dir == NORTH) {turnLeft(); return 0;}
        else if (robot_dir == SOUTH) {turnRight(); return 0;}
}

void moveDir(Direction dir) {
  if (turnDir(dir[i])) {
     move1Backward();
  } else {
     robot_dir = dir;
     move1Forward();
  }
}

void chooseDirection(Direction dir[4], int row, int col) {
  if (row >= col) {
    if (row <= (15 - col)) {
      if (row <= 8) {
        dir = {EAST, NORTH, SOUTH, WEST};
      } else {
        dir = {EAST, SOUTH, NORTH, WEST};
      }
    } else {
      if (col <= 8) {
        dir = {SOUTH, EAST, WEST, NORTH};
      } else {
        dir = {SOUTH, WEST, EAST, NORTH};
      }
    }
  } else {
    if (row <= (15 - col)) {
      if (col <= 8) {
        dir = {NORTH, EAST, WEST, SOUTH};
      } else {
        dir = {NORTH, WEST, EAST, SOUTH};
      }
    } else {
      if (row <= 8) {
        dir = {WEST, NORTH, SOUTH, EAST};
      } else {
        dir = {WEST, SOUTH, NORTH, EAST};
      }
    }
  }
}

Direction prevDir(int row, int col, int prev_row, int prev_col) {
  if (prev_row == row - 1) {
     return SOUTH;
  } else if (prev_row == row + 1) {
     return NORTH;
  } else if (prev_col == col - 1) {
     return WEST;
  } else if (prev_col == col + 1) {
     return EAST;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Room_t maze[16][16];
  for (int i = 1; i <= 16; i++) {
    for (int j = 1; j <= 16; j++) {
      room.next = NULL;
      room.prev = NULL;
      room.visited = 0;
      room.row = i;
      room.col = j;
    }
  }

  maze[0][0] = 1;
  Direction init_back_dir = SOUTH;
  dfs(0, 0, init_back_dir, maze);
  Room_t *cur_room = &maze[0][0];
  Room_t *next_room = cur_room->next;
  while (next_room != NULL) {
    moveDir(prevDir(cur_room->row, cur_room->col, next_room->row, next_room->col));
    cur_room = next_room;
    next_room = next_room->next;
  }
}

int dfs(int row, int col, Direction back_dir, int maze[][16]) {
  if ((row == 7) || (row == 8)) && ((col == 7) || (col == 8)) {
      return 1;
  } else {
    Room_t *room = &maze[row][col];
    room->visited = 1;
    Direction dir[4];
    chooseDirection(dir, row, col);
    for (int i = 0; i < 4; i++) {
      if (!isWallDir(dir[i])) {
        Room_t *neighbor = get_neighbor(16, room, dir[i], maze);
        neighbor->prev = room;
        if (!neighbor->visited) {
          moveDir(dir[i]);
          if (dfs(neighbor->row, neighbor->col, goal_row,
              goal_col, num_rows, num_cols, maze, file)) {
            room->next = neighbor;
            moveDir(oppositeDir(dir[i]));
            return 1;
          }
        }
    }
    prev = room->prev;
    moveDir(prevDir(row, col, prev->row, prev->col));
    return 0;
    }
  }

