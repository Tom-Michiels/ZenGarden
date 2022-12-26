#include "Arduino.h"
#include <cmath>
#include <exception>

#define MAX_D 2776
//#define MAX_D 277
#define STEPS_IN_ROTATION 22417 
#define TICKS 2

class Stepper {
  public:
    Stepper (int m1, int m2, int m3, int m4); 
    void step(int dir);
  private:
    int mPos;
    int m1;
    int m2;
    int m3;
    int m4;
};

Stepper::Stepper(int p1, int p2, int p3, int p4) : mPos(0), m1(p1), m2(p2), m3(p3), m4(p4) {};

void Stepper::step(int dir) {
  if (dir == 0 ) {
    return;
  }
  if (dir > 0) {
    mPos = (mPos + 1) % 4;
  } else {
    mPos = (mPos + 3) % 4;
  }

  switch (mPos) {
    case 0:  // 1010
      digitalWrite(m1, HIGH);
      digitalWrite(m2, LOW);
      digitalWrite(m3, HIGH);
      digitalWrite(m4, LOW);
    break;
    case 1:  // 0110
      digitalWrite(m1, LOW);
      digitalWrite(m2, HIGH);
      digitalWrite(m3, HIGH);
      digitalWrite(m4, LOW);
    break;
    case 2:  //0101
      digitalWrite(m1, LOW);
      digitalWrite(m2, HIGH);
      digitalWrite(m3, LOW);
      digitalWrite(m4, HIGH);
    break;
    case 3:  //1001
      digitalWrite(m1, HIGH);
      digitalWrite(m2, LOW);
      digitalWrite(m3, LOW);
      digitalWrite(m4, HIGH);
    break;
  } 

}

// in3, in1, in2, in4
Stepper myStepperRot(17, 5, 19, 18);
Stepper myStepperRad(32, 25, 33, 26);

int ticks = 2; // can be 2

void setup() {
  // put your setup code here, to run once:
  pinMode (17,OUTPUT);
  pinMode (5,OUTPUT);
  pinMode (19,OUTPUT);
  pinMode (18,OUTPUT);
  pinMode (32,OUTPUT);
  pinMode (25,OUTPUT);
  pinMode (33,OUTPUT);
  pinMode (26,OUTPUT);
  pinMode (27,PULLUP);

  Serial.begin(9600);
}

// Rot 1 ==> buiten en clock mee
// Rad 1 ==> naar buiten

float distance(float x1, float y1, float x2, float y2) {
  return std::sqrt(std::pow(x2-x1,2)+std::pow(y2-y1,2));
}

float distance_to_line(float x0, float y0, float x1, float y1, float x2, float y2) {
  return std::abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / std::sqrt(std::pow(x2-x1,2) + std::pow(y2-y1,2));
}  

class TableRouter {
  public:
    TableRouter(Stepper& r, Stepper& d);
    void get_xy(float& x, float& y, int dr=0, int dd=0);
    void goto_xy(float x, float y); 
    void home();

  private:
    Stepper &r_stepper; 
    Stepper &d_stepper;
    int r;
    int d; 

    void step(int rs, int ds);
    bool is_in_range(int ds);
    bool step_on_line(float x1, float y1, float x2, float y2);
};

TableRouter::TableRouter(Stepper& r, Stepper& d):
  r_stepper(r), d_stepper(d), r(0), d(0) {
}

void TableRouter::home() {
  r = 0;
  d = 0;
  goto_xy(0.0,1.0);
}

void TableRouter::get_xy(float& x, float& y, int dr, int dd) {
  float s = 1.0 * (d+dd) / MAX_D;
  y = std::cos(2.0 * M_PI * (r+dr) / STEPS_IN_ROTATION) * s;
  x = std::sin(2.0 * M_PI * (r+dr) / STEPS_IN_ROTATION) * s;
}

void TableRouter::goto_xy(float x2, float y2) {
  float x1, y1;
  bool done = false;
  get_xy(x1, y1);
  while (!done) {
    done = step_on_line(x1,y1,x2,y2);
  } 
} 

void TableRouter::step(int rs, int ds) {
  int ds2 = 0;
  r += rs;
  d += ds;
  // every 11th step in either rotational direction, correct the d-axis 
  if ((r % 11) == 0) {
      ds2 = -rs;
  }
  // do we need two ticks to make this combined step? 
  if (std::abs(ds2 + ds)>1) {    
    r_stepper.step(rs);
    d_stepper.step(ds);
    delay(TICKS);
    d_stepper.step(ds2);
    delay(TICKS);
  } else {
    r_stepper.step(rs);
    d_stepper.step(ds + ds2);
    delay(TICKS);
  }

  if (digitalRead(27) == 0 ) {
    throw 1; 
  }
}

bool TableRouter::is_in_range(int ds) {
  int new_d = d + ds; 
  bool result = ( new_d > 0 ) and (new_d < MAX_D);
  return result;
}

bool TableRouter::step_on_line(float x1, float y1, float x2, float y2) {
  bool done = true;
  const int directions[][2] = {{0,1},{1,1},{1,0},{1,-1},
                         {0,-1},{-1,-1},{-1,0},{-1,1}};  
  int best_d = 0;  
  int best_r = 0;  
  float distance_to_destination;
  float best_distance_to_line;
  float new_distance_to_line;
  float x0, y0;

  get_xy(x0,y0); 
  distance_to_destination = distance(x0,y0,x2,y2);
  best_distance_to_line = 2;
  for (int i=0;i<8;i++) {
    int rs = directions[i][0];
    int ds = directions[i][1];
    if (!is_in_range(ds)) {
      continue;
    }
    get_xy(x0,y0,rs, ds);
    // is this a step in the right direction?
    if (distance(x0,y0,x2,y2) < distance_to_destination) {
      new_distance_to_line = distance_to_line(x0,y0,x1,y1,x2,y2);
      // is this not only in the right direction, but is it closer to the line
      // than any other steps we have considered? 
      if (new_distance_to_line < best_distance_to_line) {
        best_d = ds;
        best_r = rs;
        best_distance_to_line = new_distance_to_line;
        // there is hope that more steps can be set in the right direction
        done = false;
      }
    }
  }

  if (!done) { 
    step(best_r, best_d);
  }
  return done;
};

TableRouter router(myStepperRot, myStepperRad); 

void pentagram() {
  for (int i=0;i<1000;i++) {
      router.goto_xy(sin(i*M_PI*4.05/5),cos(i*M_PI*4.06/5));
  }
}

void spiral() {
  for (int i=0;i<300*15;i++) {
      router.goto_xy(sin(i*M_PI*2/300.0)*(1-i/(300.0*15.0)),cos(i*M_PI*2/300)*(1-i/(300.0*15.0)) );
  }
}

void flower() {
  for (int i=0;i<200*60;i++) {
    float x = 0.35 * sin(i*M_PI*2 / 200.0) + 0.65 * sin(i*M_PI*2 / (200.0 * 30.0)); 
    float y = 0.35 * cos(i*M_PI*2 / 200.0) + 0.65 * cos(i*M_PI*2 / (200.0 * 30.0)); 
    router.goto_xy(x,y);
  }
} 

void flower2() {
  for (int i=0;i<200*60;i++) {
    float x = 0.65 * sin(i*M_PI*2 / 200.0) + 0.35 * sin(i*M_PI*2 / (200.0 * 30.0)); 
    float y = 0.65 * cos(i*M_PI*2 / 200.0) + 0.35 * cos(i*M_PI*2 / (200.0 * 30.0)); 
    router.goto_xy(x,y);
  }
} 

void wait_for_key_release() {
  while (true) {
    delay(100);
    if (digitalRead(27)==1) return;
  }
}

void loop() {
  try { router.home(); pentagram();   } catch (...) { wait_for_key_release(); } 
  try { router.home(); flower();      } catch (...) { wait_for_key_release(); } 
  try { router.home(); spiral();      } catch (...) { wait_for_key_release(); } 
  try { router.home(); flower2();     } catch (...) { wait_for_key_release(); } 
}