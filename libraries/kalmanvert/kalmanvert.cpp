#include <kalmanvert.h>

#include <Arduino.h>

void kalmanvert::init(double startp, double starta, double sigmap, double sigmaa, unsigned long timestamp) {
  /* init base values */
  p = startp;
  v = 0;
  a = starta;
  t = timestamp;
  calibrationDrift = 0.0;

  /* init variance */
  varp = sigmap * sigmap;
  vara = sigmaa * sigmaa;

  /* init covariance matrix */
  p11 = 0;
  p12 = 0;
  p21 = 0;
  p22 = 0;

  current_hold_cycle = 0;
  current_velocity = VELOCITY_MIN;
}

void kalmanvert::update(double mp, double ma, unsigned long timestamp) {

  /**************/
  /* delta time */
  /**************/
  unsigned long deltaTime = timestamp - t;
  double dt = ((double)deltaTime)/1000.0;
  t = timestamp;

  /**************/
  /* prediction */
  /**************/

  /* values */
  a = ma;  // we use the last acceleration value for prediction
  double dtPower = dt * dt; //dt^2
  p += dt*v + dtPower*a/2;
  v += dt*a;
  //a = ma; // uncomment to use the previous acceleration value

  /* covariance */
  double inc;

  dtPower *= dt;  // now dt^3
  inc = dt*p22+dtPower*vara/2;
  dtPower *= dt; // now dt^4
  p11 += dt*(p12 + p21 + inc) - (dtPower*vara/4);
  p21 += inc;
  p12 += inc;
  p22 += dt*dt*vara;

  /********************/
  /* gaussian product */
  /********************/

  /* kalman gain */
  double s, k11, k12, y;

  s = p11 + varp;
  k11 = p11/s;
  k12 = p12/s;
  y = mp - p;

  /* update */
  p += k11 * y;
  v += k12 * y;
  p22 -= k12 * p21;
  p12 -= k12 * p11;
  p21 -= k11 * p21;
  p11 -= k11 * p11;

}

double kalmanvert::getPosition() {

  return p;
}

double kalmanvert::getCalibratedPosition() {

  return (p + calibrationDrift);
}

double kalmanvert::getVelocity() {
  // increment and hold for another round
  if(current_hold_cycle >= VELOCITY_HOLD_CYCLES) {
    current_velocity += VELOCITY_STEP;
    current_hold_cycle = 1;
  }

  // Start over once maximum reached
  if(current_velocity >= VELOCITY_MAX) {
    current_velocity = VELOCITY_MIN;
  }

  current_hold_cycle += 1;

  return current_velocity;
}

double kalmanvert::getAcceleration() {

  return a;
}

unsigned long kalmanvert::getTimestamp() {

  return t;
}

void kalmanvert::calibratePosition(double newPosition) {

  calibrationDrift = newPosition - p;
}
