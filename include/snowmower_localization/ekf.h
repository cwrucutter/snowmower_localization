/* ekf.h */
#include <ros/ros.h>
#include <vector>

#ifndef __EKF_H_INCLUDED__
#define __EKF_H_INCLUDED__

class Ekf {
 private:
  /*************
    Parameters
  *************/



  /*******************
    Member Variables
  *******************/

  // State vector


  /*******************
    Member Fucntions
  *******************/
  // System update
  void systemUpdate();

  // Map frame measurement updates
  void measurementUpdateGPS(double x, double y);
  
  void measurementUpdateDecaWave(double dist[]);
  
  void measurementUpdateVisualBeacons(double angles[]);
  
  // Odom frame measurement updates
  void measurementUpdateEncoders(double vR, double vL);
  
  void measurementUpdateIMU(double wZ, double aX, double aY);
  
  void measurementUpdateVisualOdometry();
  

 public:
  Ekf();
  ~Ekf();

};

#endif
