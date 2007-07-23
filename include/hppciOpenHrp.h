/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux and Mathieu Poirier (LAAS-CNRS)

*/

#ifndef HPPCI_OPENHRP_H
#define HPPCI_OPENHRP_H

/*************************************
INCLUDE
**************************************/


#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include "hppPlanner.h"


KIT_PREDEF_CLASS(ChppPolyhedron);

/*************************************
STRUCTURES
**************************************/

/**
   \brief HALFSITTINGPOSITION_RAD_KINEO  \
  
   FreeFlyer : 0.0, 0.0, 0.648702, 0.0 , 0.0, 0.0
 
   legs : 0.0, 0.0, -0.453785, 0.872664, -0.418879, 0.0, 0.0, 0.0, -0.453785, 0.872664, -0.48879, 0.0
 
   chest and head : 0.0, 0.0, 0.0, 0.0, 
 
   right arm : 0.261799, -0.174532, 0.0, -0.523598, 0.0, 0.0, 0.174532, 

   right hand : -0.349065, 0.349065, -0.349065, 0.349065, -0.349065, 
 
   left arm : 0.261799,  0.174532, 0.0, -0.523598, 0.0, 0.0, 0.174532, 
 
   left hand : -0.174532, 0.174532, -0.174532, 0.174532, -0.174532  
*/
#define HALFSITTINGPOSITION_RAD_KINEO {					\
    0.0, 0.0, 0.648702, 0.0 , 0.0, 0.0,					\
      0.0, 0.0, -0.453785,0.872664,  -0.418879, 0.0, 0.0, 0.0, -0.453785,0.872664,  -0.418879, 0.0, \
      0.0, 0.0, 0.0, 0.0,						\
      0.261799, -0.174532, 0.0, -0.523598, 0.0, 0.0, 0.174532,		\
      -0.349065, 0.349065, -0.349065, 0.349065, -0.349065,		\
      0.261799,  0.174532, 0.0,  -0.523598, 0.0, 0.0, 0.174532,		\
      -0.174532, 0.174532, -0.174532, 0.174532, -0.174532		\
      };


// just a declaration : the body of this class is hppciOpenHp.cpp
class CinternalCorbaObject ;

// ==============================================================================
//
//  CLASS ChppciHrpClient
//
// ==============================================================================
/** 
    \brief OpenHRP client to load HRP2 model.
*/
class ChppciOpenHrpClient {
  
  friend class CinternalCorbaObjet ;

public:

  /** 
      \brief Constructor
  */
  ChppciOpenHrpClient(ChppPlanner *hpp);

  /**
     \brief Destroy joint absolute position matrices in associative array.
  */
  ~ChppciOpenHrpClient();

  /** 
      \brief Initialize Orb and load model of HRP2 by sending a Corba request modelLoader to OpenHRP. 
      \param argc, argv[] are passed to Orb initialization.

      This function gets the model of HRP2 from a CORBA request and initializes a ChppProblem with this robot.
      Note that the robot is set into half-sitting configuration.
  */
  ktStatus loadHrp2Model(int argc, char ** argv);

  /**
     \brief  Initialize Orb and load model of HRP2 by sending a Corba request modelLoader to OpenHRP.
     \param argc, argv[] are passed to Orb initialization.
     \retval o_device The model of HRP2 set in half-sitting position.

      Note that the robot is set into half-sitting configuration.
  */
  ktStatus loadHrp2Model(int argc, char **argv, ChppDeviceShPtr &o_device);

  /**
     \brief  Initialize Orb and load model of a robot by sending a Corba request modelLoader to OpenHRP.
     \param argc, argv[] are passed to Orb initialization.
     \retval o_device The model of robot.
  */
  ktStatus loadRobotModel(int argc, char **argv, ChppDeviceShPtr &o_device);

  /** 
      \brief Initialize Orb and load model of objects by sending a Corba request modelLoader to OpenHRP. 
      Param argc, argv[] are passed to Orb initialization.
      ** add internal map of planner to be added later
      */
  ktStatus loadObstacleModel(int _argc, char **_argv, std::vector<ChppPolyhedronShPtr> &polyList);
 
  /** 
      \brief Build a KineoWorks device from HRP2 model stored in Corba. 
      Once created, the device is store in hppPlanner.
      We add an extra dof in order to store time in the config.
      This enables us to build stationary linear direct paths
  */
  //ktStatus buildKppDevice(ChppDeviceShPtr& hppDevice, bool selfColFlag); 

  /**
     \brief return the pointer to planner
  */
  ChppPlanner *planner() {return hppPlanner; };

private :

  /**
     \brief Get Information from corba server (ModelLoader ...)
  */
  ktStatus getInfoFromCorba(int argc, char **argv);

  /** 
      \brief Pointer to Path Planner object allocated elsewhere.
  */
  ChppPlanner *hppPlanner;

  CinternalCorbaObject *privateCorbaObject ;

};


#endif
