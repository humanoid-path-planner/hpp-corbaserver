/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include "hppCorbaServer/hppciTools.h"

void ConfigurationToCkitMat4(const Configuration inConfig, CkitMat4& outMatrix4)
{
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++)
      outMatrix4(i,j) = inConfig.rot[i*3+j];
  }
  for(int i=0; i<3; i++)
    outMatrix4(i,3) = inConfig.trs[i];
  for(int i=0; i<3; i++)
    outMatrix4(3,i) = 0.0;
  outMatrix4(3,3) = 1.0;
}
