/*
 * RollAvg.h
 *
 *  Created on: 2014. 09. 10.
 *      Author: hrt
 *      Version : 0.1
 */	

#include "RollAvgSmall.h"
#include <Arduino.h>


//RollAvgSmall::RollAvgSmall(int buf_len, float init_val)
//{
//  RollAvg(buf_len);
//  initvals(init_val);
//  this->WritePtr = 0;
//}

RollAvgSmall::RollAvgSmall(int buf_len)
{
  if (buf_len < RollAvgMaxPos)
  {
    this->BufLen = buf_len;
  }
  else
  {
    this->BufLen = RollAvgMaxPos;
  }
}

void RollAvgSmall::initvals(float& init_val)
{
   AvgValue = init_val;
}

float RollAvgSmall::update(float& new_val)
{
  AvgValue = ((AvgValue * (this->BufLen - 1) + new_val)/this->BufLen);
  return AvgValue;
}

float RollAvgSmall::average()
{
  return AvgValue ;
}

float RollAvgSmall::minval()
{
  return AvgValue ;
}

float RollAvgSmall::maxval()
{
  return AvgValue ;
}

float RollAvgSmall::posval(int pos)
{
  return AvgValue;
}

RollAvgSmall::~RollAvgSmall() {
	// TODO Auto-generated destructor stub
}

