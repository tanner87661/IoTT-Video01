/*
 * RollAvg.h
 *
 *  Created on: 2014. 09. 10.
 *      Author: hrt
 *      Version : 0.1
 */	

#ifndef RollAvgSmall_H_
#define RollAvgSmall_H_
#include <Arduino.h>

#define RollAvgMaxPos 20  // Maximal number of values for Avg calculation

class RollAvgSmall {
protected:

    float AvgValue;
	int BufLen;

public:
//	RollAvgSmall(int buf_len, float init_val);
	RollAvgSmall(int buf_len);
	~RollAvgSmall();
	void initvals(float& init_val);
	float update(float& new_val);
	float average();
        float minval();
        float maxval();
	float posval(int pos);
};


#endif /* RollAvgSmall_H_ */
