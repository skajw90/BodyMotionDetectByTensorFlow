//
//  SquatHold.h
//  PoseDetector
//
//  Created by Jiwon on 2022/06/14.
//

#ifndef SquatHold_h
#define SquatHold_h

#include "PoseHoldUtil.h"

float* c_count_squat_hold(struct CJointNode **, float *, float *, struct CJointAccuracy, struct CJointAccuracy, struct CJointNode * scanned_body);
#endif /* SquatHold_h */
