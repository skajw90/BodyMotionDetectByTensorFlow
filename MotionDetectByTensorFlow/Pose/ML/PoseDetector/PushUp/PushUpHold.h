//
//  PushUpHold.h
//  PoseDetector
//
//  Created by Jiwon on 2022/06/07.
//

#ifndef PushUpHold_h
#define PushUpHold_h

#include "PoseHoldUtil.h"

float* c_count_pushup_hold(struct CJointNode **, float *, float *, struct CJointAccuracy, struct CJointAccuracy, struct CJointNode * scanned_body);

#endif /* PushUpHold_h */
