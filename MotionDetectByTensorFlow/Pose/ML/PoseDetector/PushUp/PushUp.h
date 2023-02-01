//
//  PushUp.h
//  PoseDetector
//
//  Created by Jiwon on 2022/05/24.
//

#ifndef PushUp_h
#define PushUp_h
#include "PoseUtil.h"

float c_pushup_height_ratio(float *max, float *min, float transposed_lx, float transposed_rx);

float c_bicep_ratio(float *max, float *min);

float* c_count_pushup(struct CJointNode **, float *, float *, struct CJointAccuracy, struct CJointAccuracy, struct CJointNode * scanned_body);

#endif /* PushUp_h */
