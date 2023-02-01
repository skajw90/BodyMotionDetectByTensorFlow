//
//  Squat.h
//  PoseDetector
//
//  Created by Jiwon on 2022/05/24.
//

#ifndef Squat_h
#define Squat_h

#include "PoseUtil.h"

/// main method for calculating squat's count
/// @param buffer buffer of size 10, each containing a frame's joint data
/// @param max pointer where all the maximum points for each joint are saved
/// @param min pointer where all the minimum points for each joint are saved
/// @param accuracy struct conatining the minimum requirements for count condition
/// @param optimalaccuracy struct containing the optimal requirements for user accuracy percentage calculation
float* c_count_squat(struct CJointNode ** buffer, float * max, float * min, struct CJointAccuracy accuracy, struct CJointAccuracy optimalaccuracy, struct CJointNode * scanned_body);

/// To calculate whether the user met the count condition based on the given condition where the squat is doen sideways,
/// @param max pointer where all the maximum points for each joint are saved
/// @param min pointer where all the minimum points for each joint are saved
/// @param left_knee_angle angle of left knee. Calculated using left hip, left knee and left ankle joint data
/// @param right_knee_angle angle of right knee. Calculated using right hip, right knee and right ankle joint data
/// @param min_knee_angle minimum required knee angle for user to reach to be counted.
/// @param optimal_knee_angle optimal angle of knee for user to reach.
float* c_count_side_squat(float *max, float *min, float left_knee_angle, float right_knee_angle, float min_knee_angle, float optimal_knee_angle);

/// To calculate whether the user met the count condition based on the given condition where the squat is done facing front or diagonal
/// @param max pointer where all the maximum points for each joint are saved
/// @param min pointer where all the minimum points for each joint are saved
/// @param left_hip_height_ratio height of the left hip ratio. Calculated using left hip height and minimum ankle height
/// @param right_hip_height_ratio height of the right hip ratio. Calculated using right hip height and minimum ankle height
/// @param min_hip_ratio minimum hip height ratio required  for user to reach
/// @param optimal_hip_ratio optimal hip height ratio for user to reach
float* c_count_front_squat(float *max, float *min, float left_hip_height_ratio, float right_hip_height_ratio, float min_hip_ratio, float optimal_hip_ratio);

#endif

