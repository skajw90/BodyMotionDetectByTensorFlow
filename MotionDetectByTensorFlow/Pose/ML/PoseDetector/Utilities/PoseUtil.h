//
//  PoseUtil.h
//  PoseDetector
//
//  Created by Jiwon on 2022/06/08.
//

#ifndef PoseUtil_h
#define PoseUtil_h

#define BUFFER_SIZE 10
#define JOINT_SIZE 17

#define NOSE_INDEX 0
#define EYE_INDEX 1
#define EAR_INDEX 3
#define SHOULDER_INDEX 5
#define ELBOW_INDEX 7
#define WRIST_INDEX 9
#define HIP_INDEX 11
#define KNEE_INDEX 13
#define ANKLE_INDEX 15

#define NOSE_X 0
#define NOSE_Y 1
#define EYE_LEFT_X 2
#define EYE_LEFT_Y 3
#define EYE_RIGHT_X 4
#define EYE_RIGHT_Y 5
#define EAR_LEFT_X 6
#define EAR_LEFT_Y 7
#define EAR_RIGHT_X 8
#define EAR_RIGHT_Y 9
#define SHOULDER_LEFT_X 10
#define SHOULDER_LEFT_Y 11
#define SHOULDER_RIGHT_X 12
#define SHOULDER_RIGHT_Y 13
#define ELBOW_LEFT_X 14
#define ELBOW_LEFT_Y 15
#define ELBOW_RIGHT_X 16
#define ELBOW_RIGHT_Y 17
#define WRIST_LEFT_X 18
#define WRIST_LEFT_Y 19
#define WRIST_RIGHT_X 20
#define WRIST_RIGHT_Y 21
#define HIP_LEFT_X 22
#define HIP_LEFT_Y 23
#define HIP_RIGHT_X 24
#define HIP_RIGHT_Y 25
#define KNEE_LEFT_X 26
#define KNEE_LEFT_Y 27
#define KNEE_RIGHT_X 28
#define KNEE_RIGHT_Y 29
#define ANKLE_LEFT_X 30
#define ANKLE_LEFT_Y 31
#define ANKLE_RIGHT_X 32
#define ANKLE_RIGHT_Y 33



#define PI 3.142857

#define Y_ERROR_MAX 10

#define DEGREE_FRONT_ERROR_MAX 0.1
#define DEGREE_DIAGONAL_ERROR_MAX 0.4

#define HOLD_ANKLE_ERROR_MAX 40

#define P_HOLD_MINIMUM_KNEE_DEGREE 140


#define ENSURE_int(i)   _Generic((i), int:   (i))
#define ENSURE_float(f) _Generic((f), float: (f))

#define GENERIC_MAX(x, y) ((x) > (y) ? (x) : (y))

#define MAX(type, x, y) \
  (type)GENERIC_MAX(ENSURE_##type(x), ENSURE_##type(y))

#define GENERIC_MIN(x, y) ((x) < (y) ? (x) : (y))

#define MIN(type, x, y) \
  (type)GENERIC_MIN(ENSURE_##type(x), ENSURE_##type(y))

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

struct CJointNode {
    char *joint_name;
    float x_coord;
    float y_coord;
    float joint_score;
};

struct CJointAccuracy {
    float height_ratio;
    float upper_body;
    float mid_body;
    float lower_body;
    float error_ratio;
};

struct CJointCoordinate {
    float x;
    float y;
};

void c_update_seg(struct CJointNode **, float *, float *, struct CJointAccuracy);
int c_is_body_side(struct CJointCoordinate, struct CJointCoordinate);

float c_points_to_distance(struct CJointCoordinate, struct CJointCoordinate);
float c_points_to_angle(struct CJointCoordinate, struct CJointCoordinate, struct CJointCoordinate);

void c_free_public(float *, float *);
void c_public_update(struct CJointNode *, int public_index, int joint_index, float *, float *, float error_ratio);

float c_point_to_degree(struct CJointCoordinate a, struct CJointCoordinate b);
float c_degree_diff(struct CJointCoordinate a, struct CJointCoordinate b, struct CJointCoordinate c, struct CJointCoordinate d);

float c_min_elbow_angle(float *max, float *min);

#endif /* PoseUtil_h */
