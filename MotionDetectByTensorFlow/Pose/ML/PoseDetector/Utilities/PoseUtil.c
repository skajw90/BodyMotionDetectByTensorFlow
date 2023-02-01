//
//  PoseUtil.c
//  PoseDetector
//
//  Created by Jiwon on 2022/06/08.
//

#include "PoseUtil.h"

/// Return the distance between two points
/// @param a first point's x and y data, structed in a CJointCoordinate
/// @param b second point's x and y data, structed in a CJointCoordinate
float c_points_to_distance(struct CJointCoordinate a, struct CJointCoordinate b){
    float run = (b.x - a.x) * (b.x - a.x);
    float rise = (b.y - a.y) * (b.y - a.y);
    float res = sqrtf(run + rise);
    return res;
}

/// Return the angle abc (meaning the angle created by ab and bc segment is returned)
/// @param a first point's x and y data, structed in a CJointCoordinate
/// @param b second point's x and y data, structed in a CJointCoordinate
/// @param c third point's x and y data, structed in a CJointCoordinate
float c_points_to_angle(struct CJointCoordinate a, struct CJointCoordinate b, struct CJointCoordinate c){
    float dist_a = c_points_to_distance(a, b);
    float dist_b = c_points_to_distance(b, c);
    float dist_c = c_points_to_distance(c, a);
    float top = (dist_a * dist_a) + (dist_b * dist_b) - (dist_c * dist_c);
    float bottom = 2 * dist_a * dist_b;
    float res = acosf(top/bottom);
    return res;
}

/// Method to reset the global variable, max and min
/// @param max pointer to the global variable, max
/// @param min pointer to the global variable, min
void c_free_public(float *max, float *min){
    for(int i = 0 ; i < 34; i++){
        max[i] = -1;
        min[i] = -1;
    }
}

/// method to update the global variable, max and min, according to the frame data that is passed into the parameter
/// @param frame New joint datas that will be compared with the global variable and updated accordingly
/// @param public_index indicates the index of the joint in the global variable
/// @param joint_index indicates the index of the join in the parameter 'frame' data
/// @param max pointer to the global variable, max
/// @param min pointer to the global variable, min
/// @param error_ratio parameter that indicates the minimum score that the joint should have to be qualified for comparing
void c_public_update(struct CJointNode *frame, int public_index, int joint_index, float *max, float *min, float error_ratio) {
    if (frame[joint_index].joint_score < error_ratio) { return; }
    
    if(max[public_index + 1] >= frame[joint_index].y_coord || max[public_index + 1] == -1) {
        max[public_index] = frame[joint_index].x_coord;
        max[public_index + 1] = frame[joint_index].y_coord;
    }
    
    if(min[public_index + 1] <= frame[joint_index].y_coord || min[public_index + 1] == -1) {
        min[public_index] = frame[joint_index].x_coord;
        min[public_index + 1] = frame[joint_index].y_coord;
    }
}

/// method that indicates whether the position of the person is facing sideways or not. L and R parameter indicates the decision coordinate that varies from each
/// position. For example, squat uses ankle coordinates and push up uses wrist coordinates.
/// @param l left point of the indicator in the joint data.
/// @param r  left point of the indicator in the joint data.
int c_is_body_side(struct CJointCoordinate l, struct CJointCoordinate r) {
    float avg_degree = atan2f((l.y - r.y),(l.x - r.x));
    float avg_degree_ratio = avg_degree / (PI/2);
    
    if (fabs(avg_degree_ratio) >= DEGREE_DIAGONAL_ERROR_MAX) {
        return 1;
    }
    else {
        return 0;
    }
}

/// Return the tilt degree of two points. range between 0 ~ 180
/// @param a first point's x and y data, structed in a CJointCoordinate
/// @param b second point's x and y data, structed in a CJointCoordinate
float c_point_to_degree(struct CJointCoordinate a, struct CJointCoordinate b){
    return atanf((a.y - b.y) / (a.x - b.x)) * (180/PI);
}

/// Taking 4 points, a b c and d, the degree difference is calculated between a to b degree and c to d degree.
/// The result is returned into a decimal between 0 and 1.
/// @param a  first point's x and y data, structed in a CJointCoordinate
/// @param b second point's x and y data, structed in a CJointCoordinate
/// @param c  third point's x and y data, structed in a CJointCoordinate
/// @param d  fourth point's x and y data, structed in a CJointCoordinate
float c_degree_diff(struct CJointCoordinate a, struct CJointCoordinate b, struct CJointCoordinate c, struct CJointCoordinate d){
    return 1 - ((MAX(float, c_point_to_degree(a, b), c_point_to_degree(c, d)) - MIN(float, c_point_to_degree(a, b), c_point_to_degree(c, d))) / 180);
}

float c_min_elbow_angle(float *max, float *min){
    struct CJointCoordinate point_left_shoulder = {.x = min[SHOULDER_LEFT_X], .y = min[SHOULDER_LEFT_Y]};
    struct CJointCoordinate point_left_elbow = {.x = min[ELBOW_LEFT_X], .y = min[ELBOW_LEFT_Y]};
    struct CJointCoordinate point_left_wrist = {.x = min[WRIST_LEFT_X], .y = min[WRIST_LEFT_Y]};
    
    struct CJointCoordinate point_right_shoulder = {.x = min[SHOULDER_LEFT_X], .y = min[SHOULDER_LEFT_Y]};
    struct CJointCoordinate point_right_elbow = {.x = min[ELBOW_LEFT_X], .y = min[ELBOW_LEFT_Y]};
    struct CJointCoordinate point_right_wrist = {.x = min[WRIST_LEFT_X], .y = min[WRIST_LEFT_Y]};
    
    float l_angle = c_points_to_angle(point_left_shoulder, point_left_elbow, point_left_wrist);
    float r_angle = c_points_to_angle(point_right_shoulder, point_right_elbow, point_right_wrist);
    
    return MIN(float, l_angle, r_angle);
}

