//
//  Squat.c
//  PoseDetector
//
//  Created by Jiwon on 2022/05/24.
//

#include "Squat.h"

//void c_update_seg(struct CJointNode **buffer, float *max, float *min, struct CJointAccuracy accuracy) {
//    for(int i = 0; i < BUFFER_SIZE; i++){
//        c_global_update(buffer[i], 0, NOSE_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], 30, ANKLE_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], 32, ANKLE_INDEX+1, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], 22, HIP_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], 24, HIP_INDEX+1, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], 26, KNEE_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], 28, KNEE_INDEX+1, max, min, accuracy.error_ratio);
//    }
//}


float* c_count_side_squat(float *max, float *min, float left_knee_angle, float right_knee_angle, float min_knee_angle, float optimal_knee_angle) {
    float* accuracy_res = calloc(2, sizeof(float));
    
    float measured_knee_angle = MAX(float, left_knee_angle, right_knee_angle) * (180 / PI);
    
    float angle_accuracy = MIN(float, 1 - (measured_knee_angle - optimal_knee_angle) / 180, (float) 1);
//    printf("angle accuracy : %f, minimum_angle : %f, measured_angle: %f, optimal : %f\n", angle_accuracy, min_knee_angle,  measured_knee_angle, optimal_knee_angle);
    
    accuracy_res[1] = angle_accuracy * 100;
    
    // counting condition
    if (measured_knee_angle < min_knee_angle) {
        c_free_public(max, min);
        accuracy_res[0] = 1;
    }
    
    return accuracy_res;
}


float* c_count_front_squat(float *max, float *min, float left_hip_height_ratio, float right_hip_height_ratio, float min_hip_ratio, float optimal_hip_ratio) {
    float* accuracy_res = calloc(2, sizeof(float));
    
    float hip_height_ratio = (left_hip_height_ratio + right_hip_height_ratio) / 2.0;
//    printf("hip_height_ratio : %f, min_hip_ratio : %f\n", hip_height_ratio, min_hip_ratio);
    float accuracy = MIN(float, 1 - (hip_height_ratio - optimal_hip_ratio), (float) 1);
    accuracy_res[1] = accuracy * 100;
//    printf("hip_height_ratio : %f, min_hip_ratio : %f, accuracy : %f\n", hip_height_ratio, min_hip_ratio, accuracy_res[1]);
    
    if (hip_height_ratio < min_hip_ratio) {
//        printf("accuracy : %f\n", accuracy_res[1]);
        c_free_public(max, min);
        accuracy_res[0] = 1;
    }
    
    return accuracy_res;
}



float* c_count_squat(struct CJointNode **buffer, float *max, float *min, struct CJointAccuracy accuracy, struct CJointAccuracy optimalaccuracy, struct CJointNode * scanned_body) {
    // update all seg from buffer
//    c_update_seg(buffer, max, min, accuracy);
    for(int i = 0; i < BUFFER_SIZE; i++){
        c_public_update(buffer[i], 0, NOSE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], 30, ANKLE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], 32, ANKLE_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], 22, HIP_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], 24, HIP_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], 26, KNEE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], 28, KNEE_INDEX+1, max, min, accuracy.error_ratio);
    }
    
    int is_side = c_is_body_side(
                                 (struct CJointCoordinate) { buffer[9][ANKLE_INDEX].x_coord, buffer[9][ANKLE_INDEX].y_coord },
                                 (struct CJointCoordinate) { buffer[9][ANKLE_INDEX + 1].x_coord, buffer[9][ANKLE_INDEX + 1].y_coord });

    // To detect ankle movement, return empty
    if((fabsf(max[ANKLE_LEFT_Y] - min[ANKLE_LEFT_Y]) > optimalaccuracy.error_ratio || fabsf(max[ANKLE_RIGHT_Y] - min[ANKLE_RIGHT_Y]) > optimalaccuracy.error_ratio)) {
        c_free_public(max, min);
        return calloc(2, sizeof(float));
    }
    
    // ---------------------ankle calculation-----------------------
    float left_ankle_mid = (max[ANKLE_LEFT_Y] + min[ANKLE_LEFT_Y]) / 2.0;
    float right_ankle_mid = (max[ANKLE_RIGHT_Y] + min[ANKLE_RIGHT_Y]) / 2.0;
    float both_ankle_mid = (left_ankle_mid + right_ankle_mid) / 2.0;
    
    float minimum_ratio = (both_ankle_mid - min[NOSE_Y]) / (both_ankle_mid - max[NOSE_Y]);
    // -------------------------------------------------------------
    
    
    float most_recent_frame = buffer[BUFFER_SIZE-1][NOSE_INDEX].y_coord;
    
    float* accuracy_res = calloc(2, sizeof(float));
    
    // minimum requirement
    if (
        minimum_ratio > accuracy.height_ratio ||
        (most_recent_frame > (max[NOSE_Y] + Y_ERROR_MAX))) {
        accuracy_res[1] = -1;
        return accuracy_res;
    }
    
    if(is_side == 1) {
//        printf("side\n");
        struct CJointCoordinate point_lhip = {.x = min[HIP_LEFT_X], .y = min[HIP_LEFT_Y]};
        struct CJointCoordinate point_lknee = {.x = min[KNEE_LEFT_X], .y = min[KNEE_LEFT_Y]};
        struct CJointCoordinate point_lankle = {.x = min[ANKLE_LEFT_X], .y = min[KNEE_LEFT_Y]};
        
        struct CJointCoordinate point_rhip = {.x = min[HIP_RIGHT_X], .y = min[HIP_RIGHT_Y]};
        struct CJointCoordinate point_rknee = {.x = min[KNEE_RIGHT_X], .y = min[KNEE_RIGHT_Y]};
        struct CJointCoordinate point_rankle = {.x = min[ANKLE_RIGHT_X], .y = min[KNEE_RIGHT_Y]};
        
        float l_angle = c_points_to_angle(point_lhip, point_lknee, point_lankle);
        float r_angle = c_points_to_angle(point_rhip, point_rknee, point_rankle);
        
//        printf("l_angle : %f, r_angle : %f\n", l_angle, r_angle);
        return c_count_side_squat(max, min, l_angle, r_angle, accuracy.mid_body * 180, optimalaccuracy.mid_body * 180);
        
    } else {
//        printf("not side\n");
        float l_hip = (both_ankle_mid - min[HIP_LEFT_Y]) / (both_ankle_mid - max[HIP_LEFT_Y]);
        float r_hip = (both_ankle_mid - min[HIP_RIGHT_Y]) / (both_ankle_mid - max[HIP_RIGHT_Y]);
        
//        printf("l_hip : %f, r_hip : %f\n", l_hip, r_hip);
        return c_count_front_squat(max, min, l_hip, r_hip, accuracy.upper_body, optimalaccuracy.upper_body);
    }
}
