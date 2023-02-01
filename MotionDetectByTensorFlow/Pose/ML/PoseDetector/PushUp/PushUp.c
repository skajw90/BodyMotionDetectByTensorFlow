//
//  PushUp.c
//  PoseDetector
//
//  Created by Jiwon on 2022/05/24.
//

#include "PushUp.h"

//void c_update_seg(struct CJointNode **buffer, float *max, float *min, struct CJointAccuracy accuracy) {
//    for(int i = 0; i < BUFFER_SIZE; i++){
//        c_global_update(buffer[i], SHOULDER_LEFT_X, SHOULDER_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], SHOULDER_RIGHT_X, SHOULDER_INDEX+1, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], ELBOW_LEFT_X, ELBOW_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], ELBOW_RIGHT_X, ELBOW_INDEX+1, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], WRIST_LEFT_X, WRIST_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], WRIST_RIGHT_X, WRIST_INDEX+1, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], ANKLE_LEFT_X, ANKLE_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], ANKLE_RIGHT_X, ANKLE_INDEX+1, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], KNEE_LEFT_X, KNEE_INDEX, max, min, accuracy.error_ratio);
//        c_global_update(buffer[i], KNEE_RIGHT_X, KNEE_INDEX+1, max, min, accuracy.error_ratio);
//    }
//}

float c_pushup_height_ratio(float *max, float *min, float transposed_lx, float transposed_rx){
    struct CJointCoordinate max_left_shoulder = {.x = transposed_lx, .y = max[SHOULDER_LEFT_Y]};
    struct CJointCoordinate max_right_shoulder = {.x = transposed_rx, .y = max[SHOULDER_RIGHT_Y]};
    
    struct CJointCoordinate min_left_wrist = {.x = min[WRIST_LEFT_X], .y = min[WRIST_RIGHT_Y]};
    struct CJointCoordinate min_right_wrist = {.x = min[WRIST_RIGHT_X], .y = min[WRIST_RIGHT_Y]};
    struct CJointCoordinate min_left_shoulder = {.x = transposed_lx, .y = min[SHOULDER_LEFT_Y]};
    struct CJointCoordinate min_right_shoulder= {.x = transposed_rx, .y = min[SHOULDER_RIGHT_Y]};

    float min_dist_ws = MIN(float, c_points_to_distance(min_left_wrist, min_left_shoulder), c_points_to_distance(min_right_wrist, min_right_shoulder));

    float max_dist_ws = MAX(float, c_points_to_distance(min_left_wrist, max_left_shoulder), c_points_to_distance(min_right_wrist, max_right_shoulder));
    
//    printf("min dist : %f, max dist : %f\n", min_dist_ws, max_dist_ws);
    return min_dist_ws / max_dist_ws;
}

float c_bicep_ratio(float *max, float *min){
    struct CJointCoordinate max_left_elbow = {.x = max[WRIST_LEFT_X], .y = max[WRIST_RIGHT_Y]};
    struct CJointCoordinate max_right_elbow = {.x = max[WRIST_RIGHT_X], .y = max[WRIST_RIGHT_Y]};
    struct CJointCoordinate max_left_shoulder = {.x = max[SHOULDER_LEFT_X], .y = max[SHOULDER_LEFT_Y]};
    struct CJointCoordinate max_right_shoulder = {.x = max[SHOULDER_RIGHT_X], .y = max[SHOULDER_RIGHT_Y]};
    
    struct CJointCoordinate min_left_elbow = {.x = min[WRIST_LEFT_X], .y = min[WRIST_RIGHT_Y]};
    struct CJointCoordinate min_right_elbow = {.x = min[WRIST_RIGHT_X], .y = min[WRIST_RIGHT_Y]};
    struct CJointCoordinate min_left_shoulder = {.x = max[SHOULDER_LEFT_X], .y = min[SHOULDER_LEFT_Y]};
    struct CJointCoordinate min_right_shoulder= {.x = max[SHOULDER_RIGHT_X], .y = min[SHOULDER_RIGHT_Y]};
    
    float min_dist_bicep = MIN(float, c_points_to_distance(min_left_elbow, min_left_shoulder), c_points_to_distance(min_right_elbow, min_right_shoulder));
    
    float max_dist_bicep = MAX(float, c_points_to_distance(max_left_elbow, max_left_shoulder), c_points_to_distance(max_right_elbow, max_right_shoulder));
    
    return min_dist_bicep / max_dist_bicep;
}


float* c_count_pushup(struct CJointNode **buffer, float *max, float *min, struct CJointAccuracy accuracy, struct CJointAccuracy optimalaccuracy, struct CJointNode * scanned_body){

//    c_update_seg(buffer, max, min, accuracy);
    for(int i = 0; i < BUFFER_SIZE; i++){
        c_public_update(buffer[i], SHOULDER_LEFT_X, SHOULDER_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], SHOULDER_RIGHT_X, SHOULDER_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ELBOW_LEFT_X, ELBOW_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ELBOW_RIGHT_X, ELBOW_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], WRIST_LEFT_X, WRIST_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], WRIST_RIGHT_X, WRIST_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ANKLE_LEFT_X, ANKLE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ANKLE_RIGHT_X, ANKLE_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], KNEE_LEFT_X, KNEE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], KNEE_RIGHT_X, KNEE_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], HIP_LEFT_X, HIP_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], HIP_RIGHT_X, HIP_INDEX+1, max, min, accuracy.error_ratio);
    }
    
    if((fabsf(max[WRIST_LEFT_Y] - min[WRIST_LEFT_Y]) > optimalaccuracy.error_ratio || fabsf(max[WRIST_RIGHT_Y] - min[WRIST_RIGHT_Y]) > optimalaccuracy.error_ratio) || (fabsf(max[WRIST_LEFT_X] - min[WRIST_LEFT_X]) > optimalaccuracy.error_ratio || fabsf(max[WRIST_RIGHT_X] - min[WRIST_RIGHT_X]) > optimalaccuracy.error_ratio)) {
        c_free_public(max, min);
        return calloc(2, sizeof(float));
    }
    
    // moved check
    if((fabsf(max[ANKLE_LEFT_X] - min[ANKLE_LEFT_X]) > optimalaccuracy.error_ratio || fabsf(max[ANKLE_RIGHT_X] - min[ANKLE_RIGHT_X]) > optimalaccuracy.error_ratio)) {
        c_free_public(max, min);
        return calloc(2, sizeof(float));
    }
    
    // height ratio
    float height_ratio = c_pushup_height_ratio(max, min, min[SHOULDER_LEFT_X], min[SHOULDER_RIGHT_X]);
    
    float most_recent_frame = MIN(float, buffer[BUFFER_SIZE-1][SHOULDER_INDEX].y_coord, buffer[BUFFER_SIZE-1][SHOULDER_INDEX+1].y_coord);
    

    
    if(most_recent_frame > MIN(float, max[SHOULDER_LEFT_Y] + Y_ERROR_MAX, max[SHOULDER_RIGHT_Y] + Y_ERROR_MAX)
       || height_ratio > accuracy.upper_body
       || MAX(float, min[HIP_LEFT_Y], min[HIP_RIGHT_Y]) >= MAX(float, min[ELBOW_LEFT_Y], min[ELBOW_RIGHT_Y]) + optimalaccuracy.error_ratio){
        return calloc(2, sizeof(float));
    }
    
    // is_side or is_front
    int is_side = c_is_body_side(
                                 (struct CJointCoordinate) { buffer[9][WRIST_INDEX].x_coord, buffer[9][WRIST_INDEX].y_coord },
                                 (struct CJointCoordinate) { buffer[9][WRIST_INDEX + 1].x_coord, buffer[9][WRIST_INDEX + 1].y_coord });

    
    float *accuracy_res = calloc(2, sizeof(float));
    if(is_side == 1) {
        
        struct CJointCoordinate avg_left_shoulder = {.x = (max[SHOULDER_LEFT_X] + min[SHOULDER_LEFT_X]) / 2.0, .y = (max[SHOULDER_LEFT_Y] + min[SHOULDER_LEFT_Y]) / 2.0};
        struct CJointCoordinate avg_left_hip = {.x = (max[HIP_LEFT_X] + min[HIP_LEFT_X]) / 2.0, .y = (max[HIP_LEFT_Y] + min[HIP_LEFT_Y]) / 2.0};
        struct CJointCoordinate avg_left_knee = {.x = (max[KNEE_LEFT_X] + min[KNEE_LEFT_X]) / 2.0, .y = (max[KNEE_LEFT_Y] + min[KNEE_LEFT_Y]) / 2.0};
        struct CJointCoordinate avg_left_ankle = {.x = (max[ANKLE_LEFT_X] + min[ANKLE_LEFT_X]) / 2.0, .y = (max[ANKLE_LEFT_Y] + min[ANKLE_LEFT_Y]) / 2.0};
        
        struct CJointCoordinate avg_right_shoulder = {.x = (max[SHOULDER_RIGHT_X] + min[SHOULDER_RIGHT_X]) / 2.0, .y = (max[SHOULDER_RIGHT_Y] + min[SHOULDER_RIGHT_Y]) / 2.0};
        struct CJointCoordinate avg_right_hip = {.x = (max[HIP_RIGHT_X] + min[HIP_RIGHT_X]) / 2.0, .y = (max[HIP_RIGHT_Y] + min[HIP_RIGHT_Y]) / 2.0};
        struct CJointCoordinate avg_right_knee = {.x = (max[KNEE_RIGHT_X] + min[KNEE_RIGHT_X]) / 2.0, .y = (max[KNEE_RIGHT_Y] + min[KNEE_RIGHT_Y]) / 2.0};
        struct CJointCoordinate avg_right_ankle = {.x = (max[ANKLE_RIGHT_X] + min[ANKLE_RIGHT_X]) / 2.0, .y = (max[ANKLE_RIGHT_Y] + min[ANKLE_RIGHT_Y]) / 2.0};
        
        
        float ratio_shoulder_hip_ankle = (c_degree_diff(avg_left_shoulder, avg_left_ankle, avg_left_shoulder, avg_left_hip) + c_degree_diff(avg_right_shoulder, avg_right_ankle, avg_right_shoulder, avg_right_hip)) / 2.0;
        float ratio_shoulder_knee_ankle = (c_degree_diff(avg_left_shoulder, avg_left_ankle, avg_left_knee, avg_left_ankle) + c_degree_diff(avg_right_shoulder, avg_right_ankle, avg_right_knee, avg_right_ankle)) / 2.0;
        
        float minimum_angle_elbow = c_min_elbow_angle(max, min);
        
        float angle_elbow_accuracy = minimum_angle_elbow * (180/PI) - optimalaccuracy.upper_body * (180);
        if(angle_elbow_accuracy < 0){
            accuracy_res[1] = 100.0;
        }else{
            accuracy_res[1] = (1 - (angle_elbow_accuracy / 180)) * 100;
        }
        
        if(ratio_shoulder_hip_ankle > accuracy.mid_body && ratio_shoulder_knee_ankle > accuracy.lower_body){
            c_free_public(max, min);
            accuracy_res[0] = 1;
        }
//        printf("most_recent_frame : %f, max_y : %f, height_ratio : %f\n", most_recent_frame, MAX(float, max[SHOULDER_LEFT_Y], max[SHOULDER_RIGHT_Y]), height_ratio);
//        printf("accruacy : %f, minimum anlge : %f, ratio_shoulder_hip : %f, ratio_shoulder_knee : %f\n", accuracy_res[1], minimum_angle_elbow * (180/PI), ratio_shoulder_hip_ankle, ratio_shoulder_knee_ankle);
        
        return accuracy_res;
        
    } else {
        
        float bicep_ratio = c_bicep_ratio(max, min);
        
        float bicep_accuracy = bicep_ratio - optimalaccuracy.upper_body;
        if(bicep_accuracy < 0){
            accuracy_res[1] = 100.0;
        }else{
            accuracy_res[1] = (1 - bicep_accuracy) * 100;
        }
        
//        printf("most_recent_frame : %f, max_hip : %f, min_wrist: %f, height_ratio : %f\n", most_recent_frame,  MAX(float, min[ANKLE_LEFT_Y], min[ANKLE_RIGHT_Y]), MIN(float, min[WRIST_LEFT_Y], min[WRIST_RIGHT_Y]), height_ratio);
//        printf("accruacy : %f, bicep ratio : %f\n", accuracy_res[1], bicep_ratio);
//
        if(bicep_ratio < accuracy.upper_body){
            c_free_public(max, min);
            accuracy_res[0] = 1;
        }
        return accuracy_res;
        
    }

}

