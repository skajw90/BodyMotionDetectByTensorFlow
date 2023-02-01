//
//  PushUpHold.c
//  PoseDetector
//
//  Created by Jiwon on 2022/06/07.
//

#include "PushUpHold.h"

float* c_count_pushup_hold(struct CJointNode **buffer, float *max, float *min, struct CJointAccuracy accuracy, struct CJointAccuracy optimalaccuracy, struct CJointNode * scanned_body) {
    float* accuracy_res = calloc(20, sizeof(int));
    
    // nose 0 1 eye 2345 ear 6789 shoulder 10111213 elbow 14 15 16 17 wrist 18192021 hip 22232425 knee 26272829 ankle 30 31 32 33
    
    // --------------------------------------------------------------------------------------------
    // buffer
    
    for(int i = 0; i < BUFFER_SIZE; i += 2){
        c_public_update(buffer[i], NOSE_X, NOSE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], SHOULDER_LEFT_X, SHOULDER_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], SHOULDER_RIGHT_X, SHOULDER_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ELBOW_LEFT_X, ELBOW_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ELBOW_RIGHT_X, ELBOW_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], WRIST_LEFT_X, WRIST_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], WRIST_RIGHT_X, WRIST_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], HIP_LEFT_X, HIP_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], HIP_RIGHT_X, HIP_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], KNEE_LEFT_X, KNEE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], KNEE_RIGHT_X, KNEE_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ANKLE_LEFT_X, ANKLE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ANKLE_RIGHT_X, ANKLE_INDEX+1, max, min, accuracy.error_ratio);
    }
    
    if((fabsf(max[WRIST_LEFT_Y] - min[WRIST_LEFT_Y]) > 3 * optimalaccuracy.error_ratio || fabsf(max[WRIST_RIGHT_Y] - min[WRIST_RIGHT_Y]) > 3 * optimalaccuracy.error_ratio)) {
//        printf("triiger-----------------------\n");
        accuracy_res[0] = 3;
        c_free_public(max, min);
        return calloc(2, sizeof(float));
    }
    
    // --------------------------------------------------------------------------------------------
    // minimum elbow angle calculation

    float minimum_angle_elbow = c_min_elbow_angle(max, min);
    
    // --------------------------------------------------------------------------------------------
    // Checking if knee and shoulder is facing same direction
    // ex) if it's both negative or positive
    struct CJointCoordinate point_left_hip = {.x = min[HIP_LEFT_X], .y = min[HIP_LEFT_Y]};
    struct CJointCoordinate point_left_knee = {.x = min[KNEE_LEFT_X], .y = min[KNEE_LEFT_Y]};
    struct CJointCoordinate point_left_ankle = {.x = min[ANKLE_LEFT_X], .y = min[ANKLE_LEFT_Y]};
    
    struct CJointCoordinate point_right_hip = {.x = min[HIP_RIGHT_X], .y = min[HIP_RIGHT_Y]};
    struct CJointCoordinate point_right_knee = {.x = min[KNEE_RIGHT_X], .y = min[KNEE_RIGHT_Y]};
    struct CJointCoordinate point_right_ankle = {.x = min[ANKLE_RIGHT_X], .y = min[ANKLE_RIGHT_Y]};
    
    float left_angle = c_points_to_angle(point_left_hip, point_left_knee, point_left_ankle);
    float right_angle = c_points_to_angle(point_right_hip, point_right_knee, point_right_ankle);

    float check_knee_angle = (left_angle + right_angle) / 2.0;
    
//    printf("knee : %f, avg_wrist_elbow_degree : %f\n", check_knee_angle * (180/PI), avg_wrist_elbow_degree);
    // if minimum requirement not met, abort
    
    if(check_knee_angle * (180/PI) < P_HOLD_MINIMUM_KNEE_DEGREE
       || minimum_angle_elbow * (180/PI) > accuracy.upper_body * 180
       || (MAX(float, min[HIP_LEFT_Y], min[HIP_RIGHT_Y]) >= (MIN(float, min[ELBOW_LEFT_Y], min[ELBOW_RIGHT_Y]) + optimalaccuracy.error_ratio) && MAX(float, min[HIP_LEFT_Y], min[HIP_RIGHT_Y]) != -1)){
        accuracy_res[0] = 3;
        c_free_public(max, min);
        return accuracy_res;
    }
    // --------------------------------------------------------------------------------------------
    // degree gradient difference calculation
    // degree difference from shoulder-ankle degree with shoulder-hip degree and knee-anklee degree
    struct CJointCoordinate avg_left_shoulder = {.x = (max[SHOULDER_LEFT_X] + min[SHOULDER_LEFT_X]) / 2.0, .y = (max[SHOULDER_LEFT_Y] + min[SHOULDER_LEFT_Y]) / 2.0};
    struct CJointCoordinate avg_left_hip = {.x = (max[HIP_LEFT_X] + min[HIP_LEFT_X]) / 2.0, .y = (max[HIP_LEFT_Y] + min[HIP_LEFT_Y]) / 2.0};
    struct CJointCoordinate avg_left_knee = {.x = (max[KNEE_LEFT_X] + min[KNEE_LEFT_X]) / 2.0, .y = (max[KNEE_LEFT_Y] + min[KNEE_LEFT_Y]) / 2.0};
    struct CJointCoordinate avg_left_ankle = {.x = (max[ANKLE_LEFT_X] + min[ANKLE_LEFT_X]) / 2.0, .y = (max[ANKLE_LEFT_Y] + min[ANKLE_LEFT_Y]) / 2.0};
    
    struct CJointCoordinate avg_right_ankle = {.x = (max[ANKLE_RIGHT_X] + min[ANKLE_RIGHT_X]) / 2.0, .y = (max[ANKLE_RIGHT_Y] + min[ANKLE_RIGHT_Y]) / 2.0};
    struct CJointCoordinate avg_right_shoulder = {.x = (max[SHOULDER_RIGHT_X] + min[SHOULDER_RIGHT_X]) / 2.0, .y = (max[SHOULDER_RIGHT_Y] + min[SHOULDER_RIGHT_Y]) / 2.0};
    struct CJointCoordinate avg_right_knee = {.x = (max[KNEE_RIGHT_X] + min[KNEE_RIGHT_X]) / 2.0, .y = (max[KNEE_RIGHT_Y] + min[KNEE_RIGHT_Y]) / 2.0};
    struct CJointCoordinate avg_right_hip = {.x = (max[HIP_RIGHT_X] + min[HIP_RIGHT_X]) / 2.0, .y = (max[HIP_RIGHT_Y] + min[HIP_RIGHT_Y]) / 2.0};
    
    
    float ratio_shoulder_hip_ankle = (c_degree_diff(avg_left_shoulder, avg_left_ankle, avg_left_shoulder, avg_left_hip) + c_degree_diff(avg_right_shoulder, avg_right_ankle, avg_right_shoulder, avg_right_hip)) / 2.0;
    float ratio_shoulder_knee_ankle = (c_degree_diff(avg_left_shoulder, avg_left_ankle, avg_left_knee, avg_left_ankle) + c_degree_diff(avg_right_shoulder, avg_right_ankle, avg_right_knee, avg_right_ankle)) / 2.0;

//    printf("ratio_shoulder_hip_ankle : %f, ratio_shoulder_knee_ankle : %f, minimum_angle_elbow : %f, to be angle : %f\n", ratio_shoulder_hip_ankle, ratio_shoulder_knee_ankle, minimum_angle_elbow * (180/PI), accuracy.upper_body * 180);
    // --------------------------------------------------------------------------------------------
    // percentage calculation
    
    float shoulder_hip_ankle_percentage = MIN(float, 1 - fabsf(optimalaccuracy.mid_body - ratio_shoulder_hip_ankle), (float) 1);
    float shoulder_knee_ankle_percentage = MIN(float, 1 - fabsf(optimalaccuracy.lower_body - ratio_shoulder_knee_ankle), (float) 1);
//     printf("sa_sh_percetnage: %f, sa_ka_percentage: %f\n", shoulder_hip_ankle_percentage, shoulder_knee_ankle_percentage);
    
    // final comparison
    
    if(minimum_angle_elbow * (180/PI) < accuracy.upper_body * 180 && ratio_shoulder_hip_ankle > accuracy.mid_body && ratio_shoulder_knee_ankle > accuracy.lower_body) {
        accuracy_res[1] = ((shoulder_hip_ankle_percentage + shoulder_knee_ankle_percentage) / 2.0) * 100.0;
        accuracy_res[0] = 1;
        return accuracy_res;
    }else{
        accuracy_res[0] = 3;
        c_free_public(max, min);
        return accuracy_res;
    }
}

