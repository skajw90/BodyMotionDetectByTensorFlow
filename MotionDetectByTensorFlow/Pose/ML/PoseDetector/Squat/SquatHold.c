//
//  PlankHold.c
//  PoseDetector
//
//  Created by Mingon Song on 2022-06-10.
//

#include "SquatHold.h"

float* c_count_squat_hold(struct CJointNode ** buffer, float * max, float * min, struct CJointAccuracy accuracy, struct CJointAccuracy optimalaccuracy, struct CJointNode * scanned_body){
    
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
        c_public_update(buffer[i], HIP_LEFT_X, HIP_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], HIP_RIGHT_X, HIP_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], KNEE_LEFT_X, KNEE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], KNEE_RIGHT_X, KNEE_INDEX+1, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ANKLE_LEFT_X, ANKLE_INDEX, max, min, accuracy.error_ratio);
        c_public_update(buffer[i], ANKLE_RIGHT_X, ANKLE_INDEX+1, max, min, accuracy.error_ratio);
    }

    
    if((fabsf(max[ANKLE_LEFT_Y] - min[ANKLE_LEFT_Y]) > optimalaccuracy.error_ratio
        || fabsf(max[ANKLE_RIGHT_Y] - min[ANKLE_RIGHT_Y]) > optimalaccuracy.error_ratio)) {
        accuracy_res[0] = 3;
        c_free_public(max, min);
        return calloc(2, sizeof(float));
    }

    // --------------------------------------------------------------------------------------------
    // last leg angle

    struct CJointCoordinate point_left_hip = {.x = buffer[BUFFER_SIZE-1][HIP_INDEX].x_coord, .y = buffer[BUFFER_SIZE-1][HIP_INDEX].y_coord};
    struct CJointCoordinate point_left_knee = {.x = buffer[BUFFER_SIZE-1][KNEE_INDEX].x_coord, .y = buffer[BUFFER_SIZE-1][KNEE_INDEX].y_coord};
    struct CJointCoordinate point_left_ankle = {.x = buffer[BUFFER_SIZE-1][ANKLE_INDEX].x_coord, .y = buffer[BUFFER_SIZE-1][ANKLE_INDEX].y_coord};
    
    struct CJointCoordinate point_right_hip = {.x = buffer[BUFFER_SIZE-1][HIP_INDEX+1].x_coord, .y = buffer[BUFFER_SIZE-1][HIP_INDEX+1].y_coord};
    struct CJointCoordinate point_right_knee = {.x = buffer[BUFFER_SIZE-1][KNEE_INDEX+1].x_coord, .y = buffer[BUFFER_SIZE-1][KNEE_INDEX+1].y_coord};
    struct CJointCoordinate point_right_ankle = {.x = buffer[BUFFER_SIZE-1][ANKLE_INDEX+1].x_coord, .y = buffer[BUFFER_SIZE-1][ANKLE_INDEX+1].y_coord};
    
    float l_angle = c_points_to_angle(point_left_hip, point_left_knee, point_left_ankle);
    float r_angle = c_points_to_angle(point_right_hip, point_right_knee, point_right_ankle);
    
    float last_angle_knee = (fabs(l_angle + r_angle) / 2.0) * (180/PI);
    // --------------------------------------------------------------------------------------------
    // final comparison
    
    //float last_hip_frame = MIN(float, buffer[BUFFER_SIZE-1][HIP_RIGHT_Y].y_coord, buffer[BUFFER_SIZE-1][HIP_LEFT_Y].y_coord)
    
    float mid_angle = (optimalaccuracy.mid_body + optimalaccuracy.lower_body) / 2.0;
    if(accuracy.mid_body * 180 > last_angle_knee && last_angle_knee > accuracy.lower_body * 180) {
        accuracy_res[0] = 1;
        if(last_angle_knee > optimalaccuracy.mid_body * 180 || last_angle_knee < optimalaccuracy.lower_body * 180){
//            printf("trigger too big\n");
            accuracy_res[1] = 0;
        }
        else{
            if(last_angle_knee < (mid_angle + optimalaccuracy.height_ratio) * 180 && last_angle_knee > (mid_angle - optimalaccuracy.height_ratio) * 180){
//                printf("trigger too perfect\n");
                accuracy_res[1] = 100;
            }else{
                float angle_range_for_calc = ((optimalaccuracy.mid_body) - (mid_angle + optimalaccuracy.height_ratio)) * 180;
                if(last_angle_knee > (mid_angle + optimalaccuracy.height_ratio) * 180){
                    accuracy_res[1] = (1 - ((last_angle_knee - ((mid_angle + optimalaccuracy.height_ratio) * 180))/angle_range_for_calc)) * 100;
//                    printf("calc for %f / %f == %f : ", (last_angle_knee - ((mid_angle + optimalaccuracy.height_ratio) * 180)), angle_range_for_calc, accuracy_res[1]);
                }else{
                    accuracy_res[1] = ((last_angle_knee - (optimalaccuracy.lower_body * 180)) /angle_range_for_calc) * 100;
//                    printf("calc for %f / %f == %f : ", (last_angle_knee - ((mid_angle + optimalaccuracy.height_ratio) * 180)), angle_range_for_calc, accuracy_res[1]);
                }
            }
        }
        
//        printf("minimum angle : %f, maximum : %f, minimum : %f, mid_max : %f, mid_min : %f, mid : %f\n", last_angle_knee, accuracy.mid_body * 180, accuracy.lower_body * 180, ((mid_angle + optimalaccuracy.height_ratio) * 180), ((mid_angle - optimalaccuracy.height_ratio) * 180), mid_angle * 180);
        return accuracy_res;
    }
    accuracy_res[0] = 3;
    c_free_public(max, min);
    return accuracy_res;
}
