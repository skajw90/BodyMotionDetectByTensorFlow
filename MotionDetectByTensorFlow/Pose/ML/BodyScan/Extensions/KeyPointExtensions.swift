//
//  KeyPointExtensions.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

import UIKit

extension KeyPoint {
    func getAngleWith(_ keypoint1: KeyPoint, _ keypoint2: KeyPoint) -> CGFloat {
        return coordinate.getAngleWith(point1: keypoint1.coordinate, point2: keypoint2.coordinate)
    }
    
    func isOnLeft(to: KeyPoint, error: CGFloat = 10) -> Bool {
        return coordinate.x > to.coordinate.x + error
    }
    
    func isOnRight(to: KeyPoint, error: CGFloat = 10) -> Bool {
        return coordinate.x < to.coordinate.x - error
    }
    
    func isOnTop(to: KeyPoint, error: CGFloat = 10) -> Bool {
        return coordinate.y < to.coordinate.y - error
    }
    
    func isOnBottom(to: KeyPoint, error: CGFloat = 10) -> Bool {
        return coordinate.y > to.coordinate.y + 10
    }
    
    func isHorizontallySimilar(to: KeyPoint, error: CGFloat = 10) -> Bool {
        return !isOnLeft(to: to, error: error) && !isOnRight(to: to, error: error)
    }
    
    func isVerticallySimilar(to: KeyPoint, error: CGFloat = 10) -> Bool {
        return !isOnTop(to: to, error: error) && !isOnBottom(to: to, error: error)
    }
}
