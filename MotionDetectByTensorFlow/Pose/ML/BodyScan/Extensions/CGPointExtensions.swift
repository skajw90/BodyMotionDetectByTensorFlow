//
//  CGPointExtensions.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

import UIKit

extension CGPoint {
    func within(_ point: CGPoint, boundary: CGFloat) -> Bool {
        return (point.x >= x - boundary && point.x <= x + boundary)
        && (point.y >= y - boundary && point.y <= y + boundary)
    }
    
    func calculateAverage(newValue: CGPoint, count: Int) -> CGPoint {
        let n = CGFloat(count)
        let x = (self.x * (n - 1) + newValue.x) / n
        let y = (self.y * (n - 1) + newValue.y) / n
        return CGPoint(x: x, y: y)
    }
    
    func getAngleWith(point1: CGPoint, point2: CGPoint) -> CGFloat {
        let vec1 = CGVector(dx: point1.x - x, dy: point1.y - y)
        let vec2 = CGVector(dx: point2.x - x, dy: point2.y - y)
        
        let theta1 = atan2(vec1.dy, vec1.dx)
        let theta2 = atan2(vec2.dy, vec2.dx)
        
        let angle = (theta2 - theta1) / CGFloat.pi * 180
        
        return angle
    }
}

