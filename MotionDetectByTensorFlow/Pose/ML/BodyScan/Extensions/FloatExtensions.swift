//
//  FloatExtensions.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

extension Float {
    func calculateAverage(newValue: Float, count: Int) -> Float {
        let n = Float(count)
        return (self * (n - 1) + newValue) / n
    }
}
