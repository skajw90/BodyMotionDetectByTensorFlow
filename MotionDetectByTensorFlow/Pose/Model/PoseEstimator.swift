//
//  PoseEstimator.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

import UIKit

/// Protocol to  run a pose estimator.
protocol PoseEstimator {
    func estimateSinglePose(on pixelbuffer: CVPixelBuffer) throws -> (Person, Times)
}

// MARK: - Custom Errors
enum PoseEstimationError: Error {
    case modelBusy
    case preprocessingFailed
    case inferenceFailed
    case postProcessingFailed
}
