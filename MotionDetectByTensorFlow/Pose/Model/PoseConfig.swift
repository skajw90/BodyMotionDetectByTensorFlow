//
//  PoseConfig.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

import Foundation

import Foundation
import UIKit

// MARK: Run configurations

/// TFLite Delegate used to run the model.
enum Delegates: String, CaseIterable {
    case cpu = "CPU"
    case gpu = "GPU"
    case npu = "NPU"
}

/// Information about a TFLite model file.
struct FileInfo {
    var name: String
    var ext: String
}

/// Type of the pose estimation model to be used.
enum ModelType: String, CaseIterable {
//    case posenet = "Posenet"
    case movenetLighting = "Lightning"  // Movenet lightning
    case movenetThunder = "Thunder"  // Movenet thunder
}

/// Pose Detecting Type sequences.
///
/// Classifying Pose type and detecting method
///
enum PoseDetectType {
    enum Unit {
        case count
        case time
    }
    case Squat(Unit)
    case PushUp(Unit)
    case Plank(Unit)
    case None
    
    var title: String {
        switch self {
        case .Squat(let unit):
            switch unit {
            case .count: return "SQUAT"
            case .time: return "SQUAT HOLD"
            }
        case .PushUp(let unit):
            switch unit {
            case .count: return "PUSH UP"
            case .time: return "PUSH UP HOLD"
            }
        case .Plank: return "PLANK"
        case .None: return "NULL"
        }
    }
    
    var isCounting: Bool {
        switch self {
        case .Squat(let unit): return unit == .count
        case .PushUp(let unit): return unit == .count
        case .Plank(let unit): return unit == .count
        case .None: return true
        }
    }
}

struct PoseDetectRatio: Decodable {
    var heightValue: Float
    var upperValue: Float
    var midValue: Float
    var bottomValue: Float
    var overallValue: Float
}

/// Pose Detecting States
enum PoseDetectState {
    enum DetectingState {
        case New
        case Continued
    }
    enum DetectedState {
        case Stabled
        case UnStabled
    }
    case Detecting(DetectingState)
    case Detected(DetectedState)
    
    var detectingColor: CGColor {
        switch self {
        case .Detecting:
            return UIColor.red.cgColor
        case .Detected(let detectedState):
            switch detectedState {
            case .Stabled: return UIColor.green.cgColor
            case .UnStabled: return UIColor.yellow.cgColor
            }
        }
    }
}
