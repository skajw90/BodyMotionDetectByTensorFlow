//
//  PoseDetectorInterpreter.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//


import CoreGraphics
import SwiftUI

private var min_height_ratio: Float = 0.7
private var upper_accuracy: Float = 0.7
private var mid_accuracy: Float = 0.9
private var bottom_accuracy: Float = 0.9
private var min_error_ratio: Float = 0.4

private var ideal_height_ratio: Float = 0.7
private var ideal_upper_accuracy: Float = 0.2
private var ideal_mid_accuracy: Float = 0.9
private var ideal_bottom_accuracy: Float = 0.9
private var ideal_min_error_ratio: Float = 0.4

protocol PoseDetectorInterpreterDelegate: AnyObject {
    func updatePoseCount(_ count: Int)
    func updatePoseAccuracy(_ accuracy: Float)
}

class PoseDetectorInterpreter {
    private let BUFFER_SIZE = 10
    
    weak var delegate: PoseDetectorInterpreterDelegate?
    private var frameBuffer: [UnsafeMutablePointer<CJointNode>?] = []
    // minimum counting condition
    private var preRequisite: CJointAccuracy =
    CJointAccuracy(
        height_ratio: min_height_ratio,     // 횟수 체크 최저 판별 조건 (횟수만 사용)
        upper_body: upper_accuracy,         // 상체의 횟수 또는 버티기 체크 판별 조건
        mid_body: mid_accuracy,             // 중부의 횟수 또는 버티기 체크 판별 조건
        lower_body: bottom_accuracy,        // 하체의 횟수 또는 버티기 체크 판별 조건
        error_ratio: min_error_ratio)       // 각 관절 인식 정확도 최저 판별 조건
    private var accuracies: CJointAccuracy =
    CJointAccuracy(
        height_ratio: ideal_height_ratio,   // 횟수 체크시 높이 정확도 판별 조건
        upper_body: ideal_upper_accuracy,   // 상체의 횟수 또는 버티기 체크 정확도 판별 조건
        mid_body: ideal_mid_accuracy,       // 상체의 횟수 또는 버티기 체크 정확도 판별 조건
        lower_body: ideal_bottom_accuracy,  // 상체의 횟수 또는 버티기 체크 정확도 판별 조건
        error_ratio: ideal_min_error_ratio) // 각 관절 노이즈 오차 판별 조건
    
    private var scannedBody: UnsafeMutablePointer<CJointNode>?
    
    private var minPos = [Float](repeating: -1, count: 34)
    private var maxPos = [Float](repeating: -1, count: 34)
    var count = 0
    
    var countAccuracies: [Float] = []
    
    private var timer: Timer?
    
    init() { }
    
    /**
     * Note: - This configuration must call when adjust custom pre requisite and accuracies
     */
    func configuration(preRequisite: PoseDetectRatio? = nil, accuracies: PoseDetectRatio? = nil, scannedBody: [KeyPoint]? = nil) {
        if let preRequisite = preRequisite {
            self.preRequisite = CJointAccuracy(height_ratio: preRequisite.heightValue, upper_body: preRequisite.upperValue, mid_body: preRequisite.midValue, lower_body: preRequisite.bottomValue, error_ratio: preRequisite.overallValue)
        }
        
        if let accuracies = accuracies {
            self.accuracies = CJointAccuracy(height_ratio: accuracies.heightValue, upper_body: accuracies.upperValue, mid_body: accuracies.midValue, lower_body: accuracies.bottomValue, error_ratio: accuracies.overallValue)
        }
        
        self.scannedBody = UnsafeMutablePointer<CJointNode>.allocate(capacity: 0)
        
        if let scannedBody = scannedBody {
            self.scannedBody = UnsafeMutablePointer<CJointNode>.allocate(capacity: scannedBody.count)
            
            for i in 0 ..< scannedBody.count {
                let value = scannedBody[i]
                self.scannedBody?[i] = CJointNode(
                    joint_name: strdup(value.bodyPart.rawValue),
                    x_coord: Float(value.coordinate.x),
                    y_coord: Float(value.coordinate.y),
                    joint_score: value.score)
            }
        }
    }
    
    func clear() {
        frameBuffer = []
        minPos = [Float](repeating: -1, count: 34)
        maxPos = [Float](repeating: -1, count: 34)
        stopTimer()
    }
    
    func detect(_ type: PoseDetectType? = nil, keypoint: [KeyPoint]) -> [BodyPart]? {
        if frameBuffer.count == BUFFER_SIZE {
            frameBuffer.removeFirst()
        }
        let bodyPartBuffer = UnsafeMutablePointer<CJointNode>.allocate(capacity: keypoint.count)
        for i in 0 ..< keypoint.count {
            let value = keypoint[i]
            bodyPartBuffer[i] = CJointNode(
                joint_name: strdup(value.bodyPart.rawValue),
                x_coord: Float(value.coordinate.x),
                y_coord: Float(value.coordinate.y),
                joint_score: value.score)
        }
        frameBuffer.append(bodyPartBuffer)
        if frameBuffer.count < BUFFER_SIZE { return nil }
        let buffer = UnsafeMutablePointer<UnsafeMutablePointer<CJointNode>?>.allocate(capacity: BUFFER_SIZE)
        buffer.initialize(from: &frameBuffer, count: frameBuffer.count)
        
        guard let type = type else { return nil }

        return runCScript(type, buffer: buffer)
    }
    
    private func runCScript(_ type: PoseDetectType, buffer: UnsafeMutablePointer<UnsafeMutablePointer<CJointNode>?>) -> [BodyPart]? {
        var result: UnsafeMutablePointer<Float32>?
        
        switch type {
        case .Squat(let unit):
            switch unit {
            case .count: result = c_count_squat(buffer, &maxPos, &minPos, preRequisite, accuracies, scannedBody)
            case .time: result = c_count_squat_hold(buffer, &maxPos, &minPos, preRequisite, accuracies, scannedBody)
            }
        case .PushUp(let unit):
            switch unit {
            case .count: result = c_count_pushup(buffer, &maxPos, &minPos, preRequisite, accuracies, scannedBody)
            case .time: result = c_count_pushup_hold(buffer, &maxPos, &minPos, preRequisite, accuracies, scannedBody)
            }
        case .Plank:
            result = c_count_plank_hold(buffer, &maxPos, &minPos, preRequisite, accuracies, scannedBody)
        default:
            print("detect mode unsupported.")
            break
        }
        guard let result = result else { return nil }

        let unstabledDots: [BodyPart] = []
        var state: Int = 0
        var accuracy: Float = 0
        for i in 0 ..< 2 {
            if i == 0 {
                state = Int(result[i])
            }
            else if i == 1 {
                accuracy = result[i]
            }
            else {
                print("invalid variable detected.")
            }
//            else if i == 2 {
//                rate = Float(result[i])
//            }
//            else if result[i] == 1,
//                    let part = BodyPart.getBodyPart(index: i - 3) {
//                unstabledDots.append(part)
//            }
        }
        print("state: \(state)")
        var isCounting: Bool = false
        switch type {
        case .Squat(let unit): isCounting = unit == .count
        case .PushUp(let unit): isCounting = unit == .count
        case .Plank(let unit): isCounting = unit == .count
        case .None: break
        }
        switch state {
        case -1: clear()
        case 0: break;
        case 1:
            if !isCounting && timer == nil {
                startTimer()
            }
            else if isCounting {
                count += 1
                delegate?.updatePoseCount(count)
                print("count: \(count)")
                clear()
            }
            
            print("accuray: \(accuracy)")
            if isCounting {
                countAccuracies.append(accuracy)
            }
            else {
                if countAccuracies.count == 2,
                   let oldAvg = countAccuracies.first,
                   let floatCount = countAccuracies.last {
                    let c = Int(floatCount)
                    countAccuracies[0] = oldAvg / Float(c + 1) * Float(c) + accuracy / Float(c + 1)
                    countAccuracies[1] = Float(c + 1)
                }
                else {
                    countAccuracies.append(accuracy)
                    countAccuracies.append(1)
                }
            }
            delegate?.updatePoseAccuracy(accuracy)
            
        case 2: break;
        case 3:
            if !isCounting && timer != nil {
                stopTimer()
            }
        default: break;
        }
        // set accuracy
//        let accuracy = rate == 0 ? 0 : accuracyValue / rate
//        delegate.updatePoseAccuracy(accuracy)
        return unstabledDots
    }
    
    private func startTimer() {
        print("Timer Started.")
        timer?.invalidate()
        timer = nil
        timer = Timer.scheduledTimer(timeInterval: 0.01, target: self, selector: #selector(updateCountTime), userInfo: nil, repeats: true)
    }
    
    func stopTimer() {
        print("Timer Ended.")
        timer?.invalidate()
        timer = nil
    }
    
    @objc private func updateCountTime() {
        count += 1
        delegate?.updatePoseCount(count)
    }
    
}


