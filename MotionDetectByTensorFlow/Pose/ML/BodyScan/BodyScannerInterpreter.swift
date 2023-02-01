//
//  BodyScannerInterpreter.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

import UIKit

class BodyScannerInterpreter {
    enum State {
        case BodyDetected
        case StartScanning
        case Moved
        case NotFront
        case UnBalanced
        case Complete
        case Timeover
    }
    
    typealias BodyPoint = (avgCoordinate: CGPoint, avgScore: Float, count: Int)
    let MIN_BODY_SCAN_SCORE: Float = 0.6
    let MIN_JOINT_ERROR_RATIO: CGFloat = 20
    let MIN_ELBOW_ANGLE_RATE: CGFloat = 20
    let MIN_KNEE_ANGLE_RATE: CGFloat = 10
    let MIN_BODY_ERROR_RATE: Float = 0.6
    
    var timeOverTimer: Timer?
    var scanningTimer: Timer?
    
    var isScanStarted: Bool = false
    var isTimeOver: Bool = false
    
    let MIN_TIME_LIMIT: TimeInterval = 10
    let MIN_DETECT_TIME_LIMIT: TimeInterval = 3
    let MIN_JOINT_DATA_COUNT: Int = 100
    
    private var scannedBuffer: [BodyPart: BodyPoint] = [:]
    var scannedData: [KeyPoint] = []
    
    var isBodyDetected: Bool = false
    
    func reset() {
        scanningTimer?.invalidate()
        scanningTimer = nil
        timeOverTimer?.invalidate()
        timeOverTimer = nil
        scannedBuffer = [:]
        isScanStarted = false
    }
    
    func scanning(_ keyPoints: [KeyPoint]?, isValid: Bool, completion: @escaping (State) -> Void) {
        guard let keyPoints = keyPoints, isValid else {
            reset()
            return
        }
        
        if isTimeOver {
            isTimeOver = false
            isBodyDetected = false
            completion(.Timeover)
            return
        }
        
        if !isBodyDetected {
            isBodyDetected = true
            completion(.BodyDetected)
        }
        
        if isScanStarted && timeOverTimer == nil {
            completion(.StartScanning)
            // 5. if scanning time is greater than 10 sec, show alert
            timeOverTimer = Timer.scheduledTimer(timeInterval: MIN_TIME_LIMIT, target: self, selector: #selector(timeOverActionHandler), userInfo: nil, repeats: false)
        }
        if !isScanStarted && scanningTimer == nil {
            scanningTimer = Timer.scheduledTimer(timeInterval: MIN_DETECT_TIME_LIMIT, target: self, selector: #selector(scanningTimerHandler), userInfo: nil, repeats: false)
        }
        
        if isScanStarted {
            // 3. check body pos validation check
            // compare ankle angle horizontally
            if !isBodyFrontSide(keyPoints) {
                print("body not front.")
                reset()
                completion(.NotFront)
                return
            }
            // compare body angles valid
            //      - shoulder line
            //      - hip line
            //      - arm angles
            //      - knee angles
            if !isBalancedPose(keyPoints) {
                print("body unbalanced.")
                reset()
                completion(.UnBalanced)
                return
            }
        }
        var isEnoughData: Bool = true
        
        for keyPoint in keyPoints {
            // 1. ignore irregulars
            //      - if score is less than min score, drop the point
            if keyPoint.score < MIN_BODY_SCAN_SCORE { continue }
            //      - if joint moved, alert, and reset while scanning body
            // 2. store joint data if qualify two conditions
            if detectMovementAndStoreData(keyPoint: keyPoint) {
                completion(.Moved)
                reset()
                return
            }
            // 4. check minimum scanned data count at least 100
            if (scannedBuffer[keyPoint.bodyPart]?.count ?? 0) < MIN_JOINT_DATA_COUNT {
                isEnoughData = false
            }
        }
        // 6. point out data if valid
        if isScanStarted && isEnoughData && scannedBuffer.count == 17 {
            // store data
            var tempData: [KeyPoint] = []
            for buffer in scannedBuffer.sorted(by: { $0.key.position < $1.key.position }) {
                tempData.append(KeyPoint(bodyPart: buffer.key, coordinate: buffer.value.avgCoordinate, score: buffer.value.avgScore))
            }
            scannedData = makeUnitPosition(data: tempData)
            reset()
            completion(.Complete)
        }
    }
    
    private func isBodyFrontSide(_ keyPoints: [KeyPoint]) -> Bool {
        // compare left and right correct position
        for i in stride(from: 1, to: 17, by: 2) {
            // 0: nose
            // i + 1 : right body part, i : left body part
            if keyPoints[i + 1].isOnLeft(to: keyPoints[i]) { return false }
        }
        
        // compare ankle angle horizontally, compare y pos
        let ankleIndex = BodyPart.leftAnkle.position
        if keyPoints[ankleIndex].score >= MIN_BODY_ERROR_RATE
            && keyPoints[ankleIndex + 1].score >= MIN_BODY_ERROR_RATE
            && !keyPoints[ankleIndex].isVerticallySimilar(to: keyPoints[ankleIndex + 1]) {
            return false
        }
        
        return true
    }
    
    private func isBalancedPose(_ keyPoints: [KeyPoint]) -> Bool {
        let hipIndex = BodyPart.leftHip.position
        let shoulderIndex = BodyPart.leftShoulder.position
        
        let elbowIndex = BodyPart.leftElbow.position
        let wristIndex = BodyPart.leftWrist.position
        
        let kneeIndex = BodyPart.leftKnee.position
        let ankleIndex = BodyPart.leftAnkle.position
        
        var isVaild = true
        
        // each group need to be checked valid joint score
        // compare hip angle horizontally parallel
        if keyPoints[hipIndex].score >= MIN_BODY_ERROR_RATE
            && keyPoints[hipIndex + 1].score  >= MIN_BODY_ERROR_RATE
            && !keyPoints[hipIndex].isVerticallySimilar(to: keyPoints[hipIndex + 1]) {
            print("hip is not horizontally parallel.")
            isVaild = false
        }
        // compare shoulder angle horizontally parallel
        if keyPoints[shoulderIndex].score >= MIN_BODY_ERROR_RATE
            && keyPoints[shoulderIndex + 1].score  >= MIN_BODY_ERROR_RATE
            && !keyPoints[shoulderIndex].isVerticallySimilar(to: keyPoints[shoulderIndex + 1]) {
            print("shoulder is not horizontally parallel.")
            isVaild = false
        }
        // compare arm angle straight: left and right
        for i in 0...1 {
            if keyPoints[elbowIndex + i].score >= MIN_BODY_ERROR_RATE
                && keyPoints[wristIndex + i].score  >= MIN_BODY_ERROR_RATE
                && keyPoints[shoulderIndex + i].score  >= MIN_BODY_ERROR_RATE {
                let angle = abs(keyPoints[elbowIndex + i].getAngleWith(keyPoints[wristIndex + i], keyPoints[shoulderIndex + i]))
                if !angle.isValueSimilar(with: 180, error: MIN_ELBOW_ANGLE_RATE) {
                    print("\(i == 0 ? "left" : "right") arm angle: \(angle)")
                    print("\(i == 0 ? "left" : "right") arm is not straight.")
                    isVaild = false
                }
            }
        }
        // compare knee angle straight: left and right
        for i in 0...1 {
            if keyPoints[hipIndex + i].score >= MIN_BODY_ERROR_RATE
                && keyPoints[kneeIndex + i].score  >= MIN_BODY_ERROR_RATE
                && keyPoints[ankleIndex + i].score  >= MIN_BODY_ERROR_RATE {
                let angle = abs(keyPoints[kneeIndex + i].getAngleWith(keyPoints[hipIndex + i], keyPoints[ankleIndex + i]))
                if !angle.isValueSimilar(with: 180, error: MIN_KNEE_ANGLE_RATE) {
                    print("\(i == 0 ? "left" : "right") leg angle: \(angle)")
                    print("\(i == 0 ? "left" : "right") leg is not straight.")
                    isVaild = false
                }
            }
        }
        
        return isVaild
    }
    
    private func detectMovementAndStoreData(keyPoint: KeyPoint) -> Bool {
        if scannedBuffer.contains(where: { $0.key == keyPoint.bodyPart }) {
            if keyPoint.coordinate.within(scannedBuffer[keyPoint.bodyPart]!.avgCoordinate, boundary: MIN_JOINT_ERROR_RATIO) {
                let prev = scannedBuffer[keyPoint.bodyPart]!
                let prevAverage = prev.avgCoordinate
                let prevScore = prev.avgScore
                let n = prev.count + 1
                scannedBuffer[keyPoint.bodyPart] = BodyPoint(prevAverage.calculateAverage(newValue: keyPoint.coordinate, count: n), prevScore.calculateAverage(newValue: keyPoint.score, count: n) , n)
            }
            else {
                return isScanStarted
            }
        }
        else {
            scannedBuffer[keyPoint.bodyPart] = BodyPoint(keyPoint.coordinate, keyPoint.score, 1)
        }
        return false
    }
    
    private func makeUnitPosition(data: [KeyPoint]) -> [KeyPoint] {
        var minX: CGFloat = .infinity // left
        var maxX: CGFloat = 0 //right
        var minY: CGFloat = .infinity // bottom
        var maxY: CGFloat = 0 // top
        
        for point in data {
            minX = min(point.coordinate.x, minX)
            maxX = max(point.coordinate.x, maxX)
            minY = min(point.coordinate.y, minY)
            maxY = max(point.coordinate.y, maxY)
        }
        
        let maxWidth = maxX - minX
        let maxHeight = maxY - minY
        
        let maxUnit = max(maxHeight, maxWidth)
        
        return data.map {
            return KeyPoint(
                bodyPart: $0.bodyPart,
                coordinate: CGPoint(
                    x: ($0.coordinate.x - minX) / maxUnit,
                    y: ($0.coordinate.y - minY) / maxUnit),
                score: $0.score)
        }
    }
    
    @objc private func scanningTimerHandler() {
        scanningTimer?.invalidate()
        scanningTimer = nil
        isScanStarted = true
    }
    
    @objc private func timeOverActionHandler() {
        isTimeOver = true
        print("time over")
        reset()
        scannedData = []
    }
}
