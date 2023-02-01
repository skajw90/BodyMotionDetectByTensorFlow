//
//  Constants.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

enum Constants {
    // Configs for the TFLite interpreter.
    static let defaultThreadCount = 4
    static let defaultDelegate: Delegates = .cpu
    static let defaultModelType: ModelType = .movenetThunder
    
    // Minimum score to render the result.
    static let minimumScore: Float32 = 0.2
    static let unstabledScore: Float32 = 0.35
}
