//
//  CGFloatExtensions.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

import UIKit

extension CGFloat {
    func isValueSimilar(with: CGFloat, error: CGFloat = 10) -> Bool {
        return self < with + error && self > with - error
    }
}
