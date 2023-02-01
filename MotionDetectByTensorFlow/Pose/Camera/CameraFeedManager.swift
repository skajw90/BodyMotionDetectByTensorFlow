//
//  CameraFeedManager.swift
//  MotionDetectByTensorFlow
//
//  Created by Jiwon on 2023/02/01.
//

import AVFoundation
import Accelerate.vImage
import UIKit

/// Delegate to receive the frames captured from the device's camera.
protocol CameraFeedManagerDelegate: AnyObject {
    
    /// Callback method that receives frames from the camera.
    /// - Parameters:
    ///     - cameraFeedManager: The CameraFeedManager instance which calls the delegate.
    ///     - pixelBuffer: The frame received from the camera.
    func cameraFeedManager(
        _ cameraFeedManager: CameraFeedManager, didOutput pixelBuffer: CVPixelBuffer)
}

/// Manage the camera pipeline.
final class CameraFeedManager: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    let captureSession = AVCaptureSession()
    var captureDevice: AVCaptureDevice!
    var isFrontCamera: Bool = false
    /// Delegate to receive the frames captured by the device's camera.
    weak var delegate: CameraFeedManagerDelegate?
    
    init(isFrontCamera: Bool) {
        super.init()
        self.isFrontCamera = isFrontCamera
        configureSession()
    }
    
    /// Start capturing frames from the camera.
    func startRunning() {
        captureSession.startRunning()
    }
    
    /// Stop capturing frames from the camera.
    func stopRunning() {
        captureSession.stopRunning()
    }
    
    /// Initialize the capture session.
    private func configureSession() {
        captureSession.sessionPreset = AVCaptureSession.Preset.photo
        
        let defaultDevice = AVCaptureDevice.default(
            .builtInWideAngleCamera, for: .video, position: isFrontCamera ? .front : .back)
        
        if #available(iOS 13, *), !isFrontCamera {
            captureDevice = AVCaptureDevice.default(.builtInUltraWideCamera, for: .video, position: .back)
        }

        if captureDevice == nil {
            captureDevice = defaultDevice
        }
        guard let captureDevice = captureDevice else {
            print("invalid device type. can't open camera")
            return
        }
        captureDevice.set(frameRate: 30)
        do {
            let input = try AVCaptureDeviceInput(device: captureDevice)
            captureSession.addInput(input)
        } catch {
            return
        }
        
        let videoOutput = AVCaptureVideoDataOutput()
        videoOutput.videoSettings = [
            (kCVPixelBufferPixelFormatTypeKey as String): NSNumber(value: kCVPixelFormatType_32BGRA)
        ]
        videoOutput.alwaysDiscardsLateVideoFrames = true
        let dataOutputQueue = DispatchQueue(
            label: "video data queue",
            qos: .userInitiated,
            attributes: [],
            autoreleaseFrequency: .workItem)
        if captureSession.canAddOutput(videoOutput) {
            captureSession.addOutput(videoOutput)
            videoOutput.connection(with: .video)?.videoOrientation = .portrait
            captureSession.startRunning()
        }
        
        videoOutput.setSampleBufferDelegate(self, queue: dataOutputQueue)
    }
    
    // MARK: Methods of the AVCaptureVideoDataOutputSampleBufferDelegate
    func captureOutput(
        _ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer,
        from connection: AVCaptureConnection
    ) {
        connection.isVideoMirrored = isFrontCamera
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }
        CVPixelBufferLockBaseAddress(pixelBuffer, CVPixelBufferLockFlags.readOnly)
        delegate?.cameraFeedManager(self, didOutput: pixelBuffer)
        CVPixelBufferUnlockBaseAddress(pixelBuffer, CVPixelBufferLockFlags.readOnly)
    }
}
