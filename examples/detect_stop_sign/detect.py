import argparse
import sys
import time
import signal
import os

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision


#Return Category detection results
def print_cat(detection_result):
    if detection_result.detections !=[]:
        category = detection_result.detections[0].categories[0]
        category_name = category.category_name
        probability = round(category.score, 2)
    else:
        category_name='None'
        probability = 0.00
        
    return [category_name, probability]

def detect():
    parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='efficientdet_lite0.tflite')
    parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
    parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=640)
    parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=480)
    parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
    parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=False)
    parser.add_argument(
            '--pid',
            help='Pid of a running process where SIGALRM should be sent to',
            required=True)
    args = parser.parse_args()
    pid = int(args.pid)
    print(pid)
    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()

    # Start capturing video input from the camera
    cap = cv2.VideoCapture(int(args.cameraId))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frameWidth)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frameHeight)

    base_options = core.BaseOptions(
      file_name=args.model, use_coral=bool(args.enableEdgeTPU), num_threads=args.numThreads)
    detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)



    while cap.isOpened():
        success, image = cap.read()
        if not success:
          sys.exit(
              'ERROR: Unable to read from webcam. Please verify your webcam settings.'
          )

        counter += 1
        image = cv2.flip(image, 1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the RGB image.
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # Run object detection estimation using the model.
        detection_result = detector.detect(input_tensor)

        # Draw keypoints and edges on input image
        #image = utils.visualize(image, detection_result)
        category_detected=print_cat(detection_result)
        if category_detected[0]=='stop sign':
            os.kill(pid,signal.SIGALRM)
            
            
        print(category_detected)
        
        # Calculate the FPS
    #     if counter % fps_avg_frame_count == 0:
    #       end_time = time.time()
    #       fps = fps_avg_frame_count / (end_time - start_time)
    #       start_time = time.time()

        # Show the FPS
    #     fps_text = 'FPS = {:.1f}'.format(fps)
    #     text_location = (left_margin, row_size)
    #     cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
    #                 font_size, text_color, font_thickness)

        # Stop the program if the ESC key is pressed.
        if cv2.waitKey(1) == 27:
          break
    #     cv2.imshow('object_detector', image)
    cap.release()
    cv2.destroyAllWindows()

detect()
 
