# BabyRobot Integration

## Integrated Services

- Audio Stream
  - A raw audio stream as captured by the front facing Kinect or the mic. 
  - Outputs a byte64 encoded wav audio segment.
  - Dummy segmentation by timing (e.g. publish a new message every 10 seconds).
- Color Stream
  - A color image stream as captured by the front facing Kinect.
  - Outputs byte64 encoded image frames.
- Depth Stream
  - A depth image stream as captured by the front facing Kinect.
  - Outputs byte64 depth frames.
- Object Recognition Module
  - A stream of the detected objects in the current image.
  - Outputs the bounding box an the label for every detected object along with the frame id.
  - Optionally subscribes to the depth stream and outputs the proximity for each object. 
- ASR Module
  - A stream of the transcribed text for an audio segment.
  - Outputs the text transcription along with the audio segment id.
- Emotion Recognition Module
  - A stream of emotions extracted from the visual and the audio modality. 
    The emotion prediction is the combined prediction of the 2 modalities.
  - Outputs a list of predicted emotions
- Speech Features Module
  - A stream of low level audio features (MFCCs, fMMLR, i-vectors, functionals)
  - Uses OpenSmile or Kaldi to extract features from raw audio segment
  - Outputs the predicted features along with the audio segment id.
- Kinect2StreamsRecorder
  - Recording of all Kinect V2 Streams at 30 fps.
- Gesture and Action Recognition Module
  - The Gesture Recognition module is responsible for recognizing the gestures and actions that are performed by children both in real time as well as in offline tasks. 
-  Distant speech recognition module 
    - The Distant Speech Recognition module is employed for real time and continuous recognition of children’s speech (can also be used in offline tasks).
- AudioVisual Diarization module
  - The audiovisual diarization module receives input from the speaker localization module and the kinect API in order to increase both the accuracy of localization as well as achieve speaker diarization. 
- Speaker localization module
    - The speaker localization module is used in order to recognize the position/angle of a speaker so that the robot can face him/her while talking. 
- 3D Object Tracking Module
  - The Object Tracking module estimates the 6-DoF poses (positions and orientations) of a number of movable objects that the children are expected to interact with.
- Wizard-of-Oz
  - Wizard of Oz interface for autonomous, semi-autonomous, and manual mode. Allows the system to work in 3 modes: autonomous, semi autonomous and manual.
- Visual Emotion Recognition module 
  - The visual emotion recognition module can be used to recognize the emotion of a child based on its body posture and facial expressions. 
- Engagement Detection Module
  - Visual child engagement estimation.
- Speech-based Emotion recognition
  - The speech based emotion recognition module can be used to recognize the emotion of a child based on its speech.
- Text-based emotion recognition
  - Emotion recognition based on the child's speech (text).
- Text-based cognitive state recognition
  - Cognitive state recognition based on the child's speech (text).


The outputs are timestamped for all modules.

## System Architecture

### ROS Topics

All modules will publish the outputs in the respective ROS topic:
- `/audio`
- `/color`
- `/depth` 
- `/object-detect`
- `/asr`
- `/emotion-predict`
- `/speech-features`

### Message Format

The messages can be 
- JSON formatted strings (George +1)
- [ROS messages](http://wiki.ros.org/msg) 

Vote so that we can choose.

#### Audio

Message fields:
- ID
- Timestamp
- Clip duration
- base64 encoded audio segment

#### Color

Message fields:
- ID
- Timestamp
- Image dimensions
- base64 RGB image

#### Depth

Message fields:
- ID
- Timestamp
- Image dimentions
- base64 encoded depth image

#### Object Detection

Message fields:
- ID
- Timestamp
- Related frame ID
- List of `(object_label, bounding_box, proximity)` tuples

#### ASR

Message fields:
- ID
- Timestamp
- Related audio segment ID
- Transcribed text
- Lemmatized text??

#### Emotion Prediction

Message fields:
- ID
- Timestamp
- Emotion from visual modality
  - Related frame ID
  - List of tuples `(predicted_emotion, truth_value)`
- Emotion from audio modality
  - Related audio segment ID
  - List of tuples `(predicted_emotion, truth_value)`
- Combined prediction
  - List of tuples `(predicted_emotion, truth_value)`
  
#### Speech Features

Message fields:
- ID
- Timestamp
- Related audio segment ID
- Map of `{feature_name: extracted_features}`
