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
- JSON formatted strings
- [ROS messages](http://wiki.ros.org/msg) (George +1)

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
