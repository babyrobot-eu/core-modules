# BabyRobot Integration

## Integrated Services

- Audio Stream
  - A raw audio stream as captured by the front facing Kinect or the mic. 
  - Outputs a byte64 encoded audio segment.
  - Dummy segmentation by timing (e.g. publish a new message every 10 seconds).
- Color Stream
  - A color image stream as captured by the front facing Kinect.
  - Outputs byte64 encoded image frames.
- Depth Stream
  - A depth image stream as captured by the front facing Kinect.
  - Outputs byte64 depth frames.
- Object Recognition Module
  - A stream of the detected objects in the current image.
  - Outputs the bounding box an the label for every detected object.
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
