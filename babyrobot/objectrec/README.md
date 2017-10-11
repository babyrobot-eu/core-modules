# How to test ObjectRec

### 1 - Run in debug mode
Go to `babyrobot/objectrec/config.py` and set `debug = True`. 
When the service runs in debug mode, it must show 3 images.
 1. The first is the image that the objectrec client captures from the camera.
At the moment, it just picks up a random image from the folder 
`babyrobot/objectrec/images`. When we hook the service with the camera, 
the only that is left for us to do, is to just change the `capture()` function,
in the client.
 2. The second image is the image that the server receives. It is displayed, 
  in order to verify that the communication works OK and that the image data,
  is serialized and deserialized properly.
 3. The third image is displayed from the client, 
 showing the recognized objects (as identified by the server), 
 with their bounding boxes, confidence and labels.
 
 
### 2 - Fire up the object recognition service
Just run `babyrobot/src/objectrec/src/objectrec_srv.py`
 
### 3 - Run the object recognition client
Just run `babyrobot/objectrec/client.py`, which will:
 1. capture the image
 2. make the request to the server (by sending the image to it)
 3. receive the recognized objects
 4. display the recognized objects, onto the original image
 5. exit
 
 
### How to evaluate the model

In order to evaluate the performance of the model, you can use  
`babyrobot/objectrec/eval_model.py`. This script, uses the object recognition
model, reads *all* the images in the directory 
`babyrobot/objectrec/images/_test_images`
and save the results (images with bounding boxes) to
`babyrobot/objectrec/images/_test_results`.