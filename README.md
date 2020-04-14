# a-brief-tutorial-on-opencv

1. Learn about basic image processing operations with OpenCV (i.e., resizing, 
rotation, blurring, etc.).
2. Discover how to count basic object shapes using contours.
3. Perform image subtraction.
4. Utilize OpenCV for face and object detection.

1. **basics.py**: Contains our implementation of basic image processing 
operations so we can use them to get hands-on experience.
2. **count_shapes.py**: Performs shape counting via OpenCV and contour/outline 
detection.
3. **image_sub.py**: Uses image background subtraction to segment the background 
of a scene from the foreground.
4. **detect_faces.py**: Detects faces in video streams.

The .xml file in the project structure is Haar cascade that has been trained by 
the OpenCV library and then serialized to disk.

We’ll load the model and then use it to perform face detection inside 
detect_faces.py.

The images/ directory contains various images that we’ll be using as examples in 
this chapter.
