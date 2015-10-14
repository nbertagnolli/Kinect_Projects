
// Nicolas Bertagnolli
// Red object tracking using open kinect

// https://github.com/nbertagnolli/Kinect_Projects
// http://nbertagnolli.com

import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import blobDetection.*;



Kinect kinect;               //  Create Kinect object
BlobDetection blobDetector;  // Create a blob Detector

PImage currentImage, redImage;  // Images to display
PImage blobs;                   //An initial image to hold blobs

int threshold = 190;  // The threshold value for red channel
int filterSize = 2;  // Median filter box size 

boolean irState = false;    // Use IR camera
boolean medFilter = false;  // Use a median filter on the red channel

void setup() {
  size(1280, 500);  // double wide to show thresholding
  
  // Initialize the kinect and video
  kinect = new Kinect(this);
  kinect.initVideo();
  
  // Display the current image
  currentImage = kinect.getVideoImage();
  image(currentImage, 0, 0);
  
  // Create an image of the thresholded red channel
  redImage = currentImage.get();
  redImage = redThreshold(redImage, threshold);
  image(redImage, 640, 0);
  
  // Do blob detection and add bounding boxes with v3ga's blob detection http://www.v3ga.net/processing/BlobDetection/
  blobDetector = new BlobDetection(redImage.width, redImage.height);
  blobDetector.setThreshold(threshold / 255.0);  
  blobDetector.computeBlobs(redImage.pixels);
  
}

void draw() {
  
  // Grab/Display the current video image
  currentImage = kinect.getVideoImage();
  image(currentImage, 0, 0);
  
  // Create an image of the thresholded red channel
  redImage = currentImage.get();
  redImage = redThreshold(redImage, threshold);
  
  // Perform median filtering
  if(medFilter) {
    redImage = medianFilter(redImage, filterSize);
  }
  
  image(redImage, 640, 0);
  
  // Do blob detection and add bounding boxes with v3ga's blob detection http://www.v3ga.net/processing/BlobDetection/
  blobDetector.computeBlobs(redImage.pixels);
  drawBlobsAndEdges(redImage, true,false);
}


/**
* Thresholds and image based on the red channel
* @param img - The image to trheshold 
* @param threshold -  The value between 0 and 255 for which to threshold the image
* @return img - the thresholded image with 1's and 0's
**/
PImage redThreshold(PImage img, int threshold){
  img.loadPixels();
  
  // Step through every pixel and see if the redness > threshold
  for (int i = 0; i < img.width*img.height; i += 1) {
    if (calcRedVal(img.pixels[i]) >= threshold) { 
      // Set pixel to white
      img.pixels[i] = color(255, 255, 255);
    } else {
      // Set pixel to black
      img.pixels[i] = color(0, 0, 0);
    }
  }
  img.updatePixels();
  return img;
}


/**
* This method extracts the red channel from a color object using bit
* shifts.
* @param c - a color object for which to extract the red channel
* @return - the value of the red channel
**/
float calcRedVal(color c) {
 return  c >> 16 & 0xFF - (c >> 16 & 0xFF + c >> 8 & 0xFF + c & 0xFF) / 3;
}


/**
* 
* @param img  - The image to apply the median filter to
* @param size - The size of the filter creates a box of size  (2*size+1)^2 pixels
* @return img - The updated image with median filtering applied  
**/
PImage medianFilter(PImage img, int size) { 
  img.loadPixels();
  PImage temp;
  int[] tempPixels = img.get().pixels;
  
  // Step through every pixel in the image that is not a border pixel
  for(int y = size; y < img.height - size; y++) {
    for(int x = size; x < img.width - size; x++) {
      // Get a block of pixels of size (2*size+1)^2 around each 
      temp = img.get(x-size, y-size, 2*size+1, 2*size+1);
      // Find the median element
      tempPixels[y * img.width + x] = sort(temp.pixels)[(2*(2*size+1)-1) / 2];
    } 
  }
  
  // Update the pixels in the image
  img.pixels = tempPixels;
  img.updatePixels();
  return img; 
}


// ==================================================
// The below code performs blob detection and bounding
// bounding boxes.  It was taken from the examples in
// The v3ga library.  The comments below describe how
// it has been modified.
// ==================================================
void drawBlobsAndEdges(PImage img, boolean drawBlobs, boolean drawEdges)
{
  noFill();
  Blob b;
  EdgeVertex eA, eB;
  for (int n=0; n<blobDetector.getBlobNb(); n++) {
    b=blobDetector.getBlob(n);
    // Ignore all blobs that are less than 10% of image size
    if (b!=null && b.w >= .1 && b.h >= .1) {
      // Edges
      if (drawEdges) {
        strokeWeight(2);
        stroke(0, 255, 0);
        for (int m=0;m<b.getEdgeNb();m++) {
          eA = b.getEdgeVertexA(m);
          eB = b.getEdgeVertexB(m);
          if (eA !=null && eB !=null)
            line(eA.x*img.width, eA.y*img.height, eB.x*img.width, eB.y*img.height);
        }
      }

      // Blobs
      if (drawBlobs) {
        strokeWeight(1);
        stroke(255, 0, 0);
        // Use image width/height instead
        rect(b.xMin*img.width, b.yMin*img.height, b.w*img.width, b.h*img.height);
      }
    }
  }
}



/**
* 'w' -- increases the red threshold
* 's' -- decreases the red threshold
* 'i' -- switches between video and infrared
* 'm' -- applies median filtering to thresholded image
**/
void keyPressed() {
 if (key == 'w') { 
   threshold++;
   blobDetector.setThreshold(threshold / 255.0);
 } else if (key == 's') { 
   threshold--;
   blobDetector.setThreshold(threshold / 255.0);
 } else if (key  == 'i') {
   irState = !irState; 
   kinect.enableIR(irState);
 } else if (key == 'm') {
   medFilter = !medFilter; 
 }
 
}
