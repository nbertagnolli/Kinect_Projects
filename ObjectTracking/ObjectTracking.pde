
import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import blobDetection.*;



Kinect kinect;  //  Create Kinect object
BlobDetection blobDetector;  // Create a blob Detector

PImage currentImage, redImage;  // Images to display
PImage blobs;  //An initial image to hold blobs

int threshold = 190;

boolean irState = false;
boolean medFilter = false;

void setup() {
  //frameRate(40);    // Set framerate 
  size(1280, 520);  // double wide to show thresholding
  
  kinect = new Kinect(this);
  kinect.initVideo();
  
  currentImage = kinect.getVideoImage();
  image(currentImage, 0, 0);
  redImage = currentImage.get();
  redImage = redThreshold(redImage, threshold);
  image(redImage, 640, 0);
  
  blobDetector = new BlobDetection(redImage.width, redImage.height);
  blobDetector.setThreshold(threshold / 255.0);
  
  blobDetector.computeBlobs(redImage.pixels);
  
}

void draw() {
  currentImage = kinect.getVideoImage();
  image(currentImage, 0, 0);
  redImage = currentImage.get();
  redImage = redThreshold(redImage, threshold);
  image(redImage, 640, 0);
  blobDetector.computeBlobs(redImage.pixels);
  drawBlobsAndEdges(redImage, true,false);
  //image(currentImage, 640 , 0);
}


PImage redThreshold(PImage img, int threshold){
  img.loadPixels();
  for (int i = 0; i < img.width*img.height; i += 1) {
    if (calcRedVal(img.pixels[i]) >= threshold) { 
      img.pixels[i] = color(255, 255, 255);
    } else {
      img.pixels[i] = color(0, 0, 0);
    }
  }
  img.updatePixels();
  return img;
}

float calcRedVal(color c) {
 return  c >> 16 & 0xFF - (c >> 16 & 0xFF + c >> 8 & 0xFF + c & 0xFF) / 3;
}




// ==================================================
// drawBlobsAndEdges()
// ==================================================
void drawBlobsAndEdges(PImage img, boolean drawBlobs, boolean drawEdges)
{
  noFill();
  Blob b;
  EdgeVertex eA, eB;
  for (int n=0 ; n<blobDetector.getBlobNb() ; n++)
  {
    b=blobDetector.getBlob(n);
    if (b!=null && b.w >= .1)
    {
      // Edges
      if (drawEdges)
      {
        strokeWeight(2);
        stroke(0, 255, 0);
        for (int m=0;m<b.getEdgeNb();m++)
        {
          eA = b.getEdgeVertexA(m);
          eB = b.getEdgeVertexB(m);
          if (eA !=null && eB !=null)
            line(
            eA.x*img.width, eA.y*img.height, 
            eB.x*img.width, eB.y*img.height
              );
        }
      }

      // Blobs
      if (drawBlobs)
      {
        strokeWeight(1);
        stroke(255, 0, 0);
        rect(
        b.xMin*img.width, b.yMin*img.height, 
        b.w*img.width, b.h*img.height
          );
      }
    }
  }
}




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
 }
 
}

