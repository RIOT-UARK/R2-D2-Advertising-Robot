//This code controls an AMB-82 mini microcontroller in R2's head. This AMB-82 mini runs an open source (https://www.amebaiot.com/en/amebapro2-arduino-neuralnework-face-detection/)
//TensorFlow Lite model for facial detection. The program then finds the closest (my measurement of width) face to the camera, and sends commands 
//to the dome motor to center on that face, providing interactability with R2. 

#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNFaceDetection.h"
#include "VideoStreamOverlay.h"
#include <AmebaServo.h>

#define CHANNEL   0
#define CHANNELNN 3

// Lower resolution for NN processing
#define NNWIDTH  576
#define NNHEIGHT 320

VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 20, VIDEO_RGB, 0);
NNFaceDetection facedet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerNN(1, 1);

//For RTSP streaming of video. Code left in here for testing/debugging purposes.
/*
char ssid[] = "Test0";    // your network SSID (name)
char pass[] = "DontPanic0042";       // your network password
int status = WL_IDLE_STATUS;
*/
AmebaServo servo;
float faceMiddle;        //Middle Pixel of the user's face
#define SERVO_PIN 1       //CONFIGURE CONFIGURE CONFIGURE

IPAddress ip;
int rtsp_portnum; 

void setup() {
    Serial.begin(115200);

    // attempt to connect to Wifi network:
  /*  while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);

        // wait 2 seconds for connection:
        delay(2000);
    }
    ip = WiFi.localIP(); */

    // Configure camera video channels with video format information
    // Adjust the bitrate based on your WiFi network quality
    config.setBitrate(1 * 1024 * 1024);     // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
    Camera.configVideoChannel(CHANNEL, config);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();

    // Configure RTSP with corresponding video format information
  //  rtsp.configVideo(config);
  //  rtsp.begin();
  //  rtsp_portnum = rtsp.getPort();

    // Configure face detection with corresponding video format information
    // Select Neural Network(NN) task and models
    facedet.configVideo(configNN);
    facedet.setResultCallback(FDPostProcess);
    facedet.modelSelect(FACE_DETECTION, NA_MODEL, DEFAULT_SCRFD, NA_MODEL);
    facedet.begin();

    // Configure StreamIO object to stream data from video channel to RTSP
  //  videoStreamer.registerInput(Camera.getStream(CHANNEL));
  //  videoStreamer.registerOutput(rtsp);
  //  if (videoStreamer.begin() != 0) {
  //      Serial.println("StreamIO link start failed");
  //  }

    // Start data stream from video channel
    //Camera.channelBegin(CHANNEL);

    // Configure StreamIO object to stream data from RGB video channel to face detection
    videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerNN.setStackSize();
    videoStreamerNN.setTaskPriority();
    videoStreamerNN.registerOutput(facedet);
    if (videoStreamerNN.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start video channel for NN
    Camera.channelBegin(CHANNELNN);

    // Start OSD drawing on RTSP video channel
    OSD.configVideo(CHANNEL, config);
    OSD.begin();

    servo.attach(SERVO_PIN); 
    //servo.write(71);
}

void loop() {
    // Do nothing
}

// User callback function for post processing of face detection results
void FDPostProcess(std::vector<FaceDetectionResult> results) {
    uint16_t im_h = config.height();
    uint16_t im_w = config.width();
    float maxSize = 0;
    int8_t maxIndex = -1;
    
    /*&
    Serial.print("Network URL for RTSP Streaming: ");
    Serial.print("rtsp://");
    Serial.print(ip);
    Serial.print(":");
    Serial.println(rtsp_portnum);
    Serial.println(" ");
    */

    //printf("Total number of faces detected = %d\r\n", facedet.getResultCount());
    OSD.createBitmap(CHANNEL);

    if (facedet.getResultCount() > 0) {
      float faceSize[facedet.getResultCount()];
        for (uint32_t i = 0; i < facedet.getResultCount(); i++) {
            FaceDetectionResult item = results[i];
            
            // Result coordinates are floats ranging from 0.00 to 1.00
            // Multiply with RTSP resolution to get coordinates in pixels
            int xmin = (int)(item.xMin() * im_w);
            int xmax = (int)(item.xMax() * im_w);
            int ymin = (int)(item.yMin() * im_h);
            int ymax = (int)(item.yMax() * im_h);

            faceSize[i] = xmax - xmin; 
            if (faceSize[i] > maxSize) { //Find face with the highest confidence
              maxSize = faceSize[i];
              maxIndex = i;
            }

            // Draw boundary box
            //printf("Face %d confidence %d:\t%d %d %d %d\n\r", i, item.score(), xmin, xmax, ymin, ymax);
            OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

            // Print identification text above boundary box
            char text_str[40];
            snprintf(text_str, sizeof(text_str), "%s %d", item.name(), item.score());
            OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN);

            // Draw facial feature points
            for (int j = 0; j < 5; j++) {
                int x = (int)(item.xFeature(j) * im_w);
                int y = (int)(item.yFeature(j) * im_h);
                OSD.drawPoint(CHANNEL, x, y, 8, OSD_COLOR_RED);
            }
        }

        if (maxIndex != -1) {
          faceMiddle = (results[maxIndex].xMin() + results[maxIndex].xMax()) / 2;
          /*
          Serial.print("Middle of face detected at X = ");
          Serial.println(faceMiddle);
          Serial.println(results[maxIndex].xMin());
          Serial.println(results[maxIndex].xMax());
          */
        
          if (faceMiddle > 0.63) {
            servo.write(84);
            Serial.println("S83");
          }
          else if (faceMiddle < 0.37) {
            servo.write(93);
            Serial.println("S94");
          }
          else {
          Serial.println("S89");
          servo.write(89);
        } 
        

        }
        
      

    }
    else {
      //Serial.println("SNO");
      servo.write(89);
    } 
    OSD.update(CHANNEL);
}
