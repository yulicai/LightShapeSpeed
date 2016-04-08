
/*
 Thomas Sanchez Lengeling.
 http://codigogenerativo.com/
 
 KinectPV2, Kinect for Windows v2 library for processing
 
 3D Skeleton.
 Some features a not implemented, such as orientation
 */


//import libraries to work with
import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;

//first skeleton
KJoint[] joints1;
int trailingJointIndex1;
ArrayList<PVector>trail1 = new ArrayList<PVector>();

//second skeleton
KJoint[] joints2;
int trailingJointIndex2;
ArrayList<PVector>trail2 = new ArrayList<PVector>();

float diag;
float scaleAmount = 500;
boolean invert =false;

void setup() {
  //size(1024, 768, P3D);
  fullScreen(P2D);
  smooth();
  rectMode(CENTER);
  diag = sqrt(sq(width)+sq(height))/2;
  kinect = new KinectPV2(this);

  //enable 3d  with (x,y,z) position
  kinect.enableSkeleton3DMap(true);
  kinect.init();
}


PVector trailingJointPosition_1= new PVector(0, 0);
PVector trailingJointPosition_2 = new PVector(0, 0);
PVector trailingJointPosition_3;

PVector pTrailingJointPosition_1 = new PVector(0, 0);
PVector pTrailingJointPosition_2 = new PVector(0, 0);

PVector leftHandSpeed = new PVector(0, 0);
float leftHandSpeedMag =0;

PVector invertSpeed = new PVector(0, 0);
float invertSpeedMag =0;

void draw() {
  if (!invert) {
    background(0);
  } else {
    background(255);
  }  /*
  /*Translate the scene to the center 
   Scale it x scaleAmount
   Rotate it around the x-axis 180-degrees
   */
  pushMatrix();
  translate(width/2, height/2, 0);
  scale(scaleAmount);
  rotateX(PI);

  ArrayList<KSkeleton> skeletonArray1 =  kinect.getSkeleton3d();
  //second skeleton
  ArrayList<KSkeleton> skeletonArray2 =  kinect.getSkeleton3d();

  // Get the skeleton
  if (skeletonArray1.size() > 0) {
    KSkeleton skeleton1 = (KSkeleton) skeletonArray1.get(0);
    if (skeleton1.isTracked()) {
      joints1 = skeleton1.getJoints();      

      //Set which joint is drawing
      // Get the index number for the joint
      //left hand
      int trailingJointIndex_1 = getTrailingJointIndex(1);
      //right hand
      int trailingJointIndex_2 = getTrailingJointIndex(2);
      //head
      int trailingJointIndex_3 = getTrailingJointIndex(3);

      // Retrieve the joint using the index number
      KJoint trailingJoint_1 = joints1[trailingJointIndex_1];
      KJoint trailingJoint_2 = joints1[trailingJointIndex_2];
      KJoint trailingJoint_3 = joints1[trailingJointIndex_3];

      // Get the PVector containing the xyz position of the joint
      trailingJointPosition_1 = trailingJoint_1.getPosition().copy();
      trailingJointPosition_2 = trailingJoint_2.getPosition().copy();
      trailingJointPosition_3 = trailingJoint_3.getPosition().copy();



      leftHandSpeed = pTrailingJointPosition_1.sub(trailingJointPosition_1);
      leftHandSpeedMag = leftHandSpeed.mag();
      //println(leftHandSpeedMag);

      invertSpeed = pTrailingJointPosition_2.sub(trailingJointPosition_2);
      invertSpeedMag = invertSpeed.mag();
      if (invertSpeedMag > 0.27) {
        invert = !invert;
      }// else {
      //invert = false;
      //}
      for (int x = 0; x < width; x+=40) {
        for (int y = 0; y < height; y+=40) {
          float trailingJointX =width-( width/2+trailingJointPosition_1.x*scaleAmount);
          float trailingJointY =  height-(trailingJointPosition_1.y*scaleAmount+height/2);
          float d = dist(x, y, trailingJointX, trailingJointY);
          float f = map(d, 0, diag, 255, 0);
          fill(f);
          //stroke(255,20);
          if (invert) {
            stroke(0, 20);
          } else {
            stroke(255, 200);
          }          
          strokeWeight(3/scaleAmount);
          float s = map(d, 0, diag, 40, 0);
          float rectSize = s/scaleAmount;
          rect(x/scaleAmount, y/scaleAmount, rectSize, rectSize);
        }
      }


      //Draw joints
      float strokeColor = map(leftHandSpeedMag, 0, 0.6, 255, 0);
      //map the speed of one of our joints to the size of the joint rect
      float strokeSize = map(leftHandSpeedMag, 0, 0.6, 0, 300);
      stroke(strokeColor);
      strokeWeight(strokeSize/scaleAmount);
      drawJoints();
      //get the previous PVector of the joints
      pTrailingJointPosition_1 = trailingJoint_1.getPosition().copy();
    }
  }
  pTrailingJointPosition_1 = trailingJointPosition_1;
  pTrailingJointPosition_2 = trailingJointPosition_2;

  popMatrix();
}

int getTrailingJointIndex(int index) {

  //Bust
  //return KinectPV2.JointType_Head;
  //return KinectPV2.JointType_Neck;
  //return KinectPV2.JointType_SpineShoulder;

  //Torso
  //return KinectPV2.JointType_SpineMid;
  //return KinectPV2.JointType_SpineBase;

  // Right Arm    
  //return KinectPV2.JointType_ShoulderRight;
  //return KinectPV2.JointType_ElbowRight;
  //return KinectPV2.JointType_WristRight;

  if (index == 1) {
    return KinectPV2.JointType_HandLeft;
  } else if (index == 2) {
    return KinectPV2.JointType_KneeRight;
  } else if (index ==3) {
    return KinectPV2.JointType_Head;
  } else {
    return 0;
  }


  //return KinectPV2.JointType_HandTipRight;
  //return KinectPV2.JointType_ThumbRight;

  // Left Arm
  //return KinectPV2.JointType_ShoulderLeft;
  //return KinectPV2.JointType_ElbowLeft;
  //return KinectPV2.JointType_WristLeft;
  //// return KinectPV2.JointType_HandLeft;
  //return KinectPV2.JointType_HandTipLeft;
  //return KinectPV2.JointType_ThumbLeft;

  // Right Leg
  //return KinectPV2.JointType_HipRight;
  //return KinectPV2.JointType_KneeRight;
  //return KinectPV2.JointType_AnkleRight;
  //return KinectPV2.JointType_FootRight;

  // Left Leg
  //return KinectPV2.JointType_HipLeft;
  //return KinectPV2.JointType_KneeLeft;
  //return KinectPV2.JointType_AnkleLeft;
  //return KinectPV2.JointType_FootLeft;
}

void drawJoint(int jointType) {
  KJoint joint = joints1[jointType];
  PVector jointPosition = joint.getPosition();
  float py = height/scaleAmount-(jointPosition.y+height/(2*scaleAmount));
  float px = width/scaleAmount-(jointPosition.x+width/(2*scaleAmount));
  //point(px, py, jointPosition.z);
  float fillColor = map(leftHandSpeedMag, 0, 0.6, 0, 255);
  println(fillColor);
  fill(fillColor);
  rect(px, py-200/scaleAmount, leftHandSpeed.x/scaleAmount, leftHandSpeed.y/scaleAmount);
}

void drawJoints() {
  //Bust
  drawJoint(KinectPV2.JointType_Head);
  //drawJoint(KinectPV2.JointType_Neck);
  //drawJoint(KinectPV2.JointType_SpineShoulder);

  //Torso
  // drawJoint(KinectPV2.JointType_SpineMid);
  // drawJoint(KinectPV2.JointType_SpineBase);

  // Right Arm    
  // drawJoint(KinectPV2.JointType_ShoulderRight);
  // drawJoint(KinectPV2.JointType_ElbowRight);
  //drawJoint(KinectPV2.JointType_WristRight);
  drawJoint(KinectPV2.JointType_HandRight);
  // drawJoint(KinectPV2.JointType_HandTipRight);
  //drawJoint(KinectPV2.JointType_ThumbRight);

  // Left Arm
  //drawJoint(KinectPV2.JointType_ShoulderLeft);
  // drawJoint(KinectPV2.JointType_ElbowLeft);
  //drawJoint(KinectPV2.JointType_WristLeft);
  drawJoint(KinectPV2.JointType_HandLeft);
  //drawJoint(KinectPV2.JointType_HandTipLeft);
  //drawJoint(KinectPV2.JointType_ThumbLeft);

  // Right Leg
  // drawJoint(KinectPV2.JointType_HipRight);
  // drawJoint(KinectPV2.JointType_KneeRight);
  // drawJoint(KinectPV2.JointType_AnkleRight);
  drawJoint(KinectPV2.JointType_FootRight);

  // Left Leg
  // drawJoint(KinectPV2.JointType_HipLeft);
  //drawJoint(KinectPV2.JointType_KneeLeft);
  //drawJoint(KinectPV2.JointType_AnkleLeft);
  drawJoint(KinectPV2.JointType_FootLeft);
}