//
//  many_Myos.cpp
//  Ours
//
//  Created by Taylor Henderson on 2/7/15.
//  Copyright (c) 2015 Taylor Henderson. All rights reserved.
//

#include "many_Myos.h"
// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to interface with multiple Myo armbands and distinguish between them.

#include <iostream>
#include <stdexcept>
#include <vector>

//////I added
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <myo/myo.hpp>

#include <fstream>
#include <iomanip>
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    // Every time Myo Connect successfully pairs with a Myo armband, this function will be called.
    //
    // You can rely on the following rules:
    //  - onPair() will only be called once for each Myo device
    //  - no other events will occur involving a given Myo device before onPair() is called with it
    //
    // If you need to do some kind of per-Myo preparation before handling events, you can safely do it in onPair().
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
    {
        // Print out the MAC address of the armband we paired with.
        
        // The pointer address we get for a Myo is unique - in other words, it's safe to compare two Myo pointers to
        // see if they're referring to the same Myo.
        
        // Add the Myo pointer to our list of known Myo devices. This list is used to implement identifyMyo() below so
        // that we can give each Myo a nice short identifier.
        knownMyos.push_back(myo);
        
        roll_w.push_back(0);
        pitch_w.push_back(0);
        yaw_w.push_back(0);
        onArm.push_back(false);
        isUnlocked.push_back(false);
        currentPose.push_back(myo::Pose::unknown);
        
        
        
        // Now that we've added it to our list, get our short ID for it and print it out.
        std::cout << "Paired with " << identifyMyo(myo) << "." << std::endl;
    }
    
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        long curMyo = identifyMyo(myo);
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w[curMyo] = 0;
        pitch_w[curMyo] = 0;
        yaw_w[curMyo] = 0;
        onArm[curMyo] = false;
        isUnlocked[curMyo] = false;
    }
    
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        long curMyo = identifyMyo(myo);
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
	int temp0 = yaw_w[0];
	int temp1 = yaw_w[1];
	// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        // Convert the floating point angles in radians to a scale from 0 to 18.
        roll_w[curMyo] = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w[curMyo] = /*static_cast<int>*/(pitch);//((pitch + (float)M_PI/2.0f)/M_PI * 18);
        if (scale == 1) {
            yaw_w[curMyo] = static_cast<int>((yaw*10)/3.5)-2;//static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        }
        if (scale == 2) {
            yaw_w[curMyo] = static_cast<int>((yaw*10)/3.5)-2;//static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        }
	if (scale == 3) {
	    yaw_w[curMyo] = static_cast<int>((yaw*10)/3.5);//static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
	}
	if (scale == 4) {
	    yaw_w[curMyo] = static_cast<int>((yaw*10)/2.15)-1;//static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
	}
	if (scale == 6) {
	  yaw_w[curMyo] = static_cast<int>((yaw*10)/3.5);//static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
	}
        else{
            yaw_w[curMyo] = static_cast<int>((yaw*10)/3.5)-2;//static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        }
	//int theD1 = temp1-yaw_w[1];
	//	int theD0 = temp0-yaw_w[0];
	
        if (scale != 7 && scale != 8 && pitch_w[1] < -0.4 && pitch_w[1] > -0.5 && !struming) {
            struming = true;
            strum();
            //std::cout << "Strum"<< std::endl;
        }
        else if(scale != 7 && scale != 8 && (pitch_w[1]> -0.4 || pitch_w[1]< -0.5)){
            struming = false;
        }
	if ((scale == 7 || scale == 8 )  &&  pitch_w[1] < -0.5 && !struming) {
	  struming = true;
	  strum();
	  //std::cout << "Strum"<< std::endl;
	}
	else if((scale == 7 || scale == 8 )  && pitch_w[1]> -0.5){
	    struming = false;
	}
	if(scale == 7){
	  if ( pitch_w[0] < -0.5 && !struming2) {
	    struming2 = true;
	    strum();
	    //std::cout << "Strum"<< std::endl;
	  }
	  else if( pitch_w[0]> -0.5){
	    struming2 = false;
	  }
	}
        //mine
        //notes
        //std::cout<<pitch_w[1]<<std::endl;
    }
    
    void strum(){
      //      std::cout << "Strum "<<std::endl;
        
        if (scale == 0) {
            if (yaw_w[0] < 0.1) {
                playSound("C4.wav");
            } else if(yaw_w[0] == 1){
                playSound("A#.wav");
            } else if(yaw_w[0] == 2){
                playSound("G.wav");
            } else if(yaw_w[0] == 3){
                playSound("F#.wav");
            } else if(yaw_w[0] == 4){
                playSound("F.wav");
            } else if(yaw_w[0] == 5){
                playSound("D#.wav");
            } else {
                playSound("C.wav");
            }
        }
        if (scale == 1) {
	  if (yaw_w[0] < 0.1) {
	    playSound("F4.wav");
	  } else if(yaw_w[0] == 1){
	    playSound("D#4.wav");
	  } else if(yaw_w[0] == 2){
	    playSound("C.wav");
	  } else if(yaw_w[0] == 3){
	    playSound("B.wav");
	  } else if(yaw_w[0] == 4){
	    playSound("A#.wav");
	  } else if(yaw_w[0] == 5){
	    playSound("G#.wav");
	  } else {
	    playSound("F.wav");
	  }
	}
	if (scale == 2) {
	  if (yaw_w[0] < 0.1) {
	    playSound("G4.wav");
	  } else if(yaw_w[0] == 1){
	    playSound("F4.wav");
	  } else if(yaw_w[0] == 2){
	    playSound("D4.wav");
	  } else if(yaw_w[0] == 3){
	    playSound("C#4.wav");
	  } else if(yaw_w[0] == 4){
	    playSound("C.wav");
	  } else if(yaw_w[0] == 5){
	    playSound("A#.wav");
	  } else {
	    playSound("G.wav");
	  }
	}
	if (scale == 3 && !cords) {
	  if (yaw_w[0] < 0.1) {
	    playSound("C4.wav");
	  } else if(yaw_w[0] == 1){
	    playSound("B.wav");
	  } else if(yaw_w[0] == 2){
	    playSound("A.wav");
	  } else if(yaw_w[0] == 3){
	    playSound("G.wav");
	  } else if(yaw_w[0] == 4){
	    playSound("F.wav");
	  } else if(yaw_w[0] == 5){
	    playSound("E.wav");
	  } else if(yaw_w[0] == 6){
	    playSound("D.wav");
	  } else {
	    playSound("C.wav");
	  }
	}
	if (scale == 3 && cords) {
	  if (yaw_w[0] < 0.1) {
	    playSound("C4chord.wav");
	  } else if(yaw_w[0] == 1){
	    playSound("Achord.wav");
	  } else if(yaw_w[0] == 2){
	    playSound("Gchord.wav");
	  } else if(yaw_w[0] == 3){
	    playSound("Fchord.wav");
	  } else if(yaw_w[0] == 4){
	    playSound("Echord.wav");
	  } else if(yaw_w[0] == 5){
	    playSound("Dchord.wav");
	  } else {
	    playSound("Cchord.wav");
	  }
	}
	if (scale == 4) {
	  if (yaw_w[0] < 0.1) {
	    playSound("C4.wav");
	  } else if(yaw_w[0] == 1){
	    playSound("B.wav");
	  } else if(yaw_w[0] == 2){
	    playSound("A#.wav");
	  } else if(yaw_w[0] == 3){
	    playSound("A.wav");
	  } else if(yaw_w[0] == 4){
	    playSound("G#.wav");
	  } else if(yaw_w[0] == 5){
	    playSound("G.wav");
	  } else if(yaw_w[0] == 6){
	    playSound("F#.wav");
	  } else if(yaw_w[0] == 7){
	    playSound("F.wav");
	  } else if(yaw_w[0] == 8){
	    playSound("E.wav");
	  } else if(yaw_w[0] == 9){
	    playSound("D#.wav");
	  } else if(yaw_w[0] == 10){
	    playSound("D.wav");
	  } else if(yaw_w[0] == 11){
	    playSound("C#.wav");
	  } else {
	    playSound("C.wav");
	  }
	}
	if(scale == 5){
	  playSound("openingchord.wav");
	  canChange = true;
	}
	if (scale == 6) {
	  if (yaw_w[0] < 0.1) {
	    playSound("C4a.wav");
	  } else if(yaw_w[0] == 1){
	    playSound("Ba.wav");
	  } else if(yaw_w[0] == 2){
	    playSound("Aa.wav");
	  } else if(yaw_w[0] == 3){
	    playSound("Ga.wav");
	  } else if(yaw_w[0] == 4){
	    playSound("Fa.wav");
	  } else if(yaw_w[0] == 5){
	    playSound("Ea.wav");
	  } else if(yaw_w[0] == 6){
	    playSound("Da.wav");
	  } else {
	    playSound("Ca.wav");
	  }
	}
	if (scale == 7 && struming2) {
	  if (yaw_w[1] < 0.1) {
	    playSound("lowC4.wav");
	  } else if(yaw_w[1] == 1){
	    playSound("lowB.wav");
	  } else if(yaw_w[1] == 2){
	    playSound("lowA.wav");
	  } else if(yaw_w[1] == 3){
	    playSound("lowG.wav");
	  } else if(yaw_w[1] == 4){
	    playSound("lowF.wav");
	  } else if(yaw_w[1] == 5){
	    playSound("lowE.wav");
	  } else if(yaw_w[1] == 6){
	    playSound("lowD.wav");
	  } else {
	    playSound("lowC.wav");
	  }
	}
	if (scale == 7 && !struming2) {
	  if (yaw_w[1] < 0.1) {
	    playSound("C47.wav");
	  } else if(yaw_w[1] == 1){
	    playSound("B7.wav");
	  } else if(yaw_w[1] == 2){
	    playSound("A7.wav");
	  } else if(yaw_w[1] == 3){
	    playSound("G7.wav");
	  } else if(yaw_w[1] == 4){
	    playSound("F7.wav");
	  } else if(yaw_w[1] == 5){
	    playSound("E7.wav");
	  } else if(yaw_w[1] == 6){
	    playSound("D7.wav");
	  } else {
	    playSound("C7.wav");
	  }
	}
	if (scale == 8 && rickc<14) {
	  playSound(rick[rickc]);
	  rickc = rickc + 1;
	}
    }
    void playSound(std::string fname){
      std::ofstream myfile;
      if(scale == 0){
	std::cout << "C Blues" << std::endl<< std::endl << std::endl;
      }else if(scale == 1){
	std::cout << "F Blues" << std::endl<< std::endl<< std::endl;
      }else if(scale == 2){
	std::cout << "G Blues" << std::endl<< std::endl<< std::endl;
      }else if(scale == 3){
	std::cout << "C Major" << std::endl<< std::endl<< std::endl;
      }else if(scale == 4){
	std::cout << "Chromatic" << std::endl<< std::endl<< std::endl;
      }else if(scale == 5){
	std::cout << "Opening" << std::endl<< std::endl<< std::endl;
      }else if(scale == 6){
	std::cout << "Acoustic" << std::endl<< std::endl<< std::endl;
      }else if(scale == 7){
	std::cout << "Piano" << std::endl<< std::endl<< std::endl;
      }else if(scale == 8){
	std::cout << "Piano 2" << std::endl<< std::endl<< std::endl;
      }
      
      if(cords){
	std::cout << "Chords On" << std::endl<< std::endl<< std::endl;
      }else{
	std::cout << "Chords Off" << std::endl<< std::endl<< std::endl;
      }

      std::cout << fname.substr(0,fname.length()-4)  << std::endl<< std::endl<< std::endl;
      
      myfile.open(std::to_string(noteCnt));
      myfile << fname;
      myfile.close();
      noteCnt = noteCnt + 1;
    }
    
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        //std::cout << "Myo " << identifyMyo(myo) << " switched to pose " << pose.toString() << "." << std::endl;
        
        long curMyo = identifyMyo(myo);
        currentPose[curMyo] = pose;
        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);
            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
            
            
//          //mine
            if(curMyo == 1 && pose == myo::Pose::fist){
                //strum both ways
                m_both = true;
		//                std::cout << "Both" << std::endl;
            }
            if(curMyo == 1 && pose == myo::Pose::fingersSpread){
                //strum both ways
                //sound.stop();
                //std::cout << "Stop" << std::endl;
		//playSound("null");
            }
            if(curMyo == 0 &&( pose == myo::Pose::waveIn || pose == myo::Pose::fist)){
                //strum both ways
                cords = true;
                //std::cout << "Cords on" << std::endl;
            }
            if(curMyo == 0 && pose != myo::Pose::waveIn && pose != myo::Pose::fist){
                //strum both ways
                cords = false;
                //std::cout << "Cords off" << std::endl;
            }
            if(curMyo == 1 && pose == myo::Pose::waveOut){
                //strum both ways
                if (scale != 8) {
                    scale = scale + 1;
                }
                else{
                    scale = 0;
                }
                //std::cout << "Scale " << scale << std::endl;
            }
            if(curMyo == 1 && pose == myo::Pose::waveIn){
                //strum both ways
                if (scale != 0) {
                    scale = scale - 1;
                }
                else{
                    scale = 8;
                }
                //std::cout << "Scale " << scale << std::endl;
            }
        } else {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            myo->unlock(myo::Myo::unlockTimed);
        }
    }
    
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
    {
        std::cout << "Myo " << identifyMyo(myo) << " has connected." << std::endl;
    }
    
    void onDisconnect(myo::Myo* myo, uint64_t timestamp)
    {
        std::cout << "Myo " << identifyMyo(myo) << " has disconnected." << std::endl;
    }
    
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        long curMyo = identifyMyo(myo);
        whichArm.push_back(arm);
        onArm[curMyo] = true;
    }
    
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        long curMyo = identifyMyo(myo);
        onArm[curMyo] = false;
    }
    
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        long curMyo = identifyMyo(myo);
        isUnlocked[curMyo] = true;
    }
    
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        long curMyo = identifyMyo(myo);
        isUnlocked[curMyo] = false;
    }
    
    // This is a utility function implemented for this sample that maps a myo::Myo* to a unique ID starting at 1.
    // It does so by looking for the Myo pointer in knownMyos, which onPair() adds each Myo into as it is paired.
    size_t identifyMyo(myo::Myo* myo) {
        // Walk through the list of Myo devices that we've seen pairing events for.
        for (size_t i = 0; i < knownMyos.size(); ++i) {
            // If two Myo pointers compare equal, they refer to the same Myo device.
            if (knownMyos[i] == myo) {
                return i;
            }
        }
        
        return 0;
    }
    
    // We store each Myo pointer that we pair with in this list, so that we can keep track of the order we've seen
    // each Myo and give it a unique short identifier (see onPair() and identifyMyo() above).
    static std::vector<myo::Myo*> knownMyos;
    
    // These values are set by onArmSync() and onArmUnsync() above.
    std::vector<bool> onArm;
    std::vector<myo::Arm> whichArm;
    // This is set by onUnlocked() and onLocked() above.
    std::vector<bool> isUnlocked;
    // These values are set by onOrientationData() and onPose() above.
    std::vector<float> roll_w, pitch_w, yaw_w;
    std::vector<myo::Pose> currentPose;
    static bool m_both;
    static bool cords;
    static int scale;
    static bool struming;
    static int noteCnt;
    static bool canChange;
    static std::string rick[];
    static int rickc;
    static bool struming2;
};
bool
DataCollector::struming2 = false;
int
DataCollector::rickc = 0;
bool
DataCollector::canChange = false;
std::vector<myo::Myo*> 
DataCollector::knownMyos;
std::string
DataCollector::rick[] = {"rick1.wav", "rick2.wav", "rick3.wav", "rick4.wav", "rick5.wav", "rick6.wav", "rick7.wav", "rick8.wav", "rick9.wav", "rick10.wav", "rick11.wav", "rick12.wav", "rick13.wav", "rick14.wav"};
bool
DataCollector::m_both = false;
bool
DataCollector::cords = false;
int
DataCollector::scale = 6;
bool
DataCollector::struming = false;
int
DataCollector::noteCnt = 0;

int main(int argc, char** argv)
{

    try {
        myo::Hub hub("com.example.multiple-myos");
        
        // Instantiate the PrintMyoEvents class we defined above, and attach it as a listener to our Hub.
        DataCollector printer;
        hub.addListener(&printer);
        
        while (1) {
            // Process events for 10 milliseconds at a time.
            hub.run(10);
	    if(DataCollector::canChange){
	      DataCollector::knownMyos[0]->unlock(myo::Myo::unlockHold);
	      DataCollector::knownMyos[1]->unlock(myo::Myo::unlockHold);
	    }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
