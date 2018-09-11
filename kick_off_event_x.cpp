//      _____         __        __                               ____                                        __
//     / ___/ ____ _ / /____   / /_   __  __ _____ ____ _       / __ \ ___   _____ ___   ____ _ _____ _____ / /_ _
//     \__ \ / __ `// //_  /  / __ \ / / / // ___// __ `/      / /_/ // _ \ / ___// _ \ / __ `// ___// ___// __  /
//    ___/ // /_/ // /  / /_ / /_/ // /_/ // /   / /_/ /      / _, _//  __/(__  )/  __// /_/ // /   / /__ / / / /
//   /____/ \__,_//_/  /___//_.___/ \__,_//_/    \__, /      /_/ |_| \___//____/ \___/ \__,_//_/    \___//_/ /_/
//                                              /____/                                                         
// Salzburg Research ForschungsgesmbH
// IoT
//
// libfranka - Dominik Hofer & Michaela Muehlberger
// protobuf, control by arguments, connection to python opc-ua - Armin Niedermueller


// Panda DTZ Demo Controller


#include <cmath>
#include <iostream>
#include <thread>
#include <string>

#include <sys/socket.h>
#include <arpa/inet.h>

#include "dtz_robot_message.pb.h"
#include "examples_common.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>


using namespace std;


    // Es müssen hier mehrer Positions abgefahren werden, um garantieren zu können, dass der Roboter nicht was zerdeppert
    // via Digital Cages (die im weiteren Verlauf des Projektes hinzugefügt werden können/sollten) kann das Problem
    // gelöst werden

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Die eine Funktion, die alle Bewegungen berechnet/ausführt. Die Joint Positions sind variabel, weshalb sie immer je nach
// gewünschter Position/Regalplatz/etc. geändert werden müssen. Die Kommentare können gelöscht werden. Dienen nur als
// Hilfestellung.

void moveFunction(franka::Robot &robot, float speed, std::array<double, 7> q_goal){
    MotionGenerator motion_generator(speed, q_goal);
    bool notCorrectPosition = true;

    while(notCorrectPosition){
      try{
        robot.control(motion_generator);
        //std::cout << "Moved robot to initial position" << std::endl;
        notCorrectPosition = false;
      }
      catch (const franka::Exception& ex) {
        string exception_string = ex.what();
        if((exception_string.rfind("discontinuity") == string::npos)
          && (exception_string.find("communication") == string::npos)){
          throw ex;
        }
        else{
          robot.automaticErrorRecovery();
          //std::cout << "Exception jumped: " << exception_string << std::endl;
        }
      }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToInitialPosition(franka::Robot &robot, float speed, franka::Gripper &gripper){
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    moveFunction(robot, speed, q_goal);
    gripper.move(0.08, 0.1);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToPrinter(franka::Robot &robot, float speed){
     
    // Fängt an das Objekt vom Drucker zu heben
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-2.136935,            // Joint 1
                -0.829897,            // Joint 2
                +2.508779,            // Joint 3
                -1.023524,           // Joint 4
                +0.956515,           // Joint 5
                +0.849816,            // Joint 6
                +1.410623             // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);

    // Position direkt beim Drucker
    q_goal = 
              {{-2.021821,            // Joint 1
                -1.397909,            // Joint 2
                +2.402339,            // Joint 3
                -1.073298,           // Joint 4
                +1.213987,           // Joint 5
                +1.125178,            // Joint 6
                +1.118048             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromPrinter(franka::Robot &robot, float speed){
     
    // Fängt an das Objekt vom Drucker zu heben
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-2.136935,            // Joint 1
                -0.829897,            // Joint 2
                +2.508779,            // Joint 3
                -1.023524,           // Joint 4
                +0.956515,           // Joint 5
                +0.849816,            // Joint 6
                +1.410623             // Joint 7  
                }};
   
   moveFunction(robot, speed, q_goal);
   return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToOutput(franka::Robot &robot, float speed){
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-1.675980,            // Joint 1
                -0.067966 ,            // Joint 2
                +0.801821,            // Joint 3
                -1.808331,           // Joint 4
                -0.241695,           // Joint 5
                +2.056401,            // Joint 6
                -1.637750              // Joint 7  
                }};
    
    moveFunction(robot, speed, q_goal);

// Zweite Position
    q_goal = 
              {{-1.521378,            // Joint 1
                -0.678895,            // Joint 2
                +0.988494,            // Joint 3
                -2.171208,           // Joint 4
                +0.300746,           // Joint 5
                +1.933613,            // Joint 6
                -1.462331              // Joint 7  
                }};
    
    moveFunction(robot, speed, q_goal);

// Position über dem Förderband
    q_goal = 
              {{-2.600428,            // Joint 1
                -0.694723,            // Joint 2
                +2.300770,            // Joint 3
                -2.332446,           // Joint 4
                -1.497476,           // Joint 5
                +2.798764,           // Joint 6
                +0.365014             // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);
   return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromOutput(franka::Robot &robot, float speed){
   
    // Erste Position
   std::array<double, 7> q_goal = 
              {{-1.521378,            // Joint 1
                -0.678895,            // Joint 2
                +0.988494,            // Joint 3
                -2.171208,           // Joint 4
                +0.300746,           // Joint 5
                +1.933613,            // Joint 6
                -1.462331              // Joint 7  
                }};
    
    moveFunction(robot, speed, q_goal);
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToStorage(franka::Robot &robot, float speed){ // Passt

// Hier ne Position um vor dem Regal zu "Schweben"
    std::array<double, 7> q_goal = 
              {{-0.099726,            // Joint 1
                +1.237739,            // Joint 2
                -1.972287,            // Joint 3
                -2.136524,           // Joint 4
                +0.355537,           // Joint 5
                +2.225301,            // Joint 6
                -1.403869             // Joint 7  
                }};
  
    moveFunction(robot, speed, q_goal);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Alle 9 Funktionen, die für das Finden des richtigen Lagerplatzes gedacht sind, fahren folgendes Muster:
// *Von Regal schwebend zu Platz schwebend
// *Von Platz schwebend zu ablegen/aufnehmen
// *Von ablegen/aufnehmen zu Platz schwebend

bool findPlaceOne(franka::Robot &robot, float speed){


     std::array<double, 7> q_goal = 
              {{-0.516120,            // Joint 1
                +0.820823,            // Joint 2
                -1.837487,            // Joint 3
                -1.347663,           // Joint 4
                +0.079926,           // Joint 5
                +1.910257,            // Joint 6
                -1.434026             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

              q_goal = 
              {{-0.632862,            // Joint 1
                +0.793258,            // Joint 2
                -1.771385,            // Joint 3
                -1.542701,           // Joint 4
                +0.047447,           // Joint 5
                +2.234886,            // Joint 6
                -1.452819             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

return true;
}

bool leavePlaceOne(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.516120,            // Joint 1
                +0.820823,            // Joint 2
                -1.837487,            // Joint 3
                -1.347663,           // Joint 4
                +0.079926,           // Joint 5
                +1.910257,            // Joint 6
                -1.434026             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);
return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceTwo(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.256274,            // Joint 1
                +0.859337,            // Joint 2
                -2.170525,            // Joint 3
                -1.627315,           // Joint 4
                +0.280060,           // Joint 5
                +1.833001,            // Joint 6
                -1.646919             // Joint 7  
                }};
  
    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.337015,            // Joint 1
                +0.925032,            // Joint 2
                -2.101915,            // Joint 3
                -1.935825,           // Joint 4
                +0.227259,           // Joint 5
                +2.343030,            // Joint 6
                -1.684558            // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);

return true;
}

bool leavePlaceTwo(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.256274,            // Joint 1
                +0.859337,            // Joint 2
                -2.170525,            // Joint 3
                -1.627315,           // Joint 4
                +0.280060,           // Joint 5
                +1.833001,            // Joint 6
                -1.646919             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceThree(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.000185,            // Joint 1
                +0.976473,            // Joint 2
                -2.466721,            // Joint 3
                -1.934626,           // Joint 4
                +0.272974,           // Joint 5
                +1.871381,            // Joint 6
                -1.827909             // Joint 7  
                }};
  
    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.135728,            // Joint 1
                +1.045247,            // Joint 2
                -2.334923,            // Joint 3
                -2.235124,           // Joint 4
                +0.262659,           // Joint 5
                +2.419076,            // Joint 6
                -1.839854            // Joint 7  
                }};
  
    moveFunction(robot, speed, q_goal);

return true;
}

void leavePlaceThree(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.000185,            // Joint 1
                +0.976473,            // Joint 2
                -2.466721,            // Joint 3
                -1.934626,           // Joint 4
                +0.272974,           // Joint 5
                +1.871381,            // Joint 6
                -1.827909             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSeven(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.428316,            // Joint 1
                +1.122033,            // Joint 2
                -1.603507,            // Joint 3
                -1.884648,           // Joint 4
                +0.362244,           // Joint 5
                +2.221434,            // Joint 6
                -1.200215             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.663267,            // Joint 1
                +1.130119,            // Joint 2
                -1.470890,            // Joint 3
                -1.948712,           // Joint 4
                +0.512178,           // Joint 5
                +2.588224,            // Joint 6
                -1.428654             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

return true;
}

bool leavePlaceSeven(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.428316,            // Joint 1
                +1.122033,            // Joint 2
                -1.603507,            // Joint 3
                -1.884648,           // Joint 4
                +0.362244,           // Joint 5
                +2.221434,            // Joint 6
                -1.200215             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceEight(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.145144,            // Joint 1
                +1.120037,            // Joint 2
                -1.758764,            // Joint 3
                -2.318438,           // Joint 4
                +0.466797,           // Joint 5
                +2.419395,            // Joint 6
                -1.436929             // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.391746,            // Joint 1
                +1.030484,            // Joint 2
                -1.582915,            // Joint 3
                -2.403232,           // Joint 4
                +0.616726,           // Joint 5
                +2.841888,            // Joint 6
                -1.605796             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

return true;
}

bool leavePlaceEight(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.145144,            // Joint 1
                +1.120037,            // Joint 2
                -1.758764,            // Joint 3
                -2.318438,           // Joint 4
                +0.466797,           // Joint 5
                +2.419395,            // Joint 6
                -1.436929             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceNine(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{+0.222048,            // Joint 1
                +1.211415,            // Joint 2
                -2.023364,            // Joint 3
                -2.721475,           // Joint 4
                +0.524792,           // Joint 5
                +2.463421,            // Joint 6
                -1.656938             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.174449,            // Joint 1
                +1.002473,            // Joint 2
                -1.725353,            // Joint 3
                -2.705326,           // Joint 4
                +0.615969,           // Joint 5
                +2.954883,            // Joint 6
                -1.709142             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

return true;
}

void leavePlaceNine(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{+0.222048,            // Joint 1
                +1.211415,            // Joint 2
                -2.023364,            // Joint 3
                -2.721475,           // Joint 4
                +0.524792,           // Joint 5
                +2.463421,            // Joint 6
                -1.656938             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFour(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.427535,            // Joint 1
                +0.979701,            // Joint 2
                -1.802566,            // Joint 3
                -1.729782,           // Joint 4
                +0.396360,           // Joint 5
                +2.114495,            // Joint 6
                -1.510005             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.581477,            // Joint 1
                +1.065810,            // Joint 2
                -1.718917,            // Joint 3
                -1.883813,           // Joint 4
                +0.380246,           // Joint 5
                +2.482093,            // Joint 6
                -1.476558             // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);


return true;
}

void leavePlaceFour(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.427535,            // Joint 1
                +0.979701,            // Joint 2
                -1.802566,            // Joint 3
                -1.729782,           // Joint 4
                +0.396360,           // Joint 5
                +2.114495,            // Joint 6
                -1.510005             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFive(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.064589,            // Joint 1
                +1.006084,            // Joint 2
                -2.079785,            // Joint 3
                -2.105988,           // Joint 4
                +0.302475,           // Joint 5
                +2.155265,            // Joint 6
                -1.555202             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.295086,            // Joint 1
                +0.921615,            // Joint 2
                -1.920725,            // Joint 3
                -2.197950,           // Joint 4
                +0.302841,           // Joint 5
                +2.526494,            // Joint 6
                -1.580266             // Joint 7  
                }};
   
    moveFunction(robot, speed, q_goal);

return true;
}

void leavePlaceFive(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{-0.064589,            // Joint 1
                +1.006084,            // Joint 2
                -2.079785,            // Joint 3
                -2.105988,           // Joint 4
                +0.302475,           // Joint 5
                +2.155265,            // Joint 6
                -1.555202             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSix(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{+0.045320,            // Joint 1
                +1.277773,            // Joint 2
                -2.265168,            // Joint 3
                -2.379450,           // Joint 4
                +0.353567,           // Joint 5
                +2.044280,            // Joint 6
                -1.691176             // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);

    q_goal = 
              {{-0.034831,            // Joint 1
                +1.023587,            // Joint 2
                -2.185950,            // Joint 3
                -2.526037,           // Joint 4
                +0.361866,           // Joint 5
                +2.577958,            // Joint 6
                -1.790957             // Joint 7  
                }};
  
    moveFunction(robot, speed, q_goal);

return true;
}

void leavePlaceSix(franka::Robot &robot, float speed){

     std::array<double, 7> q_goal = 
              {{+0.045320,            // Joint 1
                +1.277773,            // Joint 2
                -2.265168,            // Joint 3
                -2.379450,           // Joint 4
                +0.353567,           // Joint 5
                +2.044280,            // Joint 6
                -1.691176             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findRightSpot(franka::Robot &robot, float speed, int place){


// Hier die Abfrage
// wenn 1 - 3: oberes Regal
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnen
// wenn 4 - 6: keine Änderung
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnen
// wenn 7 - 9: unteres Regal fahren
//    auf Position des jeweiligen Plates fahren (davor schweben)
//    auf Ablageposition des Plates fahren und Gripper öffnenbool notCorrectPosition = true;
   

if (place < 4) {
// Hier Position für erstes Fach
        std::array<double, 7> q_goal = 
              {{-0.202298,            // Joint 1
                +1.118617,            // Joint 2
                -2.215788,            // Joint 3
                -1.792656,           // Joint 4
                +0.262851,           // Joint 5
                +1.940611,            // Joint 6
                -1.581900             // Joint 7  
                }};
 
    moveFunction(robot, speed, q_goal);


  if(place == 1) {
    findPlaceOne(robot, speed);
  }
  else if(place == 2) {
  // Hier Position für zweiten Platz
    findPlaceTwo(robot, speed);
  }
  else{
  // Hier Position für dritten Platz
    findPlaceThree(robot, speed);
  }
}
else if (place > 6) {
// Hier Position für drittes Fach

        std::array<double, 7> q_goal = 
              {{-0.087449,            // Joint 1
                +1.312051,            // Joint 2
                -1.758231,            // Joint 3
                -2.331742,           // Joint 4
                +0.478561,           // Joint 5
                +2.410676,            // Joint 6
                -1.306896             // Joint 7  
                }}; 

    moveFunction(robot, speed, q_goal);

  if(place == 7) {
    findPlaceSeven(robot, speed);  

  }
  else if(place == 8) {
    findPlaceEight(robot, speed);

  }
  else{
    findPlaceNine(robot, speed);

  }
}
else {

  if(place == 4) {
  findPlaceFour(robot, speed);
  }
  else if(place == 5) {
  findPlaceFive(robot, speed);
  }
  else{
  findPlaceSix(robot, speed);
  }
}
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool leaveRightSpot(franka::Robot &robot, float speed, int place){

if (place < 4) {
  if(place == 1) {
    leavePlaceOne(robot, speed);
  }
  else if(place == 2) {
    leavePlaceTwo(robot, speed);
  }
  else{
    leavePlaceThree(robot, speed);
  }
}
else if (place > 6) {
  if(place == 7) {
    leavePlaceSeven(robot, speed);  
  }
  else if(place == 8) {
    leavePlaceEight(robot, speed);
  }
  else{
    leavePlaceNine(robot, speed);
  }
}
else {

  if(place == 4) {
  leavePlaceFour(robot, speed);
  }
  else if(place == 5) {
  leavePlaceFive(robot, speed);
  }
  else{
  leavePlaceSix(robot, speed);
  }
}
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromStorage(franka::Robot &robot, float speed, int place){

if (place < 4) {
// Hier Position für erstes Fach
        std::array<double, 7> q_goal=
              {{-0.202298,            // Joint 1
                +1.118617,            // Joint 2
                -2.215788,            // Joint 3
                -1.792656,           // Joint 4
                +0.262851,           // Joint 5
                +1.940611,            // Joint 6
                -1.581900             // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);

}
else if (place > 6) {
// Hier Position für drittes Fach
    std::array<double, 7> q_goal=
              {{-0.087449,            // Joint 1
                +1.312051,            // Joint 2
                -1.758231,            // Joint 3
                -2.331742,           // Joint 4
                +0.478561,           // Joint 5
                +2.410676,            // Joint 6
                -1.306896             // Joint 7  
                }};     

    moveFunction(robot, speed, q_goal);

}

// In Schwebeposition zurück fahren
    std::array<double, 7> q_goal= 
              {{-0.099726,            // Joint 1
                +1.237739,            // Joint 2
                -1.972287,            // Joint 3
                -2.136524,           // Joint 4
                +0.355537,           // Joint 5
                +2.225301,            // Joint 6
                -1.403869             // Joint 7  
                }};

    moveFunction(robot, speed, q_goal);

return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromPrinterToOutput(franka::Robot &robot, float speed, float grasping_width, franka::Gripper &gripper){

    moveToPrinter(robot, speed);

    gripper.grasp(grasping_width, 0.05, 20);
    std::cout << "Log grasped" << std::endl;

    moveFromPrinter(robot, speed);
    moveToOutput(robot, speed);

  // Objekt wird in die "Freiheit" entlassen
  gripper.stop();
  gripper.move(0.08, 0.01);
  moveFromOutput(robot, speed);

  moveToInitialPosition(robot, speed, gripper);

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool getBlockFromPrinterToStorage(franka::Robot &robot, float speed, float grasping_width, franka::Gripper &gripper, int place){

    moveToPrinter(robot, speed);

    gripper.grasp(grasping_width, 0.4, 60);
    std::cout << "Log grasped" << std::endl;

    moveFromPrinter(robot, speed);
    moveToStorage(robot, speed);
    findRightSpot(robot, speed, place);

    gripper.stop();
    gripper.move(0.08, 0.1);
    leaveRightSpot(robot, speed, place);

    moveFromStorage(robot, speed, place);

    moveToInitialPosition(robot, speed, gripper);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromStorageToOutput(franka::Robot &robot, float speed, float grasping_width, franka::Gripper &gripper, int place){

  moveToStorage(robot, speed);
  findRightSpot(robot, speed, place);

    gripper.grasp(grasping_width, 0.4, 60);
    std::cout << "Log grasped" << std::endl;

  leaveRightSpot(robot, speed, place);
  moveFromStorage(robot, speed, place);
  moveToOutput(robot, speed);

  // Objekt wird in die "Freiheit" entlassen
  gripper.stop();
  gripper.move(0.08, 0.1);

  moveFromOutput(robot, speed);
  moveToInitialPosition(robot, speed, gripper);

  return true;
}


////////////////////////////////////////////////////  M A I N  /////////////////////////////////////////////////////////////////////////

// argv[0]
// argv[1] = IP-address
// argv[2] = movement   - "PS"      // Printer to Storage
//                      - "SO"      // Storage to Output
//                      - "PO"      // Printer to Output
// argv[3] = Place in Storage - "1" - "9"


int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << " <movement> " << " <place> " << std::endl;
    std::cerr << "<movement> -> \"PS\", \"SO\", \"PO\"" << std::endl;
    std::cerr << "<place> - > 1-9" << std::endl;
    std::cout << argc;
    return -1;
  } else if (!(strcmp(argv[2],"PS") || strcmp(argv[2], "SO") || strcmp(argv[3], "PO"))){
      std::cerr << "<movement> only valids are PS, SO, PO" << std::endl;
      return -1;
  }

  int iplace = atoi(argv[3]);

//   //// P R O T O B U F - C O M  ////
//   struct sockaddr_in addr;

//   addr.sin_family = AF_INET;
//   inet_aton("127.0.0.1", &addr.sin_addr);
//   addr.sin_port = htons(5555);

//   //// P R O T O B U F - I N I T ////
//   GOOGLE_PROTOBUF_VERIFY_VERSION;
//   prototest::RobotMessage robot_message;
//   std::string buf;
//   int sock = socket(PF_INET, SOCK_DGRAM, 0  );

//   // Set instance members
//   robot_message.set_id(4);
//   robot_message.set_state("init");

//   // send via protobuf
//   robot_message.SerializeToString(&buf);
//   sendto(sock, buf.data(), strlen(buf.c_str()), 0, (struct sockaddr *)&addr, sizeof(addr));

  /////////////////////////////////  I N I T  /////////////////////////////////////////
  franka::Robot robot(argv[1]);
  float speed = 0.1;
  float grasping_width = 0.044;

  franka::Gripper gripper(argv[1]);

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();


  /////////////////////////////////  S T A R T  ////////////////////////////////////////
  try {

    //// I N I T ////
    setDefaultBehavior(robot);
    robot.setFilters(1000, 1000, 1000, 1000, 1000);
    moveToInitialPosition(robot, speed, gripper);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    gripper.homing();

    // // PROTOBUF SEND
    // robot_message.set_state("moving");
    // robot_message.SerializeToString(&buf);
    // sendto(sock, buf.data(), strlen(buf.c_str()), 0, (struct sockaddr *)&addr, sizeof(addr));

    //// CHECK & MOVE ////
    if(!(strcmp(argv[2],"PS"))){
        if (iplace < 1 && iplace >9) { std::cerr << "<place> only valid from numbers 1-9" << std::endl; return -1;}
        getBlockFromPrinterToStorage(robot, speed, grasping_width, gripper, iplace);
    } else if(!(strcmp(argv[2], "SO"))){
        if (iplace < 1 && iplace >9) { std::cerr << "<place> only valid from numbers 1-9" << std::endl; return -1;}
        getBlockFromStorageToOutput(robot, speed,grasping_width, gripper, iplace);
    } else if(!(strcmp(argv[2],"PO"))){
        getBlockFromPrinterToOutput(robot, speed, grasping_width, gripper);
    }

    // // PROTOBUF SEND
    // robot_message.set_state("stop");
    // robot_message.SerializeToString(&buf);
    // sendto(sock, buf.data(), strlen(buf.c_str()), 0, (struct sockaddr *)&addr, sizeof(addr));

}
  catch (const franka::Exception& ex) {
  std::cerr << ex.what() << std::endl;
  std::cin.ignore();
  robot.automaticErrorRecovery();
}
  // std::cout << "Press Any Key to close.." << std::endl;
  // std::cin.ignore();
  return 0;
}
