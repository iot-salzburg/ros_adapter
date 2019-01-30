#include <cmath>
#include <iostream>
#include <thread>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "examples_common.h"
using namespace std;

    
    // Es müssen hier mehrer Positions abgefahren werden, um garantieren zu können, dass der Roboter nicht was zerdeppert
    // via Digital Cages (die im weiteren Verlauf des Projektes hinzugefügt werden können/sollten) kann das Problem
    // gelöst werden

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToInitialPosition(franka::Robot &robot, float speed, franka::Gripper &gripper){// Passt
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(speed, q_goal);
    bool notCorrectPosition = true;

    while(notCorrectPosition){
      try{
        robot.control(motion_generator);
        std::cout << "Moved robot to initial position" << std::endl;
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
          std::cout << "Exception jumped: " << exception_string << std::endl;
        }
      }
    }

    gripper.move(0.08, 0.1);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToPrinter(franka::Robot &robot, float speed){ // Passt
     bool notCorrectPosition = true;
     
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
 
    MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }   

  notCorrectPosition = true;

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
 
MotionGenerator motion_generator2(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator2);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }   
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromPrinter(franka::Robot &robot, float speed){ // Passt
     bool notCorrectPosition = true;
     
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
    MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
   return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToOutput(franka::Robot &robot, float speed){ // Passt
     bool notCorrectPosition = true;
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-2.596118,            // Joint 1
                -0.377315,            // Joint 2
                +2.351006,            // Joint 3
                -1.445618,           // Joint 4
                -0.761017,           // Joint 5
                +1.976400,            // Joint 6
                -0.890843             // Joint 7  
                }};
    
    MotionGenerator motion_generator3(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator3);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

// Position über dem Förderband
    q_goal = 
              {{-2.766183,            // Joint 1
                -1.121700,            // Joint 2
                +2.376853,            // Joint 3
                -1.638175,           // Joint 4
                -0.804047,           // Joint 5
                +2.683076,            // Joint 6
                -0.173219             // Joint 7  
                }};

    MotionGenerator motion_generator4(speed, q_goal);
    notCorrectPosition = true;

    while(notCorrectPosition){
        try{
            robot.control(motion_generator4);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

   return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromOutput(franka::Robot &robot, float speed){ // Passt
bool notCorrectPosition = true;
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-2.596118,            // Joint 1
                -0.377315,            // Joint 2
                +2.351006,            // Joint 3
                -1.445618,           // Joint 4
                -0.761017,           // Joint 5
                +1.976400,            // Joint 6
                -0.890843             // Joint 7  
                }};
    
    MotionGenerator motion_generator3(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator3);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToStorage(franka::Robot &robot, float speed){ // Passt
     bool notCorrectPosition = true;
   
    // Erste Position
/*    std::array<double, 7> q_goal = 
              {{+0.346378,            // Joint 1
                +1.459031,            // Joint 2
                -2.255549,            // Joint 3
                -1.965658,           // Joint 4
                +0.089137,           // Joint 5
                +2.261262,            // Joint 6
                -1.368361             // Joint 7  
                }}; 

    MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true; */

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
  
    MotionGenerator motion_generator3(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator3);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Alle 9 Funktionen, die für das Finden des richtigen Lagerplatzes gedacht sind, fahren folgendes Muster:
// *Von Regal schwebend zu Platz schwebend
// *Von Platz schwebend zu ablegen/aufnehmen
// *Von ablegen/aufnehmen zu Platz schwebend

bool findPlaceOne(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.516120,            // Joint 1
                +0.820823,            // Joint 2
                -1.837487,            // Joint 3
                -1.347663,           // Joint 4
                +0.079926,           // Joint 5
                +1.910257,            // Joint 6
                -1.434026             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
              q_goal = 
              {{-0.632862,            // Joint 1
                +0.793258,            // Joint 2
                -1.771385,            // Joint 3
                -1.542701,           // Joint 4
                +0.047447,           // Joint 5
                +2.234886,            // Joint 6
                -1.452819             // Joint 7  
                }};
 
  MotionGenerator motion_generator2(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator2);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}

bool leavePlaceOne(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.516120,            // Joint 1
                +0.820823,            // Joint 2
                -1.837487,            // Joint 3
                -1.347663,           // Joint 4
                +0.079926,           // Joint 5
                +1.910257,            // Joint 6
                -1.434026             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceTwo(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.256274,            // Joint 1
                +0.859337,            // Joint 2
                -2.170525,            // Joint 3
                -1.627315,           // Joint 4
                +0.280060,           // Joint 5
                +1.833001,            // Joint 6
                -1.646919             // Joint 7  
                }};
  
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.347059,            // Joint 1
                +0.844708,            // Joint 2
                -2.096429,            // Joint 3
                -1.847918,           // Joint 4
                +0.275905,           // Joint 5
                +2.172136,            // Joint 6
                -1.712208            // Joint 7  
                }};

    MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}

bool leavePlaceTwo(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.256274,            // Joint 1
                +0.859337,            // Joint 2
                -2.170525,            // Joint 3
                -1.627315,           // Joint 4
                +0.280060,           // Joint 5
                +1.833001,            // Joint 6
                -1.646919             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceThree(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.000185,            // Joint 1
                +0.976473,            // Joint 2
                -2.466721,            // Joint 3
                -1.934626,           // Joint 4
                +0.272974,           // Joint 5
                +1.871381,            // Joint 6
                -1.827909             // Joint 7  
                }};
  
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.089205,            // Joint 1
                +0.927569,            // Joint 2
                -2.378265,            // Joint 3
                -2.144840,           // Joint 4
                +0.262268,           // Joint 5
                +2.235057,            // Joint 6
                -1.830189            // Joint 7  
                }};
 
MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }  
return true;
}

void leavePlaceThree(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.000185,            // Joint 1
                +0.976473,            // Joint 2
                -2.466721,            // Joint 3
                -1.934626,           // Joint 4
                +0.272974,           // Joint 5
                +1.871381,            // Joint 6
                -1.827909             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSeven(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.428316,            // Joint 1
                +1.122033,            // Joint 2
                -1.603507,            // Joint 3
                -1.884648,           // Joint 4
                +0.362244,           // Joint 5
                +2.221434,            // Joint 6
                -1.200215             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.663267,            // Joint 1
                +1.130119,            // Joint 2
                -1.470890,            // Joint 3
                -1.948712,           // Joint 4
                +0.512178,           // Joint 5
                +2.588224,            // Joint 6
                -1.428654             // Joint 7  
                }};
 
MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}

bool leavePlaceSeven(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.428316,            // Joint 1
                +1.122033,            // Joint 2
                -1.603507,            // Joint 3
                -1.884648,           // Joint 4
                +0.362244,           // Joint 5
                +2.221434,            // Joint 6
                -1.200215             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceEight(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.145144,            // Joint 1
                +1.120037,            // Joint 2
                -1.758764,            // Joint 3
                -2.318438,           // Joint 4
                +0.466797,           // Joint 5
                +2.419395,            // Joint 6
                -1.436929             // Joint 7  
                }};

  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.391746,            // Joint 1
                +1.030484,            // Joint 2
                -1.582915,            // Joint 3
                -2.403232,           // Joint 4
                +0.616726,           // Joint 5
                +2.841888,            // Joint 6
                -1.605796             // Joint 7  
                }};
 
MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

return true;
}

bool leavePlaceEight(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.145144,            // Joint 1
                +1.120037,            // Joint 2
                -1.758764,            // Joint 3
                -2.318438,           // Joint 4
                +0.466797,           // Joint 5
                +2.419395,            // Joint 6
                -1.436929             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceNine(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{+0.222048,            // Joint 1
                +1.211415,            // Joint 2
                -2.023364,            // Joint 3
                -2.721475,           // Joint 4
                +0.524792,           // Joint 5
                +2.463421,            // Joint 6
                -1.656938             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.174449,            // Joint 1
                +1.002473,            // Joint 2
                -1.725353,            // Joint 3
                -2.705326,           // Joint 4
                +0.615969,           // Joint 5
                +2.954883,            // Joint 6
                -1.709142             // Joint 7  
                }};
 
MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }


return true;
}

void leavePlaceNine(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{+0.222048,            // Joint 1
                +1.211415,            // Joint 2
                -2.023364,            // Joint 3
                -2.721475,           // Joint 4
                +0.524792,           // Joint 5
                +2.463421,            // Joint 6
                -1.656938             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFour(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.427535,            // Joint 1
                +0.979701,            // Joint 2
                -1.802566,            // Joint 3
                -1.729782,           // Joint 4
                +0.396360,           // Joint 5
                +2.114495,            // Joint 6
                -1.510005             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.581477,            // Joint 1
                +1.065810,            // Joint 2
                -1.718917,            // Joint 3
                -1.883813,           // Joint 4
                +0.380246,           // Joint 5
                +2.482093,            // Joint 6
                -1.476558             // Joint 7  
                }};

MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }


return true;
}

void leavePlaceFour(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.427535,            // Joint 1
                +0.979701,            // Joint 2
                -1.802566,            // Joint 3
                -1.729782,           // Joint 4
                +0.396360,           // Joint 5
                +2.114495,            // Joint 6
                -1.510005             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceFive(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.064589,            // Joint 1
                +1.006084,            // Joint 2
                -2.079785,            // Joint 3
                -2.105988,           // Joint 4
                +0.302475,           // Joint 5
                +2.155265,            // Joint 6
                -1.555202             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.295086,            // Joint 1
                +0.921615,            // Joint 2
                -1.920725,            // Joint 3
                -2.197950,           // Joint 4
                +0.302841,           // Joint 5
                +2.526494,            // Joint 6
                -1.580266             // Joint 7  
                }};
   
MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

return true;
}

void leavePlaceFive(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{-0.064589,            // Joint 1
                +1.006084,            // Joint 2
                -2.079785,            // Joint 3
                -2.105988,           // Joint 4
                +0.302475,           // Joint 5
                +2.155265,            // Joint 6
                -1.555202             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findPlaceSix(franka::Robot &robot, float speed){

  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{+0.192788 ,            // Joint 1
                +1.172721,            // Joint 2
                -2.325774,            // Joint 3
                -2.439877,           // Joint 4
                +0.263223,           // Joint 5
                +2.199010,            // Joint 6
                -1.698004             // Joint 7  
                }};

  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
    q_goal = 
              {{-0.034831,            // Joint 1
                +1.023587,            // Joint 2
                -2.185950,            // Joint 3
                -2.526037,           // Joint 4
                +0.361866,           // Joint 5
                +2.577958,            // Joint 6
                -1.790957             // Joint 7  
                }};
  
MotionGenerator motion_generator7(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator7);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

return true;
}

void leavePlaceSix(franka::Robot &robot, float speed){
  bool notCorrectPosition = true;

     std::array<double, 7> q_goal = 
              {{+0.192788 ,            // Joint 1
                +1.172721,            // Joint 2
                -2.325774,            // Joint 3
                -2.439877,           // Joint 4
                +0.263223,           // Joint 5
                +2.199010,            // Joint 6
                -1.698004             // Joint 7  
                }};
 
  MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findRightSpot(franka::Robot &robot, float speed, int place){

  bool notCorrectPosition = true;

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
 
  MotionGenerator motion_generator6(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator6);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }


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
  bool notCorrectPosition = true;

        std::array<double, 7> q_goal = 
              {{-0.087449,            // Joint 1
                +1.312051,            // Joint 2
                -1.758231,            // Joint 3
                -2.331742,           // Joint 4
                +0.478561,           // Joint 5
                +2.410676,            // Joint 6
                -1.306896             // Joint 7  
                }}; 

MotionGenerator motion_generator6(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator6);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

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

  bool notCorrectPosition = true;

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

  MotionGenerator motion_generator6(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator6);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }
notCorrectPosition = true;
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

MotionGenerator motion_generator6(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator6);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

  notCorrectPosition = true;
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

    MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
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
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }
        }
    }

/*  notCorrectPosition = true;

    q_goal = 
              {{+0.346378,            // Joint 1
                +1.459031,            // Joint 2
                -2.255549,            // Joint 3
                -1.965658,           // Joint 4
                +0.089137,           // Joint 5
                +2.261262,            // Joint 6
                -1.368361             // Joint 7  
                }};
    MotionGenerator motion_generator2(speed, q_goal);    
    
    while(notCorrectPosition){
        try{
            robot.control(motion_generator2);
            std::cout << "Moved robot to up position (printer)" << std::endl;
            notCorrectPosition = false;
        }// Ende Try
        catch (const franka::Exception& ex) {
            string exception_string = ex.what();
            if((exception_string.rfind("discontinuity") == string::npos)
                && (exception_string.find("communication") == string::npos)){
                throw ex;
            }// Ende If
            else{
                robot.automaticErrorRecovery();
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }// Ende Else
        }//Ende Catch
    }// Ende While */

return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromPrinterToOutput(franka::Robot &robot, float speed, float grasping_width, franka::Gripper &gripper){

    moveToPrinter(robot, speed);

    // Grasp the log -> muss hier noch geändert werden (Weite passt noch nicht)
    //gripper.grasp(grasping_width, 0.05, 60);
    gripper.grasp(grasping_width, 0.05, 20);
    std::cout << "Log grasped" << std::endl;

    moveFromPrinter(robot, speed);
    moveToOutput(robot, speed);

// Es kann möglich sein, dass hier noch eine weitere Position eingefügt werden muss, bevor der Gripper geöffnet werden darf...

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

    // Grasp the log -> muss hier noch geändert werden (Weite passt noch nicht)
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

    // Grasp the log -> muss hier noch geändert werden (Weite passt noch nicht)
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  
  franka::Robot robot(argv[1]);
  float speed = 0.1;
  float grasping_width = 0.044;
  
  // Soll im weiteren Verlauf individuell geändert werden
  int place = 5;

  franka::Gripper gripper(argv[1]);

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

  try {
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
    
    //Experimente mit dem Gripper...
    //gripper.move(0.08, 0.05); //Voll aufmachen
    //gripper.grasp(0.072, 0.05, 20);
    //gripper.move(0.08, 0.05);
    std::cout << " " << std::endl;

// #######################################################################################################################################

    //getBlockFromPrinterToOutput(robot, speed, grasping_width, gripper);
    
    //Wenn Place dann via Input angegeben wird, muss hier eine Sicherheitsfunktion durchgeführt werden (Value 1- 9)
    if(!(place < 1 && place > 10)){
      getBlockFromPrinterToStorage(robot, speed, grasping_width, gripper, place);
      //getBlockFromStorageToOutput(robot, speed, grasping_width, gripper, place);
    }
// Nur zum Evaluieren notwendig (die unteren fünf Funktionen)
    //moveToStorage(robot, speed);
    //findRightSpot(robot, speed, place);
    //leaveRightSpot(robot, speed, place);
    //moveFromStorage(robot, speed, place);
    //moveToInitialPosition(robot, speed, gripper);

}
  catch (const franka::Exception& ex) {
  std::cerr << ex.what() << std::endl;
  std::cin.ignore();
  robot.automaticErrorRecovery();
}
  std::cout << "Press Any Key to close.." << std::endl;
  std::cin.ignore();
  return 0;
}
