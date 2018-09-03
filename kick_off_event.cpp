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

bool moveToInitialPosition(franka::Robot &robot, float speed, franka::Gripper &gripper){
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

bool moveToPrinter(franka::Robot &robot, float speed){
     bool notCorrectPosition = true;
     
    // Fängt an das Objekt vom Drucker zu heben
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-0.041735,            // Joint 1
                +0.657479,            // Joint 2
                +0.839581,            // Joint 3
                -1.654211 ,           // Joint 4
                +0.383426 ,           // Joint 5
                +1.655406,            // Joint 6
                -1.303061             // Joint 7  
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
              {{+0.270922,            // Joint 1
                +0.680191,            // Joint 2
                +0.593400,            // Joint 3
                -2.094262,           // Joint 4
                -0.890771,           // Joint 5
                +2.599196,            // Joint 6
                -0.844511             // Joint 7  
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

bool moveFromPrinter(franka::Robot &robot, float speed){
     bool notCorrectPosition = true;
     
    // Fängt an das Objekt vom Drucker zu heben
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-0.041735,            // Joint 1
                +0.657479,            // Joint 2
                +0.839581,            // Joint 3
                -1.654211 ,           // Joint 4
                +0.383426 ,           // Joint 5
                +1.655406,            // Joint 6
                -1.303061             // Joint 7  
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

bool moveToOutput(franka::Robot &robot, float speed){
     bool notCorrectPosition = true;
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{-0.679752,            // Joint 1
                +0.543608,            // Joint 2
                +1.150953,            // Joint 3
                -1.414210,           // Joint 4
                -1.927951,           // Joint 5
                +1.068629,            // Joint 6
                -0.621254             // Joint 7  
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
              {{-0.772128,            // Joint 1
                +1.319637,            // Joint 2
                +0.939751,            // Joint 3
                -1.490945,           // Joint 4
                -2.378254,           // Joint 5
                +1.157728,            // Joint 6
                -0.081711             // Joint 7  
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

bool getBlockFromPrinterToOutput(franka::Robot &robot, float speed, float grasping_width, franka::Gripper &gripper){

    moveToPrinter(robot, speed);

    // Grasp the log -> muss hier noch geändert werden (Weite passt noch nicht)
    //gripper.grasp(grasping_width, 0.05, 60);
    gripper.grasp(0.072, 0.05, 20);
    std::cout << "Log grasped" << std::endl;

    moveFromPrinter(robot, speed);
    moveToOutput(robot, speed);

// Es kann möglich sein, dass hier noch eine weitere Position eingefügt werden muss, bevor der Gripper geöffnet werden darf...

  // Objekt wird in die "Freiheit" entlassen
  gripper.move(0.08, 0.01);

  moveToInitialPosition(robot, speed, gripper);

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveToStorage(franka::Robot &robot, float speed){
     bool notCorrectPosition = true;
   
    // Erste Position
    std::array<double, 7> q_goal = 
              {{+0.255231,            // Joint 1
                +0.736121,            // Joint 2
                -2.428646,            // Joint 3
                -1.240482,           // Joint 4
                +0.139873,           // Joint 5
                +2.332304,            // Joint 6
                +1.511716             // Joint 7  
                }}; 

    MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
            notCorrectPosition = false;
        }// Ende TRY
        catch (const franka::Exception& ex) {
            string exception_string = ex.what();
            if((exception_string.rfind("discontinuity") == string::npos)
                && (exception_string.find("communication") == string::npos)){
                throw ex;
            }// Ende IF
            else{
                robot.automaticErrorRecovery();
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }// Ende Else
        }// Ende Catch
    }//Ende While

  notCorrectPosition = true;

// Hier ne Position um vor dem Regal zu "Schweben"
    q_goal = 
              {{-0.107643,            // Joint 1
                +1.580617,            // Joint 2
                -1.846718,            // Joint 3
                -2.296587,           // Joint 4
                +0.149223,           // Joint 5
                +2.367729 ,            // Joint 6
                +1.983285             // Joint 7  
                }};
  
    MotionGenerator motion_generator3(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator3);
            std::cout << "Moved robot to up position (printer)" << std::endl;
            notCorrectPosition = false;
        }//Ende Try
        catch (const franka::Exception& ex) {
            string exception_string = ex.what();
            if((exception_string.rfind("discontinuity") == string::npos)
                && (exception_string.find("communication") == string::npos)){
                throw ex;
            }//Ende If
            else{
                robot.automaticErrorRecovery();
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }//Ende Else
        }//Ende Catch
    }//Ende Try
    return true;
}// Ende Funktion

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findRightSpot(franka::Robot &robot, float speed, int place){

  bool notCorrectPosition = true;

// Nur Beispielimplementierung von q_goal
        std::array<double, 7> q_goal = 
              {{-0.413757,            // Joint 1
                +1.437414,            // Joint 2
                -2.100041,            // Joint 3
                -2.084699,           // Joint 4
                +0.129800,           // Joint 5
                +2.488492,            // Joint 6
                +1.722656             // Joint 7  
                }};
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
        q_goal = 
              {{-0.416011,            // Joint 1
                +1.501606,            // Joint 2
                -2.172211,            // Joint 3
                -1.987772,           // Joint 4
                +0.257009,           // Joint 5
                +2.265558,            // Joint 6
                +1.620679             // Joint 7  
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

  if(place == 1) {
  // Hier Position für ersten Platz
    q_goal = 
              {{-0.763770,            // Joint 1
                +1.439958,            // Joint 2
                -2.046877,            // Joint 3
                -1.620580,           // Joint 4
                +0.407060,           // Joint 5
                +2.278924,            // Joint 6
                +1.663505             // Joint 7  
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

  }
  else if(place == 2) {
  // Hier Position für zweiten Platz
    q_goal = 
              {{-0.622803,            // Joint 1
                +1.580107,            // Joint 2
                -2.149075,            // Joint 3
                -2.024942,           // Joint 4
                +0.604625,           // Joint 5
                +2.376974,            // Joint 6
                +1.410306            // Joint 7  
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
  }
  else{
  // Hier Position für dritten Platz
    q_goal = 
              {{-0.46254,            // Joint 1
                +1.619764,            // Joint 2
                -2.220176,            // Joint 3
                -2.345686,           // Joint 4
                +0.597662,           // Joint 5
                +2.483568,            // Joint 6
                +1.273108            // Joint 7  
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
  }
}
else if (place > 6) {
// Hier Position für drittes Fach
        q_goal = 
              {{-0.084923 ,            // Joint 1
                +1.734353,            // Joint 2
                -1.635213,            // Joint 3
                -2.358153,           // Joint 4
                +0.478339,           // Joint 5
                +2.358477,            // Joint 6
                +1.970922             // Joint 7  
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

  if(place == 7) {
  // Hier Position für erstes Platz
    q_goal = 
              {{-0.570026,            // Joint 1
                +1.698135,            // Joint 2
                -1.620691,            // Joint 3
                -1.925982,           // Joint 4
                +0.512478,           // Joint 5
                +2.426016,            // Joint 6
                +1.998905             // Joint 7  
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

  }
  else if(place == 8) {
  // Hier Position für zweites Platz
    q_goal = 
              {{-0.330804,            // Joint 1
                +1.698171,            // Joint 2
                -1.550879,            // Joint 3
                -2.334420,           // Joint 4
                +0.580390,           // Joint 5
                +2.593001,            // Joint 6
                +1.925377             // Joint 7  
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
  }
  else{
  // Hier Position für drittes Platz
    q_goal = 
              {{-0.093650,            // Joint 1
                +1.604140,            // Joint 2
                -1.493705,            // Joint 3
                -2.743767,           // Joint 4
                +0.681858,           // Joint 5
                +2.784748,            // Joint 6
                +1.785662            // Joint 7  
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
  }
}
else {

  if(place == 4) {
  // Hier Position für erstes Fach
        std::array<double, 7> q_goal = 
              {{-0.702617,            // Joint 1
                +1.713534,            // Joint 2
                -1.927999,            // Joint 3
                -1.829149,           // Joint 4
                +0.716279,           // Joint 5
                +2.293291,            // Joint 6
                +1.646584             // Joint 7  
                }}; 

    MotionGenerator motion_generator8(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator8);
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
  else if(place == 5) {
  // Hier Position für zweites Fach
    q_goal = 
              {{-0.480943,            // Joint 1
                +1.712293,            // Joint 2
                -1.890701,            // Joint 3
                -2.210220,           // Joint 4
                +0.763592,           // Joint 5
                +2.440438,            // Joint 6
                +1.498864             // Joint 7  
                }};
 
    MotionGenerator motion_generator8(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator8);
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
  else{ //Fach Sechs
  // Hier Position für drittes Fach
    q_goal = 
              {{-0.240080,            // Joint 1
                +1.719448,            // Joint 2
                -1.907602,            // Joint 3
                -2.576539,           // Joint 4
                +0.800656,           // Joint 5
                +2.531141,            // Joint 6
                +1.344329            // Joint 7  
                }}; 

    MotionGenerator motion_generator8(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator8);
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
}
return true;
}
//Hier muss noch ein Fehler sein, weil { passt irgendwie nicht...


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool moveFromStorage(franka::Robot &robot, float speed, int place){

  bool notCorrectPosition = true;

std::array<double, 7> q_goal = 
              {{-0.107643,            // Joint 1
                +1.580617,            // Joint 2
                -1.846718,            // Joint 3
                -2.296587,           // Joint 4
                +0.149223,           // Joint 5
                +2.367729 ,            // Joint 6
                +1.983285             // Joint 7  
                }};


if (place < 4) {
// Hier Position für erstes Fach
        q_goal = 
              {{-0.416011,            // Joint 1
                +1.501606,            // Joint 2
                -2.172211,            // Joint 3
                -1.987772,           // Joint 4
                +0.257009,           // Joint 5
                +2.265558,            // Joint 6
                +1.620679             // Joint 7  
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
        q_goal = 
              {{-0.084923 ,            // Joint 1
                +1.734353,            // Joint 2
                -1.635213,            // Joint 3
                -2.358153,           // Joint 4
                +0.478339,           // Joint 5
                +2.358477,            // Joint 6
                +1.970922             // Joint 7  
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
           q_goal = 
              {{-0.107643,            // Joint 1
                +1.580617,            // Joint 2
                -1.846718,            // Joint 3
                -2.296587,           // Joint 4
                +0.149223,           // Joint 5
                +2.367729 ,            // Joint 6
                +1.983285             // Joint 7  
                }};

    MotionGenerator motion_generator1(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator1);
            std::cout << "Moved robot to up position (printer)" << std::endl;
            notCorrectPosition = false;
        } //Ende Try
        catch (const franka::Exception& ex) {
            string exception_string = ex.what();
            if((exception_string.rfind("discontinuity") == string::npos)
                && (exception_string.find("communication") == string::npos)){
                throw ex;
            } //Ende If
            else{
                robot.automaticErrorRecovery();
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }// Ende Else
        }// Ende Catch
    }// Ende While

  notCorrectPosition = true;

// In Sicherheitsposition 2 zurück fahren
    q_goal = 
              {{+0.697996,            // Joint 1
                +1.725434,            // Joint 2
                -1.798190,            // Joint 3
                -2.389757,           // Joint 4
                +0.166944,           // Joint 5
                +2.365814,            // Joint 6
                +2.023952             // Joint 7 
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
    }// Ende While

  notCorrectPosition = true;

// In Sicherheitsposition 1 zurück fahren

    // Erste Position
   q_goal = 
              {{+0.255231,            // Joint 1
                +0.736121,            // Joint 2
                -2.428646,            // Joint 3
                -1.240482,           // Joint 4
                +0.139873,           // Joint 5
                +2.332304,            // Joint 6
                +1.511716             // Joint 7  
                }}; 

    MotionGenerator motion_generator3(speed, q_goal);

    while(notCorrectPosition){
        try{
            robot.control(motion_generator3);
            std::cout << "Moved robot to up position (printer)" << std::endl;
            notCorrectPosition = false;
        }//Ende Try
        catch (const franka::Exception& ex) {
            string exception_string = ex.what();
            if((exception_string.rfind("discontinuity") == string::npos)
                && (exception_string.find("communication") == string::npos)){
                throw ex;
            }//Ende if
            else{
                robot.automaticErrorRecovery();
                std::cout << "Exception jumped: " << exception_string << std::endl;
            }//Ende Else
        }//Ende Catch
    }//Ende While
return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromPrinterToStorage(franka::Robot &robot, float speed, float grasping_width, franka::Gripper &gripper, int place){
    
    moveToPrinter(robot, speed);

    // Grasp the log -> muss hier noch geändert werden (Weite passt noch nicht)
    gripper.grasp(0.072, 0.4, 60);
    std::cout << "Log grasped" << std::endl;

    moveFromPrinter(robot, speed);
    moveToStorage(robot, speed);
    findRightSpot(robot, speed, place);

    gripper.stop();
    gripper.move(0.08, 0.1); 
    
    moveFromStorage(robot, speed, place);

// Zurück in die Ausgangsposition
    moveToInitialPosition(robot, speed, gripper);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool getBlockFromStorageToOutput(franka::Robot &robot, float speed, float grasping_width, franka::Gripper &gripper, int place){
   
  moveToStorage(robot, speed);
  findRightSpot(robot, speed, place);

    // Grasp the log -> muss hier noch geändert werden (Weite passt noch nicht)
    gripper.grasp(0.072, 0.4, 60);
    std::cout << "Log grasped" << std::endl;

  moveFromStorage(robot, speed, place);
  moveToOutput(robot, speed);

// Es kann möglich sein, dass hier noch eine weitere Position eingefügt werden muss, bevor der Gripper geöffnet werden darf...

  // Objekt wird in die "Freiheit" entlassen
  gripper.stop();  
  gripper.move(0.08, 0.1);

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
  float speed = 0.04;
  float grasping_width = 0.072;
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
    
    int place = 1;
    // Wenn Place dann via Input angegeben wird, muss hier eine Sicherheitsfunktion durchgeführt werden (Value 1- 9)
    //getBlockFromPrinterToStorage(robot, speed, grasping_width, gripper, place);
    getBlockFromStorageToOutput(robot, speed, grasping_width, gripper, place);

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
