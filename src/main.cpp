/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../src/main_window.hpp"
#include "roswindowwrapper.h"
#include <iostream>
#include <string.h>
#include "../src/rosnode.h"
#ifndef Q_MOC_RUN
#include "koordinator.hpp"
#endif
#include "wartend.h"
#include "verabschiedend.h"

#include <stdio.h>
#include "Bildanalysator_Proxy.h"
#include <ros/callback_queue.h>


using namespace std;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  // Initialize Qt
  QApplication app(argc, argv);

  // Parse command line args for ROS enable/disable
  bool ros_enabled = true;
  for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--no-ros") {
          ros_enabled = false;
          break;
      }
  }

  // Initialize ROS if enabled
  if (ros_enabled) {
      ros::init(argc, argv, "SP4Koordinator");
  }
  printf("--- SP4Koordinator ---");
  // Create the main components
  Koordinator dialog(argc, argv);
  pds::MainWindow* window = new pds::MainWindow(&dialog);

  // Create ROS wrapper if enabled
  // Using raw pointer instead of make_unique for C++11 compatibility
  pds::ROSWindowWrapper* ros_wrapper = nullptr;
  if (ros_enabled) {
      ros_wrapper = new pds::ROSWindowWrapper(window);
  }

  // Set initial state
  dialog.setZustand(new Wartend(&dialog));

  // Show window
  window->show();

  int result = 0;

  // Main event loop
  if (ros_enabled) {
      // Combined Qt and ROS event loop
      ros::AsyncSpinner spinner(1);
      spinner.start();

      while (ros::ok() && window->isVisible()) {
          QApplication::processEvents();
          ros::spinOnce();
      }

      spinner.stop();
  } else {
      // Qt-only event loop
      result = app.exec();
  }

  // Clean up
  delete ros_wrapper;  // Safe to delete nullptr if ROS was disabled
  delete window;

  return result;

/*****************************************/

//    // Graphical Interface System
//    QApplication app(argc, argv);
//    int a=1;


//    Koordinator pD;
//    pds::MainWindow w(&pD);
//    pD.setZustand(new Wartend(&pD));
//    w.show();
//    printf("%s\n",setlocale(LC_NUMERIC,"de_DE.UTF-8"));
//    int result = app.exec();

//    return result;

/*****************************************/

    // Text Interface System
//    Koordinator pD;
//    TUIManager tm(&pD);
//    pD.setZustand(new Wartend(&pD));
//    std::cout << "Schleife";
//    while(1)
//        sleep(1);
//    getchar();


//    return 0;
}
