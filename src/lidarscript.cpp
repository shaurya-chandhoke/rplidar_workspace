#include <iostream>
#include <signal.h>
#include <thread>
#include <mutex> //For thread locking
#include <map>
#include <string>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sstream>
#include <fstream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stropts.h>

#include "rplidar.h" //All in one inclusive header for A2M8

#include <lcm/lcm-cpp.hpp>
#include "lidarlcm/transmitter.hpp"

#ifndef _countof
#define _countof(_Array)(int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std;
using namespace rp::standalone::rplidar;
using std::mutex;

mutex mutex1; //For getting encoder data sent to lcm
mutex mutex2; //For thread termination check
mutex mutex3; //For robot movement

string leftEnc = "";
string rightEnc = "";

int fd1=open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY); //Open robot file


//Used throughout all threads as terminating variable -- Is true when ctrl+c is pressed
bool exitKey_pressed = false;
void keyPressed(int)
{
  exitKey_pressed = true;
}

//Get current system time
unsigned long micros()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return 1000000 * tv.tv_sec + tv.tv_usec;
}

//Used by thread1 to constantly get encoder val from robot
int robotencoder()
{
  char *buff;
  int wr,rd;
  int size = 25;
  bool stopsig = false;

  buff = new char[size];

  if(fd1 == -1)
  {
    return -1;
  }
  else
  {
    fcntl(fd1, F_SETFL,0);
    printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
  }
  tcflush(fd1, TCIOFLUSH);

  while(true)
  {

    mutex3.lock();
    wr=write(fd1,"d",1);
    mutex3.unlock();

    usleep(100);
    rd=read(fd1,buff,size);

    usleep(100);
    tcflush(fd1, TCIOFLUSH);
    usleep(100);

    string s(buff);
    string foo = "";
    string enc1, enc2, compass;
    for(unsigned int i = 0; i < s.length(); i++)
    {
      char el = s[i];
      foo += el;
      if(i == 2)
      {
        compass = foo;
        foo = "";
      }
      else if(i == 13)
      {
        enc1 = foo;
        foo = "";
      }
      else if(i == (s.length()-1))
      {
        enc2 = foo;
        foo = "";
      }
    }

    enc1 = enc1.erase(0, min(enc1.find_first_not_of('0'), enc1.size()-1));
    enc2 = enc2.erase(0, min(enc2.find_first_not_of('0'), enc2.size()-1));

    mutex1.lock();
    leftEnc = enc1;
    rightEnc = enc2;
    mutex1.unlock();

    mutex2.lock();
    stopsig = exitKey_pressed;
    mutex2.unlock();

    if(stopsig)
    {
      break;
    }
  }

  close(fd1);
  return 0;
}

/*
Used by thread2 to control robot movement based on string from stdin

Available commands:
f -> forward
b -> backward
s -> stop
l -> turn right
k -> turn left
m -> rotate clockwise
n -> rotate counterclockwise

*/

int robotmovement()
{
  if(fd1 == -1)
  {
    cout << "open_port: Unable to open /dev/ttyACM0 – " << endl;
    return -1;
  }
  string key = "";
  bool stopsig = false;
  int wr;

  while(true)
  {
    cout << "Enter key: ";
    cin >> key;

    if(key == "f")
    {
        mutex3.lock();
        wr=write(fd1,"f",1);
        mutex3.unlock();
        usleep(100);
    }
    else if(key == "b")
    {
        mutex3.lock();
        wr=write(fd1,"b",1);
        mutex3.unlock();
	usleep(100);
    }
    else if(key == "s")
    {
        mutex3.lock();
        wr=write(fd1,"s",1);
        mutex3.unlock();
        usleep(100);
    }
    else if(key == "l")
    {
        mutex3.lock();
        wr=write(fd1,"l",1);
        mutex3.unlock();
        usleep(100);
   }

   else if(key == "k")
    {
        mutex3.lock();
        wr=write(fd1,"k",1);
        mutex3.unlock();
        usleep(100);
    }
    else if(key == "m")
    {
        mutex3.lock();
        wr=write(fd1,"m",1);
        mutex3.unlock();
        usleep(100);
    }
    else if(key == "n")
    {
        mutex3.lock();
        wr=write(fd1,"m",1);
        mutex3.unlock();
        usleep(100);
    }

    mutex2.lock();
    stopsig = exitKey_pressed;
    mutex2.unlock();

    if(stopsig)
    {
       break;
    }

  }

  return 0;

}

void print_usage()
{
  cout << "********************************************************" << endl;
  cout << "***      Simple LIDAR data grabber for RPLIDAR       *** \n";
  cout << "***                  Version: ";
  printf(RPLIDAR_SDK_VERSION "                  ***\n"
    "************************"
    "************************"
    "******** \n");

  cout << "********         Press ctrl + c to exit         ******** \n\n\n";
  cout << "********         Starting Initial Tests         ******** \n\n";
}

int main()
{

  const char * opt_com_path = "/dev/ttyUSB0";
  _u32 opt_com_baudrate = 115200;
  rplidar_response_device_info_t devinfo;
  rplidar_response_device_health_t healthinfo;

  u_result op_result;
  u_result op_result2;
  RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

  print_usage();

  lcm::LCM lcm;
  if (!lcm.good())
  {
    return 1;
  }
  lidarlcm::transmitter my_data;

  std::map <int, int> datamap;
  for(int i = 0; i < 360; i++)
  {
    datamap[i] = 0;
  }

  if(IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate)))
  {
    cout << "Error, cannot bind to the specified serial port" << endl;
    return -1;
  }
  else
  {
    op_result = drv->getDeviceInfo(devinfo);
  }

  if(IS_FAIL(op_result))
  {
    if(op_result == RESULT_OPERATION_TIMEOUT)
    {
      cout << "Error, the operation times out" << endl;
    }
  }

  cout << "RPLIDAR Serial No: ";

  for(int i = 0; i < 16; i++)
  {
    printf("%02X", devinfo.serialnum[i]);
  }
  cout << endl;


  op_result = drv->getHealth(healthinfo);

  if(IS_OK(op_result))
  {

    printf("%s", "RPLIDAR Health Status: ");
    switch(healthinfo.status)
    {
      case RPLIDAR_STATUS_OK:
        cout << "[OK]" << endl;;
        break;
      case RPLIDAR_STATUS_WARNING:
        cout << "[WARNING]" << endl;
        break;
      case RPLIDAR_STATUS_ERROR:
        cout << "[ERROR]" << endl;
        break;
    }

  }

  std::thread thread1(robotencoder);
  std::thread thread2(robotmovement);

  bool stopsig = false;
  signal(SIGINT, keyPressed);

  drv->startMotor();
  drv->startScan(0,1);
  int counter = 0;
  while (true)
  {
    rplidar_response_measurement_node_t nodes[8192];
    size_t count = _countof(nodes);

    op_result = drv->grabScanData(nodes, count);

    if(IS_OK(op_result))
    {

      drv->ascendScanData(nodes, count);

      for(int pos = 0; pos < (int) count; ++pos)
      {

        int qualityfilter = nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        if(qualityfilter)
        {
          // printf("Theta: %03.2f     | Dist: %08.2f     | Quality: %d\n",
          // (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
          // nodes[pos].distance_q2 / 4.0f, qualityfilter);
          int angle = (int)(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
          float dist = (int)nodes[pos].distance_q2/4.0f;
          if(datamap[angle] == 0)
          {
            datamap[angle] = dist;
          }
          else if(datamap[angle] != 0)
          {
            datamap[angle] = ((datamap[angle]) + dist)/2;
          }
        }
      }

      my_data.row[counter] = micros();
      counter++;

      mutex1.lock();
      my_data.row[counter] = atoi(leftEnc.c_str());
      counter++;
      my_data.row[counter] = atoi(rightEnc.c_str());
      counter++;
      mutex1.unlock();

      for(int i = 4; i < 24; i++)
      {
        my_data.row[counter] = 0;
        counter++;
      }

      for(std::map<int, int>::iterator it = datamap.begin(); it != datamap.end(); it++)
      {
        my_data.row[counter] = it->second;
        counter++;
      }

      my_data.row[counter] = 0;

      mutex2.lock();
      stopsig = exitKey_pressed;
      mutex2.unlock();

      if(stopsig)
      {
        break;
      }

      lcm.publish("PIPE", &my_data);
      counter = 0;
      std::fill_n(my_data.row, 385, 0);
    }
  }

  drv->stop();
  drv->stopMotor();

  thread1.join();
  thread2.join();

  RPlidarDriver::DisposeDriver(drv);
  drv = NULL;
  return 0;

}

