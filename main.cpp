#include <boost/array.hpp>
#include <iostream>
using namespace std;

#include <cassert>
#include <chrono>
#include <thread>

// Include this header file to get access to VectorNav sensors.
#include "vn/compositedata.h"
#include "vn/sensors.h"
#include "vn/util.h"

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Custom user data to pass to packet callback function
struct UserData {
  // the vectornav device identifier
  int device_family;
  // frame id used only for Odom header.frame_id
  std::string map_frame_id;
  // frame id used for header.frame_id of other messages and for Odom
  // child_frame_id
  std::string frame_id;
  // Boolean to use ned or enu frame. Defaults to enu which is data format from
  // sensor.
  bool tf_ned_to_enu;
  bool frame_based_enu;
  // Initial position after getting a GPS fix.
  vec3d initial_position;
  bool initial_position_set = false;

  // TODO: test this later
  // Unused covariances initialized to zero's
  boost::array<double, 9ul> linear_accel_covariance = {};
  boost::array<double, 9ul> angular_vel_covariance = {};
  boost::array<double, 9ul> orientation_covariance = {};

  // ROS header time stamp adjustments
  // double average_time_difference{0};

  // TODO: just removed ROS time, add cpp standard time here

  bool adjust_ros_timestamp{false};

  // strides
  unsigned int imu_stride;
  unsigned int output_stride;
};

#include <Poco/DOM/DOMParser.h>
#include <Poco/DOM/Document.h>
#include <Poco/DOM/Element.h>
#include <Poco/DOM/Node.h>
#include <Poco/DOM/NodeList.h>

#include <Poco/AutoPtr.h>
#include <array>
#include <string>

std::array<double, 9> setCov(const std::string &xmlData) {
  std::array<double, 9> output = {0.0};

  // Parse the XML data
  Poco::XML::DOMParser parser;
  Poco::AutoPtr<Poco::XML::Document> pDoc = parser.parseString(xmlData);

  // Get the root element of the XML
  Poco::XML::Element *pRoot = pDoc->documentElement();

  // Get the child nodes of the root element
  Poco::XML::NodeList *pNodeList = pRoot->childNodes();

  // Iterate over the child nodes and extract the values
  for (Poco::XML::Node *pNode = pNodeList->item(0); pNode;
       pNode = pNode->nextSibling()) {
    if (pNode->nodeType() == Poco::XML::Node::ELEMENT_NODE) {
      Poco::XML::Element *pElement = dynamic_cast<Poco::XML::Element *>(pNode);
      if (pElement->nodeName() == "value") {
        int index = std::stoi(pElement->getAttribute("index"));
        output[index] = std::stod(pElement->innerText());
      }
    }
  }

  return output;
}

// Assure that the serial port is set to async low latency in order to reduce
// delays and package pilup. These changes will stay effective until the device
// is unplugged
#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
bool optimize_serial_communication(std::string portName) {
  int portFd = -1;

  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    // ROS_WARN("Can't open port for optimization");
    cout << "Can't open port for optimization" << endl;
    return false;
  }

  // ROS_INFO("Set port to ASYNCY_LOW_LATENCY");
  cout << "\nSet port to ASYNCY_LOW_LATENCY" << endl;
  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ::close(portFd);
  return true;
}
#elif
bool optimize_serial_communication(str::string portName) { return true; }
#endif

int main() {
  cout << "running VIN-100T imu reader..." << endl;

  UserData user_data;

  // set parameters
  string sensor_port;
  int sensor_baudrate;
  int async_output_rate;
  int imu_output_rate;

  int sensor_imu_rate;

  user_data.frame_based_enu = "map";
  user_data.frame_id = "vectormap";
  user_data.tf_ned_to_enu = false;
  user_data.frame_based_enu = false;
  user_data.adjust_ros_timestamp = false;
  async_output_rate = 40;
  imu_output_rate = async_output_rate;

  sensor_port = "/dev/ttyUSB0";
  sensor_baudrate = 115200;
  sensor_imu_rate = 800;

  // Linear Acceleration Covariances not produced by the sensor
  boost::array<double, 9ul> linear_accel_covariance = {
      0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
  // Angular Velocity Covariances not produced by the sensor
  boost::array<double, 9ul> angular_vel_covariance = {0.01, 0.0, 0.0, 0.0, 0.01,
                                                      0.0,  0.0, 0.0, 0.01};

  // Orientation covariance overwritten in driver, this is included just
  // as an extra
  boost::array<double, 9ul> orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01,
                                                      0.0,  0.0, 0.0, 0.01};

  user_data.linear_accel_covariance = linear_accel_covariance;
  user_data.angular_vel_covariance = angular_vel_covariance;
  user_data.orientation_covariance = orientation_covariance;

  cout << "Connecting to : " << sensor_port << " " << sensor_baudrate
       << " Baud";
  optimize_serial_communication(sensor_port);

  // Create a VnSensor object and connect to sensor
  VnSensor vs;

  // Default baudrate variable
  int defaultBaudrate;
  // Run through all of the acceptable baud rates until we are connected
  // Looping in case someone has changed the default
  bool baudSet = false;
  // Lets add the set baudrate to the top of the list, so that it will try
  // to connect with that value first (speed initialization up)
  std::vector<unsigned int> supportedBaudrates = vs.supportedBaudrates();
  supportedBaudrates.insert(supportedBaudrates.begin(), sensor_baudrate);
  while (!baudSet) {
    // Make this variable only accessible in the while loop
    static int i = 0;
    defaultBaudrate = supportedBaudrates[i];
    cout << "Connecting with default at " << defaultBaudrate << endl;
    // Default response was too low and retransmit time was too long by default.
    // They would cause errors
    vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);   // Retransmit every 50 ms

    // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400,
    // 460800, 921600 Data sheet says 128000 is a valid baud rate. It doesn't
    // work with the VN100 so it is excluded. All other values seem to work
    // fine.
    try {
      // Connect to sensor at it's default rate
      if (defaultBaudrate != 128000 && sensor_baudrate != 128000) {
        vs.connect(sensor_port, defaultBaudrate);
        // Issues a change baudrate to the VectorNav sensor and then
        // reconnects the attached serial port at the new baudrate.
        vs.changeBaudRate(sensor_baudrate);
        // Only makes it here once we have the default correct
        // ROS_INFO("Connected baud rate is %d", vs.baudrate());
        cout << "Connectefd baud rate is" << vs.baudrate() << endl;
        baudSet = true;
      }
    }
    // Catch all oddities
    catch (...) {
      // Disconnect if we had the wrong default and we were connected
      vs.disconnect();
      // Sleep for 0.2 seconds
      // Sleep for 0.2 seconds
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    // Increment the default iterator
    i++;
    // There are only 9 available data rates, if no connection
    // made yet possibly a hardware malfunction?
    if (i > 8) {
      break;
    }
  }

  return 0;
}