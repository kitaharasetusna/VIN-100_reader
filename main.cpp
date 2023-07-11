#include <boost/array.hpp>
#include <iostream>
using namespace std;

#include <cassert>

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
    return false;
  }

  // ROS_INFO("Set port to ASYNCY_LOW_LATENCY");
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

  string sensor_port;
  int sensor_baudrate;
  int async_output_rate;
  int imu_output_rate;

  return 0;
}