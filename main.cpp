#include <boost/array.hpp>
#include <iostream>
#include <sstream>
#include <string>
using namespace std;

#include <cassert>
#include <chrono>
#include <thread>

void log_warn(const std::string &input) {
  std::cout << "WARN:" << input << std::endl;
}

void log_error(const std::string &input) {
  std::cout << "ERROR:" << input << std::endl;
}

void log_info(const std::string &input) {
  std::cout << "INFO:" << input << std::endl;
}

std::string arrayToString(const boost::array<double, 9ul> &inputArray) {
  std::ostringstream oss;
  oss << "[";
  for (const auto &element : inputArray) {
    oss << " " << element << ",";
  }
  oss << "]";
  return oss.str();
}

// Include this header file to get access to VectorNav sensors.
#include "vn/compositedata.h"
#include "vn/sensors.h"
#include "vn/util.h"

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void BinaryAsyncMessageReceived(void *userData, Packet &p, size_t index);

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
  log_info("Set port to ASYNCY_LOW_LATENCY");
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

  log_info("Connecting to: " + sensor_port + " " + to_string(sensor_baudrate) +
           "Baud");
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
    log_info("Connecting with default at" + to_string(defaultBaudrate));
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
        log_info("Connectefd baud rate is " + to_string(vs.baudrate()));
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

  if (vs.verifySensorConnectivity()) {
    log_info("Device connection established");
  } else {
    log_error("ERROR: NO device communication");
    log_warn("WARN: please input a valid baud rate , Valid are: ");
    log_warn(
        "9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
    log_warn("With the test IMU 128000 did not work, all others worked fine.");
  }

  // Query the sensor's model number.
  string mn = vs.readModelNumber();
  string fv = vs.readFirmwareVersion();
  uint32_t hv = vs.readHardwareRevision();
  uint32_t sn = vs.readSerialNumber();

  log_info("Model Number: " + mn + ", Firmware version: " + fv);
  log_info("Hardware Revision: " + to_string(hv) +
           ", Serial Number: " + to_string(sn));

  // calculate the least common multiple of the two rate and assure it is a
  // valid package rate, also calculate the imu and output strides
  int package_rate = 0;
  for (int allowed_rate : {1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200, 0}) {
    package_rate = allowed_rate;
    if ((package_rate % async_output_rate) == 0 &&
        (package_rate % imu_output_rate) == 0)
      break;
  }
  log_info("imu_output_rate: " + to_string(imu_output_rate) +
           ", async_output_rate: " + to_string(async_output_rate) +
           ", package_rate: " + to_string(package_rate));
  // Set the device info for passing to the packet callback function
  user_data.device_family = vs.determineDeviceFamily();

  // Make sure no generic async output is registered
  vs.writeAsyncDataOutputType(VNOFF);
  // Configure binary output message
  BinaryOutputRegister bor(
      ASYNCMODE_PORT1,
      sensor_imu_rate / package_rate, // update rate [ms]
      COMMONGROUP_QUATERNION | COMMONGROUP_YAWPITCHROLL |
          COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_ACCEL |
          COMMONGROUP_MAGPRES |
          (user_data.adjust_ros_timestamp ? COMMONGROUP_TIMESTARTUP : 0),
      TIMEGROUP_NONE | TIMEGROUP_GPSTOW | TIMEGROUP_GPSWEEK | TIMEGROUP_TIMEUTC,
      IMUGROUP_NONE, GPSGROUP_NONE,
      ATTITUDEGROUP_YPRU, //<-- returning yaw pitch roll uncertainties
      INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELBODY |
          INSGROUP_ACCELECEF | INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU,
      GPSGROUP_NONE);

  // An empty output register for disabling output 2 and 3 if previously set
  BinaryOutputRegister bor_none(
      0, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE,
      ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

  vs.writeBinaryOutput1(bor);
  vs.writeBinaryOutput2(bor_none);
  vs.writeBinaryOutput3(bor_none);

  // Register async callback function
  vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  // vs.disconnect();
  return 0;
}

void BinaryAsyncMessageReceived(void *userData, Packet &p, size_t index) {
  // package counter to calculate strides
  static unsigned long long pkg_count = 0;
  string log_str = "";
  log_str += "this is pkg no. " + to_string(pkg_count);
  // evaluate time first, to have it as close to the measurement time as
  // possible const ros::Time ros_time = ros::Time::now();

  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  UserData *user_data = static_cast<UserData *>(userData);
  // ros::Time time = get_time_stamp(cd, user_data, ros_time);

  if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration()) {
    vec4f q = cd.quaternion();
    vec3f ar = cd.angularRate();
    vec3f al = cd.acceleration();
    // log_info("q: " + to_string(q[0]) + " " + to_string(q[1]) + " " +
    //          to_string(q[2]));
    // log_info("ar: " + to_string(ar[0]) + " " + to_string(ar[1]) + " " +
    //          to_string(ar[2]));
    // log_info("al: " + to_string(al[0]) + " " + to_string(al[1]) + " " +
    //          to_string(al[2]));

    if (cd.hasAttitudeUncertainty()) {
      vec3f orientationStdDev = cd.attitudeUncertainty();
      user_data->orientation_covariance[0] =
          pow(orientationStdDev[2] * M_PI / 180, 2); // Convert to radians Roll
      user_data->orientation_covariance[4] =
          pow(orientationStdDev[1] * M_PI / 180, 2); // Convert to radians Pitch
      user_data->orientation_covariance[8] =
          pow(orientationStdDev[0] * M_PI / 180, 2); // Convert to radians Yaw
      // log_info(to_string(user_data->orientation_covariance[0]) + " " +
      //          to_string(user_data->orientation_covariance[4]) + " " +
      //          to_string(user_data->orientation_covariance[8]));
    }

    // TODO:
    // add quan in first condition
    if (user_data->tf_ned_to_enu) {
      // // If we want the orientation to be based on the reference label on the
      // // imu
      // tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      // geometry_msgs::Quaternion quat_msg;

      // if (user_data->frame_based_enu) {
      //   // Create a rotation from NED -> ENU
      //   tf2::Quaternion q_rotate;
      //   q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
      //   // Apply the NED to ENU rotation such that the coordinate frame
      //   matches tf2_quat = q_rotate * tf2_quat; quat_msg =
      //   tf2::toMsg(tf2_quat);

      //   // Since everything is in the normal frame, no flipping required
      //   msgIMU.angular_velocity.x = ar[0];
      //   msgIMU.angular_velocity.y = ar[1];
      //   msgIMU.angular_velocity.z = ar[2];

      //   msgIMU.linear_acceleration.x = al[0];
      //   msgIMU.linear_acceleration.y = al[1];
      //   msgIMU.linear_acceleration.z = al[2];
      // } else {
      //   // put into ENU - swap X/Y, invert Z
      //   quat_msg.x = q[1];
      //   quat_msg.y = q[0];
      //   quat_msg.z = -q[2];
      //   quat_msg.w = q[3];

      //   // Flip x and y then invert z
      //   msgIMU.angular_velocity.x = ar[1];
      //   msgIMU.angular_velocity.y = ar[0];
      //   msgIMU.angular_velocity.z = -ar[2];
      //   // Flip x and y then invert z
      //   msgIMU.linear_acceleration.x = al[1];
      //   msgIMU.linear_acceleration.y = al[0];
      //   msgIMU.linear_acceleration.z = -al[2];

      //   if (cd.hasAttitudeUncertainty()) {
      //     vec3f orientationStdDev = cd.attitudeUncertainty();
      //     msgIMU.orientation_covariance[0] = pow(
      //         orientationStdDev[1] * M_PI / 180, 2); // Convert to radians
      //         pitch
      //     msgIMU.orientation_covariance[4] = pow(
      //         orientationStdDev[0] * M_PI / 180, 2); // Convert to radians
      //         Roll
      //     msgIMU.orientation_covariance[8] = pow(
      //         orientationStdDev[2] * M_PI / 180, 2); // Convert to radians
      //         Yaw
      //   }
      // }

      // msgIMU.orientation = quat_msg;
    } else {
      float orientation_x = q[0];
      float orientation_y = q[1];
      float orientation_z = q[2];
      float orientation_w = q[3];

      float angular_velocity_x = ar[0];
      float angular_velocity_y = ar[1];
      float angular_velocity_z = ar[2];
      float linear_acceleration_x = al[0];
      float linear_acceleration_y = al[1];
      float linear_acceleration_z = al[2];
      // log_info(to_string(orientation_x) + " " + to_string(orientation_y) +
      //          to_string(orientation_z) + to_string(orientation_w));
      // log_info("\n orientation:\n x: " + to_string(orientation_x) + "\n" +
      //          " y: " + to_string(orientation_y) +
      //          "\n z: " + to_string(orientation_z));
      log_str += "\n orientation:\n x: " + to_string(orientation_x) + "\n" +
                 " y: " + to_string(orientation_y) +
                 "\n z: " + to_string(orientation_z);
      log_str += "\n orientation_covariance: " +
                 arrayToString(user_data->orientation_covariance);
      log_str += "\n angular_velocity:\n x: " + to_string(angular_velocity_x) +
                 "\n" + " y: " + to_string(angular_velocity_y) +
                 "\n z: " + to_string(angular_velocity_z);
      log_str += "\n angular_acceleration_covariance: " +
                 arrayToString(user_data->angular_vel_covariance);
      log_str +=
          "\n linear_acceleration:\n x: " + to_string(linear_acceleration_x) +
          "\n" + " y: " + to_string(linear_acceleration_y) +
          "\n z: " + to_string(linear_acceleration_z);
      log_str += "\n linear_acceleration_covariance: " +
                 arrayToString(user_data->linear_accel_covariance);
    }
    // TODO:
    // Covariances pulled from parameters
    // msgIMU.angular_velocity_covariance = user_data->angular_vel_covariance;
    // msgIMU.linear_acceleration_covariance =
    // user_data->linear_accel_covariance;
    log_info(log_str);
  }

  pkg_count += 1;
}