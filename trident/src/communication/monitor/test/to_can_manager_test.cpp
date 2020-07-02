// Dont use exceptions because google doesnt us them
#include <auvic_msgs/devices_to_monitor.h>

#include <can_msgs/Frame.h>
#include <auvic_msgs/protocol_allMessages_U.h>
#include <auvic_msgs/protocol_deviceName_S.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <list>
#include <memory>
#include <string>
#include <vector>

class msgCollector
{
  public:
    std::list<auvic_msgs::device_to_monitor> messages;

    msgCollector() {}

    void msgCallback(const auvic_msgs::device_to_monitor& f)
    {
      messages.push_back(f);
    }
};
// send a defined message on protocol.h
TEST(CANtoMonitorTEST, checkFullMessageKnownID_S)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup();  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  driver_->init("string_not_used", true);

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a can frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x1337;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  driver_->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(1, message_collector_.messages.size());

  // compare the received can_msgs::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::Frame msg = message_collector_.messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  EXPECT_EQ(received.data, f.data);
}
// send an undefined message on protocol.h
TEST(CANtoMonitorTEST, checkFullMessageUnknownID_S)
{
  // - tries to send a non-extended frame with an id larger than 11 bits.
  //   that should not be sent.
  // - verifies that sending one larger than 11 bits actually works.

  // sending a message with a dlc > 8 is not possible as the DummyInterface
  // causes a crash then.

  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup();  // initiate the message callbacks

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a message
  can::Frame f;
  f.is_extended = false;
  f.id = (1<<11)+1;  // this is an illegal CAN packet... should not be sent.

  // send the can::Frame over the driver.
  // driver_->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(message_collector_.messages.size(), 0);

  f.is_extended = true;
  f.id = (1<<11)+1;  // now it should be alright.

  driver_->send(f);
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(message_collector_.messages.size(), 1);
}
// send an empty message with known id
TEST(CANtoMonitorTEST, checkEmptyMessageKnownID_S)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // create can_id vector with id that should be passed and published to ros
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x1337);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup(can::tofilters(pass_can_ids));  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  driver_->init("string_not_used", true);

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a can frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x1337;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  driver_->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  ASSERT_EQ(1, message_collector_.messages.size());

  // compare the received can_msgs::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::Frame msg = message_collector_.messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  EXPECT_EQ(received.data, f.data);
}
// send an unknown message id with message
TEST(CANtoMonitorTEST, checkEmptyMessageUnknownID_S)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  can::DummyInterfaceSharedPtr driver_ = std::make_shared<can::DummyInterface>(true);

  // create can_id vector with id that should not be received on can bus
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x300);

  // start the to topic bridge.
  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver_);
  to_topic_bridge.setup(can::tofilters(pass_can_ids));  // initiate the message callbacks

  // init the driver to test stateListener (not checked automatically).
  driver_->init("string_not_used", true);

  // create a frame collector.
  msgCollector message_collector_;

  // register for messages on received_messages.
  ros::Subscriber subscriber_ = nh.subscribe("received_messages", 1, &msgCollector::msgCallback, &message_collector_);

  // create a can frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x1337;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  driver_->send(f);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  EXPECT_EQ(0, message_collector_.messages.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_to_can_manager");
  ros::NodeHandle nh;
  ros::WallDuration(1.0).sleep();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
