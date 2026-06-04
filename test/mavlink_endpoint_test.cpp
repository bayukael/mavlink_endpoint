#include <byte_transport/ByteTransportFactory.h>
#include <byte_transport/ByteTransport.h>
#include <chrono>
#include <functional>
#include <gtest/gtest.h>
#include <mavlink.h>
#include <mavlink_endpoint/MavlinkEndpoint.h>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using MavlinkEndpoint = pendarlab::lib::comm::MavlinkEndpoint;
using MavlinkEndpointState = pendarlab::lib::comm::MavlinkEndpointState;
using MavlinkEndpointToken = pendarlab::lib::comm::MavlinkEndpointToken;
using MavlinkEndpointPacket = pendarlab::lib::comm::MavlinkEndpointPacket;

class MockByteTransport : public pendarlab::lib::comm::ByteTransport
{
public:
  MockByteTransport() = default;
  ~MockByteTransport() = default;
  static std::unique_ptr<ByteTransport> create(const std::unordered_map<std::string, std::string>& cfg);
  static pendarlab::lib::comm::ByteTransportFactory::ValidationResult validateConfig(const std::unordered_map<std::string, std::string>&);
  int read(unsigned char* buf, unsigned int buf_size) override;
  int write(const unsigned char* buf, unsigned int length) override;
};

REGISTER_BYTE_TRANSPORT("MockTransport", &MockByteTransport::create, &MockByteTransport::validateConfig);

std::unique_ptr<pendarlab::lib::comm::ByteTransport> MockByteTransport::create(const std::unordered_map<std::string, std::string>& cfg)
{
  return std::make_unique<MockByteTransport>();
}

pendarlab::lib::comm::ByteTransportFactory::ValidationResult
    MockByteTransport::validateConfig(const std::unordered_map<std::string, std::string>&)
{
  pendarlab::lib::comm::ByteTransportFactory::ValidationResult res;
  res.ok = true;
  return res;
}

int MockByteTransport::read(unsigned char* buf, unsigned int buf_size)
{
  auto time_to_wake_up = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
  mavlink_heartbeat_t heartbeat_msg = { 0 };
  mavlink_message_t msg;
  mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat_msg);
  unsigned int len = mavlink_msg_to_send_buffer(buf, &msg); // 12 bytes of packet header + 9 bytes of heartbeat payload = 21 bytes
  std::this_thread::sleep_until(time_to_wake_up);

  return len;
}

int MockByteTransport::write(const unsigned char* buf, unsigned int length)
{
  return length;
}

class MockCallback : public std::enable_shared_from_this<MockCallback>
{
public:
  static std::shared_ptr<MockCallback> create();
  static void theCallback(std::weak_ptr<MockCallback> worker, const MavlinkEndpointPacket& packet);
  static void listeningCallback(std::weak_ptr<MockCallback> worker, const MavlinkEndpointPacket& packet);
  bool isHasBeenCalled();
  MavlinkEndpointPacket latestPacket();

private:
  MockCallback();
  bool has_been_called_;
  MavlinkEndpointPacket latest_packet_;
};

std::shared_ptr<MockCallback> MockCallback::create()
{
  return std::make_shared<MockCallback>(MockCallback());
}

MockCallback::MockCallback() : has_been_called_(false), latest_packet_{0}
{
}

void MockCallback::theCallback(std::weak_ptr<MockCallback> worker, const MavlinkEndpointPacket& packet)
{
  auto self = worker.lock();
  if (self) {
    self->has_been_called_ = true;
  }
}

void MockCallback::listeningCallback(std::weak_ptr<MockCallback> worker, const MavlinkEndpointPacket& packet)
{
  auto self = worker.lock();
  if (self) {
    self->latest_packet_ = packet;
  }
}

bool MockCallback::isHasBeenCalled()
{
  return has_been_called_;
}

MavlinkEndpointPacket MockCallback::latestPacket()
{
  return latest_packet_;
}

class MavlinkEndpointTestSetup
{
public:
  MavlinkEndpointTestSetup() : mav_ep_(MavlinkEndpoint::create()) {}

protected:
  std::shared_ptr<MavlinkEndpoint> mav_ep_;
};

class MavlinkEndpointInitialTest : public testing::Test, public MavlinkEndpointTestSetup
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(MavlinkEndpointInitialTest, InitialStateShouldBeDisconnected)
{
  EXPECT_EQ(mav_ep_->getState(), MavlinkEndpointState::DISCONNECTED);
}

TEST_F(MavlinkEndpointInitialTest, InitialListenerShouldBeZero)
{
  EXPECT_EQ(mav_ep_->getNumOfListener(), 0);
  auto listeners_id = mav_ep_->getListenersID();
  EXPECT_EQ(listeners_id.empty(), true);
}

TEST_F(MavlinkEndpointInitialTest, ValidateConfigReturnsTrueForExistingType)
{
  EXPECT_EQ(MavlinkEndpoint::validateConfig("MockTransport", std::unordered_map<std::string,std::string>()).ok, true);
}

TEST_F(MavlinkEndpointInitialTest, ValidateConfigReturnsFalseForNonExistantType)
{
  EXPECT_EQ(MavlinkEndpoint::validateConfig("NonExistantTransport", std::unordered_map<std::string,std::string>()).ok, false);
}

class MavlinkEndpointConnectionTest : public testing::Test, public MavlinkEndpointTestSetup
{
protected:
  void SetUp() override { connect_result_ = mav_ep_->connect("MockTransport", std::unordered_map<std::string, std::string>()); }
  void TearDown() override {}

  bool connect_result_;
};

TEST_F(MavlinkEndpointConnectionTest, ConnectShouldChangeStateToConnected)
{
  ASSERT_EQ(connect_result_, true);
  EXPECT_EQ(mav_ep_->getState(), MavlinkEndpointState::CONNECTED);
}

TEST_F(MavlinkEndpointConnectionTest, DisconnectShouldChangeStateToDisconnected)
{
  mav_ep_->disconnect();
  EXPECT_EQ(mav_ep_->getState(), MavlinkEndpointState::DISCONNECTED);
}

TEST_F(MavlinkEndpointConnectionTest, ConnectWhenConnectedReturnsFalse)
{
  bool connect_result = mav_ep_->connect("MockTransport", std::unordered_map<std::string, std::string>());
  EXPECT_EQ(connect_result, false);
}

class MavlinkEndpointCallbackAndTokenTest : public testing::Test, public MavlinkEndpointTestSetup
{
protected:
  void SetUp() override
  {
    for (size_t i = 0; i < num_of_cb_; i++) {
      v_mock_callback_.push_back(MockCallback::create());
    }
    for (size_t i = 0; i < num_of_cb_; i++) {
      auto token = mav_ep_->createListener(std::bind(&MockCallback::theCallback, v_mock_callback_[i], std::placeholders::_1));
      v_token_.push_back(std::move(token));
    }
  }
  void TearDown() override {}

  std::vector<std::shared_ptr<MockCallback>> v_mock_callback_;
  std::vector<std::unique_ptr<MavlinkEndpointToken>> v_token_;
  const size_t num_of_cb_ = 20;
};

TEST_F(MavlinkEndpointCallbackAndTokenTest, NoRegisteredCallbackShouldBeInvokedWhenDisconnected)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Wait for 1 s
  for (size_t i = 0; i < num_of_cb_; i++) {
    EXPECT_EQ(v_mock_callback_[i]->isHasBeenCalled(), false);
  }
}

TEST_F(MavlinkEndpointCallbackAndTokenTest, AllRegisteredCallbacksShouldBeInvokedWhenConnected)
{
  mav_ep_->connect("MockTransport", std::unordered_map<std::string, std::string>());
  std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Wait for 5 ms
  for (size_t i = 0; i < num_of_cb_; i++) {
    EXPECT_EQ(v_mock_callback_[i]->isHasBeenCalled(), true);
  }
}

TEST_F(MavlinkEndpointCallbackAndTokenTest, RegisterCallbackWhenConnectedShouldWork)
{
  mav_ep_->connect("MockTransport", std::unordered_map<std::string, std::string>());
  v_mock_callback_.push_back(MockCallback::create());
  ASSERT_EQ(mav_ep_->getState(), MavlinkEndpointState::CONNECTED);
  auto token = mav_ep_->createListener(std::bind(&MockCallback::theCallback, v_mock_callback_[num_of_cb_], std::placeholders::_1));
  v_token_.push_back(std::move(token));
  EXPECT_EQ(mav_ep_->getNumOfListener(), num_of_cb_ + 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Wait for 5 ms
  EXPECT_EQ(v_mock_callback_[num_of_cb_]->isHasBeenCalled(), true);
}

TEST_F(MavlinkEndpointCallbackAndTokenTest, UnregisterCallbackWhenDisconnectedShouldWork)
{
  ASSERT_EQ(mav_ep_->getNumOfListener(), num_of_cb_);
  auto token = std::move(v_token_.back());
  token->release();
  EXPECT_EQ(mav_ep_->getNumOfListener(), num_of_cb_ - 1);
}

TEST_F(MavlinkEndpointCallbackAndTokenTest, UnregisterCallbackWhenConnectedShouldWork)
{
  ASSERT_EQ(mav_ep_->getNumOfListener(), num_of_cb_);
  mav_ep_->connect("MockTransport", std::unordered_map<std::string, std::string>());
  auto token = std::move(v_token_.back());
  token->release();
  EXPECT_EQ(mav_ep_->getNumOfListener(), num_of_cb_ - 1);
}

TEST_F(MavlinkEndpointCallbackAndTokenTest, DeletingTokenShouldUnregisterAssociatedCallback)
{
  ASSERT_EQ(mav_ep_->getNumOfListener(), num_of_cb_);
  int token_id = v_token_.back()->getID();
  v_token_.pop_back();
  size_t current_num_of_cb = num_of_cb_ - 1;
  EXPECT_EQ(mav_ep_->getNumOfListener(), current_num_of_cb);
  for (size_t i = 0; i < current_num_of_cb; i++) {
    EXPECT_NE(v_token_[i]->getID(), token_id);
  }
}

class MavlinkEndpointTransmissionTest : public testing::Test, public MavlinkEndpointTestSetup
{
protected:
  void SetUp() override
  {
    mock_callback_ = MockCallback::create();
    mav_ep_->connect("MockTransport", std::unordered_map<std::string, std::string>());
    ASSERT_EQ(mav_ep_->getState(), MavlinkEndpointState::CONNECTED);
  }
  void TearDown() override {}

  std::shared_ptr<MockCallback> mock_callback_;
  std::unique_ptr<MavlinkEndpointToken> token_;
};

TEST_F(MavlinkEndpointTransmissionTest, IncomingHeartbeatMessageShouldBeParsedProperly)
{
  EXPECT_EQ(mock_callback_->latestPacket().msg.msgid, 0);
  EXPECT_EQ(mock_callback_->latestPacket().msg.sysid, 0);
  mock_callback_ = MockCallback::create();
  token_ = mav_ep_->createListener(std::bind(&MockCallback::listeningCallback, mock_callback_, std::placeholders::_1));
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(mock_callback_->latestPacket().msg.msgid, MAVLINK_MSG_ID_HEARTBEAT);
  EXPECT_EQ(mock_callback_->latestPacket().msg.sysid, 1);
}

TEST_F(MavlinkEndpointTransmissionTest, SendingHeartbeatMessageWhenDisconnectedShouldReturnNegativeOne)
{
  mav_ep_->disconnect();
  ASSERT_EQ(mav_ep_->getState(), MavlinkEndpointState::DISCONNECTED);

  mavlink_heartbeat_t heartbeat_msg = { 0 };
  mavlink_message_t msg;
  mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat_msg);

  int write_result = mav_ep_->writeMessage(msg);
  EXPECT_EQ(write_result, -1);
}

TEST_F(MavlinkEndpointTransmissionTest, SendingHeartbeatMessageWhenConnectedShouldReturn21)
{
  mavlink_heartbeat_t heartbeat_msg = { 0 };
  mavlink_message_t msg;
  mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat_msg);

  int write_result = mav_ep_->writeMessage(msg);
  EXPECT_EQ(write_result, 21); // 21 is the number of bytes encoded in a mavlink heartbeat message
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  int test_result = RUN_ALL_TESTS();
  return test_result;
}