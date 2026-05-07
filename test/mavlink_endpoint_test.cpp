#include <byte_transport/ByteTransportFactory.h>
#include <byte_transport/IByteTransport.h>
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

class MockByteTransport : public pendarlab::lib::comm::IByteTransport
{
public:
  MockByteTransport() = default;
  ~MockByteTransport() = default;
  static std::unique_ptr<IByteTransport> create(const std::unordered_map<std::string, std::string>& cfg);
  static pendarlab::lib::comm::ByteTransportFactory::ValidationResult validateConfig(const std::unordered_map<std::string, std::string>&);
  int read(unsigned char* buf, unsigned int buf_size) override;
  int write(const unsigned char* buf, unsigned int length) override;
};

REGISTER_BYTE_TRANSPORT("MockTransport", &MockByteTransport::create, &MockByteTransport::validateConfig);

std::unique_ptr<pendarlab::lib::comm::IByteTransport> MockByteTransport::create(const std::unordered_map<std::string, std::string>& cfg)
{
  return std::make_unique<MockByteTransport>();
}

pendarlab::lib::comm::ByteTransportFactory::ValidationResult
    MockByteTransport::validateConfig(const std::unordered_map<std::string, std::string>&)
{
  pendarlab::lib::comm::ByteTransportFactory::ValidationResult res;
  res.ok = true;
  res.msg = "";
  return res;
}

int MockByteTransport::read(unsigned char* buf, unsigned int buf_size)
{
  mavlink_heartbeat_t heartbeat_msg = { 0 };
  mavlink_message_t msg;
  mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat_msg);
  unsigned int len = mavlink_msg_to_send_buffer(buf, &msg); // 12 bytes of packet header + 9 bytes of heartbeat payload = 21 bytes

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
  bool isHasBeenCalled();

private:
  MockCallback();
  bool has_been_called_;
};

std::shared_ptr<MockCallback> MockCallback::create()
{
  return std::make_shared<MockCallback>(MockCallback());
}

MockCallback::MockCallback() : has_been_called_(false)
{
}

void MockCallback::theCallback(std::weak_ptr<MockCallback> worker, const MavlinkEndpointPacket& packet)
{
  auto self = worker.lock();
  if (self) {
    self->has_been_called_ = true;
  }
}

bool MockCallback::isHasBeenCalled()
{
  return has_been_called_;
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
  std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Wait for 1s
  for (size_t i = 0; i < num_of_cb_; i++) {
    EXPECT_EQ(v_mock_callback_[i]->isHasBeenCalled(), true);
  }
}

TEST_F(MavlinkEndpointCallbackAndTokenTest, RegisterCallbackWhenConnectedShouldWork)
{
  mav_ep_->connect("MockTransport", std::unordered_map<std::string, std::string>());
  v_mock_callback_.push_back(MockCallback::create());
  auto token = mav_ep_->createListener(std::bind(&MockCallback::theCallback, v_mock_callback_[num_of_cb_], std::placeholders::_1));
  v_token_.push_back(std::move(token));
  EXPECT_EQ(mav_ep_->getState(), MavlinkEndpointState::CONNECTED);
  EXPECT_EQ(mav_ep_->getNumOfListener(), num_of_cb_ + 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Wait for 1s
  EXPECT_EQ(v_mock_callback_[num_of_cb_]->isHasBeenCalled(), true);
}

// TODO:
//  # Test MavlinkEndpointState: DISCONNECTED, CONNECTING, CONNECTED, DISCONNECTING
//  # Test callback should not be called when not CONNECTED
//  # Test callback should be called when CONNECTED
//  - Test register callback in all states should be successful
//  - Test unregister callback in all states should be successful
//  - Test deleting token should unregister the associated callback
//  - Test Mavlink data received by mavlink endpoint should be parsed properly
//  - Test Mavlink data sent by mavlink endpoint should be written properly

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  int test_result = RUN_ALL_TESTS();
  return test_result;
}