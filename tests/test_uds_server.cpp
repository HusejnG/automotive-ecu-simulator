// test_uds_server.cpp

#include <gtest/gtest.h>

#include "can_bus/virtual_can_bus.h"
#include "diagnostics/uds_server.h"
#include "ecu_nodes/bms_node.h"

namespace {

constexpr std::uint32_t kRequestId = 0x7A0;
constexpr std::uint32_t kResponseId = 0x7A8;

// Records every frame delivered on the given ID, so tests can inspect
// the UDS server's responses.
class RecordingNode : public ECUNode {
public:
    void onFrameReceived(const CanFrame& frame) override {
        frames.push_back(frame);
    }
    std::vector<CanFrame> frames;
};

struct UdsFixture {
    VirtualCanBus bus;
    BmsNode bms{bus, 0x200};
    UdsServer uds{bus, kRequestId, kResponseId, bms};
    RecordingNode recorder;

    UdsFixture() {
        bus.subscribe(&recorder);
        bus.subscribe(&uds);
    }

    void sendRequest(const std::vector<std::uint8_t>& data) {
        bus.send(CanFrame(kRequestId, data));
        bus.process(); // delivers the request to UdsServer, which queues its response
        bus.process(); // delivers that response to subscribers (including recorder)
    }

    // Returns the most recent frame seen on the response ID, if any.
    const CanFrame* lastResponse() const {
        for (auto it = frames_snapshot().rbegin(); it != frames_snapshot().rend(); ++it) {
            if (it->id == kResponseId) return &(*it);
        }
        return nullptr;
    }

    const std::vector<CanFrame>& frames_snapshot() const { return recorder.frames; }
};

} // namespace

// ---- DiagnosticSessionControl ---------------------------------------------

TEST(UdsServer, StartsInDefaultSession) {
    UdsFixture f;
    EXPECT_EQ(f.uds.currentSession(), DiagnosticSession::Default);
}

TEST(UdsServer, SessionControlSwitchesToExtendedAndAcks) {
    UdsFixture f;
    f.sendRequest({0x10, 0x03}); // DiagnosticSessionControl -> Extended

    EXPECT_EQ(f.uds.currentSession(), DiagnosticSession::Extended);
    const CanFrame* resp = f.lastResponse();
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->data[0], 0x50); // 0x10 + 0x40 positive response
    EXPECT_EQ(resp->data[1], 0x03);
}

TEST(UdsServer, SessionControlRejectsUnknownSession) {
    UdsFixture f;
    f.sendRequest({0x10, 0x99}); // invalid session type

    const CanFrame* resp = f.lastResponse();
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->data[0], 0x7F); // negative response
    EXPECT_EQ(resp->data[1], 0x10);
    EXPECT_EQ(resp->data[2], static_cast<std::uint8_t>(UdsNrc::SubFunctionNotSupported));
}

// ---- ReadDataByIdentifier ---------------------------------------------------

TEST(UdsServer, ReadsLiveBmsVoltage) {
    UdsFixture f;
    f.bms.updateSensors(364.80, 0.0, 25.0, 60.0);
    f.bms.tick();
    f.bus.process();

    f.sendRequest({0x22, 0x10, 0x01}); // DID 0x1001 = pack voltage

    const CanFrame* resp = f.lastResponse();
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->data[0], 0x62); // 0x22 + 0x40
    EXPECT_EQ(resp->data[1], 0x10);
    EXPECT_EQ(resp->data[2], 0x01);
    // data[3..4] hold the 16-bit encoded voltage; decode and check.
    std::uint16_t raw = (static_cast<std::uint16_t>(resp->data[3]) << 8) | resp->data[4];
    EXPECT_NEAR(raw * 0.01, 364.80, 0.01);
}

TEST(UdsServer, ReadUnknownDidReturnsNegativeResponse) {
    UdsFixture f;
    f.sendRequest({0x22, 0xFF, 0xFF}); // no such DID

    const CanFrame* resp = f.lastResponse();
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->data[0], 0x7F);
    EXPECT_EQ(resp->data[2], static_cast<std::uint8_t>(UdsNrc::SubFunctionNotSupported));
}

TEST(UdsServer, ReadActiveSessionDid) {
    UdsFixture f;
    f.sendRequest({0x10, 0x03}); // switch to Extended first
    f.sendRequest({0x22, 0xF1, 0x86}); // standard-ish "active session" DID

    const CanFrame* resp = f.lastResponse();
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->data[3], static_cast<std::uint8_t>(DiagnosticSession::Extended));
}

// ---- SecurityAccess -----------------------------------------------------------

TEST(UdsServer, StartsLocked) {
    UdsFixture f;
    EXPECT_FALSE(f.uds.securityUnlocked());
}

TEST(UdsServer, CorrectKeyUnlocksSecurity) {
    UdsFixture f;
    f.sendRequest({0x27, 0x01}); // requestSeed
    const CanFrame* seedResp = f.lastResponse();
    ASSERT_NE(seedResp, nullptr);
    ASSERT_EQ(seedResp->data[0], 0x67);
    std::uint16_t seed = (static_cast<std::uint16_t>(seedResp->data[2]) << 8) | seedResp->data[3];

    std::uint16_t key = seed ^ 0xA5A5; // same transform the server uses
    f.sendRequest({0x27, 0x02, static_cast<std::uint8_t>(key >> 8), static_cast<std::uint8_t>(key & 0xFF)});

    EXPECT_TRUE(f.uds.securityUnlocked());
    const CanFrame* keyResp = f.lastResponse();
    ASSERT_NE(keyResp, nullptr);
    EXPECT_EQ(keyResp->data[0], 0x67);
}

TEST(UdsServer, WrongKeyStaysLocked) {
    UdsFixture f;
    f.sendRequest({0x27, 0x01}); // requestSeed

    f.sendRequest({0x27, 0x02, 0x00, 0x00}); // almost certainly wrong

    EXPECT_FALSE(f.uds.securityUnlocked());
    const CanFrame* resp = f.lastResponse();
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->data[0], 0x7F);
    EXPECT_EQ(resp->data[2], static_cast<std::uint8_t>(UdsNrc::InvalidKey));
}

TEST(UdsServer, SendKeyWithoutSeedRequestIsRejected) {
    UdsFixture f;
    f.sendRequest({0x27, 0x02, 0x12, 0x34}); // sendKey with no prior requestSeed

    EXPECT_FALSE(f.uds.securityUnlocked());
    const CanFrame* resp = f.lastResponse();
    ASSERT_NE(resp, nullptr);
    EXPECT_EQ(resp->data[0], 0x7F);
    EXPECT_EQ(resp->data[2], static_cast<std::uint8_t>(UdsNrc::RequestSequenceError));
}

TEST(UdsServer, LeavingDefaultSessionResetsSecurityUnlock) {
    UdsFixture f;
    f.sendRequest({0x27, 0x01});
    const CanFrame* seedResp = f.lastResponse();
    std::uint16_t seed = (static_cast<std::uint16_t>(seedResp->data[2]) << 8) | seedResp->data[3];
    std::uint16_t key = seed ^ 0xA5A5;
    f.sendRequest({0x27, 0x02, static_cast<std::uint8_t>(key >> 8), static_cast<std::uint8_t>(key & 0xFF)});
    ASSERT_TRUE(f.uds.securityUnlocked());

    f.sendRequest({0x10, 0x03}); // Extended
    f.sendRequest({0x10, 0x01}); // back to Default

    EXPECT_FALSE(f.uds.securityUnlocked());
}

// ---- Unrelated traffic ---------------------------------------------------------

TEST(UdsServer, IgnoresFramesNotAddressedToIt) {
    UdsFixture f;
    f.bus.send(CanFrame(0x999, {0x10, 0x03})); // different ID entirely
    f.bus.process();

    EXPECT_EQ(f.uds.currentSession(), DiagnosticSession::Default); // unchanged
}
