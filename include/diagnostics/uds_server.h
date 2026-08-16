// uds_server.h
//
// A UDS (ISO 14229) diagnostic server: receives request frames on the
// virtual CAN bus, dispatches them to the appropriate service handler,
// and publishes a response frame. Implements three of the most commonly
// used services:
//
//   0x10 DiagnosticSessionControl  -- switch between Default/Programming/
//                                      Extended sessions
//   0x22 ReadDataByIdentifier      -- read a live value by a 2-byte DID,
//                                      wired up to the BMS node's sensors
//   0x27 SecurityAccess            -- seed/key challenge-response
//
// Positive responses echo the request SID with bit 6 set (SID + 0x40).
// Negative responses are [0x7F, requestSid, NRC].

#pragma once

#include "can_bus/can_frame.h"
#include "can_bus/virtual_can_bus.h"
#include "ecu_nodes/bms_node.h"
#include "signal_codec.h"

#include <cstdint>
#include <vector>

enum class UdsSid : std::uint8_t {
    DiagnosticSessionControl = 0x10,
    SecurityAccess           = 0x27,
    ReadDataByIdentifier     = 0x22,
};

enum class DiagnosticSession : std::uint8_t {
    Default     = 0x01,
    Programming = 0x02,
    Extended    = 0x03,
};

enum class UdsNrc : std::uint8_t {
    ServiceNotSupported     = 0x11,
    SubFunctionNotSupported = 0x12,
    InvalidKey              = 0x35,
    RequestSequenceError    = 0x24, // e.g. sendKey without a prior requestSeed
};

class UdsServer : public ECUNode {
public:
    UdsServer(VirtualCanBus& bus, std::uint32_t requestId, std::uint32_t responseId, BmsNode& bms)
        : bus_(bus), requestId_(requestId), responseId_(responseId), bms_(bms) {}

    void onFrameReceived(const CanFrame& frame) override {
        if (frame.id != requestId_ || frame.dlc == 0) {
            return; // not addressed to this diagnostic server
        }

        std::uint8_t sid = frame.data[0];
        switch (static_cast<UdsSid>(sid)) {
            case UdsSid::DiagnosticSessionControl:
                handleSessionControl(frame);
                break;
            case UdsSid::ReadDataByIdentifier:
                handleReadDataByIdentifier(frame);
                break;
            case UdsSid::SecurityAccess:
                handleSecurityAccess(frame);
                break;
            default:
                sendNegativeResponse(sid, UdsNrc::ServiceNotSupported);
                break;
        }
    }

    DiagnosticSession currentSession() const { return session_; }
    bool securityUnlocked() const { return securityUnlocked_; }

private:
    // ---- 0x10 DiagnosticSessionControl ------------------------------------

    void handleSessionControl(const CanFrame& req) {
        if (req.dlc < 2) {
            sendNegativeResponse(static_cast<std::uint8_t>(UdsSid::DiagnosticSessionControl),
                                  UdsNrc::SubFunctionNotSupported);
            return;
        }
        std::uint8_t requestedSession = req.data[1];
        if (requestedSession != static_cast<std::uint8_t>(DiagnosticSession::Default) &&
            requestedSession != static_cast<std::uint8_t>(DiagnosticSession::Programming) &&
            requestedSession != static_cast<std::uint8_t>(DiagnosticSession::Extended)) {
            sendNegativeResponse(static_cast<std::uint8_t>(UdsSid::DiagnosticSessionControl),
                                  UdsNrc::SubFunctionNotSupported);
            return;
        }

        session_ = static_cast<DiagnosticSession>(requestedSession);
        // Leaving a non-default session drops any security unlock, same
        // as most real ECUs -- you don't keep privileged access across
        // an unrelated session change.
        if (session_ == DiagnosticSession::Default) {
            securityUnlocked_ = false;
        }

        sendPositiveResponse(UdsSid::DiagnosticSessionControl, {requestedSession});
    }

    // ---- 0x22 ReadDataByIdentifier -----------------------------------------

    void handleReadDataByIdentifier(const CanFrame& req) {
        if (req.dlc < 3) {
            sendNegativeResponse(static_cast<std::uint8_t>(UdsSid::ReadDataByIdentifier),
                                  UdsNrc::SubFunctionNotSupported);
            return;
        }
        std::uint16_t did = (static_cast<std::uint16_t>(req.data[1]) << 8) | req.data[2];

        // A handful of custom DIDs (0x1001-0x1004) exposing live BMS
        // readings, plus the standard-ish 0xF186 "active session" DID.
        std::vector<std::uint8_t> value;
        switch (did) {
            case 0x1001: value = encodeOneSignal(16, false, 0.01, 0.0, bms_.voltage()); break;
            case 0x1002: value = encodeOneSignal(16, true,  0.1,  0.0, bms_.current()); break;
            case 0x1003: value = encodeOneSignal(8,  false, 0.5,  0.0, bms_.stateOfCharge()); break;
            case 0x1004: value = encodeOneSignal(8,  false, 1.0, -40.0, bms_.temperature()); break;
            case 0xF186: value = {static_cast<std::uint8_t>(session_)}; break;
            default:
                sendNegativeResponse(static_cast<std::uint8_t>(UdsSid::ReadDataByIdentifier),
                                      UdsNrc::SubFunctionNotSupported);
                return;
        }

        std::vector<std::uint8_t> responseData = {req.data[1], req.data[2]};
        responseData.insert(responseData.end(), value.begin(), value.end());
        sendPositiveResponse(UdsSid::ReadDataByIdentifier, responseData);
    }

    // ---- 0x27 SecurityAccess ------------------------------------------------

    void handleSecurityAccess(const CanFrame& req) {
        if (req.dlc < 2) {
            sendNegativeResponse(static_cast<std::uint8_t>(UdsSid::SecurityAccess),
                                  UdsNrc::SubFunctionNotSupported);
            return;
        }
        std::uint8_t subFunction = req.data[1];
        bool isRequestSeed = (subFunction % 2) == 1; // odd = requestSeed, even = sendKey

        if (isRequestSeed) {
            // Simplified, deterministic "seed" for simulation purposes --
            // a real ECU would use a hardware RNG. Documented as
            // intentionally non-cryptographic; see project README.
            seedCounter_ += 0x1111;
            lastSeed_ = seedCounter_;
            seedPending_ = true;

            std::uint8_t seedHi = static_cast<std::uint8_t>(lastSeed_ >> 8);
            std::uint8_t seedLo = static_cast<std::uint8_t>(lastSeed_ & 0xFF);
            sendPositiveResponse(UdsSid::SecurityAccess, {subFunction, seedHi, seedLo});
            return;
        }

        // sendKey
        if (!seedPending_ || req.dlc < 4) {
            sendNegativeResponse(static_cast<std::uint8_t>(UdsSid::SecurityAccess),
                                  UdsNrc::RequestSequenceError);
            return;
        }
        std::uint16_t receivedKey = (static_cast<std::uint16_t>(req.data[2]) << 8) | req.data[3];
        seedPending_ = false;

        if (receivedKey == computeKey(lastSeed_)) {
            securityUnlocked_ = true;
            sendPositiveResponse(UdsSid::SecurityAccess, {subFunction});
        } else {
            sendNegativeResponse(static_cast<std::uint8_t>(UdsSid::SecurityAccess), UdsNrc::InvalidKey);
        }
    }

    // Deliberately simple, reversible transform -- NOT cryptographically
    // secure. Real seed/key algorithms are OEM-proprietary and far more
    // involved; this exists to demonstrate the challenge-response
    // *mechanism*, not to be a real security boundary.
    static std::uint16_t computeKey(std::uint16_t seed) {
        return static_cast<std::uint16_t>(seed ^ 0xA5A5);
    }

    static std::vector<std::uint8_t> encodeOneSignal(unsigned bits, bool isSigned,
                                                       double scale, double offset, double value) {
        SignalSpec spec{"value", bits, isSigned, scale, offset};
        std::vector<std::uint8_t> buffer;
        BitWriter writer(buffer);
        encodeSignal(writer, spec, value);
        return buffer;
    }

    // ---- response helpers ----------------------------------------------------

    void sendPositiveResponse(UdsSid sid, const std::vector<std::uint8_t>& payload) {
        std::vector<std::uint8_t> data = {static_cast<std::uint8_t>(
            static_cast<std::uint8_t>(sid) + 0x40)};
        data.insert(data.end(), payload.begin(), payload.end());
        bus_.send(CanFrame(responseId_, data));
    }

    void sendNegativeResponse(std::uint8_t requestSid, UdsNrc nrc) {
        std::vector<std::uint8_t> data = {0x7F, requestSid, static_cast<std::uint8_t>(nrc)};
        bus_.send(CanFrame(responseId_, data));
    }

    VirtualCanBus& bus_;
    std::uint32_t requestId_;
    std::uint32_t responseId_;
    BmsNode& bms_;

    DiagnosticSession session_ = DiagnosticSession::Default;
    bool securityUnlocked_ = false;
    bool seedPending_ = false;
    std::uint16_t lastSeed_ = 0;
    std::uint16_t seedCounter_ = 0x1000;
};
