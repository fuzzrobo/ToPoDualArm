// ToPoDualArm: 2026-10-23にProtocol 2.0受信処理を修正。由来と変更点は../README.mdを参照。
#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>

// Protocol 2.0: CRCはstuffingを含む受信列で確認し、その後で挿入FDを除く。
// https://emanual.robotis.com/docs/en/dxl/protocol2/#packet-process
inline bool decode_protocol2_status(const uint8_t* raw, size_t size, size_t data_size,
                                    std::vector<uint8_t>& data, uint8_t& error,
                                    uint16_t (*crc)(const uint8_t*, size_t)) {
    if (size < 11 || raw[0]!=0xff || raw[1]!=0xff || raw[2]!=0xfd || raw[3]!=0 ||
        size != 7u + (raw[5] | (uint16_t(raw[6])<<8))) return false;
    const uint16_t expected = raw[size-2] | (uint16_t(raw[size-1])<<8);
    if (crc(raw,size-2)!=expected) return false;
    std::vector<uint8_t> body;
    for (size_t i=7;i<size-2;i++) {
        body.push_back(raw[i]);
        if (i>=9 && raw[i-2]==0xff && raw[i-1]==0xff && raw[i]==0xfd) {
            if (i+1>=size-2 || raw[i+1]!=0xfd) return false;
            ++i;
        }
    }
    if (body.size()!=data_size+2 || body[0]!=0x55) return false;
    error=body[1]; data.assign(body.begin()+2,body.end()); return true;
}
