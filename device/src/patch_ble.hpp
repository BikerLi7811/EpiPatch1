// The Bluetooth LE service that links the two patches. The thigh patch is the
// GATT server (peripheral); the chest patch connects to it (central).
//
//   service      EpiPatch link
//   to-thigh     write without response   patch_msg::Decision / Cancel / ReportNow
//   to-chest     notify                   patch_msg::Report
//
// Both characteristics need an encrypted link. The patches pair once (LE
// Secure Connections, bonded), and each one refuses pairing while it already
// has a bond, so only its partner can ever write a decision. Pairing again
// (for example after replacing a patch) needs the bond erased on both.
#pragma once

#include <zephyr/bluetooth/uuid.h>

#define EPIPATCH_UUID_SVC_VAL BT_UUID_128_ENCODE(0x5e6a0001, 0x7c1d, 0x4c55, 0x9f0e, 0x2b51e2a0d001)
#define EPIPATCH_UUID_TO_THIGH_VAL BT_UUID_128_ENCODE(0x5e6a0002, 0x7c1d, 0x4c55, 0x9f0e, 0x2b51e2a0d001)
#define EPIPATCH_UUID_TO_CHEST_VAL BT_UUID_128_ENCODE(0x5e6a0003, 0x7c1d, 0x4c55, 0x9f0e, 0x2b51e2a0d001)

#define EPIPATCH_UUID_SVC BT_UUID_DECLARE_128(EPIPATCH_UUID_SVC_VAL)
#define EPIPATCH_UUID_TO_THIGH BT_UUID_DECLARE_128(EPIPATCH_UUID_TO_THIGH_VAL)
#define EPIPATCH_UUID_TO_CHEST BT_UUID_DECLARE_128(EPIPATCH_UUID_TO_CHEST_VAL)
