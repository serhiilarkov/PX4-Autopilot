/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *   Portions Copyright (C) 2014 Pavel Kirienko (libuavcan driver interface),
 *   MIT-licensed, re-expressed here in a framework-neutral namespace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

/**
 * px4can -- framework-neutral CAN bus driver interface.
 *
 * A self-contained re-expression of the small libuavcan driver-layer surface
 * (CanFrame, ICanIface/ICanDriver, the monotonic/UTC time types, and the handful of
 * container/trait helpers the bare-metal drivers were written against) in a neutral
 * namespace with NO dependency on the libuavcan protocol library. It lets the
 * battle-tested STM32 bxCAN/FDCAN register drivers be shared by both the legacy
 * libuavcan-based `uavcan` stack and the new libcanard-based `dronecan` stack: each
 * driver is written against px4can, and each protocol stack adapts at its edge.
 *
 * Layout note: px4can::CanFrame matches both uavcan::CanFrame and the legacy
 * libcanard CanardCANFrame (same FlagEFF/FlagRTR bits, MaskExtID, data[8], dlc).
 */

#include <stdint.h>

namespace px4can
{

// Re-export fixed-width integer types so driver code written as `px4can::uint32_t`
// (or `uavcan::uint32_t` via a local alias) resolves here.
using ::int8_t;
using ::int16_t;
using ::int32_t;
using ::int64_t;
using ::uint8_t;
using ::uint16_t;
using ::uint32_t;
using ::uint64_t;

/**
 * This limit is defined by the DroneCAN/UAVCAN specification.
 */
enum { MaxCanIfaces = 3 };

// ---- small trait / container helpers the drivers reference -------------------

template <bool> struct StaticAssert;
template <> struct StaticAssert<true> { static void check() { } };

class Noncopyable
{
	Noncopyable(const Noncopyable &) = delete;
	Noncopyable &operator=(const Noncopyable &) = delete;
protected:
	Noncopyable() = default;
	~Noncopyable() = default;
};

template <typename T> inline const T &min(const T &a, const T &b) { return (b < a) ? b : a; }
template <typename T> inline const T &max(const T &a, const T &b) { return (a < b) ? b : a; }

template <typename In, typename Out>
inline Out copy(In first, In last, Out out) { while (first != last) { *out++ = *first++; } return out; }

template <typename Fwd, typename T>
inline void fill(Fwd first, Fwd last, const T &value) { while (first != last) { *first++ = value; } }

template <typename Out, typename Size, typename T>
inline Out fill_n(Out first, Size n, const T &value)
{
	// Count via an integer so Size may be an enum (e.g. a driver's NumTxMailboxes).
	for (unsigned long count = static_cast<unsigned long>(n); count > 0; --count) { *first++ = value; }

	return first;
}

template <typename In1, typename In2>
inline bool equal(In1 first, In1 last, In2 other) { while (first != last) { if (!(*first++ == *other++)) { return false; } } return true; }

// ---- time (neutralized copy of uavcan/time.hpp) ------------------------------

template <typename D>
class DurationBase
{
	int64_t usec_;

protected:
	~DurationBase() { }
	DurationBase() : usec_(0) { static_assert(sizeof(D) == 8, "Duration must be 8 bytes"); }

public:
	static D getInfinite() { return fromUSec(INT64_MAX); }

	static D fromUSec(int64_t us) { D d; d.usec_ = us; return d; }
	static D fromMSec(int64_t ms) { return fromUSec(ms * 1000); }

	int64_t toUSec() const { return usec_; }
	int64_t toMSec() const { return usec_ / 1000; }

	D getAbs() const { return D::fromUSec((usec_ < 0) ? (-usec_) : usec_); }

	bool isPositive() const { return usec_ > 0; }
	bool isNegative() const { return usec_ < 0; }
	bool isZero() const { return usec_ == 0; }

	bool operator==(const D &r) const { return usec_ == r.usec_; }
	bool operator!=(const D &r) const { return !operator==(r); }
	bool operator<(const D &r) const { return usec_ < r.usec_; }
	bool operator>(const D &r) const { return usec_ > r.usec_; }
	bool operator<=(const D &r) const { return usec_ <= r.usec_; }
	bool operator>=(const D &r) const { return usec_ >= r.usec_; }

	D operator+(const D &r) const { return fromUSec(usec_ + r.usec_); }
	D operator-(const D &r) const { return fromUSec(usec_ - r.usec_); }
	D operator-() const { return fromUSec(-usec_); }

	D &operator+=(const D &r) { *this = *this + r; return *static_cast<D *>(this); }
	D &operator-=(const D &r) { *this = *this - r; return *static_cast<D *>(this); }

	template <typename Scale> D operator*(Scale scale) const { return fromUSec(usec_ * scale); }
	template <typename Scale> D &operator*=(Scale scale) { *this = *this * scale; return *static_cast<D *>(this); }
};

template <typename T, typename D>
class TimeBase
{
	uint64_t usec_;

protected:
	~TimeBase() { }
	TimeBase() : usec_(0) { static_assert(sizeof(T) == 8 && sizeof(D) == 8, "Time/Duration must be 8 bytes"); }

public:
	static T getMax() { return fromUSec(UINT64_MAX); }

	static T fromUSec(uint64_t us) { T d; d.usec_ = us; return d; }
	static T fromMSec(uint64_t ms) { return fromUSec(ms * 1000); }

	uint64_t toUSec() const { return usec_; }
	uint64_t toMSec() const { return usec_ / 1000; }

	bool isZero() const { return usec_ == 0; }

	bool operator==(const T &r) const { return usec_ == r.usec_; }
	bool operator!=(const T &r) const { return !operator==(r); }
	bool operator<(const T &r) const { return usec_ < r.usec_; }
	bool operator>(const T &r) const { return usec_ > r.usec_; }
	bool operator<=(const T &r) const { return usec_ <= r.usec_; }
	bool operator>=(const T &r) const { return usec_ >= r.usec_; }

	T operator+(const D &r) const
	{
		if (r.isNegative()) {
			if (uint64_t(r.getAbs().toUSec()) > usec_) { return fromUSec(0); }

		} else {
			if (uint64_t(int64_t(usec_) + r.toUSec()) < usec_) { return fromUSec(UINT64_MAX); }
		}

		return fromUSec(uint64_t(int64_t(usec_) + r.toUSec()));
	}

	T operator-(const D &r) const { return *static_cast<const T *>(this) + (-r); }
	D operator-(const T &r) const
	{
		return D::fromUSec((usec_ > r.usec_) ? int64_t(usec_ - r.usec_) : -int64_t(r.usec_ - usec_));
	}

	T &operator+=(const D &r) { *this = *this + r; return *static_cast<T *>(this); }
	T &operator-=(const D &r) { *this = *this - r; return *static_cast<T *>(this); }
};

class MonotonicDuration : public DurationBase<MonotonicDuration> { };
class MonotonicTime : public TimeBase<MonotonicTime, MonotonicDuration> { };
class UtcDuration : public DurationBase<UtcDuration> { };
class UtcTime : public TimeBase<UtcTime, UtcDuration> { };

/**
 * Abstract monotonic/UTC system clock (provided by the concrete driver).
 */
class ISystemClock
{
public:
	virtual ~ISystemClock() { }
	virtual MonotonicTime getMonotonic() const = 0;
	virtual UtcTime getUtc() const = 0;
	virtual void adjustUtc(UtcDuration adjustment) = 0;
};

// ---- CAN frame + interfaces (neutralized copy of uavcan/driver/can.hpp) -------

/**
 * Raw CAN frame, as passed to/from the CAN driver.
 */
struct CanFrame {
	static constexpr uint32_t MaskStdID = 0x000007FFU;
	static constexpr uint32_t MaskExtID = 0x1FFFFFFFU;
	static constexpr uint32_t FlagEFF = 1U << 31;   ///< Extended frame format
	static constexpr uint32_t FlagRTR = 1U << 30;   ///< Remote transmission request
	static constexpr uint32_t FlagERR = 1U << 29;   ///< Error frame

	static constexpr uint8_t MaxDataLen = 8;

	uint32_t id;                ///< CAN ID with flags (above)
	uint8_t data[MaxDataLen];
	uint8_t dlc;                ///< Data Length Code

	CanFrame() : id(0), dlc(0) { fill(data, data + MaxDataLen, uint8_t(0)); }

	CanFrame(uint32_t can_id, const uint8_t *can_data, uint8_t data_len) :
		id(can_id),
		dlc((data_len > MaxDataLen) ? MaxDataLen : data_len)
	{
		(void)copy(can_data, can_data + dlc, this->data);
	}

	bool operator!=(const CanFrame &rhs) const { return !operator==(rhs); }
	bool operator==(const CanFrame &rhs) const
	{
		return (id == rhs.id) && (dlc == rhs.dlc) && equal(data, data + dlc, rhs.data);
	}

	bool isExtended()                  const { return id & FlagEFF; }
	bool isRemoteTransmissionRequest() const { return id & FlagRTR; }
	bool isErrorFrame()                const { return id & FlagERR; }

	/**
	 * CAN frame arbitration rules, particularly STD vs EXT. Ported verbatim from
	 * libuavcan (uc_can.cpp).
	 */
	bool priorityHigherThan(const CanFrame &rhs) const
	{
		const uint32_t clean_id     = id     & MaskExtID;
		const uint32_t rhs_clean_id = rhs.id & MaskExtID;

		// STD vs EXT - if 11 most significant bits are the same, EXT loses.
		const bool ext     = id     & FlagEFF;
		const bool rhs_ext = rhs.id & FlagEFF;

		if (ext != rhs_ext) {
			const uint32_t arb11     = ext     ? (clean_id >> 18)     : clean_id;
			const uint32_t rhs_arb11 = rhs_ext ? (rhs_clean_id >> 18) : rhs_clean_id;

			if (arb11 != rhs_arb11) {
				return arb11 < rhs_arb11;

			} else {
				return rhs_ext;
			}
		}

		// RTR vs Data frame - if frame identifiers and frame types are the same, RTR loses.
		const bool rtr     = id     & FlagRTR;
		const bool rhs_rtr = rhs.id & FlagRTR;

		if (clean_id == rhs_clean_id && rtr != rhs_rtr) {
			return rhs_rtr;
		}

		// Plain ID arbitration - greater value loses.
		return clean_id < rhs_clean_id;
	}

	bool priorityLowerThan(const CanFrame &rhs) const { return rhs.priorityHigherThan(*this); }
};

/**
 * CAN hardware filter config struct.
 */
struct CanFilterConfig {
	uint32_t id;
	uint32_t mask;

	bool operator==(const CanFilterConfig &rhs) const { return rhs.id == id && rhs.mask == mask; }

	CanFilterConfig() : id(0), mask(0) { }
};

/**
 * Events to look for during ICanDriver::select(). Bit position == iface index.
 */
struct CanSelectMasks {
	uint8_t read;
	uint8_t write;

	CanSelectMasks() : read(0), write(0) { }
};

/**
 * Special IO flags. Loopback echoes the frame back to RX; AbortOnError aborts TX on
 * the first bus error (anonymous-message CSMA), per the spec.
 */
typedef uint16_t CanIOFlags;
static constexpr CanIOFlags CanIOFlagLoopback = 1;
static constexpr CanIOFlags CanIOFlagAbortOnError = 2;

/**
 * Single non-blocking CAN interface.
 */
class ICanIface
{
public:
	virtual ~ICanIface() { }

	/// Non-blocking TX. 1 = transmitted, 0 = TX buffer full, negative = error.
	virtual int16_t send(const CanFrame &frame, MonotonicTime tx_deadline, CanIOFlags flags) = 0;

	/// Non-blocking RX. 1 = received, 0 = RX buffer empty, negative = error.
	virtual int16_t receive(CanFrame &out_frame, MonotonicTime &out_ts_monotonic, UtcTime &out_ts_utc,
				CanIOFlags &out_flags) = 0;

	virtual int16_t configureFilters(const CanFilterConfig *filter_configs, uint16_t num_configs) = 0;
	virtual uint16_t getNumFilters() const = 0;
	virtual uint64_t getErrorCount() const = 0;
};

/**
 * Generic CAN driver, incorporating all available interfaces.
 */
class ICanDriver
{
public:
	virtual ~ICanDriver() { }

	virtual ICanIface *getIface(uint8_t iface_index) = 0;
	virtual const ICanIface *getIface(uint8_t iface_index) const
	{
		return const_cast<ICanDriver *>(this)->getIface(iface_index);
	}

	virtual uint8_t getNumIfaces() const = 0;

	virtual int16_t select(CanSelectMasks &inout_masks,
			       const CanFrame * (&pending_tx)[MaxCanIfaces],
			       MonotonicTime blocking_deadline) = 0;
};

} // namespace px4can
