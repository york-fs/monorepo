#pragma once

#include <util/flag_bitset.hh>

#include <cstdint>

namespace bms {

/**
 * @brief Flags that specify an error condition on the BMS master.
 */
enum class MasterError : std::uint16_t {
    /**
     * @brief Specifies that the CAN bus is unavailable or unreliable.
     */
    CanOffline,

    /**
     * @brief Specifies that there is no usable config available.
     */
    NoConfig,

    /**
     * @brief Specifies that a task has overrun its deadline.
     */
    DeadlineOverrun,

    /**
     * @brief Specifies that the onboard precision reference used for current sensing is bad.
     */
    BadReference,

    /**
     * @brief Specifies that the master board is overtemperature.
     */
    Overtemperature,

    /**
     * @brief Specifies that a current sensor's zero voltage is bad.
     */
    BadCurrentSensor,

    /**
     * @brief Specifies that an overcurrent has been detected by one or both of the current sensors' overcurrent pins.
     */
    OvercurrentThreshold,

    /**
     * @brief Specifies that an overcurrent has been measured by one or both of the current sensors.
     */
    OvercurrentMeasured,

    /**
     * @brief Specifies that one or more segments have one or more errors.
     */
    SegmentError,

    /**
     * @brief Specifies that the number of segments connected is not as expected, for example due to communication
     * dropout or segment power failure.
     */
    BadSegmentCount,
};

using MasterErrorFlags = util::FlagBitset<MasterError>;

/**
 * @brief Flags that specify an error condition on a BMS segment.
 */
enum class SegmentError : std::uint16_t {
    /**
     * @brief Specifies that the segment is disconnected and not responding.
     */
    Disconnected,

    /**
     * @brief Specifies that the number of connected cells to the segment is not equal to the expected amount set in the
     * master's config.
     */
    BadCellCount,

    /**
     * @brief Specifies that available number of thermistors connected to the segment is less than the expected amount
     * set in the master's config.
     */
    BadThermistorCount,

    /**
     * @brief Specifies that the segment's power rail voltage is out of tolerance.
     */
    BadRailVoltage,

    /**
     * @brief Specifies that the segment sent an incorrect number of bytes or corrupted data.
     */
    UnreliableCommunication,

    /**
     * @brief Specifies that the segment's data is unreliable due to degraded cell voltage readings or bad rail voltage.
     */
    UnreliableMeasurement,

    /**
     * @brief Specifies that one or more cells is in an undervoltage condition.
     */
    Undervoltage,

    /**
     * @brief Specifies that one or more cells is in an overvoltage condition.
     */
    Overvoltage,

    /**
     * @brief Specifies that one or more cells is in an undertemperature condition.
     */
    Undertemperature,

    /**
     * @brief Specifies that one or more cells is in an overtemperature condition.
     */
    Overtemperature,
};

using SegmentErrorFlags = util::FlagBitset<SegmentError>;

} // namespace bms
