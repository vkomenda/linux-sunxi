#ifndef __CEC_SUNXI_H
#define __CEC_SUNXI_H

/* CEC version 1.3a timings, µs */
#define CEC_START_LOW      3700
#define CEC_START_LOW_MAX  3900
#define CEC_START_HIGH      800
#define CEC_START_HIGH_MAX 1200
#define CEC_START_MAX      4700
#define CEC_BIT_NOMINAL    2400
#define CEC_BIT_MAX        2750
#define CEC_0_LOW          1500
#define CEC_0_LOW_MAX      1700
#define CEC_0_HIGH          900
#define CEC_1_LOW           600
#define CEC_1_LOW_MAX       800
#define CEC_1_HIGH         1800
#define CEC_ACK_SAMPLE      400
#define CEC_ACK_REST       1400

#define CEC_WAIT_SEND_FAILED    (3 * CEC_BIT_NOMINAL)
#define CEC_WAIT_NEW_INITIATOR  (5 * CEC_BIT_NOMINAL)
#define CEC_WAIT_NEXT_FRAME     (7 * CEC_BIT_NOMINAL)

/* Arbitration and general receive sampling period, µs */
#define SAMPLE_PERIOD 200
/* Start pulse polling period, ms */
#define INTERFRAME_SAMPLE_PERIOD 3
/* High rate polling period, µs */
#define FAST_SAMPLE_PERIOD 100

/* CEC device tag, a.k.a. "logical address". */
enum CecLAddr {
	CecLAddrTV,
	CecLAddrRecorder1,
	CecLAddrRecorder2,
	CecLAddrTuner1,
	CecLAddrPlayer1,
	CecLAddrAudioSystem,
	CecLAddrTuner2,
	CecLAddrTuner3,
	CecLAddrPlayer2,
	CecLAddrRecorder3,
	CecLAddrTuner4,
	CecLAddrPlayer3,
	CecLAddrReserved1,
	CecLAddrReserved2,
	CecLAddrCustom,
	CecLAddrBroadcast,
};

/* CEC device mode on the bus. */
enum CecMode {
	CecModeIdle,
	CecModeInitiator,
	CecModeFollower,
};

#endif /* __CEC_SUNXI_H */
