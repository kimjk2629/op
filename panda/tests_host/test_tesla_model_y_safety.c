#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define MIN(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a < _b) ? _a : _b; })

#define MAX(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a > _b) ? _a : _b; })

#define ABS(a) \
 ({ __typeof__ (a) _a = (a); \
   (_a > 0) ? _a : (-_a); })

#define UNUSED(x) (void)(x)

#define CANPACKET_DATA_SIZE_MAX 8U
#include "can_definitions.h"

#define FAULT_RELAY_MALFUNCTION (1U << 0)
uint32_t fault_status_test = 0U;
void fault_occurred(uint32_t fault) { fault_status_test |= fault; }
void fault_recovered(uint32_t fault) { fault_status_test &= ~fault; }

typedef void (*board_set_can_mode)(uint8_t mode);
typedef struct {
  const bool has_obd;
  board_set_can_mode set_can_mode;
} board_t;
#define CAN_MODE_NORMAL 0U
#define CAN_MODE_OBD_CAN2 3U
const board_t board_stub = {.has_obd = false, .set_can_mode = NULL};
const board_t *current_board = &board_stub;

static uint32_t fake_us_clock = 0U;
void puth(unsigned int i) { UNUSED(i); }
uint32_t microsecond_timer_get(void) { return fake_us_clock; }
uint32_t get_ts_elapsed(uint32_t ts, uint32_t ts_last) { return ts - ts_last; }

#define ALLOW_DEBUG
#include "safety.h"

/* ---------------------------------------------------------------------- */
/* test helpers                                                            */
/* ---------------------------------------------------------------------- */
static int tests_run = 0;
static int tests_failed = 0;

#define CHECK(desc, cond) do { \
  tests_run++; \
  if (!(cond)) { \
    tests_failed++; \
    printf("FAIL: %s\n", desc); \
  } else { \
    printf("ok:   %s\n", desc); \
  } \
} while (0)

static CANPacket_t make_pkt(int addr, int bus, int len) {
  CANPacket_t p;
  memset(&p, 0, sizeof(p));
  p.addr = (unsigned int)addr;
  p.bus = (unsigned char)bus;
  /* dlc_to_len[len] == len for len <= 8, so data_len_code == len works for our messages */
  p.data_len_code = (unsigned char)len;
  return p;
}

static CANPacket_t make_di_state(int cruise_state, int autopark_state) {
  CANPacket_t p = make_pkt(0x286, 0, 8);
  p.data[1] = (uint8_t)((cruise_state & 0x7) << 4);
  p.data[3] = (uint8_t)((autopark_state & 0xF) << 1);
  return p;
}

static CANPacket_t make_epas(int angle_tenth_deg) {
  /* angle_meas_new = (((byte4 & 0x3F) << 8) | byte5) - 8192 */
  int raw = angle_tenth_deg + 8192;
  CANPacket_t p = make_pkt(0x370, 0, 8);
  p.data[4] = (uint8_t)((raw >> 8) & 0x3F);
  p.data[5] = (uint8_t)(raw & 0xFF);
  return p;
}

static CANPacket_t make_steering(int desired_angle_tenth_deg, int steer_type, int bus) {
  int raw = desired_angle_tenth_deg + 16384;
  CANPacket_t p = make_pkt(0x488, bus, 4);
  p.data[0] = (uint8_t)((raw >> 8) & 0x7F);
  p.data[1] = (uint8_t)(raw & 0xFF);
  p.data[2] = (uint8_t)((steer_type & 0x3) << 6);
  return p;
}

static CANPacket_t make_das_control(int acc_state, int aeb_event, int raw_accel_max, int raw_accel_min, int bus) {
  CANPacket_t p = make_pkt(0x2b9, bus, 8);
  p.data[1] = (uint8_t)((acc_state & 0xF) << 4);
  p.data[2] = (uint8_t)(aeb_event & 0x3);
  p.data[4] = (uint8_t)((raw_accel_min & 0x1F) << 3);
  p.data[5] = (uint8_t)(((raw_accel_max & 0xF) << 4) | ((raw_accel_min >> 5) & 0xF));
  p.data[6] = (uint8_t)((raw_accel_max >> 4) & 0x1F);
  return p;
}

static CANPacket_t make_eac_monitor(int bus) {
  return make_pkt(0x27d, bus, 3);
}

int main(void) {
  /* ===================== init ===================== */
  int status = set_safety_hooks(SAFETY_TESLA, TESLA_FLAG_MODEL3_Y);
  CHECK("set_safety_hooks(SAFETY_TESLA, MODEL3_Y) succeeds", status == 0);
  CHECK("tesla_model3_y flag set", tesla_model3_y == true);
  CHECK("tesla_my_longitudinal flag NOT set (long control off by default)", tesla_my_longitudinal == false);
  CHECK("rx_checks switched to Model Y/3 set", current_rx_checks == &tesla_my_rx_checks);

  /* ===================== rx: cruise engage ===================== */
  CANPacket_t di_off = make_di_state(1 /* STANDBY */, 0);
  safety_rx_hook(&di_off);
  CHECK("controls_allowed is 0 before cruise engaged", controls_allowed == 0);

  CANPacket_t di_on = make_di_state(2 /* ENABLED */, 0);
  safety_rx_hook(&di_on);
  CHECK("controls_allowed becomes 1 on cruise ENABLED", controls_allowed == 1);

  /* ===================== rx: summon blocks controls ===================== */
  cruise_engaged_prev = false; /* force rising-edge path like a fresh summon session */
  CANPacket_t di_summon = make_di_state(2, 3 /* ACTIVE */);
  safety_rx_hook(&di_summon);
  CHECK("summon ACTIVE blocks cruise_engaged -> controls_allowed 0", controls_allowed == 0);
  CHECK("tesla_my_summon latched true", tesla_my_summon == true);

  /* leave summon state, re-engage normal cruise for the rest of the tests */
  CANPacket_t di_summon_off = make_di_state(2, 0 /* UNAVAILABLE */);
  safety_rx_hook(&di_summon_off);
  CANPacket_t di_reengage = make_di_state(2, 0);
  cruise_engaged_prev = false;
  safety_rx_hook(&di_reengage);
  CHECK("controls_allowed back to 1 after leaving summon", controls_allowed == 1);
  CHECK("tesla_my_summon cleared", tesla_my_summon == false);

  /* ===================== rx: steering angle sample ===================== */
  CANPacket_t epas = make_epas(0);
  safety_rx_hook(&epas);
  CHECK("angle_meas centered near 0 after EPAS3S sample", angle_meas.min <= 0 && angle_meas.max >= 0);

  /* ===================== tx: msg_allowed bus filtering ===================== */
  CANPacket_t steer_wrong_bus = make_steering(0, 1 /* ANGLE_CONTROL */, 1 /* wrong bus */);
  CHECK("steering tx on wrong bus rejected by msg_allowed", safety_tx_hook(&steer_wrong_bus) == 0);

  /* ===================== tx: steering angle 0, controls allowed -> should pass ===================== */
  desired_angle_last = 0;
  vehicle_speed = 0.0f;
  CANPacket_t steer_zero = make_steering(0, 1, 0);
  CHECK("steering tx angle=0 within rate limit accepted", safety_tx_hook(&steer_zero) == 1);

  /* ===================== tx: huge angle jump at standstill -> rate-limit violation ===================== */
  CANPacket_t steer_jump = make_steering(1000 /* 100.0 deg, single-frame jump */, 1, 0);
  CHECK("steering tx large single-frame jump rejected (rate limit)", safety_tx_hook(&steer_jump) == 0);

  /* ===================== tx: steering while controls not allowed -> rejected ===================== */
  controls_allowed = 0;
  desired_angle_last = 0;
  CANPacket_t steer_not_allowed = make_steering(0, 1, 0);
  CHECK("steering tx with ANGLE_CONTROL while controls_allowed=0 rejected", safety_tx_hook(&steer_not_allowed) == 0);
  controls_allowed = 1;

  /* ===================== tx: DAS_control cancel-only path (long control OFF) ===================== */
  desired_angle_last = 0;
  CANPacket_t cancel_ok = make_das_control(13 /* ACC_CANCEL_GENERIC_SILENT */, 0, 375, 375, 0);
  CHECK("DAS_control silent-cancel with inactive accel accepted when long control is off", safety_tx_hook(&cancel_ok) == 1);

  CANPacket_t cancel_bad_state = make_das_control(4 /* ACC_ON, not cancel */, 0, 375, 375, 0);
  CHECK("DAS_control non-cancel acc_state rejected when long control is off", safety_tx_hook(&cancel_bad_state) == 0);

  CANPacket_t cancel_with_accel = make_das_control(13, 0, 400, 375, 0);
  CHECK("DAS_control cancel with non-inactive accel rejected when long control is off", safety_tx_hook(&cancel_with_accel) == 0);

  /* ===================== tx: DAS_control WITH long control enabled ===================== */
  status = set_safety_hooks(SAFETY_TESLA, TESLA_FLAG_MODEL3_Y | TESLA_FLAG_MODEL3_Y_LONG_CONTROL);
  CHECK("set_safety_hooks re-init with LONG_CONTROL succeeds", status == 0);
  CHECK("tesla_my_longitudinal now true", tesla_my_longitudinal == true);
  controls_allowed = 1;

  /* accel within [-3.48, 2.0] m/s^2 range: raw = (accel + 15) / 0.04 */
  int raw_1ms2 = (int)((1.0f + 15.0f) / 0.04f);   /* ~1.0 m/s^2 -> should be allowed */
  int raw_5ms2 = (int)((5.0f + 15.0f) / 0.04f);   /* ~5.0 m/s^2 -> should be rejected, over max */
  CANPacket_t accel_ok = make_das_control(4, 0, raw_1ms2, 375, 0);
  CHECK("DAS_control accel within limits accepted under long control", safety_tx_hook(&accel_ok) == 1);

  CANPacket_t accel_over = make_das_control(4, 0, raw_5ms2, 375, 0);
  CHECK("DAS_control accel above TESLA_MY_MAX_ACCEL rejected", safety_tx_hook(&accel_over) == 0);

  CANPacket_t aeb_flagged = make_das_control(4, 1 /* AEB_ACTIVE, not allowed from us */, raw_1ms2, 375, 0);
  CHECK("DAS_control with AEB event bit set rejected", safety_tx_hook(&aeb_flagged) == 0);

  /* ===================== fwd_hook ===================== */
  CANPacket_t any_bus0 = make_pkt(0x123, 0, 8);
  CHECK("fwd bus0->bus2 always forwards (party -> autopilot)", tesla_fwd_hook(0, &any_bus0) == 2);

  tesla_my_stock_steering_control = false;
  CANPacket_t stock_steer_from_ap = make_steering(0, 1, 2);
  CHECK("fwd bus2->bus0 blocks our own DAS_steeringControl addr when no stock conflict", tesla_fwd_hook(2, &stock_steer_from_ap) == -1);

  CANPacket_t eac_from_ap = make_eac_monitor(2);
  CHECK("fwd bus2->bus0 blocks APS_eacMonitor echo", tesla_fwd_hook(2, &eac_from_ap) == -1);

  CANPacket_t other_from_ap = make_pkt(0x999, 2, 8);
  CHECK("fwd bus2->bus0 forwards unrelated addr", tesla_fwd_hook(2, &other_from_ap) == 0);

  /* ===================== regression: AP1/AP2 path untouched ===================== */
  status = set_safety_hooks(SAFETY_TESLA, 0 /* no flags: plain AP2, no powertrain, no long */);
  CHECK("set_safety_hooks re-init AP1/AP2 (no flags) succeeds", status == 0);
  CHECK("tesla_model3_y is false in AP1/AP2 mode", tesla_model3_y == false);
  CHECK("rx_checks switched back to AP1/AP2 set", current_rx_checks == &tesla_rx_checks);

  CANPacket_t ap_di_state_off = make_pkt(0x368, 0, 8);
  ap_di_state_off.data[1] = (uint8_t)(1 << 4); /* STANDBY */
  safety_rx_hook(&ap_di_state_off);
  CHECK("AP1/AP2: controls_allowed 0 before cruise engaged", controls_allowed == 0);

  CANPacket_t ap_di_state_on = make_pkt(0x368, 0, 8);
  ap_di_state_on.data[1] = (uint8_t)(2 << 4); /* ENABLED */
  safety_rx_hook(&ap_di_state_on);
  CHECK("AP1/AP2: controls_allowed 1 after cruise ENABLED", controls_allowed == 1);

  desired_angle_last = 0;
  vehicle_speed = 0.0f;
  CANPacket_t ap_steer_zero = make_steering(0, 1, 0);
  CHECK("AP1/AP2: steering tx angle=0 accepted", safety_tx_hook(&ap_steer_zero) == 1);

  CANPacket_t ap_steer_jump = make_steering(1000, 1, 0);
  CHECK("AP1/AP2: steering tx large jump rejected (old rate table)", safety_tx_hook(&ap_steer_jump) == 0);

  CANPacket_t ap_steer_wrong_bus = make_steering(0, 1, 1);
  CHECK("AP1/AP2: steering tx on bus 1 rejected by msg_allowed (only bus 0/2 allowed via STW addr, not steering)", safety_tx_hook(&ap_steer_wrong_bus) == 0);

  printf("\n%d/%d checks passed\n", tests_run - tests_failed, tests_run);
  return tests_failed == 0 ? 0 : 1;
}
