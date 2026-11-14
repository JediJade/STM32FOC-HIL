#include "console.h"
#include "usb_cdc.h"
#include "pwmin.h"
#include "sensors.h"
#include "iout.h"
#include "vmotor.h"
#include "foc.h"

#define HIL_VERSION   1

/* ------------------------------------------------------- formatting ----- */

static char *put_str(char *p, const char *s)
{
  while (*s != '\0') {
    *p++ = *s++;
  }
  return p;
}

static char *put_int(char *p, int32_t v)
{
  char tmp[12];
  int  n = 0;
  uint32_t u;

  if (v < 0) {
    *p++ = '-';
    u = (uint32_t)(-v);
  } else {
    u = (uint32_t)v;
  }
  do {
    tmp[n++] = (char)('0' + (u % 10u));
    u /= 10u;
  } while (u != 0u);
  while (n > 0) {
    *p++ = tmp[--n];
  }
  return p;
}

/*
 * One call does not always take the lot. The CDC driver caps a packet at 63
 * bytes - the drive firmware found out the hard way that a reply of exactly
 * 64 arrives one request late, because the host keeps waiting for a short
 * packet that never comes - so anything longer has to go out in pieces.
 *
 * The guard is there because this runs in the main loop: if the host has
 * stopped reading, the console gives up on the line rather than hanging the
 * board.
 */
static void emit(const char *buf, uint16_t len)
{
  uint16_t sent = 0u;
  uint32_t idle = 0u;

  while (sent < len) {
    uint16_t n = usb_cdc_write((const uint8_t *)buf + sent,
                               (uint16_t)(len - sent));

    if (n == 0u) {
      if (++idle > 200000u) {
        return;
      }
    } else {
      sent += n;
      idle = 0u;
    }
  }
}

static void line(const char *text)
{
  char buf[96];
  char *p = put_str(buf, text);

  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

/*
 * One `name value ` pair, and the end of the buffer it may not pass.
 *
 * The state line is where every new feature adds a field, and it had
 * already grown past the buffer it was built in: the overflow wrote over
 * console_poll's return address and the board took a UsageFault, but only
 * once the drive was turning, because only then were the numbers long
 * enough to reach the end. A truncated line is a cheap bug to notice. A
 * smashed stack is not.
 *
 * Twelve bytes of headroom: the longest int32 is eleven characters plus
 * the space, and emit() still needs two for the line ending.
 */
static char *pair_lim(char *p, const char *end, const char *name,
                      int32_t value)
{
  if ((end - p) < 24) {
    return p;
  }
  p = put_str(p, name);
  *p++ = ' ';
  p = put_int(p, value);
  *p++ = ' ';
  return p;
}

/* ---------------------------------------------------------- the state --- */

static uint8_t telemetry;
static uint32_t next_telem;
static int32_t lock_setting;

static void reply_int(const char *name, int32_t value)
{
  char buf[32];
  char *p = put_str(buf, name);

  *p++ = ' ';
  p = put_int(p, value);
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void print_state(void)
{
  char buf[256];
  const char *end = buf + sizeof(buf) - 4;
  char *p = put_str(buf, "s ");
  int16_t va, vb, vc, ia, ib;
  uint16_t period, high;

  pwmin_phase_voltages(&va, &vb, &vc);
  vmotor_phase_currents(&ia, &ib);

  /*
   * The duties go out in the drive's own units - a compare value out of
   * 1800 - not in this board's timer ticks. One window then reads both
   * boards without converting anything, and a duty on this screen is the
   * number the drive wrote into its own CCR.
   */
  for (int i = 0; i < 3; i++) {
    int32_t duty = 0;

    pwmin_raw(i, &period, &high);
    if (period != 0u) {
      duty = ((int32_t)high * 1800) / (int32_t)period;
    }
    p = pair_lim(p, end, (i == 0) ? "da" : ((i == 1) ? "db" : "dc"), duty);
  }
  p = pair_lim(p, end, "per", period);
  p = pair_lim(p, end, "va", va);
  p = pair_lim(p, end, "vb", vb);
  p = pair_lim(p, end, "vc", vc);
  p = pair_lim(p, end, "ang", (int32_t)vmotor_true_angle());
  /* units per ms, the same unit the drive speaks, so the two state lines
     can be read side by side. vmotor keeps it as units per period in Q8. */
  p = pair_lim(p, end, "spd", (vmotor_true_speed() * 20) >> 8);
  p = pair_lim(p, end, "ia", ia);
  p = pair_lim(p, end, "ib", ib);
  p = pair_lim(p, end, "hall", sensors_hall_code());
  p = pair_lim(p, end, "live", pwmin_live());
  p = pair_lim(p, end, "moe", pwmin_outputs_on());
  p = pair_lim(p, end, "ovl", (int32_t)pwmin_overlaps_total());
  p = pair_lim(p, end, "late", (int32_t)hil_late_ticks());
  p = pair_lim(p, end, "tick", (int32_t)hil_model_ticks());
  p = pair_lim(p, end, "iout", iout_enabled());
  p = pair_lim(p, end, "oa", (int32_t)iout_duty_a());
  p = pair_lim(p, end, "ob", (int32_t)iout_duty_b());
  p = pair_lim(p, end, "load", vmotor_load());
  p = pair_lim(p, end, "lock", lock_setting);
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void print_duties(void)
{
  char buf[120];
  char *p = put_str(buf, "d ");

  for (int i = 0; i < 3; i++) {
    uint16_t period, high;

    pwmin_raw(i, &period, &high);
    p = pair_lim(p, buf + sizeof(buf) - 4, (i == 0) ? "a" : ((i == 1) ? "b" : "c"), high);
    p = pair_lim(p, buf + sizeof(buf) - 4, "of", period);
  }
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void print_help(void)
{
  line("commands:");
  line("  s            state: duties, voltages, angle, speed, currents, hall");
  line("  d            the three raw captures: high time of period");
  line("  m            toggle 10 Hz telemetry");
  line("  ver          hil <version>");
  line("  set mot <n>  motor model: 0 gimbal, 1 hub");
  line("  set load <n> drag on the shaft, 0 none; 1 is the hub motor");
  line("  set lock <n> 0 free, 1 shaft clamped");
  line("  set hwid <n> how much of a turn a Hall sensor stays on");
  line("  set dead <n> dead-time added back, timer ticks; 72 = 1.000 us");
  line("  set ovl 0    forget the overlaps counted so far");
  line("  set hall <n> drive a fixed hall pattern 0..7; -1 gives the model back");
  line("  set iout <n> 0 off, 1 send the currents out on PA8 and PA9");
  line("  set ispan <n> steps either side of mid-rail at full scale");
  line("");
  line("`ovl` counts samples that caught a phase with both transistors on.");
  line("On a real bridge that is a short across the bus. It must stay 0.");
}

/* ------------------------------------------------------------ parsing --- */

static char    cmd[48];
static uint8_t cmd_len;

static int starts_with(const char *s, const char *word)
{
  while (*word != '\0') {
    if (*s++ != *word++) {
      return 0;
    }
  }
  return 1;
}

static int32_t parse_int(const char *s)
{
  int32_t v = 0;
  int neg = 0;

  while (*s == ' ') { s++; }
  if (*s == '-') { neg = 1; s++; }
  while ((*s >= '0') && (*s <= '9')) {
    v = (v * 10) + (*s++ - '0');
  }
  return neg ? -v : v;
}

static void do_set(const char *args)
{
  if (starts_with(args, "mot ")) {
    int32_t v = parse_int(args + 4);

    vmotor_set_preset((v != 0) ? 1 : 0);
    line((v != 0) ? "mot 1" : "mot 0");
  } else if (starts_with(args, "load ")) {
    vmotor_set_load(parse_int(args + 5));
    reply_int("load", vmotor_load());
  } else if (starts_with(args, "lock ")) {
    lock_setting = (parse_int(args + 5) != 0) ? 1 : 0;
    vmotor_set_mech((lock_setting != 0) ? VM_MECH_LOCKED : VM_MECH_FREE, 0);
    reply_int("lock", lock_setting);
  } else if (starts_with(args, "hwid ")) {
    int32_t v = parse_int(args + 5);

    if (v < 1000)  { v = 1000; }
    if (v > 60000) { v = 60000; }
    sensors_hall_width((uint16_t)v);
    line("hwid set");
  } else if (starts_with(args, "dead ")) {
    int32_t v = parse_int(args + 5);

    if (v < 0)   { v = 0; }
    if (v > 500) { v = 500; }
    pwmin_deadtime((uint16_t)v);
    line("dead set");
  } else if (starts_with(args, "iout ")) {
    iout_enable(parse_int(args + 5) != 0);
    reply_int("iout", iout_enabled());
  } else if (starts_with(args, "ispan ")) {
    iout_set_span(parse_int(args + 6));
    reply_int("ispan", iout_span());
  } else if (starts_with(args, "hall ")) {
    int32_t v = parse_int(args + 5);

    sensors_force((v < 0) ? -1 : (int)v);
    line((v < 0) ? "hall from the model" : "hall forced");
  } else if (starts_with(args, "ovl ")) {
    pwmin_overlaps_clear();
    line("ovl 0");
  } else {
    line("? unknown setting");
  }
}

static void dispatch(void)
{
  if (cmd_len == 0U) {
    return;
  }
  if ((cmd[0] == '?') || starts_with(cmd, "help")) {
    print_help();
  } else if (starts_with(cmd, "ver")) {
    char buf[32];
    char *p = put_str(buf, "hil ");

    p = put_int(p, HIL_VERSION);
    *p++ = '\r';
    *p++ = '\n';
    emit(buf, (uint16_t)(p - buf));
  } else if (starts_with(cmd, "set ")) {
    do_set(&cmd[4]);
  } else if (cmd[0] == 'd') {
    print_duties();
  } else if (cmd[0] == 's') {
    print_state();
  } else if (cmd[0] == 'm') {
    telemetry = (uint8_t)((telemetry != 0U) ? 0U : 1U);
    line((telemetry != 0U) ? "telemetry on" : "telemetry off");
  } else {
    line("? for help");
  }
}

void console_init(void)
{
  cmd_len = 0U;
  telemetry = 0U;
  next_telem = 0U;
  lock_setting = 0;
}

void console_poll(void)
{
  uint8_t c;

  if (usb_cdc_configured() == 0) {
    return;
  }
  while (usb_cdc_read_byte(&c) != 0) {
    if ((c == '\r') || (c == '\n')) {
      cmd[cmd_len] = '\0';
      dispatch();
      cmd_len = 0U;
    } else if (cmd_len < (sizeof(cmd) - 1u)) {
      cmd[cmd_len++] = (char)c;
    }
  }
  if ((telemetry != 0U) && (millis() >= next_telem)) {
    next_telem = millis() + 100u;
    print_state();
  }
}
