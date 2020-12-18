# BREAK Timing in DMX512-A

This document discusses BREAK and Mark after BREAK (MAB) timing in
[ANSI E1.11 DMX512-A](https://tsp.esta.org/tsp/documents/published_docs.php)
using [serial port](https://en.wikipedia.org/wiki/Serial_port) terms.

A BREAK can be represented by sending a zero (SPACE bits, in serial port terms)
to a serial port that's configured for a specific baud rate, parity setting, and
stop-bit setting. For no parity, only the START bit joins the data bits as part
of the SPACE bits, and the STOP bit(s) become part of the MAB.

For EVEN parity, when sending a zero, the start bit and the parity bit become
part of the SPACE bits, and the STOP bit(s) become part of the MAB. For ODD
parity, only the START bit becomes part of the SPACE bits; the parity bit and
STOP bit(s) become part of the MAB.

This document describes baud rate calculation, however, if a microprocessor can
instead specify bit timings using a divisor, then it is possible to have timings
expressed exactly.

## Bit counts

Serial ports are configured with a varying number of data bits and stop bits.
When sending a zero, the following table shows how many bits appear for each
setting. These are constraints.

**Table 1—Possible bit layouts**

| Format | BREAK Bits | MAB Bits |
| :----- | ---------: | -------: |
| 8N1    |          9 |        1 |
| 8N2    |          9 |        2 |
| 8E1    |         10 |        1 |
| 8E2    |         10 |        2 |
| 8O1    |          9 |        2 |
| 8O2    |          9 |        3 |
| 7E1    |          9 |        1 |
| 7O1    |          8 |        2 |
| 9N1    |         10 |        1 |
| 9E1    |         11 |        1 |
| 9O1    |         10 |        2 |

Note that several of the formats produce the same timings. Also, 8N1 and 8E1
have a MAB time of one bit; for the baud rate table below, these are the rows
used to calculate the associated baud rates, for illustration purposes.

We can use this information to construct a wide variety of theoretical BREAK and
MAB durations using different theoretical baud rates. Baud rates are theoretical
because it is often the case that a microprocessor cannot generate an exact baud
rate; there will be some relative error. BREAK and MAB durations are also
theoretical because of additional practical code considerations outlined in a
later section.

Having said that, if the microprocessor generates baud rates via a system clock
divider, the actual baud rates are mostly irrelevant because the bit time can be
specified instead. See the section below that discusses Cortex-M divisors.

## Valid timings

The
[ANSI E1.11 DMX512-A specification](https://tsp.esta.org/tsp/documents/published_docs.php)
describes a range of valid BREAK and MAB timings. For transmitters, the minimum
BREAK time is 92µs and the typical duration is 176µs; there is no maximum. The
minimum MAB time is 12µs and the maximum is less than 1s.

For RDM (start code 0xCC), the BREAK time can range from 176–352µs and the MAB
time can range from 12–88µs.

For test packets (start code 0x55), the BREAK time can range from 88–120µs and
the MAB time can range from 8-16µs.

The following table shows BREAK and MAB times and which theoretical baud rates
will produce them. The rows are sorted by BREAK time and then by MAB time. Note
that we are not restricted to integer times, but it's useful to use them to
construct this table.

**Table 2—Example timings**

| BREAK time (µs) | MAB time (µs) | Baud Rate (1/bitTime) | Format |
| --------------: | ------------: | --------------------: | :----- |
|            _90_ |           _9_ |             111111.11 | 8E1    |
|            _90_ |          _10_ |                100000 | 8N1    |
|            _90_ |            18 |             111111.11 | 8E2    |
|            _90_ |            20 |                100000 | 8N2    |
|            _90_ |            30 |                100000 | 8O2    |
|              99 |          _11_ |              90909.09 | 8N1    |
|              99 |            22 |              90909.09 | 8N2    |
|              99 |            33 |              90909.09 | 8O2    |
|             100 |          _10_ |                100000 | 8E1    |
|             100 |            20 |                100000 | 8E2    |
|             108 |            12 |              83333.33 | 8N1    |
|             108 |            24 |              83333.33 | 8N2    |
|             108 |            36 |              83333.33 | 8O2    |
|             110 |          _11_ |              90909.09 | 8E1    |
|             110 |            22 |              90909.09 | 8E2    |
|             120 |            12 |              83333.33 | 8E1    |
|             120 |            24 |              83333.33 | 8E2    |
|             160 |            16 |                 62500 | 8E1    |
|             160 |            32 |                 62500 | 8E2    |
|             170 |            17 |              58823.53 | 8E1    |
|             170 |            34 |              58823.53 | 8E2    |
|             171 |            19 |              52631.58 | 8N1    |
|             171 |            38 |              52631.58 | 8N2    |
|             171 |            57 |              52631.58 | 8O2    |
|             180 |            18 |              55555.56 | 8E1    |
|             180 |            20 |                 50000 | 8N1    |
|             180 |            36 |              55555.56 | 8E2    |
|             180 |            40 |                 50000 | 8N2    |
|             180 |            60 |                 50000 | 8O2    |
|             189 |            21 |              47619.05 | 8N1    |
|             189 |            42 |              47619.05 | 8N2    |
|             189 |            63 |              47619.05 | 8O2    |
|             190 |            19 |              52631.58 | 8E1    |
|             190 |            38 |              52631.58 | 8E2    |
|             198 |            22 |              45454.55 | 8N1    |
|             198 |            44 |              45454.55 | 8N2    |
|             198 |            66 |              45454.55 | 8O2    |
|             200 |            20 |                 50000 | 8E1    |
|             200 |            40 |                 50000 | 8E2    |
|             220 |            22 |              45454.55 | 8E1    |
|             220 |            44 |              45454.55 | 8E2    |
|             350 |            35 |              28571.43 | 8E1    |
|             350 |            70 |              28571.43 | 8E2    |
|             351 |            39 |              25641.03 | 8N1    |
|             351 |            78 |              25641.03 | 8N2    |
|             351 |         _117_ |              25641.03 | 8O2    |

Again, the baud rates here are irrelevant if the microprocessor generates baud
rates via a divisor.

## Practical considerations

There are some practical considerations that must be taken into account when
choosing baud rates and the MAB time.

Assuming we limit ourselves to a BREAK time of 92–352µs and a MAB time of
12–88µs, the largest baud rate is 10/92µs (at 8E2), about 108695.65, and the
smallest is 9/352µs (at 8N1 or 8N2), about 25568.18.

If we are limited to only one stop bit (8N1 or 8E1), then the largest baud rate
is 1/12µs, about 83333.33.

Remember that with a serial port, we can only create a BREAK plus MAB using a
single zero character (unless there's built-in BREAK support) plus a delay, and
using the bit layout constraints outlined in Table&nbsp;1.

### Baud rate error

Baud rate generation on a microprocessor is not exact for every choice. There
will be some relative error. On an ARM Cortex-M processor, the baud rate is
generated by this formula: `clock/(16*divider)`. (See
[UART - Cortex-M](https://cortex-m.com/uart/).) This means that, practically,
some baud rates will have an error (if using baud rate to specify timings).

The [Teensy UART](https://www.pjrc.com/teensy/td_uart.html) documentation also
has a discussion on relative error when using specific baud rates.

Additionally, it's possible to choose something close to the baud rate specified
in the table and still get something close to the  desired timings, and with
less (or more) error. For example, for a 22µs bit time, 45500 could be used
instead of 45454.55 to get something close.

Again, that's only if we need to specify baud rates. If we can use a divisor to
specify bit times instead, then as long as the formulas produce integers, the
timings will be expressed exactly.

### Some math for close baud rates on a Cortex-M

An exact baud rate can be generated if `divider` mentioned in the Cortex-M
formula can be expressed as either an integer, exact multiple of 1/32, or
something else that depends on the chip type.

Let's calculate the divider and replace `baud` with `1/bitTime`:
`divider=clock*bitTime/16`

If `divider` is an integer, great. If `divider` can be a multiple of 1/32, then
it is sufficient that `clock*2*bitTime` is an integer. Etc.

### Latency

When actually implementing DMX packet transmission, the MAB is sent before
characters which are sent at the DMX baud rate of 250000. This means that the
UART must change baud rates between the BREAK and the slot data. Practically,
due to interrupt and other code latency, the MAB may be output for a longer
duration than expected. Table&nbsp;2 shows values that are far shorter than the
maximum MAB, so it shouldn't be an issue for receiving equipment; the MAB won't
be much larger, and so shouldn't hit the 1s limit for DMX512-A, nor the 88µs
limit for RDM.

---

Copyright (c) 2018 Shawn Silverman
