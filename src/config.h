#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Regeltakt - die EINZIGE Stelle, an der die Schleifenfrequenz bestimmt wird.
// ---------------------------------------------------------------------------
// Timer2 laeuft weiter mit 20-us-Ticks (1000 Ticks = 20 ms = 50-Hz-ESC-Rahmen,
// unveraendert). CTRL_DIV teilt diesen Takt fuer den Regeltakt:
//
//   CTRL_RATE_HZ  CTRL_DIV  tatsaechlich  Bemerkung
//        100        500      100,00 Hz     alter Standard (exakt)
//        120        416      120,19 Hz
//        125        400      125,00 Hz     exakt
//        160        312      160,26 Hz     Standard
//
// Die tatsaechliche Rate weicht um < 0,2 % von der Nennung ab; alle
// Konstanten rechnen mit der Nennung - dieser Fehler liegt weit unter den
// uebrigen Fehlern und wird von der Acc-Korrektur ohnehin aufgefangen.
//
// CPU-Budget bei 160 Hz (6,25 ms je Takt): Gyro-Lesen (14 Byte ueber
// 400-kHz-I2C ~0,4 ms), Acc-Poll (6 Byte ~0,2 ms alle 2 ms) und Rechnen
// (~0,2 ms) fuellen rund 1,2 ms. Das passt locker; sollten je Takt mehr
// Takte verlorengehen (Monitor zeigt < Nennrate), CTRL_RATE_HZ auf 125
// oder 120 senken.
#define CTRL_RATE_HZ 160
#define CTRL_DIV (50000 / CTRL_RATE_HZ) // 20-us-Ticks je Regeltakt

// ---------------------------------------------------------------------------
// Umrechnung "bei 100 Hz gewaehlt" -> "bei CTRL_RATE_HZ gleich wirksam"
// ---------------------------------------------------------------------------
// Alle Konstanten dieses Projekts wurden pro TAKT gewaehlt (z. B. 8/256 je
// Probe), weil die Schleife 100 Hz lief. Wer die Rate aendert, aendert damit
// sonst jede Zeitkonstante mit:
//
//   PROBE_KEEPS_TAU(v): fuer Glaettungen und Verstaerkungen pro Probe, die
//     die ZEITKONSTANTE halten sollen (ACCEL_ALPHA, BIAS_KI, D_FILTER_ALPHA,
//     MOTOR_LPF, MOTOR_SLEW). Zeitkonstante ~ 256/(v*Rate) -> v ~ 1/Rate,
//     mit kaufmaennischer Rundung.
//
//   PROBE_RAISES_D(v): fuer den D-Anteil. pid.c bildet die Ableitung AUF
//     EINE PROBE (Messdifferenz je Takt); dieselbe physikalische Ableitung
//     ist bei hoeherer Rate pro Probe kleiner. Kd muss also MIT der Rate
//     steigen (sonst waechst der Daempfanteil umgekehrt zur Rate).
//     (Kp ist rate-unabhaengig, Ki waechst wie hier - Ki steht auf 0.)
#define PROBE_KEEPS_TAU(v)                                                     \
  (((int32_t)(v) * 100 + CTRL_RATE_HZ / 2) / CTRL_RATE_HZ)
#define PROBE_RAISES_D(v) (((int32_t)(v) * CTRL_RATE_HZ + 50) / 100)

#endif
