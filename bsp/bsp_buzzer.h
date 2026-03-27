#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "typedef.h"

#define NOTE_D0 (-1)
#define NOTE_D1 523
#define NOTE_D2 587
#define NOTE_D3 659
#define NOTE_D4 698
#define NOTE_D5 784
#define NOTE_D5_ 831
#define NOTE_D6 880
#define NOTE_D6_ 932
#define NOTE_D7 988

#define NOTE_DL1 262
#define NOTE_DL2 294
#define NOTE_DL3 330
#define NOTE_DL4 349
#define NOTE_DL5 392
#define NOTE_DL6 440
#define NOTE_DL7 494

#define NOTE_DH1 1046
#define NOTE_DH1_ 1109
#define NOTE_DH2 1175
#define NOTE_DH3 1318
#define NOTE_DH4 1397
#define NOTE_DH5 1568
#define NOTE_DH5_ 1661
#define NOTE_DH6 1760
#define NOTE_DH6_ 1865
#define NOTE_DH7 1976

void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);
void buzzer_tone(int frequency);
void music();

#endif
