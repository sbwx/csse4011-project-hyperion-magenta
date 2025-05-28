#ifndef TONES
#define TONES

/* Tone definition */
struct tone {
    float frequency;      // Frequency in Hz
    uint32_t duration_ms; // Duration in milliseconds
};

/* Note frequencies (Hz) */
#define NOTE_C0   16.35f
#define NOTE_CS0  17.32f  // C#0 / Db0
#define NOTE_D0   18.35f
#define NOTE_DS0  19.45f
#define NOTE_E0   20.60f
#define NOTE_F0   21.83f
#define NOTE_FS0  23.12f
#define NOTE_G0   24.50f
#define NOTE_GS0  25.96f
#define NOTE_A0   27.50f
#define NOTE_AS0  29.14f
#define NOTE_B0   30.87f

#define NOTE_C1   32.70f
#define NOTE_CS1  34.65f
#define NOTE_D1   36.71f
#define NOTE_DS1  38.89f
#define NOTE_E1   41.20f
#define NOTE_F1   43.65f
#define NOTE_FS1  46.25f
#define NOTE_G1   49.00f
#define NOTE_GS1  51.91f
#define NOTE_A1   55.00f
#define NOTE_AS1  58.27f
#define NOTE_B1   61.74f

#define NOTE_C2   65.41f
#define NOTE_CS2  69.30f
#define NOTE_D2   73.42f
#define NOTE_DS2  77.78f
#define NOTE_E2   82.41f
#define NOTE_F2   87.31f
#define NOTE_FS2  92.50f
#define NOTE_G2   98.00f
#define NOTE_GS2  103.83f
#define NOTE_A2   110.00f
#define NOTE_AS2  116.54f
#define NOTE_B2   123.47f

#define NOTE_C3   130.81f
#define NOTE_CS3  138.59f
#define NOTE_D3   146.83f
#define NOTE_DS3  155.56f
#define NOTE_E3   164.81f
#define NOTE_F3   174.61f
#define NOTE_FS3  185.00f
#define NOTE_G3   196.00f
#define NOTE_GS3  207.65f
#define NOTE_A3   220.00f
#define NOTE_AS3  233.08f
#define NOTE_B3   246.94f

#define NOTE_C4   261.63f
#define NOTE_CS4  277.18f
#define NOTE_D4   293.66f
#define NOTE_DS4  311.13f
#define NOTE_E4   329.63f
#define NOTE_F4   349.23f
#define NOTE_FS4  369.99f
#define NOTE_G4   392.00f
#define NOTE_GS4  415.30f
#define NOTE_A4   440.00f
#define NOTE_AS4  466.16f
#define NOTE_B4   493.88f

#define NOTE_C5   523.25f
#define NOTE_CS5  554.37f
#define NOTE_D5   587.33f
#define NOTE_DS5  622.25f
#define NOTE_E5   659.26f
#define NOTE_F5   698.46f
#define NOTE_FS5  739.99f
#define NOTE_G5   783.99f
#define NOTE_GS5  830.61f
#define NOTE_A5   880.00f
#define NOTE_AS5  932.33f
#define NOTE_B5   987.77f

#define NOTE_C6   1046.50f
#define NOTE_CS6  1108.73f
#define NOTE_D6   1174.66f
#define NOTE_DS6  1244.51f
#define NOTE_E6   1318.51f
#define NOTE_F6   1396.91f
#define NOTE_FS6  1479.98f
#define NOTE_G6   1567.98f
#define NOTE_GS6  1661.22f
#define NOTE_A6   1760.00f
#define NOTE_AS6  1864.66f
#define NOTE_B6   1975.53f

#define NOTE_C7   2093.00f
#define NOTE_CS7  2217.46f
#define NOTE_D7   2349.32f
#define NOTE_DS7  2489.02f
#define NOTE_E7   2637.02f
#define NOTE_F7   2793.83f
#define NOTE_FS7  2959.96f
#define NOTE_G7   3135.96f
#define NOTE_GS7  3322.44f
#define NOTE_A7   3520.00f
#define NOTE_AS7  3729.31f
#define NOTE_B7   3951.07f
#define PAUSE     0.1f

#endif