#define BAUD_RATE 9600

typedef struct{
    uint16_t unLed_Pattern;
    uint32_t unLed_Colour;
    uint16_t unBlink_Pattern;
    uint16_t unBuzz_Freq;
    uint8_t unBuzz_time;
    uint16_t unBuzz_Pattern;
    bool bnewData;
}stMessage;


extern stMessage UARTmsg;
