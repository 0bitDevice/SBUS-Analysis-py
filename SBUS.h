#ifndef SBUS_h
#define SBUS_h

#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00

void process();
int  SBUS_getChannel(int channel);
int  SBUS_getNormalizedChannel(int channel);
int  SBUS_getFailsafeStatus();
int  SBUS_getFrameLoss();
long SBUS_getGoodFrames();
long SBUS_getLostFrames();
long SBUS_getDecoderErrorFrames();

int _channels[18];
int _failsafe;
long _goodFrames;
long _lostFrames;
long _decoderErrorFrames;
};

#endif
