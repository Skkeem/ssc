#ifndef SSC_h
#define SSC_h

#include <inttypes.h>

typedef enum {SERIAL, PWM} TYPE;

class SSCClass
{
public:
  SSCClass();
  void attachSourceSERIAL(unsigned int, int, int);
  void attachSourcePWM(int, int, int, int, int);
  void attachSelect(void);
  void attachControlPWM(unsigned int);

  int getSource(void);
  void putControl(int);
};

extern SSCClass SSC;

#endif
