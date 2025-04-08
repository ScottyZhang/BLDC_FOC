#ifndef INA240_H
#define INA240_H

#include <Arduino.h>
#define V_REF           3.3f
#define SHUNT_RES       0.012f
#define INA240_GAIN     20.0f

// GPIO Defines
#define U_READ_PIN  35
#define V_READ_PIN  36 
#define W_READ_PIN  39



class INA240{
    public:
        INA240();
        void INA240_init();
        float getUCurrentValue();
        float getVCurrentValue();
        float getWCurrentValue();

    private:
        float _offset_iu = 0;
        float _offset_iv = 0;
        float _offset_iw = 0;

        void calibrateOffsets();


};


#endif