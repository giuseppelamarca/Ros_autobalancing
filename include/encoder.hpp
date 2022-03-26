#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__
#include "Arduino.h"
#include <array>

#define CW         1
#define CCW       -1
#define MAX_ENCODERS  5

std::array<int, MAX_ENCODERS> phaseA;
std::array<int, MAX_ENCODERS> phaseB;
std::array<int, MAX_ENCODERS> cnt_encoder;

template <int N>
ICACHE_RAM_ATTR void handleInterruptA() {
    if (digitalRead(phaseA[N]))
        if (digitalRead(phaseB[N]))
            cnt_encoder[N] += CCW;
        else
            cnt_encoder[N] += CW;
    else
        if (digitalRead(phaseB[N]))
            cnt_encoder[N] += CW;
        else
            cnt_encoder[N] += CCW;  
}

template <int N>
ICACHE_RAM_ATTR void handleInterruptB() {
    if (digitalRead(phaseB[N]))
        if (digitalRead(phaseA[N]))
            cnt_encoder[N] += CW;
        else
            cnt_encoder[N] += CCW;
    else
        if (digitalRead(phaseA[N]))
            cnt_encoder[N] += CCW;
        else
            cnt_encoder[N] += CW;  
}

template <int N>
class Encoder{
    public:
        Encoder(int pin_phaseA, int pin_phaseB){
            phaseA[N] = pin_phaseA;
            phaseB[N] = pin_phaseB;
            createInterrupt(phaseA[N], CHANGE, handleInterruptA<N>);
            createInterrupt(phaseB[N], CHANGE, handleInterruptB<N>);
        };
        Encoder(Encoder const&);
        Encoder operator= (Encoder const &);

        void createInterrupt(int pin, int mode, void (*callback)()){
            pinMode(pin, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(pin), callback, mode);
        }

    private:


    public:
        long int getCnt(){return cnt_encoder[N];}
};


#endif