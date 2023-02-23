#pragma once


#include <iostream>
#include "irrKlang-64bit-1.6.0/include/irrKlang.h"
#include <windows.h>
#include <mmsystem.h>
#include <string>
#pragma comment( lib, "Winmm.lib" )
#pragma comment(lib, "irrKlang.lib")

using namespace irrklang;

class SoundManager {

public:
    SoundManager();
    void PlayHitSound();
    void PlayPickupSound();
    void PlayHealthSound();
    void PlayGameSound();
    void PlayNextLevel();
    void PlayGameEndSound();
    void PlayGameWinnerSound();

private:
    ISoundEngine* engine;
};