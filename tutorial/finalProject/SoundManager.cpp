#include <windows.h>
#include <cstdio>
#include <iostream>

#include "SoundManager.h"
#include <string>
#include <mmsystem.h>
#pragma comment( lib, "Winmm.lib" )

SoundManager::SoundManager()
{
    engine = createIrrKlangDevice();
    if (!engine)
        return;
}

void SoundManager::PlayHitSound()
{
    engine->play2D("C:\\Users\\grubm\\OneDrive\\Desktop\\Itay\\School\\3D\\Animation\\build\\Sounds\\box.wav");
}
void SoundManager::PlayPickupSound()
{    
    engine->play2D("C:\\Users\\grubm\\OneDrive\\Desktop\\Itay\\School\\3D\\Animation\\build\\Sounds\\food.wav");
}
void SoundManager::PlayHealthSound()
{
    engine->play2D("C:\\Users\\grubm\\OneDrive\\Desktop\\Itay\\School\\3D\\Animation\\build\\Sounds\\health.wav");
}
void SoundManager::PlayGameSound()
{
    engine->play2D("C:\\Users\\grubm\\OneDrive\\Desktop\\Itay\\School\\3D\\Animation\\build\\Sounds\\game.wav",true);
}
void SoundManager::PlayNextLevel()
{
    engine->stopAllSounds();
    engine->play2D("C:\\Users\\grubm\\OneDrive\\Desktop\\Itay\\School\\3D\\Animation\\build\\Sounds\\nextLevel.wav");
}
void SoundManager::PlayGameEndSound()
{
    engine->stopAllSounds();
    engine->play2D("C:\\Users\\grubm\\OneDrive\\Desktop\\Itay\\School\\3D\\Animation\\build\\Sounds\\gameOver.wav");
}
void SoundManager::PlayGameWinnerSound()
{
    engine->stopAllSounds();
    auto x = engine->play2D("C:\\Users\\grubm\\OneDrive\\Desktop\\Itay\\School\\3D\\Animation\\build\\Sounds\\gameWinner.wav");
}
