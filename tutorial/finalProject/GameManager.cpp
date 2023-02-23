#include "GameManager.h"
#include "Util.h"
#include "Snake.h"
#include "SpawnManager.h"
#include "SoundManager.h"



namespace Game{
//TODO implement json configuration loading

    GameManager::GameManager(SpawnManager* spawnManager, SoundManager* soundManager)
    {
        this->soundManager = soundManager;
        this->spawnManager = spawnManager;
        shouldSpawnNextlevel = false;
        this->scene = spawnManager->scene;
        SetHighScore(0);
        InitValues();
        
    }

    void GameManager::InitValues()
    {
        SetScore(0);
        currlevel=0;
    }

    void GameManager::GameStart()
    {
        // init values that are not highscore
        Util::DebugPrint("Game Starting...");
        InitValues(); 
        scene->SetActive();
        isStarted = true;
        scene->snake->InitGameValues();  
        // signal level manager
        Nextlevel();
    }

    void GameManager::GameEnd()
    {
        // launch menu
        // update high score?
        Util::DebugPrint("Score is: " + std::to_string(score));
        // see score and high score
        scene->SetActive(false);
        shouldSpawnNextlevel = false;
        isStarted = false;

        if (currlevel == 6)
            soundManager->PlayGameWinnerSound();
        else
            soundManager->PlayGameEndSound();
    }

    void GameManager::Nextlevel()
    {
        soundManager->PlayGameSound();
        currlevel++;
        scene->InitBackground();
        scene->SetActive(true);
        //TEMP change to last level number
        if(currlevel == 6){
            Util::DebugPrint("Congratulations! you have reached the end!");
            GameEnd();
        }
        else{
            Util::DebugPrint("level Started: " + std::to_string(GetCurrlevel()));
            spawnManager->Spawnlevel(GetCurrlevel());
        }
        shouldSpawnNextlevel=false;
        // signal level manager?
    }

    void GameManager::Restart()
    {
        if (scene->IsActive())
            GameEnd();
        InitValues();
        scene->SetActive();
        isStarted = true;
        scene->snake->InitGameValues();
        Nextlevel();
        
    }

    void GameManager::IncreaseScore(float amount)
    {
        score += amount;
        if (score >GetHighScore())
            SetHighScore(score);
        Util::DebugPrint("score is: " + std::to_string(score));
    }

    void GameManager::DecreaseScore(float amount)
    {
        score = (score-amount)>=0 ? score-amount : 0;
    }

    float GameManager::GetHighScore()
    {
        return highScore;
    }

    float GameManager::GetScore()
    {
        return score;
    }

    int GameManager::GetCurrlevel()
    {
        return currlevel;
    }

    void GameManager::SetHighScore(float amount)
    {
        highScore = amount;
        Util::DebugPrint("High score is: " + std::to_string(score));
    }

    void GameManager::SetScore(float amount){
        score = amount>=0 ? amount : 0;
    }
}