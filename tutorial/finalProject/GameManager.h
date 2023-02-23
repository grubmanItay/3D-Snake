#pragma once

#include <utility>
#include <vector>
#include <string>
#include <cstdint>
#include "Movable.h"
#include "GameObject.h"
#include "SoundManager.h"


class Snake;

namespace Game{
    class SpawnManager;

    class GameManager {


        public:
            
            GameManager(SpawnManager* spawnManager, SoundManager* soundManager);

            void GameStart();
            void GameEnd();
            bool GetisStarted() { return isStarted; }
            void Nextlevel();
            void Restart();
            void IncreaseScore(float amount);
            void DecreaseScore(float amount);
            float GetHighScore();
            float GetScore();
            int GetCurrlevel();            
            void SetHighScore(float amount);

            SpawnManager* spawnManager;
            SoundManager* soundManager;
            std::vector<std::shared_ptr<Game::GameObject>> gameObjects;
            std::shared_ptr<Snake> snake;
            SnakeGame* scene;
            bool shouldSpawnNextlevel;

        private:

            float score;
            int currlevel;
            float highScore;
            bool isStarted = false;
            
            void InitValues();
            void SetScore(float amount);

    };

}