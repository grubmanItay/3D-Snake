## 3D Snake Final project
This project was written as the final project of 3D Animation course by Itay Grubman and Gil Negrin.

#### Game Description: 
The game have 6 levels in total that you must pass to win the game. 
The purpose of the player is to collect as much points as he can possibly without dying, dying is possible if you collide with an obstacle. 
In each level we change background, add more obstacles and pickup items, and increase the speed of the items to make it more difficult.

### This is a 3D snake game with the following features:

1. Objects in the game - There are 3 different type of objects in our game:

* Health - The helath object changes it's location on the screen, and collected when collided with the snake. The health object increases health of the snake by 5 points.

* Score - The score object changes it's location on the screen, and collected when collided with the snake. The score object increases score by 5 points.

* obstacles - The obstacles number increases each level. if the snake collide with them they will decrease his health by 5 points.

2. camera - player can change the camera view to be on top of the head of the snake or above the snake. 

3. Collision detection - we started by creating an AlignedBox3d for each of the joints. In total there are 16 joints that check the collision with each pickup item.

4. Menu - The game multiple states, you can start the game by pressing space or "play" button, the snake will appear along with the obstacle and pickup. the player can pause the game, continue or restart while in play. after the player finishes a level, he can press the "next level" button to move up the levels.

5. Scoring - the player can keep track on his score during the game, and the highest score played in the current session.

6. Music - For each situation in the game we have predefined a music file that is played. there is in-play music, and sounds for each action, health pick up, score pickup, obstacle, next level, losing the game and of course big claps for winning the game.

7. Snake motion - the snake in the game simulates the motion of a real snake, no sharp turns, but real-like motion. the first joints moves in the direction the player presses, the rest of the joints move in the opposite direction in order to cancel the movement of first joints so the snake will not move as one long unit without joints.

