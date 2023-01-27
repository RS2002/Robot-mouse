# Robot-mouse

There are three models in this project: origin, PPO, ETG_RL. We put the codes in different folder in order to classify them. But if you want to run the given model, for example you want to run PPO, please copy the content of the PPO folder to the main folder(Robot-mouse folder).

## origin

The control data of this part is computed by mathematical model. But it can’t cross any obstacles.

## PPO

You can train the model in flat ground first and train it in obstacle scenes next.

## ETG_RL

We write the code of this part based on [PaddleRobotics/QuadrupedalRobots/ETGRL at main · PaddlePaddle/PaddleRobotics · GitHub](https://github.com/PaddlePaddle/PaddleRobotics/tree/main/QuadrupedalRobots/ETGRL). So you can find the way to run this code in the above website.