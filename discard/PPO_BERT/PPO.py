from PPO_Sim import PPO_SimModel
from PPO_Agent import PPO,Memory
import torch
# import matplotlib.pyplot as plt

state_dim=14 #10+4
action_dim=14
action_std=0.5  # constant std for action distribution (Multivariate Normal) ???
lr=1e-4
betas=(0.9,0.999)
gamma=0.99
K_epochs=80
eps_clip=0.2
max_steps=5000
max_episodes=3000  # max training episodes
update_step=50000  # update policy every n steps     i.e. (update_step/max_steps) episodes
save_episode=2 # the steps to save the model and test
load_model=False
test=True
enviroment_path="./models/dynamic_4l_t3.xml"

def eval_model():
    theMouse = PPO_SimModel(enviroment_path, max_steps)
    theMouse.set_savepath("ctrldata_PPO")
    agent = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip)
    agent.policy.load_state_dict(torch.load("./PPO.pkl"))
    state = theMouse.reset()
    done = False
    reward_test = 0
    distance_test = 0
    useless_memory = Memory()
    while not done:
        action = agent.select_action(state, useless_memory)
        state, reward, done, pos, action = theMouse.runStep(action)
        reward_test += reward
        if done:
            distance_test = -pos[1]
    print("Test result: distance:", distance_test, "reward:", reward_test)
    # glfw.terminate()

def main():
    theMouse = PPO_SimModel(enviroment_path, max_steps)
    memory = Memory()
    agent = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip)
    if load_model:
        agent.policy.load_state_dict(torch.load("./model_parameters/PPO.pkl"))
    distance = []
    steps = 0
    Step=0
    best_distance=-100
    for i_episode in range(1, max_episodes + 1):
        state = theMouse.reset()
        done = False
        while not done:
            Step+=1
            steps += 1
            action = agent.select_action(state, memory)
            state, reward, done, pos, action = theMouse.runStep(action)
            agent.store(action, memory)
            memory.rewards.append(reward)
            memory.is_terminals.append(done)
            if steps % update_step == 0:
                agent.update(memory)
                memory.clear_memory()
                steps = 0
            if done:
                distance.append(-pos[1])
                break
        if i_episode % save_episode == 0:
            print("Episode: " + str(i_episode) + " The average distance: " + str(sum(distance[-save_episode:]) / save_episode))
            if test:
                useless_memory = Memory()
                state = theMouse.reset()
                done = False
                reward_test = 0
                distance_test = 0
                while not done:
                    action = agent.select_action(state, useless_memory)
                    state, reward, done, pos, action = theMouse.runStep(action)
                    reward_test += reward
                    if done:
                        distance_test = -pos[1]
                print("Step: " + str(Step)+" distance: ", distance_test, "reward: ", reward_test)
                with open("log.txt",'a') as file:
                    file.write("Step: " + str(Step) +" distance: "+str(distance_test)+" reward: "+str(reward_test)+"\n")
                if distance_test>best_distance:
                    best_distance=distance_test
                    # torch.save(agent.policy.state_dict(), "./PPO.pkl")
                    torch.save(agent.policy.state_dict(), "./PPO.pth")

    # plt.plot(distance)
    # plt.show()

def eval():
    theMouse = PPO_SimModel(enviroment_path, max_steps,view=True)
    agent = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip)
    # agent.policy.load_state_dict(torch.load("./PPO.pkl",map_location=torch.device('cpu')))
    agent.policy.load_state_dict(torch.load("./PPO.pkl", map_location=torch.device('cpu')),strict=False)
    state = theMouse.reset()
    done = False
    reward_test = 0
    distance_test = 0
    useless_memory = Memory()

    while not done:
        action = agent.select_action(state, useless_memory)
        state, reward, done, pos, action = theMouse.runStep(action)
        reward_test += reward
        if done:
            distance_test = -pos[1]
    print("Test result: distance:", distance_test, "reward:", reward_test)


if __name__ == '__main__':
    #main()
    eval()
