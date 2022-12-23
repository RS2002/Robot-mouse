from PPO_Sim import PPO_SimModel
from PPO_Agent import PPO,Memory
import torch
import matplotlib.pyplot as plt

state_dim=19
action_dim=12
action_std=0.5  # constant std for action distribution (Multivariate Normal) ???
lr=1e-4
betas=(0.9, 0.999)
gamma=0.99
K_epochs=80
eps_clip=0.2
max_steps=2000
max_episodes = 100000  # max training episodes
update_step = 50000  # update policy every n steps
load_model=False

if __name__ == '__main__':
    theMouse = PPO_SimModel("./models/dynamic_4l_t3.xml",max_steps)
    memory=Memory()
    agent = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip)
    if load_model:
        agent.policy.load_state_dict(torch.load("./model_parameters/PPO.pkl"))
    distance=[]
    steps=0
    for i_episode in range(1, max_episodes + 1):
        state=theMouse.reset()
        done=False
        while not done:
            steps+=1
            action = agent.select_action(state, memory)
            state, reward, done, pos = theMouse.runStep(action)
            memory.rewards.append(reward)
            memory.is_terminals.append(done)
            if steps % update_step == 0:
                agent.update(memory)
                memory.clear_memory()
                steps=0
            if done:
                distance.append(-pos[1])
                break
        if i_episode % 50 == 0:
            print("Episode"+str(i_episode)+":"+str(distance[-1]))
            torch.save(agent.policy.state_dict(), "./model_parameters/PPO.pkl")
    plt.plot(distance)
    plt.show()
    #glfw.terminate()