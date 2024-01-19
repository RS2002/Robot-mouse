from ARS_agent import ARS_agent,Policy,Normalizer
from ARS_env import SimModel

max_steps = 5e6
steps_per_epoch = 5e3
state_dim=14
action_dim=14

def main():
    env = SimModel(modelPath="./models/dynamic_4l_t3.xml",max_steps=steps_per_epoch)
    policy = Policy(state_dim,action_dim)
    normalizer = Normalizer(state_dim)
    agent = ARS_agent(env,policy,normalizer)
    step = 0
    epoch = 0
    while step<max_steps:
        agent.train()
        step+=env.get_step()
        reward=agent.eval()
        current_step=env.get_step()
        print("Step:{}, Episode Timestep:{}, Reward:{:.2f}, Reward Per Step:{:.5f}".format(step,current_step,reward,reward/current_step))
        with open("log.txt", 'a') as file:
            file.write("Step:{}, Episode Timestep:{}, Reward:{:.2f}, Reward Per Step:{:.5f}\n".format(step, current_step,
                                                                                               reward,
                                                                                               reward / current_step))
        if epoch%10==0:
            agent.save("./ARS.npy")
        epoch+=1

def eval():
    env = SimModel(modelPath="./models/my_test.xml",max_steps=steps_per_epoch,render=True)
    policy = Policy(state_dim,action_dim)
    normalizer = Normalizer(state_dim)
    agent = ARS_agent(env,policy,normalizer)
    agent.load("./ARS.npy")
    agent.eval()


if __name__ == '__main__':
    main()
    #eval()