import torch
import torch.nn as nn
from torch.distributions import MultivariateNormal
from transformers import BertConfig,BertModel
device = torch.device("cuda:4" if torch.cuda.is_available() else "cpu")

class BERT(nn.Module):
    def __init__(self,bertconfig,state_dim):
        super().__init__()
        self.Linear=nn.Linear(state_dim,bertconfig.hidden_size)
        self.bertconfig=bertconfig
        self.bert=BertModel(bertconfig)

    def forward(self,x,attn_mask=None):
        x=self.Linear(x)
        y=self.bert(inputs_embeds=x,attention_mask=attn_mask, output_hidden_states=True)
        y=y.hidden_states[-1]
        return y


class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
        self.next_state_values = []

    def clear_memory(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]
        del self.next_state_values[:]


class ActorCritic(nn.Module):
    def __init__(self, state_dim, bertconfig, action_dim, action_std):
        super(ActorCritic, self).__init__()
        self.bert=BERT(bertconfig,state_dim)
        # action mean range -1 to 1
        self.actor = nn.Sequential(
            nn.Linear(bertconfig.hidden_size, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
            nn.Tanh(),
            nn.Linear(32, action_dim),
            nn.Tanh()
        )
        self.critic = nn.Sequential(
            nn.Linear(bertconfig.hidden_size*bertconfig.max_position_embeddings, 64),
            nn.Tanh(),
            nn.Linear(64, 32),
            nn.Tanh(),
            nn.Linear(32, 1)
        )
        self.action_var = torch.full((action_dim,), action_std * action_std).to(device)

    def store(self,action, memory):
        action=torch.tensor(action)
        action=action.reshape(1,-1).to(device)
        action_logprob = self.dist.log_prob(action)
        memory.actions.append(action)
        memory.logprobs.append(action_logprob)

    def act(self, state, memory):
        action_mean = self.actor(self.bert(state))
        action_mean=action_mean.squeeze()
        action_var = self.action_var.expand_as(action_mean)
        cov_mat = torch.diag_embed(action_var).to(device)
        dist = MultivariateNormal(action_mean, cov_mat)
        self.dist = MultivariateNormal(action_mean[0], torch.diag(self.action_var).to(device))
        action = dist.sample()
        memory.states.append(state)
        return action.detach()

    def evaluate(self, state, action):
        action_mean = self.actor(self.bert(state))
        action_mean = action_mean[:,0,:]

        action_var = self.action_var.expand_as(action_mean)
        # torch.diag_embed(input, offset=0, dim1=-2, dim2=-1) → Tensor
        # Creates a tensor whose diagonals of certain 2D planes (specified by dim1 and dim2) are filled by input
        cov_mat = torch.diag_embed(action_var).to(device)
        # 生成一个多元高斯分布矩阵
        dist = MultivariateNormal(action_mean, cov_mat)
        # 我们的目的是要用这个随机的去逼近真正的选择动作action的高斯分布
        action_logprobs = dist.log_prob(action)
        # log_prob 是action在前面那个正太分布的概率的log ，我们相信action是对的 ，
        # 那么我们要求的正态分布曲线中点应该在action这里，所以最大化正太分布的概率的log， 改变mu,sigma得出一条中心点更加在a的正太分布。
        dist_entropy = dist.entropy()

        batch=state.shape[0]
        state=self.bert(state)
        state=state.reshape([batch,-1])
        state_value = self.critic(state)

        return action_logprobs, torch.squeeze(state_value), dist_entropy

class PPO:
    def __init__(self, state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip):
        self.lr = lr
        self.betas = betas
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs

        self.bertconfig=BertConfig(max_position_embeddings=3, hidden_size=64, num_hidden_layers=4, num_attention_heads=4, intermediate_size=128)

        self.policy = ActorCritic(state_dim,self.bertconfig, action_dim, action_std).to(device)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr, betas=betas)

        self.MseLoss = nn.MSELoss()

    def select_action(self, state, memory):
        state = torch.FloatTensor(state.reshape(1,3, -1)).to(device)
        return self.policy.act(state, memory).cpu().data.numpy()

    def store(self,action, memory):
        self.policy.store(action, memory)

    def update(self, memory):
        # Monte Carlo estimate of rewards:
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)

        # Normalizing the rewards:
        rewards = torch.tensor(rewards, dtype=torch.float32).to(device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)

        # convert list to tensor
        # 使用stack可以保留两个信息：[1. 序列] 和 [2. 张量矩阵] 信息，属于【扩张再拼接】的函数；
        old_states = torch.squeeze(torch.stack(memory.states).to(device), 1).detach()
        old_actions = torch.squeeze(torch.stack(memory.actions).to(device), 1).detach()
        old_logprobs = torch.squeeze(torch.stack(memory.logprobs), 1).to(device).detach()
        #这里即可以对样本进行多次利用，提高利用率
        # Optimize policy for K epochs:
        for _ in range(self.K_epochs):
            # Evaluating old actions and values :
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)

            # Finding the ratio (pi_theta / pi_theta__old):
            ratios = torch.exp(logprobs - old_logprobs.detach())

            # Finding Surrogate Loss:
            advantages = rewards - state_values.detach()
            '''target = (memory.rewards + memory.next_state_values * self.gamma).detach()
            advantages = (target - state_values).detach()'''
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
            #loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, target) - 0.01 * dist_entropy

            # take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()