import numpy as np
import pickle
from Bezier import get_Bezier_point,pos_2_angle

L_span = 0.03
psi_min = 0
psi_max = 0.01
delta_max = 0
delta_min = -0.005
delta_St = np.array([0, 0.5, 0.5, 0])
dt = 0.012
df = [0.00, -0.045]
dh = [0.00, -0.05]
#start_step=20
start_step=0



class Normalizer():
    """ this ensures that the policy puts equal weight upon
        each state component.
    """

    # Normalizes the states
    def __init__(self, state_dim):
        """ Initialize state space (all zero)
        """
        self.state = np.zeros(state_dim)
        self.mean = np.zeros(state_dim)
        self.mean_diff = np.zeros(state_dim)
        self.var = np.zeros(state_dim)

    def observe(self, x):
        """ Compute running average and variance
            clip variance >0 to avoid division by zero
        """
        self.state += 1.0
        last_mean = self.mean.copy()

        # running avg
        self.mean += (x - self.mean) / self.state

        # used to compute variance
        self.mean_diff += (x - last_mean) * (x - self.mean)
        # variance
        self.var = (self.mean_diff / self.state).clip(min=1e-2)

    def normalize(self, states):
        """ subtract mean state value from current state
            and divide by standard deviation (sqrt(var))
            to normalize
        """
        state_mean = self.mean
        state_std = np.sqrt(self.var)
        return (states - state_mean) / state_std

class Policy():
    """ state --> action
    """
    def __init__(
            self,
            state_dim,
            action_dim,
            # how much weights are changed each step
            learning_rate=0.03,
            # number of random expl_noise variations generated
            # each step
            # each one will be run for 2 epochs, + and -
            num_deltas=16,
            # used to update weights, sorted by highest rwrd
            num_best_deltas=16,
            # weight of sampled exploration noise
            expl_noise=0.05, #TODO
            # for seed gen
            seed=0):

        # Tunable Hyperparameters
        self.learning_rate = learning_rate
        self.num_deltas = num_deltas
        self.num_best_deltas = num_best_deltas
        # there cannot be more best_deltas than there are deltas
        assert self.num_best_deltas <= self.num_deltas
        self.expl_noise = expl_noise
        self.seed = seed
        np.random.seed(seed)
        self.state_dim = state_dim
        self.action_dim = action_dim

        # input/ouput matrix with weights set to zero
        # this is the perception matrix (policy)
        self.theta = np.zeros((action_dim, state_dim))

    def evaluate(self, state, delta=None, direction=None):
        """ state --> action
        """

        # if direction is None, deployment mode: takes dot product
        # to directly sample from (use) policy
        if direction is None:
            return self.theta.dot(state)

        # otherwise, add (+-) directed expl_noise before taking dot product (policy)
        # this is where the 2*num_deltas rollouts comes from
        elif direction == "+":
            return (self.theta + self.expl_noise * delta).dot(state)
        elif direction == "-":
            return (self.theta - self.expl_noise * delta).dot(state)

    def sample_deltas(self):
        """ generate array of random expl_noise matrices. Length of
            array = num_deltas
            matrix dimension: pxn where p=observation dim and
            n=action dim
        """
        deltas = []
        # print("SHAPE THING with *: {}".format(*self.theta.shape))
        # print("SHAPE THING NORMALLY: ({}, {})".format(self.theta.shape[0],
        #                                               self.theta.shape[1]))
        # print("ACTUAL SHAPE: {}".format(self.theta.shape))
        # print("SHAPE OF EXAMPLE DELTA WITH *: {}".format(
        #     np.random.randn(*self.theta.shape).shape))
        # print("SHAPE OF EXAMPLE DELTA NOMRALLY: {}".format(
        #     np.random.randn(self.theta.shape[0], self.theta.shape[1]).shape))

        for _ in range(self.num_deltas):
            deltas.append(
                np.random.randn(self.theta.shape[0], self.theta.shape[1]))

        return deltas

    def update(self, rollouts, std_dev_rewards):
        """ Update policy weights (theta) based on rewards
            from 2*num_deltas rollouts
        """
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, delta in rollouts:
            # how much to deviate from policy
            step += (r_pos - r_neg) * delta
        self.theta += self.learning_rate / (self.num_best_deltas *
                                            std_dev_rewards) * step

alpha=0.7
class ARS_agent():
    def __init__(self,env,policy,normalizer):
        self.env=env
        self.policy=policy
        self.normalizer=normalizer

    def train(self):
        # Initializing the perturbations deltas and the positive/negative rewards
        deltas = self.policy.sample_deltas()
        # Initialize +- reward list of size num_deltas
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas

        for i in range(self.policy.num_deltas):
            old_action=0
            St=0
            done=False
            reward_sum=0
            state=self.env.reset()
            state = np.concatenate([state,St+delta_St])
            step=0
            while not done:
                step+=1
                self.normalizer.observe(state)
                state = self.normalizer.normalize(state)
                action = self.policy.evaluate(state, deltas[i], '+')
                action = np.tanh(action)
                # action = np.tanh(action) *0.025
                # action = alpha * old_action + (1 - alpha) * action
                old_action = action[:]
                ctrl = action[:-2]
                '''psi = np.clip(action[-1], psi_min, psi_max)
                delta = np.clip(action[-2], delta_min, delta_max)'''
                '''psi=(1+action[-1])/2*(psi_max-psi_min)+psi_min
                delta=(1+action[-2])/2*(delta_max-delta_min)+delta_min'''
                psi=action[-1]*0.02
                delta=action[-2]*0.02
                for i in range(4):
                    x,y = get_Bezier_point(L_span, psi, delta, St + delta_St[i])
                    if i<2:
                        leg="f"
                        dx=df[0]
                        dy=df[1]
                    else:
                        leg="h"
                        dx=dh[0]
                        dy=dh[1]
                    if step < start_step:
                        action[i*2:(i+1)*2]=0
                    '''X=x+action[i*2]+dx
                    Y=y+action[i*2+1]+dy'''
                    '''X =  action[i * 2] + dx
                    Y =  action[i * 2 + 1] + dy'''
                    X = x + dx
                    Y = y + dy
                    ctrl[i * 2:(i + 1) * 2] = pos_2_angle(X, Y, leg)
                ctrl[-4:]=0
                # ctrl = action[:12]
                # ctrl[-4:] = 0
                state, reward, done, _ = self.env.runStep(ctrl)
                St=(St+dt)%2
                state = np.concatenate([state,St+delta_St])
                reward_sum+=reward
            positive_rewards[i]=reward_sum

        for i in range(self.policy.num_deltas):
            old_action=0
            St=0
            done=False
            reward_sum=0
            state=self.env.reset()
            state = np.concatenate([state,St+delta_St])
            step=0
            while not done:
                step+=1
                self.normalizer.observe(state)
                state = self.normalizer.normalize(state)
                action = self.policy.evaluate(state, deltas[i], '-')
                action = np.tanh(action)
                # action = np.tanh(action) *0.025
                # action = alpha * old_action + (1 - alpha) * action
                old_action = action[:]
                ctrl = action[:-2]
                '''psi = np.clip(action[-1], psi_min, psi_max)
                delta = np.clip(action[-2], delta_min, delta_max)'''
                '''psi=(1+action[-1])/2*(psi_max-psi_min)+psi_min
                delta=(1+action[-2])/2*(delta_max-delta_min)+delta_min'''
                psi=action[-1]*0.02
                delta=action[-2]*0.02
                for i in range(4):
                    x,y = get_Bezier_point(L_span, psi, delta, St + delta_St[i])
                    if i<2:
                        leg="f"
                        dx=df[0]
                        dy=df[1]
                    else:
                        leg="h"
                        dx=dh[0]
                        dy=dh[1]
                    if step < start_step:
                        action[i*2:(i+1)*2]=0
                    '''X=x+action[i*2]+dx
                    Y=y+action[i*2+1]+dy'''
                    '''X =  action[i * 2] + dx
                    Y =  action[i * 2 + 1] + dy'''
                    X = x + dx
                    Y = y + dy
                    ctrl[i * 2:(i + 1) * 2] = pos_2_angle(X, Y, leg)
                ctrl[-4:]=0
                # ctrl = action[:12]
                # ctrl[-4:] = 0
                state, reward, done, _ = self.env.runStep(ctrl)
                St=(St+dt)%2
                state = np.concatenate([state,St+delta_St])
                reward_sum+=reward
            negative_rewards[i]=reward_sum


        # Calculate std dev
        std_dev_rewards = np.array(positive_rewards + negative_rewards).std()

        # Order rollouts in decreasing list using cum reward as criterion
        # take max between reward for +- disturbance as that rollout's reward
        # Store max between positive and negative reward as key for sort
        scores = {
            k: max(r_pos, r_neg)
            for k, (
                r_pos,
                r_neg) in enumerate(zip(positive_rewards, negative_rewards))
        }
        indeces = sorted(scores.keys(), key=lambda x: scores[x],
                         reverse=True)[:self.policy.num_deltas]
        # print("INDECES: ", indeces)
        rollouts = [(positive_rewards[k], negative_rewards[k], deltas[k])
                    for k in indeces]

        # Update Policy
        self.policy.update(rollouts, std_dev_rewards)


    def eval(self):
        old_action = 0
        St = 0
        done = False
        reward_sum = 0
        state = self.env.reset()
        state = np.concatenate([state, St + delta_St])
        step = 0
        while not done:
            step += 1
            self.normalizer.observe(state)
            state = self.normalizer.normalize(state)
            action = self.policy.evaluate(state)
            #print(action)
            action = np.tanh(action)
            # action = np.tanh(action) *0.025
            # print(action)
            # action = alpha * old_action + (1 - alpha) * action
            old_action = action[:]
            ctrl = action[:-2]
            '''psi = np.clip(action[-1], psi_min, psi_max)
            delta = np.clip(action[-2], delta_min, delta_max)'''
            '''psi=(1+action[-1])/2*(psi_max-psi_min)+psi_min
            delta=(1+action[-2])/2*(delta_max-delta_min)+delta_min'''
            psi = action[-1] * 0.02
            delta = action[-2] * 0.02
            '''print(psi)
            print(delta)'''
            for i in range(4):
                x, y = get_Bezier_point(L_span, psi, delta, St + delta_St[i])
                if i < 2:
                    leg = "f"
                    dx = df[0]
                    dy = df[1]
                else:
                    leg = "h"
                    dx = dh[0]
                    dy = dh[1]
                if step < start_step:
                    action[i * 2:(i + 1) * 2] = 0
                '''X=x+action[i*2]+dx
                Y=y+action[i*2+1]+dy'''
                '''X =  action[i * 2] + dx
                Y =  action[i * 2 + 1] + dy'''
                X = x + dx
                Y = y + dy
                '''print(X,Y)
                print(pos_2_angle(X, Y, leg))'''
                ctrl[i * 2:(i + 1) * 2] = pos_2_angle(X, Y, leg)
            ctrl[-4:] = 0
            #print(ctrl)
            # ctrl=action[:12]
            # ctrl[-4:] = 0
            state, reward, done, _ = self.env.runStep(ctrl)
            St = (St + dt) % 2
            state = np.concatenate([state, St + delta_St])
            reward_sum += reward
        return reward_sum



    def save(self,filename):
        with open(filename, 'wb') as filehandle:
            pickle.dump(self.policy.theta, filehandle)

    def load(self,filename):
        self.policy.theta = np.load(filename, allow_pickle=True)





