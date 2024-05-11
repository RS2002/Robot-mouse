from environment import Env
from ARS import ARS
import torch
import argparse


def get_args():
    parser = argparse.ArgumentParser(description='')
    parser.add_argument("--cpu", action="store_true",default=False)
    parser.add_argument("--cuda", type=str, default='0')

    parser.add_argument('--modelPath', type=str, default="./models/dynamic_4l_t3.xml")
    parser.add_argument('--state_dim', type=int, default=31)
    parser.add_argument('--action_dim', type=int, default=12)
    parser.add_argument('--max_steps', type=int, default=5000) # 每个episode内的最大步数
    parser.add_argument('--max_epoch', type=int, default=500) # 最大训练epoch
    parser.add_argument('--episode_num', type=int, default=16) # 每个epoch内的episode数目
    parser.add_argument('--update_num', type=int, default=10) # 每个epoch内更新的episode数目
    parser.add_argument('--learning_rate', type=float, default=0.03)
    parser.add_argument('--exploration_noise', type=float, default=0.05)


    args = parser.parse_args()
    return args


def main():
    args=get_args()

    device_name = "cuda:"+args.cuda
    device = torch.device(device_name if torch.cuda.is_available() and not args.cpu else 'cpu')

    env=Env(modelPath=args.modelPath,max_steps=args.max_steps)
    ars=ARS(state_dim=args.state_dim,action_dim=args.action_dim,device=device,env=env,max_epoch=args.max_epoch,episode_num=args.episode_num,update_num=args.update_num,learning_rate=args.learning_rate,exploration_noise=args.exploration_noise)
    ars.train()

if __name__ == '__main__':
    main()