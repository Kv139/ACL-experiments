# docs and experiment results can be found at https://docs.cleanrl.dev/rl-algorithms/ppo/#ppo_continuous_actionpy
import os
import random
import time
import scenic
from dataclasses import dataclass
from metadrive.component.sensors.semantic_camera import SemanticCamera
from custom.custom_simulator import MetaDriveSimulation, MetaDriveSimulator
from scenic import setDebuggingOptions



import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import tyro
from torch.distributions.normal import Normal
from torch.utils.tensorboard import SummaryWriter
# MODIFIED: was `from custom.custom_gym import CustomMetaDriveEnv`.
# custom_gym.CustomMetaDriveEnv.__init__ does not accept total_timesteps /
# buffer_capacity / start_genetic / nk_buffer, which make_env has passed since
# commit 536752e -- so the script raised TypeError before MetaDrive ever started.
# gym_w_buffer is also the env the policies were actually trained under.
from custom.gym_w_buffer import CustomMetaDriveEnv
from typing import Callable

@dataclass
class Args:
    exp_name: str = os.path.basename(__file__)[: -len(".py")]
    """the name of this experiment"""
    seed: int = 1
    """seed of the experiment"""
    torch_deterministic: bool = True
    """if toggled, `torch.backends.cudnn.deterministic=False`"""
    cuda: bool = False
    """if toggled, cuda will be enabled by default"""

    # Scenic specific arguments
    scenic_file: str = "./scenarios/driver.scenic"
    """the scenic program defining the enviroment"""
    max_steps: int = 1000
    """the maximum number of steps for any given episode"""
    model : str = "scenic.simulators.metadrive.model"
    """ underlying model for the scenic file """
    map: str = "./../CARLA/Town01.net.xml"
    """ Sumo map for the env"""
    sampler_type: str = "random"
    """ sampling type for generating scenes"""
    # NOTE: this default points at a run directory that does not exist under runs/.
    # Pass --model-to-evaluate-path explicitly.
    model_to_evaluate_path: str = "./runs/ACL_MetaDrive__ppo__77__1785190664/ppo.cleanrl_model"
    """Path for model to evaluate"""
    model_name: str = "ep5_timesteps_1m_random_ng"
    """ model name"""
    # MODIFIED: these three were missing type annotations, so dataclass/tyro treated
    # them as plain class attributes rather than fields -- they could not be set from
    # the CLI and did not appear in vars(args) for the hyperparameter dump.
    apply_genetic_ops: int = 0
    """ flag to signal genetic operations for scene contsruction"""
    render: int = 1
    """ Whether to render the simulation or not"""
    run_name: str = "ppo_eval_1"
    """Target name for trial runs"""
    # ADDED: number of episodes to evaluate over (was hard-coded at the call site).
    eval_episodes: int = 10
    """how many episodes to roll out for evaluation"""

    # Algorithm specific arguments
    env_id: str = "ACL_MetaDrive"
    """the id of the environment"""
    total_timesteps: int = 20000
    """total timesteps of the experiments"""
    learning_rate: float = 3e-4
    """the learning rate of the optimizer"""
    num_envs: int = 1
    """the number of parallel game environments"""
    num_steps: int = 2048
    """the number of steps to run in each environment per policy rollout"""
    anneal_lr: bool = True
    """Toggle learning rate annealing for policy and value networks"""
    gamma: float = 0.99
    """the discount factor gamma"""
    gae_lambda: float = 0.95
    """the lambda for the general advantage estimation"""
    num_minibatches: int = 32
    """the number of mini-batches"""
    update_epochs: int = 10
    """the K epochs to update the policy"""
    norm_adv: bool = True
    """Toggles advantages normalization"""
    clip_coef: float = 0.2
    """the surrogate clipping coefficient"""
    clip_vloss: bool = True
    """Toggles whether or not to use a clipped loss for the value function, as per the paper."""
    ent_coef: float = 0.0
    """coefficient of the entropy"""
    vf_coef: float = 0.5
    """coefficient of the value function"""
    max_grad_norm: float = 0.5
    """the maximum norm for the gradient clipping"""
    target_kl: float = None
    """the target KL divergence threshold"""

    # to be filled in runtime
    batch_size: int = 0
    """the batch size (computed in runtime)"""
    minibatch_size: int = 0
    """the mini-batch size (computed in runtime)"""
    num_iterations: int = 0
    """the number of iterations (computed in runtime)"""

    """Total number of scenes to store in a single buffer"""
    capacity: int = 100
    """After how many episodes should genetic ops start"""
    start_genetic: int = 25
    """Generate a n X k dimensional buffer for retaining scenes"""
    nk_buffer: bool = True


def make_env(env_id, idx, capture_video, run_name, gamma):
    def thunk():

        scenario = (scenic.scenarioFromFile(args.scenic_file,
                                        model=args.model,
                                        mode2D=True,
                                        params={"verifaiSamplerType": args.sampler_type}))

        observation_space =observation_space=gym.spaces.Box(low=-np.inf, high=np.inf, shape=(268,))

        action_space = gym.spaces.Box(low=np.array([-1,-1]), high=np.array([1,1]), shape=(2,), dtype=np.float32)  # Defines the possible actions of the agent

        # MODIFIED: log_dir added so evaluation episode CSVs land in eval_logs/ instead
        # of overwriting the training logs/ directory. Everything else is unchanged.
        # NOTE: log_episode_stats() is never called during evaluation, so the pvl /
        # reward_sum / num_steps columns in those CSVs stay at 0; the scene parameter
        # columns and termination_reason are the useful part.
        env = CustomMetaDriveEnv(
            scenario=scenario,
            simulator=MetaDriveSimulator(sumo_map=args.map),
            max_steps=args.max_steps,
            observation_space=observation_space,
            action_space=action_space,
            file = args.scenic_file,
            genetic_flag = bool(args.apply_genetic_ops),
            total_timesteps=args.total_timesteps,
            buffer_capacity=args.capacity,
            start_genetic=args.start_genetic,
            nk_buffer = bool(args.nk_buffer),
            log_dir="eval_logs",
        )

        # NOTE: the observation/reward normalization statistics are re-estimated from
        # scratch here and keep updating during evaluation, because training only
        # checkpoints agent.state_dict() and never saved its running means. Left as-is
        # to match how the policy saw observations during training; if you start saving
        # the wrapper statistics alongside the model, load them here and set
        # `update_running_mean = False` on both wrappers instead.
        env = gym.wrappers.FlattenObservation(env)  # deal with dm_control's Dict observation space
        env = gym.wrappers.RecordEpisodeStatistics(env)
        env = gym.wrappers.ClipAction(env)
        env = gym.wrappers.TransformObservation(env, lambda obs: np.clip(obs, -10, 10), observation_space=env.observation_space)
        env = gym.wrappers.TransformReward(env, lambda reward: np.clip(reward, -10, 10))
        return env

    return thunk



def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


class Agent(nn.Module):
    def __init__(self, envs):
        super().__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, np.prod(envs.single_action_space.shape)), std=0.01),
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(envs.single_action_space.shape)))

    def get_value(self, x):
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        action_mean = self.actor_mean(x)
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(x)



def evaluate(
    model_path: str,
    make_env: Callable,
    env_id: str,
    eval_episodes: int,
    run_name: str,
    Model: torch.nn.Module,
    device: torch.device = torch.device("cpu"),
    capture_video: bool = True,
    gamma: float = 0.99,
):
    # MODIFIED: the whole body is wrapped in try/finally so that envs.close() always
    # runs. This is the segfault fix. CustomMetaDriveEnv.loop is a generator suspended
    # at a `yield` *inside* `with simulator.simulateStepped(...)`. If the env is left to
    # the garbage collector, GeneratorExit unwinds that context manager at interpreter
    # shutdown, running MetaDriveSimulation.destroy() -> client.engine.agent_manager
    # .clear_objects(...) against Panda3D C++ objects that may already be torn down.
    # Closing here unwinds it deterministically while the engine is still alive.
    envs = gym.vector.SyncVectorEnv([make_env(env_id, 0, capture_video, run_name, gamma)])
    try:
        # MODIFIED: the agent is built and loaded here, from the one vector env this
        # process owns. Previously __main__ built a second SyncVectorEnv purely to shape
        # the network, which eagerly constructed a second MetaDrive DriveEnv -- and
        # MetaDrive's engine (and Panda3D's ShowBase) is a process-wide singleton.
        assert isinstance(envs.single_action_space, gym.spaces.Box), "only continuous action space is supported"

        agent = Model(envs).to(device)
        agent.load_state_dict(torch.load(model_path, map_location=device))
        agent.eval()

        obs, _ = envs.reset()
        episodic_returns = []
        while len(episodic_returns) < eval_episodes:
            # MODIFIED: no_grad added. The action sampling itself is unchanged -- this
            # is still a stochastic rollout from Normal(actor_mean, exp(actor_logstd)).
            with torch.no_grad():
                actions, _, _, _ = agent.get_action_and_value(torch.Tensor(obs).to(device))
            obs, _, _, _, infos = envs.step(actions.cpu().numpy())


            # MODIFIED: gymnasium 1.3.0 removed final_observation/final_info from the
            # vector API and moved to next-step autoreset, so the old `if "final_info"
            # in infos` branch was dead -- episodes finished but nothing was ever
            # recorded, hence the infinite while loop. (The same dead branch in ppo.py
            # is why no run under runs/ has a charts/episodic_return scalar.)
            # Under 1.3.0, RecordEpisodeStatistics on the sub-env writes
            # info["episode"] = {"r", "l", "t"} on the terminal step; SyncVectorEnv
            # batches that into infos["episode"]["r"] (shape (num_envs,)) plus the
            # completion mask infos["_episode"]. The mask is required: entries for
            # envs that did not terminate this step are zeros, not returns.
            if "episode" in infos:
                for i in np.nonzero(infos["_episode"])[0]:
                    episodic_return = float(infos["episode"]["r"][i])
                    print(f"eval_episode={len(episodic_returns)}, episodic_return={episodic_return}")
                    episodic_returns.append(episodic_return)

        return episodic_returns
    finally:
        envs.close()


if __name__ == "__main__":
    setDebuggingOptions(verbosity=0)

    args = tyro.cli(Args)
    args.batch_size = int(args.num_envs * args.num_steps)
    args.minibatch_size = int(args.batch_size // args.num_minibatches)
    args.num_iterations = args.total_timesteps // args.batch_size
    run_name = args.run_name
    writer = SummaryWriter(f"runs/{run_name}")
    writer.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n%s" % ("\n".join([f"|{key}|{value}|" for key, value in vars(args).items()])),
    )

    # TRY NOT TO MODIFY: seeding
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.backends.cudnn.deterministic = args.torch_deterministic

    device = torch.device("cuda" if torch.cuda.is_available() and args.cuda else "cpu")

    # MODIFIED: the module-level SyncVectorEnv, the isinstance assert and the throwaway
    # Agent(envs) that used to live here are gone -- they built a second MetaDrive
    # DriveEnv that was never reset, and then envs.close() at the bottom closed *that*
    # one while the env holding the live engine was never closed at all. evaluate() now
    # constructs, uses and closes the single env in this process.
    episodic_returns = evaluate(
        args.model_to_evaluate_path,
        make_env,
        args.env_id,
        eval_episodes=args.eval_episodes,
        run_name=f"{run_name}-eval",
        Model=Agent,
        device=device,
        gamma=args.gamma,
    )
    for idx, episodic_return in enumerate(episodic_returns):
        writer.add_scalar("eval/episodic_return", episodic_return, idx)

    # MODIFIED: reports the aggregate the evaluation is actually for.
    print(
        f"mean_episodic_return={np.mean(episodic_returns):.3f} "
        f"+/- {np.std(episodic_returns):.3f} over {len(episodic_returns)} episodes"
    )

    # writer.close()
