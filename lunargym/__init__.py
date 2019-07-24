
from gym.envs.registration import register

register(
    id='lunar-v0',
    entry_point='lunargym.envs:lunarEnv',
)